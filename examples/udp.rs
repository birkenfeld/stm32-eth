#![no_std]
#![no_main]

extern crate panic_itm;

use cortex_m::{iprintln, interrupt};
use cortex_m_rt::{entry, exception};
use stm32f4xx_hal::{
    gpio::GpioExt,
    stm32::{Peripherals, CorePeripherals, SYST},
};

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr};
use smoltcp::iface::{NeighborCache, EthernetInterfaceBuilder};
use smoltcp::socket::{SocketSet, UdpSocket, UdpSocketBuffer};
use smoltcp::storage::PacketMetadata;
use log::{Record, Level, Metadata, LevelFilter, info, warn};

use stm32_eth::{Eth, RingEntry};

static mut LOGGER: ItmLogger = ItmLogger;

struct ItmLogger;

impl log::Log for ItmLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= Level::Trace
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let mut cp = unsafe { CorePeripherals::steal() };
            iprintln!(&mut cp.ITM.stim[0], "[{}] {}", record.level(), record.args());
        }
    }

    fn flush(&self) {}
}

static TIME: Mutex<RefCell<u64>> = Mutex::new(RefCell::new(0));

/* *** MINE ***
pub fn setup(p: &Peripherals) {
    let pll_n_bits = 336;
    let hpre_bits = 0b111;
    let ppre2_bits = 0b100;
    let ppre1_bits = 0b101;

    // adjust flash wait states
    p.FLASH.acr.modify(|_, w| unsafe { w.latency().bits(0b110) });

    // use PLL as source
    p.RCC.pllcfgr.modify(|_, w| unsafe { w.plln().bits(pll_n_bits as u16) });

    // Enable PLL
    p.RCC.cr.modify(|_, w| w.pllon().set_bit());
    // Wait for PLL ready
    while p.RCC.cr.read().pllrdy().bit_is_clear() {}

    // enable PLL
    p.RCC.cfgr.write(|w| unsafe {
                w
                    // APB high-speed prescaler (APB2)
                    .ppre2()
                    .bits(ppre2_bits)
                    // APB Low speed prescaler (APB1)
                    .ppre1()
                    .bits(ppre1_bits)
                    // AHB prescaler
                    .hpre()
                    .bits(hpre_bits)
                    // System clock switch
                    // PLL selected as system clock
                    .sw1().bit(true)
                    .sw0().bit(false)
            });

    init_pins(&p.RCC, &p.GPIOA, &p.GPIOB, &p.GPIOC, &p.GPIOG);
*/

#[entry]
fn main() -> ! {
    unsafe { log::set_logger(&LOGGER).unwrap(); }
    log::set_max_level(LevelFilter::Debug);

    let p = Peripherals::take().unwrap();
    let mut cp = CorePeripherals::take().unwrap();

    setup_systick(&mut cp.SYST);

    stm32_eth::setup(&p.RCC, &p.SYSCFG);
    let gpioa = p.GPIOA.split();
    let gpiob = p.GPIOB.split();
    let gpioc = p.GPIOC.split();
    let gpiog = p.GPIOG.split();
    stm32_eth::setup_pins(
        gpioa.pa1, gpioa.pa2, gpioa.pa7, gpiob.pb13, gpioc.pc1,
        gpioc.pc4, gpioc.pc5, gpiog.pg11, gpiog.pg13
    );

    let mut rx_ring: [RingEntry<_>; 16] = Default::default();
    let mut tx_ring: [RingEntry<_>; 4] = Default::default();
    let mut eth = Eth::new(
        p.ETHERNET_MAC, p.ETHERNET_DMA,
        &mut rx_ring[..], &mut tx_ring[..]
    );

    // enable RNG
    p.RCC.ahb2enr.modify(|_, w| w.rngen().set_bit());
    p.RNG.cr.modify(|_, w| w.rngen().set_bit());

    let ip_addr = IpCidr::new(IpAddress::v4(10, 0, 0, 1), 24);
    let mut ip_addrs = [ip_addr];
    let mut neighbor_storage = [None; 16];
    let neighbor_cache = NeighborCache::new(&mut neighbor_storage[..]);
    let serial: u32 = unsafe {
        *(0x1FFF_7A10 as *const u32) ^
        *(0x1FFF_7A14 as *const u32) ^
        *(0x1FFF_7A18 as *const u32)
    };
    let ethernet_addr = EthernetAddress([
        0x46, 0x52, 0x4d,  // F R M
        (serial >> 16) as u8, (serial >> 8) as u8, serial as u8
    ]);
    let mut iface = EthernetInterfaceBuilder::new(&mut eth)
        .ethernet_addr(ethernet_addr)
        .ip_addrs(&mut ip_addrs[..])
        .neighbor_cache(neighbor_cache)
        .finalize();

    let mut rx_meta_buffer = [PacketMetadata::EMPTY; 4];
    let mut tx_meta_buffer = [PacketMetadata::EMPTY; 4];
    let mut rx_data_buffer = [0; 1500*4];
    let mut tx_data_buffer = [0; 1500*4];
    let mut udp_socket = UdpSocket::new(
        UdpSocketBuffer::new(&mut rx_meta_buffer[..], &mut rx_data_buffer[..]),
        UdpSocketBuffer::new(&mut tx_meta_buffer[..], &mut tx_data_buffer[..])
    );
    udp_socket.bind((IpAddress::v4(10, 0, 0, 1), 50000)).unwrap();
    let mut sockets_storage = [None, None];
    let mut sockets = SocketSet::new(&mut sockets_storage[..]);
    let handle = sockets.add(udp_socket);
    let mut target = None;

    info!("------------------------------------------------------------------------");
    let mut seq_number = 0u32;
    loop {
        //let random = p.RNG.dr.read().bits();
        {
            let mut socket = sockets.get::<UdpSocket>(handle);
            // if socket.can_recv() {
                while let Ok((msg, ep)) = socket.recv() {
                    if msg == b"start" {
                        info!("got start message");
                        target = Some(ep);
                        seq_number = 0;
                    } else if msg == b"stop " {
                        info!("got stop message");
                        target = None;
                    }
                }
            // }
            if let Some(tgt) = target {
                while let Ok(buf) = socket.send(1400, tgt) {
                    buf[0] = (seq_number >> 24) as u8;
                    buf[1] = (seq_number >> 16) as u8;
                    buf[2] = (seq_number >> 8) as u8;
                    buf[3] = seq_number as u8;
                    seq_number += 1;
                }
            }
        }
        let time = interrupt::free(|cs| *TIME.borrow(cs).borrow());
        if let Err(e) = iface.poll(&mut sockets, Instant::from_millis(time as i64)) {
            warn!("{}", e);
        }
    }
}

fn setup_systick(syst: &mut SYST) {
    syst.set_reload(SYST::get_ticks_per_10ms() / 10);
    syst.enable_counter();
    syst.enable_interrupt();
}

#[exception]
fn SysTick() {
    cortex_m::interrupt::free(|cs| *TIME.borrow(cs).borrow_mut() += 1);
}
