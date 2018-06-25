#![no_std]
#![no_main]
#![feature(cell_update)]

extern crate panic_itm;

use cortex_m::{iprintln, interrupt, peripheral};
use cortex_m_rt::{entry, exception};
use stm32f4xx_hal::{
    gpio::GpioExt,
    stm32::{Peripherals, CorePeripherals, SYST},
};

use core::cell::Cell;
use cortex_m::interrupt::Mutex;

use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, Ipv4Address, IpCidr};
use smoltcp::iface::{NeighborCache, EthernetInterfaceBuilder, Routes};
use smoltcp::socket::{SocketSet, UdpSocket, UdpSocketBuffer, RawSocketBuffer};
use smoltcp::storage::PacketMetadata;
use smoltcp::dhcp::Dhcpv4Client;
use log::{Record, Level, Metadata, LevelFilter, info, warn};

use stm32_eth::{Eth, RingEntry};

struct ItmLogger;

impl log::Log for ItmLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= Level::Trace
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let stim = unsafe { &mut (*peripheral::ITM::ptr()).stim[0] };
            iprintln!(stim, "[{}] {}", record.level(), record.args());
        }
    }

    fn flush(&self) {}
}

static LOGGER: ItmLogger = ItmLogger;
static ETH_TIME: Mutex<Cell<i64>> = Mutex::new(Cell::new(0));

#[entry]
fn main() -> ! {
    log::set_logger(&LOGGER).unwrap();
    log::set_max_level(LevelFilter::Info);

    let p = Peripherals::take().unwrap();
    let mut cp = CorePeripherals::take().unwrap();

    setup_clock(&p);
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

    let serial = read_serno();
    let ethernet_addr = EthernetAddress([
        0x46, 0x52, 0x4d,  // F R M
        (serial >> 16) as u8, (serial >> 8) as u8, serial as u8
    ]);
    let mut ip_addrs = [IpCidr::new(Ipv4Address::UNSPECIFIED.into(), 0)];
    let mut neighbor_storage = [None; 16];
    let mut routes_storage = [None; 2];
    let mut iface = EthernetInterfaceBuilder::new(&mut eth)
        .ethernet_addr(ethernet_addr)
        .ip_addrs(&mut ip_addrs[..])
        .neighbor_cache(NeighborCache::new(&mut neighbor_storage[..]))
        .routes(Routes::new(&mut routes_storage[..]))
        .finalize();

    let mut udp_rx_meta_buffer = [PacketMetadata::EMPTY; 4];
    let mut udp_tx_meta_buffer = [PacketMetadata::EMPTY; 4];
    let mut udp_rx_data_buffer = [0; 1500*4];
    let mut udp_tx_data_buffer = [0; 1500*4];
    let mut dhcp_rx_meta_buffer = [PacketMetadata::EMPTY; 1];
    let mut dhcp_tx_meta_buffer = [PacketMetadata::EMPTY; 1];
    let mut dhcp_rx_data_buffer = [0; 1500];
    let mut dhcp_tx_data_buffer = [0; 1500*2];

    let udp_socket = UdpSocket::new(
        UdpSocketBuffer::new(&mut udp_rx_meta_buffer[..], &mut udp_rx_data_buffer[..]),
        UdpSocketBuffer::new(&mut udp_tx_meta_buffer[..], &mut udp_tx_data_buffer[..])
    );
    let mut sockets_storage = [None, None];
    let mut sockets = SocketSet::new(&mut sockets_storage[..]);
    let mut target = None;

    let dhcp_rx_buffer = RawSocketBuffer::new(&mut dhcp_rx_meta_buffer[..], &mut dhcp_rx_data_buffer[..]);
    let dhcp_tx_buffer = RawSocketBuffer::new(&mut dhcp_tx_meta_buffer[..], &mut dhcp_tx_data_buffer[..]);
    let mut dhcp = Dhcpv4Client::new(&mut sockets, dhcp_rx_buffer, dhcp_tx_buffer, Instant::from_millis(0));

    let udp_handle = sockets.add(udp_socket);
    let mut seq_number = 0u32;
    let mut dhcp_msgs = 0;

    info!("------------------------------------------------------------------------");
    loop {
        {
            let mut socket = sockets.get::<UdpSocket>(udp_handle);
            while let Ok((msg, ep)) = socket.recv() {
                if msg == b"start" {
                    info!("got start message");
                    target = Some(ep);
                    seq_number = 0;
                } else if msg == b"stop" {
                    info!("got stop message");
                    target = None;
                }
            }
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
        let time = Instant::from_millis(interrupt::free(|cs| ETH_TIME.borrow(cs).get()));
        if let Err(e) = iface.poll(&mut sockets, time) {
            warn!("poll: {}", e);
        }
        if dhcp_msgs < 2 {
            match dhcp.poll(&mut iface, &mut sockets, time) {
                Err(e) => warn!("dhcp: {}", e),
                Ok(Some(config)) => match config.address {
                    Some(cidr) => {
                        info!("got {}", cidr);
                        dhcp_msgs += 1;
                        iface.update_ip_addrs(
                            |addrs| addrs.iter_mut().for_each(|addr| *addr = IpCidr::Ipv4(cidr)));
                        let _ = sockets.get::<UdpSocket>(udp_handle).bind((cidr.address(), 50000));
                    }
                    _ => {}
                },
                _ => ()
            }
        }
    }
}

fn read_serno() -> u32 {
    unsafe {
        *(0x1FFF_7A10 as *const u32) ^
        *(0x1FFF_7A14 as *const u32) ^
        *(0x1FFF_7A18 as *const u32)
    }
}

fn setup_clock(p: &Peripherals) {
    // setup for 168 MHz, 84 MHz, 42 MHz
    let pll_n_bits = 336;
    let hpre_bits = 0b111;
    let ppre2_bits = 0b100;
    let ppre1_bits = 0b101;

    // adjust flash wait states
    p.FLASH.acr.modify(|_, w| unsafe { w.latency().bits(0b110) });

    // use PLL as source
    p.RCC.pllcfgr.modify(|_, w| unsafe { w.plln().bits(pll_n_bits as u16) });

    // enable PLL
    p.RCC.cr.modify(|_, w| w.pllon().set_bit());
    // wait for PLL ready
    while p.RCC.cr.read().pllrdy().bit_is_clear() {}

    // enable PLL
    p.RCC.cfgr.write(|w| unsafe { w
                                  // APB high-speed prescaler (APB2)
                                  .ppre2().bits(ppre2_bits)
                                  // APB Low speed prescaler (APB1)
                                  .ppre1().bits(ppre1_bits)
                                  // AHB prescaler
                                  .hpre().bits(hpre_bits)
                                  // System clock switch
                                  // PLL selected as system clock
                                  .sw().bits(0b10) });
}

fn setup_systick(syst: &mut SYST) {
    syst.set_reload(SYST::get_ticks_per_10ms() / 10);
    syst.enable_counter();
    syst.enable_interrupt();
}

#[exception]
fn SysTick() {
    interrupt::free(|cs| ETH_TIME.borrow(cs).update(|v| v.wrapping_add(1)));
}
