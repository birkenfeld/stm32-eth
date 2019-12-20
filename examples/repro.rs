#![no_std]
#![no_main]

extern crate panic_itm;

use cortex_m::{iprintln, interrupt, peripheral};
use cortex_m::interrupt::Mutex;
use cortex_m_rt::{entry, exception};
use stm32f4xx_hal::gpio::GpioExt;
use stm32f4xx_hal::stm32::{Peripherals, CorePeripherals, GPIOB, SYST, TIM2};

use core::cell::Cell;

use smoltcp::time::Instant;
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr, IpEndpoint};
use smoltcp::iface::{NeighborCache, EthernetInterfaceBuilder, Routes};
use smoltcp::socket::{SocketSet, UdpSocket, UdpSocketBuffer};
use smoltcp::storage::PacketMetadata;
use log::{Record, Metadata, LevelFilter, info, warn};

use stm32_eth::{Eth, RingEntry};

const PORT: u16 = 54321;
// size of each packet
const PKT_SIZE: usize = 1446;
// one packet every 10MHz/1200
const PKT_EVERY: u32 = 1200;

struct ItmLogger;

fn itm() -> &'static mut peripheral::itm::Stim {
    unsafe { &mut (*peripheral::ITM::ptr()).stim[0] }
}

impl log::Log for ItmLogger {
    fn log(&self, record: &Record) {
        iprintln!(itm(), "[{}] {}", record.level(), record.args());
    }

    fn enabled(&self, _: &Metadata) -> bool { true }
    fn flush(&self) {}
}

static LOGGER: ItmLogger = ItmLogger;
static ETH_TIME: Mutex<Cell<i64>> = Mutex::new(Cell::new(0));

#[entry]
fn main() -> ! {
    // enable logging if someone is listening on ITM
    if itm().is_fifo_ready() {
        log::set_logger(&LOGGER).unwrap();
        log::set_max_level(LevelFilter::Trace);
    }

    let p = Peripherals::take().unwrap();
    let mut cp = CorePeripherals::take().unwrap();

    setup_clock(&p);
    setup_systick(&mut cp.SYST);
    setup_10mhz(&p);
    stm32_eth::setup(&p.RCC, &p.SYSCFG);

    let gpioa = p.GPIOA.split();
    let gpiob = p.GPIOB.split();
    let gpioc = p.GPIOC.split();
    let gpiog = p.GPIOG.split();
    stm32_eth::setup_pins(
        gpioa.pa1, gpioa.pa2, gpioa.pa7, gpiob.pb13, gpioc.pc1,
        gpioc.pc4, gpioc.pc5, gpiog.pg11, gpiog.pg13
    );

    let _led_green = gpiob.pb0.into_push_pull_output();
    let _let_blue = gpiob.pb7.into_push_pull_output();
    let _let_red = gpiob.pb14.into_push_pull_output();

    // red LED: indicate "booting"
    set_leds(true, false, false);

    // set up ring buffers for network handling tokens
    let mut rx_ring: [RingEntry<_>; 16] = Default::default();
    let mut tx_ring: [RingEntry<_>; 16] = Default::default();
    let mut eth = Eth::new(
        p.ETHERNET_MAC, p.ETHERNET_DMA,
        &mut rx_ring[..], &mut tx_ring[..]
    );

    let ethernet_addr = EthernetAddress([0x46, 0x52, 0x4d, 0x01, 0x02, 0x03]);

    // select the default Mesytec IP if static configuration
    let mut ip_addrs = [IpCidr::new(IpAddress::v4(192, 168, 1, 222), 24)];
    let mut neighbor_storage = [None; 16];
    let mut routes_storage = [None; 2];
    let mut iface = EthernetInterfaceBuilder::new(&mut eth)
        .ethernet_addr(ethernet_addr)
        .ip_addrs(&mut ip_addrs[..])
        .neighbor_cache(NeighborCache::new(&mut neighbor_storage[..]))
        .routes(Routes::new(&mut routes_storage[..]))
        .finalize();

    // set up buffers for packet content and metadata
    let mut udp_rx_meta_buffer = [PacketMetadata::EMPTY; 4];
    let mut udp_tx_meta_buffer = [PacketMetadata::EMPTY; 12];
    let mut udp_rx_data_buffer = [0; 1500*4];
    let mut udp_tx_data_buffer = [0; 1500*12];

    // create the UDP socket
    let udp_socket = UdpSocket::new(
        UdpSocketBuffer::new(&mut udp_rx_meta_buffer[..], &mut udp_rx_data_buffer[..]),
        UdpSocketBuffer::new(&mut udp_tx_meta_buffer[..], &mut udp_tx_data_buffer[..])
    );
    let mut sockets_storage = [None, None];
    let mut sockets = SocketSet::new(&mut sockets_storage[..]);

    let udp_handle = sockets.add(udp_socket);

    let mut gen = Generator::new(p.TIM2);

    let ip_addr = iface.ipv4_addr().unwrap();
    sockets.get::<UdpSocket>(udp_handle).bind((ip_addr, PORT)).unwrap();
    set_leds(false, true, false);

    gen.start();

    // let mut buf = [0; 1500];
    loop {
        // process packets
        {
            let mut socket = sockets.get::<UdpSocket>(udp_handle);
            // socket.recv_slice(&mut buf);
            gen.maybe_send_data(&mut socket);
        }
        let time_ticks = interrupt::free(|cs| ETH_TIME.borrow(cs).get());
        // blink blue LED
        set_leds(false, true, time_ticks & 0x100 == 0);
        // handle ethernet
        if let Err(e) = iface.poll(&mut sockets, Instant::from_millis(time_ticks)) {
            warn!("poll: {}", e);
        }
    }
}

struct Generator {
    endpoint: IpEndpoint,
    timer: TIM2,
    pkt_no: u32,
    time: u64,
    lastpkt: u64,
    run: bool,
}

impl Generator {
    fn new(timer: TIM2) -> Self {
        Generator { timer, endpoint: (IpAddress::v4(192, 168, 1, 1), PORT).into(),
                    run: false, pkt_no: 0, time: 0, lastpkt: 0 }
    }

    fn maybe_send_data(&mut self, sock: &mut UdpSocket) {
        if !self.run {
            return;
        }

        // keep track of 64-bit time
        let low_time = self.timer.cnt.read().bits();
        let overflow = if low_time < self.time as u32 { 1 << 32 } else { 0 };
        self.time = ((self.time & 0xFFFF_FFFF_0000_0000) + overflow) | low_time as u64;

        // calculate time since last packet
        let elapsed = (self.time - self.lastpkt) as u32;
        if elapsed < PKT_EVERY {
            return;
        }

        // now we can send a packet
        match sock.send(PKT_SIZE, self.endpoint) {
            Ok(buf) => {
                self.pkt_no += 1;
                // packet number
                buf[0] = self.pkt_no as u8;
                buf[1] = (self.pkt_no >> 8) as u8;
                buf[2] = (self.pkt_no >> 16) as u8;
                buf[3] = (self.pkt_no >> 24) as u8;
                // time
                buf[4] = self.time as u8;
                buf[5] = (self.time >> 8) as u8;
                buf[6] = (self.time >> 16) as u8;
                buf[7] = (self.time >> 24) as u8;
                buf[8] = (self.time >> 32) as u8;
                buf[9] = (self.time >> 40) as u8;
                buf[10] = (self.time >> 48) as u8;
                buf[11] = (self.time >> 56) as u8;
                self.lastpkt = self.time - (elapsed % PKT_EVERY) as u64;
            }
            Err(e) => (), //warn!("send: {}", e),
        }
    }

    fn start(&mut self) {
        self.run = true;
        self.time = 0;
        self.lastpkt = 0;
        // reset the timer
        self.timer.cnt.write(|w| unsafe { w.bits(0) });
        self.timer.cr1.write(|w| w.cen().set_bit());
    }

    fn stop(&mut self) {
        self.run = false;
        self.timer.cr1.write(|w| w.cen().clear_bit());
        set_leds(false, true, false);
    }
}

fn setup_clock(p: &Peripherals) {
    // setup clocks for 180 MHz, 90 MHz, 45 MHz from 8MHz HSE (Nucleo MCO)
    let pll_m = 8;
    let pll_n = 360; // fVCO = 360 MHz
    let pll_p = 2;
    let pll_q = 8; // to get <= 48 MHz
    let flash_latency = 5;
    let ahb_div  = 0b111;
    let apb2_div = 0b100; // div2
    let apb1_div = 0b101; // div4

    // enable HSE
    p.RCC.cr.modify(|_, w| w.hseon().set_bit());
    while p.RCC.cr.read().hserdy().bit_is_clear() {}

    // select regulator voltage scale 1
    p.RCC.apb1enr.modify(|_, w| w.pwren().set_bit());
    p.PWR.cr.modify(|_, w| unsafe { w.vos().bits(0b11) });

    // configure PLL frequency
    p.RCC.pllcfgr.modify(|_, w| unsafe { w.pllm().bits(pll_m)
                                          .plln().bits(pll_n)
                                          .pllp().bits((pll_p >> 1) - 1)
                                          .pllq().bits(pll_q)
                                          .pllsrc().set_bit() });

    // enable PLL
    p.RCC.cr.modify(|_, w| w.pllon().set_bit());
    // wait for PLL ready
    while p.RCC.cr.read().pllrdy().bit_is_clear() {}

    // enable overdrive
    p.PWR.cr.modify(|_, w| w.oden().set_bit());
    while p.PWR.csr.read().odrdy().bit_is_clear() {}
    p.PWR.cr.modify(|_, w| w.odswen().set_bit());
    while p.PWR.csr.read().odswrdy().bit_is_clear() {}

    // adjust icache and flash wait states
    p.FLASH.acr.modify(|_, w| unsafe { w.icen().set_bit()
                                       .dcen().set_bit()
                                       .prften().set_bit()
                                       .latency().bits(flash_latency) });

    // enable PLL as clock source
    p.RCC.cfgr.write(|w| unsafe { w
                                  // APB high-speed prescaler (APB2)
                                  .ppre2().bits(apb2_div)
                                  // APB low-speed prescaler (APB1)
                                  .ppre1().bits(apb1_div)
                                  // AHB prescaler
                                  .hpre().bits(ahb_div)
                                  // PLL selected as system clock
                                  .sw().pll() });
    while !p.RCC.cfgr.read().sws().is_pll() {}
}

fn setup_systick(syst: &mut SYST) {
    // systick is used for advancing the Ethernet clock for timeouts etc.
    syst.set_reload(22_500 - 1); // every ms
    syst.enable_counter();
    syst.enable_interrupt();
}

fn set_leds(red: bool, green: bool, blue: bool) {
    let gpiob = unsafe { &(*GPIOB::ptr()) };
    gpiob.odr.modify(|_, w| w.odr0().bit(green).odr7().bit(blue).odr14().bit(red));
}

fn setup_10mhz(p: &Peripherals) {
    p.RCC.apb1enr.modify(|_, w| w.tim2en().set_bit());
    p.TIM2.psc.write(|w| unsafe { w.psc().bits(8) }); // 90 MHz/9
    p.TIM2.egr.write(|w| w.ug().set_bit());
}

#[exception]
fn SysTick() {
    interrupt::free(|cs| {
        let time = ETH_TIME.borrow(cs);
        time.set(time.get().wrapping_add(1));
    });
}
