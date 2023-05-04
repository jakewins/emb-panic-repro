#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]
#![allow(incomplete_features)]

use defmt::*;
use embassy_executor as emex;
use embassy_rp::multicore as emmc;

use emex::Executor;
use static_cell as sc;

use cyw43;
use cyw43_pio;

use embassy_rp::gpio;
use embassy_rp::peripherals as empe;
use embassy_rp::pio::PioPeripherial;
use embassy_sync::blocking_mutex::raw as rawmut;
use embassy_sync::channel as emch;
use embassy_time as time;

use {defmt_rtt as _, panic_probe as _};

static mut CORE1_STACK: emmc::Stack<4096> = emmc::Stack::new();
static EXECUTOR0: sc::StaticCell<emex::Executor> = sc::StaticCell::new();
static EXECUTOR1: sc::StaticCell<emex::Executor> = sc::StaticCell::new();

const NUM_NMEA_MESSAGES: usize = 4;

static NMEA_STREAM: emch::Channel<rawmut::CriticalSectionRawMutex, usize, NUM_NMEA_MESSAGES> =
    emch::Channel::new();
static NMEA_FREE: emch::Channel<rawmut::CriticalSectionRawMutex, usize, NUM_NMEA_MESSAGES> =
    emch::Channel::new();

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());

    emmc::spawn_core1(p.CORE1, unsafe { &mut CORE1_STACK }, move || {
        let executor1 = EXECUTOR1.init(Executor::new());
        executor1.run(|spawner| {
            unwrap!(spawner.spawn(nmea_worker()));
        })
    });

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        unwrap!(spawner.spawn(server(
            p.PIO1, p.DMA_CH1, p.PIN_23, p.PIN_25, p.PIN_29, p.PIN_24
        )))
    });
}
#[embassy_executor::task]
pub async fn nmea_worker() -> ! {
    for i in 0..NUM_NMEA_MESSAGES {
        NMEA_FREE.try_send(i).unwrap();
    }

    loop {
        let msgidx = NMEA_FREE.recv().await;
        time::Timer::after(time::Duration::from_millis(100)).await;
        NMEA_STREAM.send(msgidx).await;
    }
}

// The server runs on core0 and handles interfacing with the outside world, providing an API to the worker task
#[embassy_executor::task]
pub async fn server(
    pioblock: empe::PIO1,
    spi_dma: empe::DMA_CH1,
    pin_pwr: empe::PIN_23,
    pin_cs: empe::PIN_25,
    pin_clk: empe::PIN_29,
    pin_dio: empe::PIN_24,
) {
    let fw = include_bytes!("../firmware/43439A0.bin");

    let pwr = gpio::Output::new(pin_pwr, gpio::Level::Low);
    let cs = gpio::Output::new(pin_cs, gpio::Level::High);

    let (_, sm0, ..) = pioblock.split();
    let spi = cyw43_pio::PioSpi::new(sm0, cs, pin_dio, pin_clk, spi_dma);

    static CYW43_STATE: sc::StaticCell<cyw43::State> = sc::StaticCell::new();
    let state = CYW43_STATE.init(cyw43::State::new());

    let (net_device, ..) = cyw43::new(state, pwr, spi, fw).await;

    loop {
        let msgidx = NMEA_STREAM.recv().await;
        NMEA_FREE.send(msgidx).await;
    }
}

