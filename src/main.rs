//! Etch-a-sketch in sand

#![deny(unsafe_code)]
//#![deny(warnings)]
#![feature(proc_macro)]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rtfm as rtfm;
extern crate either;
extern crate gcode;
extern crate nb;
extern crate byteorder;
extern crate cobs;
extern crate stepper_driver;
extern crate stm32f103xx_hal as hal;

use core::str;

use byteorder::{ByteOrder, LE};

use gcode::{Parser, Tokenizer};
use gcode::parser::{CommandKind, Line, Number};

use either::Either;

use stm32f103xx::{USART1};

use hal::prelude::*;
use hal::stm32f103xx;
use hal::gpio::gpioa::{PA1, PA2, PA3, PA4};
use hal::gpio::gpiob::{PB4, PB5, PB6, PB7};
use hal::gpio::{Output, PushPull};
use hal::serial::{Rx, Serial, Tx};
use hal::timer::{Timer, Event};
use hal::dma::{self, Transfer, dma1, R, W};

use cortex_m::peripheral::syst::SystClkSource;

use rtfm::{app, Threshold};

use stepper_driver::{Direction, Stepper};
use stepper_driver::ic::ULN2003;

// TODO: have these initialized over the serial connection
const GCODE: &'static str = "G0 X10 Y0\nG0 X10 Y10\nG0 X0 Y10\nG0 X0 Y0";
const X_STEPS_MM: f32 = 120.0;
const X_STEP_SIZE: f32 = 1.0 / X_STEPS_MM;
const Y_STEPS_MM: f32 = 120.0;
const Y_STEP_SIZE: f32 = 1.0 / Y_STEPS_MM;

const TX_SZ: usize = 18;
const RX_SZ: usize = 20;

// CONNECTIONS
type TX = Tx<USART1>;
type RX = Rx<USART1>;
#[allow(non_camel_case_types)]
type TX_BUF = &'static mut [u8; TX_SZ];
#[allow(non_camel_case_types)]
type RX_BUF = &'static mut [u8; RX_SZ];

#[allow(non_camel_case_types)]
type STEPPER_X = Stepper<
    PA1<Output<PushPull>>,
    PA2<Output<PushPull>>,
    PA3<Output<PushPull>>,
    PA4<Output<PushPull>>,
    ULN2003,
>;
#[allow(non_camel_case_types)]
type STEPPER_Y = Stepper<
    PB4<Output<PushPull>>,
    PB5<Output<PushPull>>,
    PB6<Output<PushPull>>,
    PB7<Output<PushPull>>,
    ULN2003,
>;

const G0: (CommandKind, Number) = (CommandKind::G, Number::Integer(0));

pub struct Move {
    pub x: f32,
    pub y: f32,
}

app! {
    device: stm32f103xx,

    resources: {
        static STEPPER_X: STEPPER_X;
        static STEPPER_Y: STEPPER_Y;
        static CURRENT: Move;
        static TIMER: Timer<stm32f103xx::TIM2>;
        static TX: Option<Either<(TX_BUF, dma1::C4, TX), Transfer<R, TX_BUF, dma1::C4, TX>>> = None;
        static RX_BUF: [u8; RX_SZ] = [0; RX_SZ];
        static RX: Option<Either<(RX_BUF, dma1::C5, RX), Transfer<W, RX_BUF, dma1::C5, RX>>> = None;
        static TX_BUF: [u8; TX_SZ] = [0; TX_SZ];
    },

    init: {
        resources: [RX_BUF, TX_BUF],
    },

    // Timer 2 handles stepper movement
    tasks: {
        SYS_TICK: {
            path: sys_tick,
            resources: [TIMER, CURRENT],
        },

        TIM2: {
            path: timer2,
            resources: [STEPPER_X, STEPPER_Y, CURRENT, TIMER],
        },

        DMA1_CHANNEL5: {
            path: rx,
            resources: [RX, TX, CURRENT],
        }
    },
}

fn init(mut p: init::Peripherals, r: init::Resources) -> init::LateResources {
    p.core.DWT.enable_cycle_counter();

    let mut flash = p.device.FLASH.constrain();
    let mut rcc = p.device.RCC.constrain();

    let clocks = rcc.cfgr
        .sysclk(64.mhz())
        .pclk1(32.mhz())
        .freeze(&mut flash.acr);
    
    
    let mut tim2 = Timer::tim2(p.device.TIM2, 180.hz(), clocks, &mut rcc.apb1);
    tim2.listen(Event::Update);

    let mut afio = p.device.AFIO.constrain(&mut rcc.apb2);

    let mut gpioa = p.device.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = p.device.GPIOB.split(&mut rcc.apb2);
    let mut channels = p.device.DMA1.split(&mut rcc.ahb);

    // SERIAL
    let pa9 = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let pa10 = gpioa.pa10;

    let serial = Serial::usart1(
        p.device.USART1,
        (pa9, pa10),
        &mut afio.mapr,
        115_200.bps(),
        clocks,
        &mut rcc.apb2,
    );

    let (mut tx, rx) = serial.split();

    // start of COBS frame
    tx.write(0x00).ok().unwrap();

    *r.TX = Some(Either::Left((r.TX_BUF, channels.4, tx)));

    channels.5.listen(dma::Event::TransferComplete);
    *r.RX = Some(Either::Right(rx.read_exact(channels.5, r.RX_BUF)));

    // configure the system timer
    p.core.SYST.set_clock_source(SystClkSource::Core);
    p.core.SYST.set_reload(100_000);
    p.core.SYST.enable_interrupt();
    p.core.SYST.enable_counter();

    let stepper_x = Stepper::uln2003(
        Direction::CW,
        gpioa.pa1.into_push_pull_output(&mut gpioa.crl),
        gpioa.pa2.into_push_pull_output(&mut gpioa.crl),
        gpioa.pa3.into_push_pull_output(&mut gpioa.crl),
        gpioa.pa4.into_push_pull_output(&mut gpioa.crl),
    );

    let stepper_y = Stepper::uln2003(
        Direction::CW,
        gpiob.pb4.into_push_pull_output(&mut gpiob.crl),
        gpiob.pb5.into_push_pull_output(&mut gpiob.crl),
        gpiob.pb6.into_push_pull_output(&mut gpiob.crl),
        gpiob.pb7.into_push_pull_output(&mut gpiob.crl),
    );

    init::LateResources {
        STEPPER_X: stepper_x,
        STEPPER_Y: stepper_y,
        CURRENT: Move { x: 0.0, y: 0.0 },
        TIMER: tim2,
    }
}

fn idle() -> ! {
    loop {
        rtfm::wfi();
    }
}

// This is the task handler of the TIM2 exception
fn sys_tick(_t: &mut Threshold, r: SYS_TICK::Resources) {
    // TODO: Send heartbeat and any other responses back through serial

    //TODO: calculate timer speeds based on accel and jerk settings
}

// This is the task handler of the TIM2 exception
fn timer2(_t: &mut Threshold, mut r: TIM2::Resources) {
    // TODO: add check if all current moves are done and grab next move from buffer
    if r.CURRENT.x > 0.0 {
        r.STEPPER_X.step();
        r.CURRENT.x -= X_STEP_SIZE;
    }
    if r.CURRENT.y > 0.0 {
        r.STEPPER_Y.step();
        r.CURRENT.y -= Y_STEP_SIZE;
    }
    r.TIMER.wait().unwrap();
}

fn rx(_t: &mut Threshold, mut r: DMA1_CHANNEL5::Resources) {
    //TODO: change this to take a string, or change from the dma interface to regular serial
    let (buf, c, rx) = match r.RX.take().unwrap() {
        Either::Left((buf, c, rx)) => (buf, c, rx),
        Either::Right(transfer) => transfer.wait(),
    };

    if let Ok(gcode) = str::from_utf8(buf) {
        let lexer = Tokenizer::new(gcode.chars());
        let tokens = lexer.filter_map(|t| t.ok());
        let parser = Parser::new(tokens);
        for line in parser {
            if let Line::Cmd(command) = line.unwrap() {
                if (command.kind, command.number) == G0 {
                    // TODO: these should add the command to the buffer, rather than current move
                    if let Some(x) = command.args.x {
                        r.CURRENT.x = x;
                    }
                    if let Some(y) = command.args.y {
                        r.CURRENT.y = y;
                    }
                }
            }
        }
    }

    *r.RX = Some(Either::Right(rx.read_exact(c, buf)));

    let (buf, c, tx) = match r.TX.take().unwrap() {
        Either::Left((buf, c, tx)) => (buf, c, tx),
        Either::Right(transfer) => transfer.wait(),
    };

    if r.CURRENT.y > 0.0 {
        //let mut data = [0; TX_SZ - 2];
        //LE::write_f32(&mut data[0..4], r.CURRENT.y);
        let data = "moving\n".as_bytes();
        cobs::encode(&data, buf);
        *r.TX = Some(Either::Right(tx.write_all(c, buf)));
    } else {
        *r.TX = Some(Either::Left((buf, c, tx)));
    }
}
