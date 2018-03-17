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
extern crate m;
extern crate byteorder;
extern crate cobs;
extern crate stepper_driver;
extern crate stm32f103xx_hal as hal;
extern crate heapless;

use core::str;

use gcode::{Parser, Tokenizer};
use gcode::parser::{CommandKind, Line, Number};

use either::Either;

use heapless::RingBuffer;

use stm32f103xx::{USART1};

use hal::prelude::*;
use hal::stm32f103xx;
use hal::gpio::gpioa::{PA1, PA2, PA3, PA4};
use hal::gpio::gpiob::{PB5, PB6, PB7, PB8};
use hal::gpio::{Output, PushPull};
use hal::serial::{Rx, Serial, Tx};
use hal::timer::{Timer, Event};
use hal::dma::{self, Transfer, dma1, R, W};

use cortex_m::peripheral::syst::SystClkSource;

use rtfm::{app, Threshold};

use stepper_driver::{Direction, Stepper};
use stepper_driver::ic::ULN2003;

// TODO: have these initialized over the serial connection and save to flash
const X_STEPS_MM: f32 = 600.0;
const X_STEP_SIZE: f32 = 1.0 / X_STEPS_MM;
const Y_STEPS_MM: f32 = 600.0;
const Y_STEP_SIZE: f32 = 1.0 / Y_STEPS_MM;
// max velocity in mm/s
const MAX_SPEED: f32 = 0.6;

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
    PB5<Output<PushPull>>,
    PB6<Output<PushPull>>,
    PB7<Output<PushPull>>,
    PB8<Output<PushPull>>,
    ULN2003,
>;

const G0: (CommandKind, Number) = (CommandKind::G, Number::Integer(0)); // Move
/*
const M0: (CommandKind, Number) = (CommandKind::M, Number::Integer(0)); // unconditional stop (unimplemented)
const M18: (CommandKind, Number) = (CommandKind::M, Number::Integer(18)); // disable steppers (unimplemented)
const M92: (CommandKind, Number) = (CommandKind::M, Number::Integer(92)); // Set steps per mm (unimplemented)
const M114: (CommandKind, Number) = (CommandKind::M, Number::Integer(114)); // get current position (unimplemented)
const M202: (CommandKind, Number) = (CommandKind::M, Number::Integer(202)); // Set max travel accel (unimplemented)
const M203: (CommandKind, Number) = (CommandKind::M, Number::Integer(203)); // Set max feedrate /speed (unimplemented)
const M204: (CommandKind, Number) = (CommandKind::M, Number::Integer(204)); // Set default accel (unimplemented)
const M205: (CommandKind, Number) = (CommandKind::M, Number::Integer(205)); // Advanced settings / jerk (unimplemented)
const M222: (CommandKind, Number) = (CommandKind::M, Number::Integer(222)); // Set speed for fast XY Moves (unimplemented)
const M223: (CommandKind, Number) = (CommandKind::M, Number::Integer(223)); // Set speed for fast Z Moves (unimplemented)
const M500: (CommandKind, Number) = (CommandKind::M, Number::Integer(500)); // Save settings (unimplemented)
const M501: (CommandKind, Number) = (CommandKind::M, Number::Integer(501)); // Restore settings (unimplemented)
*/

pub struct Move {
    pub x: Option<f32>,
    pub y: Option<f32>,
}

app! {
    device: stm32f103xx,

    resources: {
        static STEPPER_X: STEPPER_X;
        static STEPPER_Y: STEPPER_Y;
        static CURRENT: Move;
        static TIMER_X: Timer<stm32f103xx::TIM2>;
        static TIMER_Y: Timer<stm32f103xx::TIM3>;
        static TX: Option<Either<(TX_BUF, dma1::C4, TX), Transfer<R, TX_BUF, dma1::C4, TX>>> = None;
        static RX_BUF: [u8; RX_SZ] = [0; RX_SZ];
        static RX: Option<Either<(RX_BUF, dma1::C5, RX), Transfer<W, RX_BUF, dma1::C5, RX>>> = None;
        static TX_BUF: [u8; TX_SZ] = [0; TX_SZ];
        static MOVE_BUFFER: RingBuffer<Move, [Move; 5]> = RingBuffer::new();
    },

    init: {
        resources: [RX_BUF, TX_BUF],
    },

    tasks: {
        SYS_TICK: {
            path: sys_tick,
            resources: [TX, TIMER_X, TIMER_Y, CURRENT, MOVE_BUFFER, STEPPER_X, STEPPER_Y],
        },

        TIM2: {
            path: timer_x,
            resources: [STEPPER_X, CURRENT, TIMER_X],
        },

        TIM3: {
            path: timer_y,
            resources: [STEPPER_Y, CURRENT, TIMER_Y],
        },

        DMA1_CHANNEL5: {
            path: rx,
            resources: [RX, MOVE_BUFFER],
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
    
    let tim2 = Timer::tim2(p.device.TIM2, 1.hz(), clocks, &mut rcc.apb1);
    let tim3 = Timer::tim3(p.device.TIM3, 1.hz(), clocks, &mut rcc.apb1);

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

    for character in "initializing\n".as_bytes() {
        tx.write(*character);
    }

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
        gpiob.pb5.into_push_pull_output(&mut gpiob.crl),
        gpiob.pb6.into_push_pull_output(&mut gpiob.crl),
        gpiob.pb7.into_push_pull_output(&mut gpiob.crl),
        gpiob.pb8.into_push_pull_output(&mut gpiob.crh),
    );

    init::LateResources {
        STEPPER_X: stepper_x,
        STEPPER_Y: stepper_y,
        CURRENT: Move { x: None, y: None },
        TIMER_X: tim2,
        TIMER_Y: tim3,
    }
}

fn idle() -> ! {
    loop {
        rtfm::wfi();
    }
}

// This is the task handler of the TIM2 exception
fn sys_tick(_t: &mut Threshold, mut r: SYS_TICK::Resources) {
    // TODO: Send heartbeat and any other responses back through serial
    // terminate the last DMA transfer
    //let (buf, c, mut tx) = match r.TX.take().unwrap() {
    //    Either::Left((buf, c, tx)) => (buf, c, tx),
    //    Either::Right(trans) => trans.wait(),
    //};

    if r.CURRENT.x.is_none() && r.CURRENT.y.is_none() {
        if let Some(new_move) = r.MOVE_BUFFER.dequeue() {
            match new_move.x {
                Some(x) => {
                    if x > 0.0 {
                        r.STEPPER_X.direction(Direction::CW);
                        r.CURRENT.x = Some(x);
                    } else {
                        r.STEPPER_X.direction(Direction::CCW);
                        r.CURRENT.x = Some(x * -1.0);
                    }
                },
                None => r.CURRENT.x = None,
            }
            match new_move.y {
                Some(y) => {
                    if y > 0.0 {
                        r.STEPPER_Y.direction(Direction::CW);
                        r.CURRENT.y = Some(y);
                    } else {
                        r.STEPPER_Y.direction(Direction::CCW);
                        r.CURRENT.y = Some(y * -1.0);
                    }
                },
                None => r.CURRENT.y = None,
            }
            match (r.CURRENT.x, r.CURRENT.y) {
                (Some(_x), None) => {
                    r.TIMER_X.start(((MAX_SPEED / X_STEP_SIZE) as u32).hz());
                    r.TIMER_X.listen(Event::Update);
                    r.TIMER_Y.unlisten(Event::Update);
                },
                (None, Some(_y)) => {
                    r.TIMER_X.unlisten(Event::Update);
                    r.TIMER_Y.start(((MAX_SPEED / Y_STEP_SIZE) as u32).hz());
                    r.TIMER_Y.listen(Event::Update);
                },
                (Some(x), Some(y)) => {
                    if x > y {
                        r.TIMER_X.start(((MAX_SPEED / X_STEP_SIZE) as u32).hz());
                        r.TIMER_Y.start((((MAX_SPEED / Y_STEP_SIZE) / (x / y)) as u32).hz());
                    } else {
                        r.TIMER_X.start((((MAX_SPEED / X_STEP_SIZE) / (y / x)) as u32).hz());
                        r.TIMER_Y.start(((MAX_SPEED / Y_STEP_SIZE) as u32).hz());
                    }
                    r.TIMER_X.listen(Event::Update);
                    r.TIMER_Y.listen(Event::Update);
                }
                _ => {
                    r.TIMER_X.unlisten(Event::Update);
                    r.TIMER_Y.unlisten(Event::Update);
                }
            }
        }
    }
    //*r.TX = Some(Either::Left((buf, c, tx)));
}

// This is the task handler of the TIM2 exception
fn timer_x(_t: &mut Threshold, mut r: TIM2::Resources) {
    if let Some(x) = r.CURRENT.x {
        if x > 0.0 {
            r.STEPPER_X.step();
            r.CURRENT.x = Some(x - X_STEP_SIZE);
        } else {
            r.CURRENT.x = None;
        }
    } else {
        r.STEPPER_X.disable();
    }
    r.TIMER_X.wait().unwrap();
}

// This is the task handler of the TIM2 exception
fn timer_y(_t: &mut Threshold, mut r: TIM3::Resources) {
    if let Some(y) = r.CURRENT.y {
        if y > 0.0 {
            r.STEPPER_Y.step();
            r.CURRENT.y = Some(y - Y_STEP_SIZE);
        } else {
            r.CURRENT.y = None;
        }
    } else {
        r.STEPPER_Y.disable();
    }
    r.TIMER_Y.wait().unwrap();
}

fn rx(_t: &mut Threshold, mut r: DMA1_CHANNEL5::Resources) {
    //TODO: change this to take a string, or change from the dma interface to regular serial
    let (buf, c, rx) = match r.RX.take().unwrap() {
        Either::Left((buf, c, rx)) => (buf, c, rx),
        Either::Right(transfer) => transfer.wait(),
    };

    if let Ok(gcode) = str::from_utf8(buf) {
        let lexer = Tokenizer::new(gcode.chars());
        // TODO: add error handling here
        let tokens = lexer.filter_map(|t| t.ok());
        let parser = Parser::new(tokens);
        for line in parser {
            if let Line::Cmd(command) = line.unwrap() {
                if (command.kind, command.number) == G0 {
                    while let Err(_) = r.MOVE_BUFFER.enqueue(Move{x: command.args.x, y: command.args.y}) {}
                }
            }
        }
    }

    *r.RX = Some(Either::Right(rx.read_exact(c, buf)));
}
