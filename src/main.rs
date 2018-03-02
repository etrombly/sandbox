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

use core::str;

use byteorder::{ByteOrder, LE};

use m::Float;

use gcode::{Parser, Tokenizer};
use gcode::parser::{CommandKind, Line, Number};

use either::Either;

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
const MAX_SPEED_X: f32 = 0.6;
const MAX_SPEED_Y: f32 = 0.6;
// min velocity
const JERK_X: f32 = 0.3;
const JERK_Y: f32 = 0.3;
// acceleration
const ACCEL_X: f32 = 1.0;
const ACCEL_Y: f32 = 1.0;

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
        static TIMER_X: Timer<stm32f103xx::TIM2>;
        static TIMER_Y: Timer<stm32f103xx::TIM3>;
        static TX: Option<Either<(TX_BUF, dma1::C4, TX), Transfer<R, TX_BUF, dma1::C4, TX>>> = None;
        static RX_BUF: [u8; RX_SZ] = [0; RX_SZ];
        static RX: Option<Either<(RX_BUF, dma1::C5, RX), Transfer<W, RX_BUF, dma1::C5, RX>>> = None;
        static TX_BUF: [u8; TX_SZ] = [0; TX_SZ];
    },

    init: {
        resources: [RX_BUF, TX_BUF],
    },

    tasks: {
        SYS_TICK: {
            path: sys_tick,
            resources: [TIMER_X, TIMER_Y, CURRENT],
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
    
    // TODO: start at 0 and calculate velocity in sys_tick
    let mut tim2 = Timer::tim2(p.device.TIM2, ((MAX_SPEED_X / X_STEP_SIZE) as u32).hz(), clocks, &mut rcc.apb1);
    tim2.listen(Event::Update);
    let mut tim3 = Timer::tim3(p.device.TIM3, ((MAX_SPEED_Y / Y_STEP_SIZE) as u32).hz(), clocks, &mut rcc.apb1);
    tim3.listen(Event::Update);

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
        gpiob.pb5.into_push_pull_output(&mut gpiob.crl),
        gpiob.pb6.into_push_pull_output(&mut gpiob.crl),
        gpiob.pb7.into_push_pull_output(&mut gpiob.crl),
        gpiob.pb8.into_push_pull_output(&mut gpiob.crh),
    );

    init::LateResources {
        STEPPER_X: stepper_x,
        STEPPER_Y: stepper_y,
        CURRENT: Move { x: 0.0, y: 0.0 },
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
fn sys_tick(_t: &mut Threshold, r: SYS_TICK::Resources) {
    // TODO: Send heartbeat and any other responses back through serial

    //TODO: calculate timer speeds based on accel and jerk settings
    //r.X_TIMER.start((X_SPEED / X_STEP_SIZE).hz());

    let distance = 10.0; //temp value for velocity calc, replace with r.CURRENT.x or whatever
    let velocity_max_x = ((ACCEL_X * JERK_X * JERK_X) + (2.0 * (ACCEL_X * ACCEL_X * distance)) / (ACCEL_X * 2.0)).sqrt();
}

// This is the task handler of the TIM2 exception
fn timer_x(_t: &mut Threshold, mut r: TIM2::Resources) {
    // TODO: add check if all current moves are done and grab next move from buffer
    // TODO: refactor this to remove duplication
    if r.CURRENT.x - X_STEP_SIZE > 0.0 || r.CURRENT.x + X_STEP_SIZE < 0.0{
        if r.CURRENT.x > 0.0 {
            r.STEPPER_X.direction(Direction::CW);
            r.STEPPER_X.step();
            r.CURRENT.x -= X_STEP_SIZE;
        } else if r.CURRENT.x < 0.0 {
            r.STEPPER_X.direction(Direction::CW);
            r.STEPPER_X.step();
            r.CURRENT.x += X_STEP_SIZE;
        }
    } else {
        r.STEPPER_X.disable();
        r.CURRENT.x = 0.0;
    }
    r.TIMER_X.wait().unwrap();
}

// This is the task handler of the TIM2 exception
fn timer_y(_t: &mut Threshold, mut r: TIM3::Resources) {
    // TODO: add check if all current moves are done and grab next move from buffer
    // TODO: refactor this to remove duplication
    if r.CURRENT.y - Y_STEP_SIZE > 0.0 || r.CURRENT.y + Y_STEP_SIZE < 0.0{
        if r.CURRENT.y > 0.0 {
            r.STEPPER_Y.direction(Direction::CW);
            r.STEPPER_Y.step();
            r.CURRENT.y -= Y_STEP_SIZE;
        } else if r.CURRENT.y < 0.0 {
            r.STEPPER_Y.direction(Direction::CW);
            r.STEPPER_Y.step();
            r.CURRENT.y += Y_STEP_SIZE;
        }
    } else {
        r.STEPPER_Y.disable();
        r.CURRENT.y = 0.0;
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

    // TODO: take this out later
    let (buf, c, tx) = match r.TX.take().unwrap() {
        Either::Left((buf, c, tx)) => (buf, c, tx),
        Either::Right(transfer) => transfer.wait(),
    };

    if r.CURRENT.y > 0.0 {
        //let mut data = [0; TX_SZ - 2];
        //LE::write_f32(&mut data[0..4], r.CURRENT.y);
        let data = "moving\r\n".as_bytes();
        cobs::encode(&data, buf);
        *r.TX = Some(Either::Right(tx.write_all(c, buf)));
    } else {
        *r.TX = Some(Either::Left((buf, c, tx)));
    }
}
