//! Etch-a-sketch in sand

#![deny(unsafe_code)]
//#![deny(warnings)]
#![feature(proc_macro)]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rtfm as rtfm;
extern crate stm32f103xx_hal as hal;
extern crate stepper_driver;
extern crate nb;
extern crate gcode;

use gcode::{Tokenizer, Parser};
use gcode::parser::{Line, CommandKind, Number};

use hal::prelude::*;
use hal::stm32f103xx;
use hal::gpio::gpioa::{PA1, PA2, PA3, PA4};
use hal::gpio::gpiob::{PB4, PB5, PB6, PB7};
use hal::gpio::{Output, PushPull};

use cortex_m::peripheral::syst::SystClkSource;

use rtfm::{app, Threshold};

use stepper_driver::{Stepper, Direction};
use stepper_driver::ic::ULN2003;

// TODO: have these initialized over the serial connection
const GCODE: &'static str = "G0 X10 Y0\nG0 X10 Y10\nG0 X0 Y10\nG0 X0 Y0";
const X_STEPS_MM: f32 = 120.0;
const X_STEP_SIZE: f32 = 1.0 / X_STEPS_MM;
const Y_STEPS_MM: f32 = 120.0;
const Y_STEP_SIZE: f32 = 1.0 / Y_STEPS_MM;

// CONNECTIONS
type STEPPER_X = Stepper<
    PA1<Output<PushPull>>,
    PA2<Output<PushPull>>,
    PA3<Output<PushPull>>,
    PA4<Output<PushPull>>,
    ULN2003,
>;

type STEPPER_Y = Stepper<
    PB4<Output<PushPull>>,
    PB5<Output<PushPull>>,
    PB6<Output<PushPull>>,
    PB7<Output<PushPull>>,
    ULN2003,
>;

pub struct Move {
    pub x: f32,
    pub y: f32,
}

app! {
    device: stm32f103xx,

    resources: {
        static STEPPER_X: STEPPER_X;
        static CURRENT: Move;
    },

    // SYS_TICK handles stepper movement
    tasks: {
        SYS_TICK: {
            path: sys_tick,
            resources: [STEPPER_X, CURRENT],
        },
    },
}

fn init(mut p: init::Peripherals) -> init::LateResources {
    let mut flash = p.device.FLASH.constrain();
    let mut rcc = p.device.RCC.constrain();
    let _clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut gpioa = p.device.GPIOA.split(&mut rcc.apb2);

    // configure the system timer
    p.core.SYST.set_clock_source(SystClkSource::Core);
    p.core.SYST.set_reload(100_000); 
    p.core.SYST.enable_interrupt();
    p.core.SYST.enable_counter();
    let stepper_x = Stepper::uln2003(Direction::CW, gpioa.pa1.into_push_pull_output(&mut gpioa.crl), gpioa.pa2.into_push_pull_output(&mut gpioa.crl), gpioa.pa3.into_push_pull_output(&mut gpioa.crl), gpioa.pa4.into_push_pull_output(&mut gpioa.crl));
    init::LateResources {
        STEPPER_X: stepper_x,
        CURRENT: Move{x: 0.0, y: 0.0},
    }
}

fn idle() -> ! {
    loop {
        rtfm::wfi();
    }
}


// This is the task handler of the SYS_TICK exception
#[allow(unsafe_code)]
fn sys_tick(_t: &mut Threshold, mut r: SYS_TICK::Resources) {
    // TODO: add check if all current moves are done and grab next move from buffer
    if r.CURRENT.x > 0.0 {
        r.STEPPER_X.step();
        r.CURRENT.x -= X_STEP_SIZE;
    }
    
    // TODO: This section should be in the serial handling section
    let lexer = Tokenizer::new(GCODE.chars());
    let tokens = lexer.filter_map(|t| t.ok());
    let parser = Parser::new(tokens);
    for line in parser {
        if let Line::Cmd(command) = line.unwrap() {
            if (command.kind, command.number) == (CommandKind::G, Number::Integer(0)) {
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