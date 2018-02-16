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
use gcode::parser::{Line, CommandKind};

//use core::fmt::Write;

use hal::prelude::*;
use cortex_m::peripheral::syst::SystClkSource;
use hal::stm32f103xx;
use rtfm::{app, Threshold};
use stepper_driver::{Stepper, Direction};
use stepper_driver::ic::ULN2003;

use hal::gpio::gpioa::{PA1, PA2, PA3, PA4};
use hal::gpio::{Output, PushPull};

const GCODE: &'static str = "G0 X10 Y0\nG0 X10 Y10\nG0 X0 Y10\nG0 X0 Y0";
const STEPS_MM: f32 = 120.0;
const STEP_SIZE: f32 = 1.0 / STEPS_MM;


// CONNECTIONS
type STEPPER_X = Stepper<
    PA1<Output<PushPull>>,
    PA2<Output<PushPull>>,
    PA3<Output<PushPull>>,
    PA4<Output<PushPull>>,
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

    // configure the system timer to generate one interrupt every second
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
//
// `_t` is the preemption threshold token. We won't use it in this program.
//
// `r` is the set of resources this task has access to. `SYS_TICK::Resources`
// has one field per resource declared in `app!`.
#[allow(unsafe_code)]
fn sys_tick(_t: &mut Threshold, mut r: SYS_TICK::Resources) {
    // add check if all current moves are done and grab next move from buffer
    if r.CURRENT.x > 0.0 {
        r.STEPPER_X.step();
        r.CURRENT.x -= STEP_SIZE;
    }
    

    // This section should be in the serial handling section
    let lexer = Tokenizer::new(GCODE.chars());
    let tokens = lexer.filter_map(|t| t.ok());
    let parser = Parser::new(tokens);
    for line in parser {
        if let Line::Cmd(command) = line.unwrap() {
            if (command.kind, command.number) == (CommandKind::G, gcode::parser::Number::Integer(0)) {
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