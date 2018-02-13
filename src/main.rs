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

//use core::fmt::Write;

use hal::prelude::*;
use cortex_m::peripheral::syst::SystClkSource;
use hal::stm32f103xx;
use rtfm::{app, Threshold};
use stepper_driver::{Stepper, Direction};
use stepper_driver::ic::ULN2003;

use hal::gpio::gpioa::{PA1, PA2, PA3, PA4};
use hal::gpio::{Output, PushPull};

// CONNECTIONS
type STEPPER1 = Stepper<
    PA1<Output<PushPull>>,
    PA2<Output<PushPull>>,
    PA3<Output<PushPull>>,
    PA4<Output<PushPull>>,
    ULN2003,
>;

app! {
    device: stm32f103xx,

    resources: {
        static STEPPER1: STEPPER1;
    },

    // Both SYS_TICK and TIM2 have access to the `COUNTER` data
    tasks: {
        SYS_TICK: {
            path: sys_tick,
            resources: [STEPPER1],
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
    p.core.SYST.set_reload(250_000); // 1/32s
    p.core.SYST.enable_interrupt();
    p.core.SYST.enable_counter();
    let stepper1 = Stepper::uln2003(Direction::CW, gpioa.pa1.into_push_pull_output(&mut gpioa.crl), gpioa.pa2.into_push_pull_output(&mut gpioa.crl), gpioa.pa3.into_push_pull_output(&mut gpioa.crl), gpioa.pa4.into_push_pull_output(&mut gpioa.crl));
    init::LateResources {
        STEPPER1: stepper1,
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
    r.STEPPER1.step();
}