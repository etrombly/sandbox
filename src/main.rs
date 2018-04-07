//! Etch-a-sketch in sand

#![deny(unsafe_code)]
//#![deny(warnings)]
#![feature(proc_macro)]
#![feature(const_fn)]
#![feature(core_float)]
#![no_std]

extern crate byteorder;
extern crate cobs;
extern crate cortex_m;
extern crate cortex_m_rtfm as rtfm;
extern crate gcode;
#[macro_use(block)]
extern crate nb;
extern crate m;
extern crate stepper_driver;
extern crate stm32f103xx_hal as hal;

// used for encoding data to send over serial
use byteorder::{ByteOrder, LE};
// used to convert bytes to string
use core::num::Float;
use core::str;

// some float math that's not in core
use m::Float as _0;

use gcode::parser::{CommandKind, Line, Number};
use gcode::{Parser, Tokenizer};

use hal::gpio::gpioa::{PA1, PA2, PA3, PA4};
use hal::gpio::gpiob::{PB5, PB6, PB7, PB8};
use hal::gpio::{Output, PushPull};
use hal::prelude::*;
use hal::serial::{Rx, Serial, Tx};
use hal::stm32f103xx;
use hal::time::{Instant, MonoTimer};
use hal::timer::{Event, Timer};

use cortex_m::peripheral::syst::SystClkSource;

use rtfm::{app, Resource, Threshold};

use stepper_driver::ic::ULN2003;
use stepper_driver::{Direction, Stepper};

const CARRIAGE_RETURN: u8 = 13;
const NEW_LINE: u8 = 10;

// TODO: have these initialized over the serial connection and save to flash
const X_STEPS_MM: f32 = 600.0;
const X_STEP_SIZE: f32 = 1.0 / X_STEPS_MM;
const Y_STEPS_MM: f32 = 600.0;
const Y_STEP_SIZE: f32 = 1.0 / Y_STEPS_MM;
// max speed in mm/s
const MAX_SPEED: f32 = 0.6;
const MAX_FREQ_X: u32 = (MAX_SPEED / X_STEP_SIZE) as u32;
const MAX_FREQ_Y: u32 = (MAX_SPEED / Y_STEP_SIZE) as u32;
// Length of linear segment for arcs
const MM_PER_ARC_SEGMENT: f32 = 1.0;

// max characters allowed in a line of gcode
const RX_SZ: usize = 256;

// number of moves to buffer
const MOVE_SZ: usize = 256;

// GCODE command aliases
const G0: (CommandKind, Number) = (CommandKind::G, Number::Integer(0)); // Move
const G1: (CommandKind, Number) = (CommandKind::G, Number::Integer(1)); // Move
const G2: (CommandKind, Number) = (CommandKind::G, Number::Integer(2)); // Clockwise arc
const G3: (CommandKind, Number) = (CommandKind::G, Number::Integer(3)); // Counterclockwise arc
const M0: (CommandKind, Number) = (CommandKind::M, Number::Integer(0)); // unconditional stop

/* May add these back in later if I bother with path planning
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

// TODO: put the structs in a lib
// Movebuffer holds all movements parsed from gcode
// modified from heapless ringbuffer
pub struct MoveBuffer {
    head: usize,
    tail: usize,
    buffer: [Movement; MOVE_SZ],
}

impl MoveBuffer {
    const fn new() -> MoveBuffer {
        MoveBuffer {
            head: 0,
            tail: 0,
            buffer: [Movement::LinearMove(LinearMove { x: None, y: None }); MOVE_SZ],
        }
    }

    pub fn push(&mut self, movement: &Movement) -> Result<(), ()> {
        let next_tail = (self.tail + 1) % MOVE_SZ;
        if next_tail != self.head {
            self.buffer[self.tail] = *movement;
            self.tail = next_tail;
            Ok(())
        } else {
            Err(())
        }
    }

    pub fn next(&mut self) -> Option<LinearMove> {
        if self.head != self.tail {
            match &mut self.buffer[self.head] {
                // if movement is linear just return it
                Movement::LinearMove(x) => {
                    self.head = (self.head + 1) % MOVE_SZ;
                    Some(*x)
                }
                // if movement is an arc, get the next line segment
                Movement::ArcMove(ref mut x) => {
                    // TODO: final step is going to be slightly off, do a final move to the last location
                    if x.segments == 0.0 {
                        self.head = (self.head + 1) % MOVE_SZ;
                        None
                    } else {
                        Some(x.step())
                    }
                }
            }
        } else {
            None
        }
    }

    pub fn clear(&mut self) {
        self.tail = self.head;
    }
}

// Serial recieve buffer
pub struct Buffer {
    index: usize,
    buffer: [u8; RX_SZ],
}

impl Buffer {
    const fn new() -> Buffer {
        Buffer {
            index: 0,
            buffer: [0; RX_SZ],
        }
    }

    pub fn push(&mut self, data: &u8) -> Result<(), ()> {
        if self.index < RX_SZ {
            self.buffer[self.index] = *data;
            self.index += 1;
            return Ok(());
        }
        Err(())
    }

    pub fn read(&mut self) -> Option<Buffer> {
        if self.index > 0 {
            let tmp = self.index;
            self.index = 0;
            Some(Buffer {
                index: tmp,
                buffer: self.buffer,
            })
        } else {
            None
        }
    }

    pub fn split(&mut self) -> Option<(Buffer, Buffer)> {
        if let Some(index) = self.buffer
            .iter()
            .position(|&x| x == CARRIAGE_RETURN || x == NEW_LINE)
        {
            let mut line = Buffer::new();
            let mut remainder = Buffer::new();
            for c in &self.buffer[0..index] {
                // TODO: handle error here
                if line.push(c).is_err() {}
            }
            for c in &self.buffer[index + 1..self.index] {
                // TODO: handle error here
                if remainder.push(c).is_err() {}
            }
            Some((line, remainder))
        } else {
            None
        }
    }
}

pub struct Point {
    pub x: f32,
    pub y: f32,
}

// Linear movement
#[derive(Clone, Copy)]
pub struct LinearMove {
    pub x: Option<f32>,
    pub y: Option<f32>,
}

#[derive(Clone, Copy, PartialEq)]
pub enum ArcDirection {
    CW,
    CCW,
}

// Struct to handle a G2 or G3 command
// TODO: change these to 2d points (maybe)
#[derive(Clone, Copy)]
pub struct ArcMove {
    pub end_x: f32,
    pub end_y: f32,
    pub center_x: f32,
    pub center_y: f32,
    pub radius: f32,
    pub r_x: f32,
    pub r_y: f32,
    pub sin_t: f32,
    pub cos_t: f32,
    pub segments: f32,
}

// taken from marlin plan_arc in Marlin_main.cpp, removed anything dealing with the z axis or delta printers
impl ArcMove {
    pub fn new(
        curr_x: f32,
        curr_y: f32,
        end_x: f32,
        end_y: f32,
        i: f32,
        j: f32,
        direction: ArcDirection,
    ) -> ArcMove {
        let center_x = curr_x + i;
        let center_y = curr_y + j;
        let radius = ((curr_x - center_x).powi(2) + (curr_y - center_y).powi(2)).sqrt();
        let r_x = -i;
        let r_y = -j;
        let rt_x = end_x - center_x;
        let rt_y = end_y - center_y;
        let mut angular_travel = (r_x * rt_y - r_y * rt_x).atan2(r_x * rt_x + r_y * rt_y);

        if angular_travel < 0.0 {
            angular_travel += (360_f32).to_radians();
        }
        if direction == ArcDirection::CW {
            angular_travel -= (360_f32).to_radians();
        }
        // Make a circle if the angular rotation is 0 and the target is current position
        if angular_travel == 0.0 && core::num::Float::abs(curr_x - end_x) < core::f32::EPSILON
            && core::num::Float::abs(curr_y - end_y) < core::f32::EPSILON
        {
            angular_travel = (360_f32).to_radians();
        }

        let mm_of_travel = core::num::Float::abs(angular_travel * radius);

        // double cast is a cheater floor()
        let segments = (mm_of_travel / MM_PER_ARC_SEGMENT) as u32 as f32;
        let sin_t = angular_travel / segments;
        let cos_t = 1.0 - 0.5 * (sin_t).powi(2); // Small angle approximation

        ArcMove {
            end_x,
            end_y,
            center_x,
            center_y,
            radius,
            r_x,
            r_y,
            sin_t,
            cos_t,
            segments,
        }
    }

    pub fn step(&mut self) -> LinearMove {
        let r_new_y = self.r_x * self.sin_t + self.r_y * self.cos_t;
        self.r_x = self.r_x * self.cos_t - self.r_y * self.sin_t;
        self.r_y = r_new_y;

        self.segments -= 1.0;

        LinearMove {
            x: Some(self.center_x + self.r_x),
            y: Some(self.center_y + self.r_y),
        }
    }
}

// use an enum to allow the movebuffer to hold both types
#[derive(Clone, Copy)]
pub enum Movement {
    LinearMove(LinearMove),
    ArcMove(ArcMove),
}

// CONNECTIONS
// serial tx and rx
type TX = Tx<stm32f103xx::USART1>;
type RX = Rx<stm32f103xx::USART1>;

// the first stepper is connected to a ULN2003 on pins PA1-4
#[allow(non_camel_case_types)]
type STEPPER_X = Stepper<
    PA1<Output<PushPull>>,
    PA2<Output<PushPull>>,
    PA3<Output<PushPull>>,
    PA4<Output<PushPull>>,
    ULN2003,
>;

// the second stepper is connected to a ULN2003 on pins PB5-8
#[allow(non_camel_case_types)]
type STEPPER_Y = Stepper<
    PB5<Output<PushPull>>,
    PB6<Output<PushPull>>,
    PB7<Output<PushPull>>,
    PB8<Output<PushPull>>,
    ULN2003,
>;

app! {
    device: stm32f103xx,

    resources: {
        static STEPPER_X: STEPPER_X;
        static STEPPER_Y: STEPPER_Y;
        // linear move that is active
        static CURRENT: LinearMove;
        // timers to control stepper movement
        static TIMER_X: Timer<stm32f103xx::TIM2>;
        static TIMER_Y: Timer<stm32f103xx::TIM3>;
        static TX: TX;
        static RX_BUF: Buffer = Buffer::new();
        static CMD_BUF: Buffer = Buffer::new();
        static RX: RX;
        static MOVE_BUFFER: MoveBuffer = MoveBuffer::new();
        // monotimer for cpu monitor
        static MONO: MonoTimer;
        static LAST_UPDATE: Instant;
        static SLEEP: u32 = 0;
        // current location
        static LOCATION: Point = Point{x: 0.0, y: 0.0};
        // location once moves are processed (used for adding an arc movement)
        static VIRT_LOCATION: Point = Point{x: 0.0, y: 0.0};
    },

    idle: {
        resources: [SLEEP, MONO]
    },

    tasks: {
        SYS_TICK: {
            path: sys_tick,
            resources: [TX, TIMER_X, TIMER_Y, CURRENT, MOVE_BUFFER, STEPPER_X, STEPPER_Y, SLEEP, LAST_UPDATE, MONO, LOCATION, VIRT_LOCATION, RX_BUF, CMD_BUF],
        },

        TIM2: {
            path: timer_x,
            resources: [STEPPER_X, CURRENT, TIMER_X, LOCATION],
        },

        TIM3: {
            path: timer_y,
            resources: [STEPPER_Y, CURRENT, TIMER_Y, LOCATION],
        },

        USART1: {
            path: rx,
            resources: [RX, TX, MOVE_BUFFER, CURRENT, RX_BUF, LOCATION],
        }
    },
}

fn init(mut p: init::Peripherals, _r: init::Resources) -> init::LateResources {
    let mut flash = p.device.FLASH.constrain();
    let mut rcc = p.device.RCC.constrain();

    // TODO: test performance and see what this needs to actually be set to
    let clocks = rcc.cfgr
        .sysclk(64.mhz())
        .pclk1(32.mhz())
        .freeze(&mut flash.acr);

    // start timer for performance monitoring
    let mono = MonoTimer::new(p.core.DWT, clocks);

    // configure the system timer
    // TODO: test performance and see what this needs to actually be set to
    p.core.SYST.set_clock_source(SystClkSource::Core);
    p.core.SYST.set_reload(100_000);
    p.core.SYST.enable_interrupt();
    p.core.SYST.enable_counter();

    // These timers are used to control the step speed of the steppers
    // the initial update freq doesn't matter since they won't start until a move is received
    // just don't set to 0hz
    let tim2 = Timer::tim2(p.device.TIM2, 1.hz(), clocks, &mut rcc.apb1);
    let tim3 = Timer::tim3(p.device.TIM3, 1.hz(), clocks, &mut rcc.apb1);

    let mut afio = p.device.AFIO.constrain(&mut rcc.apb2);

    let mut gpioa = p.device.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = p.device.GPIOB.split(&mut rcc.apb2);

    // SERIAL
    let pa9 = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let pa10 = gpioa.pa10;

    let mut serial = Serial::usart1(
        p.device.USART1,
        (pa9, pa10),
        &mut afio.mapr,
        115_200.bps(),
        clocks,
        &mut rcc.apb2,
    );

    // Enable RX interrupt
    serial.listen(hal::serial::Event::Rxne);

    let (mut tx, rx) = serial.split();

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

    // TODO: look in to changing serial TX back to DMA
    for c in b"initialized\r\n" {
        block!(tx.write(*c)).ok();
    }

    init::LateResources {
        STEPPER_X: stepper_x,
        STEPPER_Y: stepper_y,
        CURRENT: LinearMove { x: None, y: None },
        TIMER_X: tim2,
        TIMER_Y: tim3,
        RX: rx,
        TX: tx,
        MONO: mono,
        LAST_UPDATE: mono.now(),
    }
}

fn idle(t: &mut Threshold, mut r: idle::Resources) -> ! {
    loop {
        rtfm::atomic(t, |t| {
            let before = r.MONO.borrow(t).now();
            rtfm::wfi();
            *r.SLEEP.borrow_mut(t) += before.elapsed();
        });
    }
}

// Sys_tick mainly handles the movement and command buffers, not using any path planning since the steppers are slow
fn sys_tick(_t: &mut Threshold, mut r: SYS_TICK::Resources) {
    // check if any commands have been received, if so process the gcode
    // TODO: this currently isn't fast enough to keep up if batches of commands are sent
    if let Some(data) = r.RX_BUF.read() {
        // add the data to the command buffer
        for c in &data.buffer[0..data.index] {
            // TODO: handle error here
            if r.CMD_BUF.push(c).is_err() {}
        }
        // if any lines have been received, process them
        while let Some((line, buffer)) = r.CMD_BUF.split() {
            *r.CMD_BUF = buffer;
            if let Ok(gcode) = str::from_utf8(&line.buffer[0..line.index]) {
                let lexer = Tokenizer::new(gcode.chars());
                // TODO: add error handling here
                let tokens = lexer.filter_map(|t| t.ok());
                let parser = Parser::new(tokens);
                for line in parser {
                    if let Ok(Line::Cmd(command)) = line {
                        // TODO: switch this to a match
                        if (command.kind, command.number) == G0
                            || (command.kind, command.number) == G1
                        {
                            // TODO: Handle the buffer being full
                            if r.MOVE_BUFFER
                                .push(&Movement::LinearMove(LinearMove {
                                    x: command.args.x,
                                    y: command.args.y,
                                }))
                                .is_err()
                            {
                            } else {
                                *r.VIRT_LOCATION = Point {
                                    x: command.args.x.unwrap(),
                                    y: command.args.y.unwrap(),
                                };
                            }
                        // TODO: if path planning is going to be added arcs should be converted in to linear moves here
                        } else if (command.kind, command.number) == G2 {
                            if r.MOVE_BUFFER
                                .push(&Movement::ArcMove(ArcMove::new(
                                    r.VIRT_LOCATION.x,
                                    r.VIRT_LOCATION.y,
                                    command.args.x.unwrap(),
                                    command.args.y.unwrap(),
                                    command.args.i.unwrap(),
                                    command.args.j.unwrap(),
                                    ArcDirection::CW,
                                )))
                                .is_err()
                            {
                            } else {
                                *r.VIRT_LOCATION = Point {
                                    x: command.args.x.unwrap(),
                                    y: command.args.y.unwrap(),
                                };
                            }
                        } else if (command.kind, command.number) == G3 {
                            if r.MOVE_BUFFER
                                .push(&Movement::ArcMove(ArcMove::new(
                                    r.LOCATION.x,
                                    r.LOCATION.y,
                                    command.args.x.unwrap(),
                                    command.args.y.unwrap(),
                                    command.args.i.unwrap(),
                                    command.args.j.unwrap(),
                                    ArcDirection::CCW,
                                )))
                                .is_err()
                            {}
                        } else if (command.kind, command.number) == M0 {
                            r.MOVE_BUFFER.clear();
                            r.CURRENT.x = None;
                            r.CURRENT.y = None;
                        }
                    }
                }
            }
        }
    }

    // check if current move is done
    if r.CURRENT.x.is_none() && r.CURRENT.y.is_none() {
        // if it is try to grab a move off the buffer
        if let Some(new_move) = r.MOVE_BUFFER.next() {
            match new_move.x {
                // if there's an x movement set the direction and make the movement positive
                Some(x) => {
                    let next = x - r.LOCATION.x;
                    if next > 0.0 {
                        r.STEPPER_X.direction(Direction::CW);
                    } else {
                        r.STEPPER_X.direction(Direction::CCW);
                    }
                    r.CURRENT.x = Some(core::num::Float::abs(next));
                    r.TIMER_X.listen(Event::Update);
                }
                // if there isn't a move disable the stepper
                None => {
                    r.CURRENT.x = None;
                    r.TIMER_X.unlisten(Event::Update);
                }
            }
            match new_move.y {
                // if there's an y movement set the direction and make the movement positive
                Some(y) => {
                    let next = y - r.LOCATION.y;
                    if next > 0.0 {
                        r.STEPPER_Y.direction(Direction::CW);
                    } else {
                        r.STEPPER_Y.direction(Direction::CCW);
                    }
                    r.CURRENT.y = Some(core::num::Float::abs(next));
                    r.TIMER_Y.listen(Event::Update);
                }
                // if there isn't a move disable the stepper
                None => {
                    r.CURRENT.y = None;
                    r.TIMER_Y.unlisten(Event::Update);
                }
            }

            match (r.CURRENT.x, r.CURRENT.y) {
                // TODO: test why these aren't matching up on longer moves
                // TODO: handle x or y being zero
                // calculate the stepper speeds so that x and y movement end at the same time
                // (linear interpolation)
                (Some(_x), None) => {
                    r.TIMER_X.start(MAX_FREQ_X.hz());
                }
                (None, Some(_y)) => {
                    r.TIMER_Y.start(MAX_FREQ_Y.hz());
                }
                (Some(x), Some(y)) => {
                    if x > y {
                        let freq = MAX_FREQ_Y / (x / y) as u32 + 1;
                        r.TIMER_X.start(MAX_FREQ_X.hz());
                        r.TIMER_Y.start(freq.hz());
                    } else {
                        let freq = MAX_FREQ_X / (y / x) as u32 + 1;
                        r.TIMER_X.start(freq.hz());
                        r.TIMER_Y.start(MAX_FREQ_Y.hz());
                    }
                }
                _ => {}
            }
        }
    }

    // send performance info once a second
    // TODO: figure out a good interval, 1 second may be too long to be useful
    if r.LAST_UPDATE.elapsed() > 64_000_000 {
        *r.LAST_UPDATE = r.MONO.now();
        // write the cpu monitoring data out
        let mut data = [0; 12];
        let mut buf: [u8; 13] = [0; 13];
        LE::write_u32(&mut data[0..4], *r.SLEEP);
        LE::write_f32(&mut data[4..8], r.LOCATION.x);
        LE::write_f32(&mut data[8..12], r.LOCATION.y);
        cobs::encode(&data, &mut buf);
        *r.SLEEP = 0;
        for byte in &buf {
            block!(r.TX.write(*byte)).ok();
        }
    }
}

// Handle movement for the X axis stepper
fn timer_x(_t: &mut Threshold, mut r: TIM2::Resources) {
    if let Some(x) = r.CURRENT.x {
        if x > 0.0 {
            r.STEPPER_X.step();
            r.CURRENT.x = Some(x - X_STEP_SIZE);
            match r.STEPPER_X.direction {
                Direction::CW => r.LOCATION.x += X_STEP_SIZE,
                _ => r.LOCATION.x -= X_STEP_SIZE,
            }
        } else {
            r.CURRENT.x = None;
        }
    } else {
        r.STEPPER_X.disable();
    }

    if r.TIMER_X.wait().is_err() {
        //interrupts probably disabled
    }
}

// Handle movement for the Y axis stepper
fn timer_y(_t: &mut Threshold, mut r: TIM3::Resources) {
    if let Some(y) = r.CURRENT.y {
        if y > 0.0 {
            r.STEPPER_Y.step();
            r.CURRENT.y = Some(y - Y_STEP_SIZE);
            match r.STEPPER_Y.direction {
                Direction::CW => r.LOCATION.y += Y_STEP_SIZE,
                _ => r.LOCATION.y -= Y_STEP_SIZE,
            }
        } else {
            r.CURRENT.y = None;
        }
    } else {
        r.STEPPER_Y.disable();
    }

    if r.TIMER_Y.wait().is_err() {
        //interrupts probably disabled
    }
}

// serial recieve interrupt, triggered when there is data in RX buffer on the device
// not using DMA because it's easier to process one byte at a time
// moved gcode processing to sys_tick due to serial overrun errors
// need to keep this handler as fast as possible
fn rx(_t: &mut Threshold, mut r: USART1::Resources) {
    // Read each character from serial as it comes in
    match r.RX.read() {
        Ok(c) => {
            // TODO: handle buffer being full
            if r.RX_BUF.push(&c).is_ok() {}
        }
        Err(e) => {
            match e {
                nb::Error::WouldBlock => {
                    for c in b"blocking\r\n" {
                        block!(r.TX.write(*c)).ok();
                    }
                }
                // currently no way to easily clear the overrun flag, if you hit this
                // it'll be stuck here
                nb::Error::Other(hal::serial::Error::Overrun) => {
                    for c in b"overrun error\r\n" {
                        block!(r.TX.write(*c)).ok();
                    }
                }
                _ => {
                    for c in b"other error\r\n" {
                        block!(r.TX.write(*c)).ok();
                    }
                }
            }
        }
    }
}
