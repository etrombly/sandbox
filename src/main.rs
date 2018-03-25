//! Etch-a-sketch in sand

#![deny(unsafe_code)]
//#![deny(warnings)]
#![feature(proc_macro)]
#![feature(const_fn)]
#![no_std]

extern crate byteorder;
extern crate cobs;
extern crate cortex_m;
extern crate cortex_m_rtfm as rtfm;
extern crate gcode;
#[macro_use(block)]
extern crate nb;
extern crate stepper_driver;
extern crate stm32f103xx_hal as hal;
extern crate heapless;
extern crate m;

// used for encoding data to send over serial
use byteorder::{ByteOrder, LE};
// used to convert bytes to string
use core::str;

// some float math that's not in core 
use m::Float as _0;

use gcode::{Parser, Tokenizer};
use gcode::parser::{CommandKind, Line, Number};

// used for the movement buffer
use heapless::RingBuffer;

use hal::prelude::*;
use hal::stm32f103xx;
use hal::gpio::gpioa::{PA1, PA2, PA3, PA4};
use hal::gpio::gpiob::{PB5, PB6, PB7, PB8};
use hal::gpio::{Output, PushPull};
use hal::serial::{Rx, Serial, Tx};
use hal::time::{MonoTimer, Instant};
use hal::timer::{Timer, Event};

use cortex_m::peripheral::syst::SystClkSource;

use rtfm::{app, Resource, Threshold};

use stepper_driver::{Direction, Stepper};
use stepper_driver::ic::ULN2003;

const CARRIAGE_RETURN: u8 = 13;
const NEW_LINE: u8 = 10;

// TODO: have these initialized over the serial connection and save to flash
const X_STEPS_MM: f32 = 600.0;
const X_STEP_SIZE: f32 = 1.0 / X_STEPS_MM;
const Y_STEPS_MM: f32 = 600.0;
const Y_STEP_SIZE: f32 = 1.0 / Y_STEPS_MM;
// max speed in mm/s
const MAX_SPEED: f32 = 0.6;

// max characters allowed in a line of gcode
const RX_SZ: usize = 256;

// CONNECTIONS
type TX = Tx<stm32f103xx::USART1>;
type RX = Rx<stm32f103xx::USART1>;

// Serial recieve buffer
pub struct Buffer {
    index: usize,
    buffer: [u8; RX_SZ]
}

impl Buffer {
    const fn new() -> Buffer {
        Buffer{index:0, buffer:[0; RX_SZ]}
    }

    pub fn insert(&mut self, data: &u8) -> Result<(), ()>{
        if self.index < RX_SZ {
            self.buffer[self.index] = *data;
            self.index += 1;
            return Ok(())
        }
        Err(())
    }

    pub fn clear(&mut self) {
        self.index = 0;
    }
}

// the first stepper is connect to a ULN2003 on pins PA1-4
#[allow(non_camel_case_types)]
type STEPPER_X = Stepper<
    PA1<Output<PushPull>>,
    PA2<Output<PushPull>>,
    PA3<Output<PushPull>>,
    PA4<Output<PushPull>>,
    ULN2003,
>;

// the second stepper is connect to a ULN2003 on pins PB5-8
#[allow(non_camel_case_types)]
type STEPPER_Y = Stepper<
    PB5<Output<PushPull>>,
    PB6<Output<PushPull>>,
    PB7<Output<PushPull>>,
    PB8<Output<PushPull>>,
    ULN2003,
>;

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

pub trait Square {
    fn sqr(&self) -> f32;
}

// TODO: handle overflows
impl Square for f32 {
    fn sqr(&self) -> f32 {
        self * self
    }
}

// constrain a value between min and max
pub fn clamp (min: f32, max: f32, value: f32) -> f32 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

pub struct Move {
    pub x: Option<f32>,
    pub y: Option<f32>,
}

// Struct to handle a G2 or G3 command
// TODO: current x and y should be in global state
// TODO: change these to 2d points
// TODO: add direction (CW, CCW)
struct ArcMove {
    end_x: f32,
    end_y: f32,
    center_x: f32,
    center_y: f32,
    curr_x: f32,
    curr_y: f32,
    radius: f32,
}

impl ArcMove {
    pub fn new(curr_x: f32, curr_y: f32, end_x: f32, end_y: f32, i: f32, j: f32) -> ArcMove {
        let center_x = curr_x + i;
        let center_y = curr_y + j;
        let radius = ((curr_x - center_x).sqr() + (curr_y - center_y).sqr()).sqrt();
        ArcMove {end_x, 
            end_y, 
            center_x, 
            center_y, 
            curr_x, 
            curr_y,
            radius,
        }
    }
    
    // roughly translated from a python example, this can probably be optimized better
    pub fn step(&mut self) {
        // Figure out what quadrant we're in to know the next x and if y will be positive or negative
        // TODO: find a multiple of step size that works well for drawing line segments
        let (next_x, y_sign) = match (self.curr_x > self.center_x, self.curr_y > self.center_y,
                                      self.curr_x == self.center_x, self.curr_y == self.center_y) {
            // top right or top
            (true, true, false, false) |
            (false, true, true, false)=> {
                (clamp(self.center_x - self.radius, 
                self.center_x + self.radius, 
                self.curr_x + X_STEP_SIZE),
                1.0)
            },
            // bottom right or right
            (true, false, false, false) |
            (true, false, false, true)=> {
                (clamp(self.center_x - self.radius, 
                self.center_x + self.radius, 
                self.curr_x - X_STEP_SIZE),
                - 1.0)
            },
            // bottom left or bottom
            (false, false, false, false) |
            (false, false, true, false)=> {
                (clamp(self.center_x - self.radius, 
                self.center_x + self.radius, 
                self.curr_x - X_STEP_SIZE),
                -1.0)
            },
            // top left or left
            (false, true, false, false) |
            (false, false, false, true)=> {
                (clamp(self.center_x - self.radius, 
                self.center_x + self.radius, 
                self.curr_x + X_STEP_SIZE),
                1.0)
            },
            // not actually possible to reach here
            _ => (0.0, 0.0),
        };
        let d = next_x.sqr() - (2.0 * self.center_x * next_x) + self.center_x.sqr() + self.center_y.sqr() - self.radius.sqr();
        let D = self.center_y.sqr() - d;
        let next_y = self.center_y + (D.sqrt() * y_sign);
        // TODO: change this to add a linear move to the active movement
        self.curr_x = next_x;
        self.curr_y = next_y;
    }
    
    pub fn finished(&self) -> bool {
        if (self.curr_x - X_STEP_SIZE) < self.end_x &&
           (self.curr_x + X_STEP_SIZE) > self.end_x &&
           (self.curr_y - X_STEP_SIZE) < self.end_y &&
           (self.curr_y + X_STEP_SIZE) > self.end_y{
            return true
        }
        false
    }
}

app! {
    device: stm32f103xx,

    resources: {
        static STEPPER_X: STEPPER_X;
        static STEPPER_Y: STEPPER_Y;
        static CURRENT: Move;
        static TIMER_X: Timer<stm32f103xx::TIM2>;
        static TIMER_Y: Timer<stm32f103xx::TIM3>;
        static TX: TX;
        static RX_BUF: Buffer = Buffer::new();
        static RX: RX;
        static MOVE_BUFFER: RingBuffer<Move, [Move; 20]> = RingBuffer::new();
        static MONO: MonoTimer;
        static LAST_UPDATE: Instant;
        static SLEEP: u32 = 0;
    },

    idle: {
        resources: [SLEEP, MONO]
    },

    tasks: {
        SYS_TICK: {
            path: sys_tick,
            resources: [TX, TIMER_X, TIMER_Y, CURRENT, MOVE_BUFFER, STEPPER_X, STEPPER_Y, SLEEP, LAST_UPDATE, MONO],
        },

        TIM2: {
            path: timer_x,
            resources: [STEPPER_X, CURRENT, TIMER_X],
        },

        TIM3: {
            path: timer_y,
            resources: [STEPPER_Y, CURRENT, TIMER_Y],
        },

        USART1: {
            path: rx,
            resources: [RX, TX, MOVE_BUFFER, CURRENT, RX_BUF],
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
    
    // start timer for performance monnitoring
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

    // set up timer 4 to send out performance data
    let mut tim4 = Timer::tim4(p.device.TIM4, 1.hz(), clocks, &mut rcc.apb1);
    tim4.listen(Event::Update);

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

    for c in "initialized\r\n".as_bytes() {
        block!(tx.write(*c)).ok();
    }

    init::LateResources {
        STEPPER_X: stepper_x,
        STEPPER_Y: stepper_y,
        CURRENT: Move { x: None, y: None },
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

// Sys_tick mainly handles the movement buffer, not using any path planning since the steppers are slow
fn sys_tick(_t: &mut Threshold, mut r: SYS_TICK::Resources) {
    // check if current move is done
    if r.CURRENT.x.is_none() && r.CURRENT.y.is_none() {
        // if it is try to grab a move off the buffer
        if let Some(new_move) = r.MOVE_BUFFER.dequeue() {
            match new_move.x {
                // if there's an x movement set the direction and make the movement positive
                Some(x) => {
                    if x > 0.0 {
                        r.STEPPER_X.direction(Direction::CW);
                        r.CURRENT.x = Some(x);
                    } else {
                        r.STEPPER_X.direction(Direction::CCW);
                        r.CURRENT.x = Some(x * -1.0);
                    }
                    r.TIMER_X.listen(Event::Update);
                },
                // if there isn't a move disable the stepper 
                None => {
                    r.CURRENT.x = None;
                    r.TIMER_X.unlisten(Event::Update);
                },
            }
            match new_move.y {
                // if there's an y movement set the direction and make the movement positive
                Some(y) => {
                    if y > 0.0 {
                        r.STEPPER_Y.direction(Direction::CW);
                        r.CURRENT.y = Some(y);
                    } else {
                        r.STEPPER_Y.direction(Direction::CCW);
                        r.CURRENT.y = Some(y * -1.0);
                    }
                    r.TIMER_Y.listen(Event::Update);
                },
                // if there isn't a move disable the stepper 
                None => {
                    r.CURRENT.y = None;
                    r.TIMER_Y.unlisten(Event::Update);
                },
            }
            
            
            match (r.CURRENT.x, r.CURRENT.y) {
                // calculate the stepper speeds
                (Some(_x), None) => {
                    r.TIMER_X.start(((MAX_SPEED / X_STEP_SIZE) as u32).hz());
                },
                (None, Some(_y)) => {
                    r.TIMER_Y.start(((MAX_SPEED / Y_STEP_SIZE) as u32).hz());
                },
                (Some(x), Some(y)) => {
                    if x > y {
                        r.TIMER_X.start(((MAX_SPEED / X_STEP_SIZE) as u32).hz());
                        r.TIMER_Y.start((((MAX_SPEED / Y_STEP_SIZE) / (x / y)) as u32).hz());
                    } else {
                        r.TIMER_X.start((((MAX_SPEED / X_STEP_SIZE) / (y / x)) as u32).hz());
                        r.TIMER_Y.start(((MAX_SPEED / Y_STEP_SIZE) as u32).hz());
                    }
                },
                _ => {}
            }
        }
    }

    // send performance info every second
    // TODO: change this from a snapshot usage to an average or something more useful
    if r.LAST_UPDATE.elapsed() > 60_000_000 {
        *r.LAST_UPDATE = r.MONO.now();
        // write the cpu monitoring data out
        let mut data = [0; 4];
        let mut buf: [u8; 8] = [0;8];
        LE::write_u32(&mut data[0..4], *r.SLEEP);
        cobs::encode(&data, &mut buf);
        for byte in buf.iter() {
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
        } else {
            r.CURRENT.x = None;
        }
    } else {
        r.STEPPER_X.disable();
    }

    if let Err(_) = r.TIMER_X.wait() {
        //interrupts probably disabled
    }
}

// Handle movement for the Y axis stepper
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

    if let Err(_) = r.TIMER_Y.wait(){
        //interrupts probably disabled
    }
}

// serial recieve interrupt, triggered when there is data in RX buffer on the device
fn rx(_t: &mut Threshold, mut r: USART1::Resources) {
    // Read each character from serial as it comes in
    match r.RX.read() {
        Ok(c) => {
            // screen seems to not send a newline, so just check for either
            if c == CARRIAGE_RETURN || c == NEW_LINE {
                if let Ok(gcode) = str::from_utf8(&r.RX_BUF.buffer[0..r.RX_BUF.index]) {
                    let lexer = Tokenizer::new(gcode.chars());
                    // TODO: add error handling here
                    let tokens = lexer.filter_map(|t| t.ok());
                    let parser = Parser::new(tokens);
                    for line in parser {
                        if let Ok(Line::Cmd(command)) = line {
                            if (command.kind, command.number) == G0  || (command.kind, command.number) == G1 {
                                // TODO: Handle the buffer being full
                                if let Err(_) = r.MOVE_BUFFER.enqueue(Move{x: command.args.x, y: command.args.y}) {}
                            }
                            if (command.kind, command.number) == G2 {
                                // TODO: have this create an ArcMove and add it to the movement buffer
                            }
                            else if (command.kind, command.number) == M0 {
                                while let Some(_) = r.MOVE_BUFFER.dequeue() {}
                                r.CURRENT.x = None;
                                r.CURRENT.y = None;
                            }
                        }
                    }
                }
                r.RX_BUF.clear();
            }
            // If it's not a newline add it to the input buffer
            // TODO: Handle the buffer being full
            else if let Ok(_) = r.RX_BUF.insert(&c) {} 
        },
        Err(e) => {
            match e {
                nb::Error::WouldBlock => {
                    for c in "blocking\r\n".as_bytes() {
                        block!(r.TX.write(*c)).ok();
                    }
                },
                // currently no way to easily clear the overrun flag, if you hit this
                // it'll be stuck here
                nb::Error::Other(hal::serial::Error::Overrun) => {
                    for c in "overrun error\r\n".as_bytes() {
                        block!(r.TX.write(*c)).ok();
                    }
                }
                _ => {
                    for c in "other error\r\n".as_bytes() {
                        block!(r.TX.write(*c)).ok();
                    }
                }
            }
        }
    }
}