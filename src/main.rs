//! examples/init.rs

#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_semihosting;

use cortex_m::peripheral::syst::SystClkSource;
use gcode::Mnemonic;
use nb::block;
use rtfm::app;
// atan2 and sqrt
use libm::F32Ext;
use rtfm::export::wfi;
use stepper_driver::{ic::ULN2003, Direction, Stepper};
use stm32f103xx::Interrupt;
use stm32f103xx_hal::{
    //flash::FlashExt,
    gpio::{
        gpioa::{PA1, PA2, PA3, PA4},
        gpiob::{PB5, PB6, PB7, PB8},
        Output, PushPull,
    },
    prelude::*,
    serial::{Rx, Serial, Tx},
    time::{enable_trace, Instant, MonoTimer},
    timer::{Event, Timer},
};

// used for encoding data to send over serial
use byteorder::{ByteOrder, LE};
// used to convert bytes to string
use core::str;

const CARRIAGE_RETURN: u8 = 13;
const NEW_LINE: u8 = 10;

// TODO: have these initialized over the serial connection and save to flash
const X_STEPS_MM: f32 = 600.0;
const X_STEP_SIZE: f32 = 1.0 / X_STEPS_MM;
const Y_STEPS_MM: f32 = 600.0;
const Y_STEP_SIZE: f32 = 1.0 / Y_STEPS_MM;
// max speed in mm/s
const MAX_SPEED: f32 = 0.3;
const MAX_FREQ_X: u32 = (MAX_SPEED / X_STEP_SIZE) as u32;
const MAX_FREQ_Y: u32 = (MAX_SPEED / Y_STEP_SIZE) as u32;
// Length of linear segment for arcs
const MM_PER_ARC_SEGMENT: f32 = 1.0;

// max characters allowed in a line of gcode
const RX_SZ: usize = 256;

// number of moves to buffer
const MOVE_SZ: usize = 256;

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

// Serial receive buffer
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

    pub fn push(&mut self, data: u8) -> Result<(), ()> {
        if self.index < RX_SZ {
            self.buffer[self.index] = data;
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
        if let Some(index) = self
            .buffer
            .iter()
            .position(|&x| x == CARRIAGE_RETURN || x == NEW_LINE)
        {
            let mut line = Buffer::new();
            let mut remainder = Buffer::new();
            for c in &self.buffer[0..index] {
                // TODO: handle error here
                if line.push(*c).is_err() {}
            }
            for c in &self.buffer[index + 1..self.index] {
                // TODO: handle error here
                if remainder.push(*c).is_err() {}
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

impl LinearMove {
    pub const fn new() -> LinearMove {
        LinearMove { x: None, y: None }
    }
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
        let radius = ((curr_x - center_x).powf(2.0) + (curr_y - center_y).powf(2.0)).sqrt();
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
        if angular_travel == 0.0
            && (curr_x - end_x).abs() < core::f32::EPSILON
            && (curr_y - end_y).abs() < core::f32::EPSILON
        {
            angular_travel = (360_f32).to_radians();
        }

        let mm_of_travel = (angular_travel * radius).abs();

        let segments = (mm_of_travel / MM_PER_ARC_SEGMENT).floor();
        let sin_t = angular_travel / segments;
        let cos_t = 1.0 - 0.5 * (sin_t).powf(2.0); // Small angle approximation

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

#[app(device = stm32f103xx)]
const APP: () = {
    // linear move that is active
    static mut CURRENT: LinearMove = LinearMove::new();
    static mut RX_BUF: Buffer = Buffer::new();
    static mut CMD_BUF: Buffer = Buffer::new();
    static mut MOVE_BUFFER: MoveBuffer = MoveBuffer::new();
    // current location
    static mut LOCATION: Point = Point { x: 0.0, y: 0.0 };
    // location once moves are processed (used for adding an arc movement)
    static mut VIRT_LOCATION: Point = Point { x: 0.0, y: 0.0 };
    static mut STEPPER_X: STEPPER_X = ();
    static mut STEPPER_Y: STEPPER_Y = ();
    static mut TIMER_X: Timer<stm32f103xx::TIM2> = ();
    static mut TIMER_Y: Timer<stm32f103xx::TIM3> = ();
    static mut TX: TX = ();
    static mut RX: RX = ();
    static mut LAST_UPDATE: Instant = ();
    // monotimer for cpu monitor
    static MONO: MonoTimer = ();
    static mut SLEEP: u32 = 0;

    #[init]
    fn init() {
        let device: stm32f103xx::Peripherals = device;
        let mut core: rtfm::Peripherals = core;
        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();

        // TODO: test performance and see what this needs to actually be set to
        let clocks = rcc
            .cfgr
            .sysclk(64.mhz())
            .pclk1(32.mhz())
            .freeze(&mut flash.acr);

        let trace = enable_trace(core.DCB);
        // start timer for performance monitoring
        let mono = MonoTimer::new(core.DWT, trace, clocks);

        // configure the system timer
        // TODO: test performance and see what this needs to actually be set to
        core.SYST.set_clock_source(SystClkSource::Core);
        core.SYST.set_reload(100_000);
        core.SYST.enable_interrupt();
        core.SYST.enable_counter();

        // These timers are used to control the step speed of the steppers
        // the initial update freq doesn't matter since they won't start until a move is received
        // just don't set to 0hz
        let tim2 = Timer::tim2(device.TIM2, 1.hz(), clocks, &mut rcc.apb1);
        let tim3 = Timer::tim3(device.TIM3, 1.hz(), clocks, &mut rcc.apb1);

        let mut afio = device.AFIO.constrain(&mut rcc.apb2);

        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = device.GPIOB.split(&mut rcc.apb2);

        // SERIAL
        let pa9 = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let pa10 = gpioa.pa10;

        let mut serial = Serial::usart1(
            device.USART1,
            (pa9, pa10),
            &mut afio.mapr,
            115_200.bps(),
            clocks,
            &mut rcc.apb2,
        );

        // Enable RX interrupt
        serial.listen(stm32f103xx_hal::serial::Event::Rxne);

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

        STEPPER_X = stepper_x;
        STEPPER_Y = stepper_y;
        TIMER_X = tim2;
        TIMER_Y = tim3;
        LAST_UPDATE = mono.now();
        MONO = mono;
        TX = tx;
        RX = rx;
    }

    #[idle(resources = [MONO, SLEEP])]
    fn idle() -> ! {
        loop {
            let before = resources.MONO.now();
            wfi();
            resources.SLEEP.lock(|sleep| *sleep += before.elapsed());
            rtfm::pend(Interrupt::TIM2);
            rtfm::pend(Interrupt::TIM3);
            rtfm::pend(Interrupt::USART1);
        }
    }

    #[exception(resources = [CURRENT, RX_BUF, CMD_BUF, MOVE_BUFFER, LOCATION, VIRT_LOCATION, SLEEP, STEPPER_X, STEPPER_Y, TX, TIMER_X, TIMER_Y, LAST_UPDATE, MONO])]
    fn SysTick() {
        // check if any commands have been received, if so process the gcode
        // TODO: this currently isn't fast enough to keep up if batches of commands are sent
        if let Some(data) = resources.RX_BUF.read() {
            // add the data to the command buffer
            for c in &data.buffer[0..data.index] {
                // TODO: handle error here
                if resources.CMD_BUF.push(*c).is_err() {}
            }
            // if any lines have been received, process them
            while let Some((line, buffer)) = resources.CMD_BUF.split() {
                *resources.CMD_BUF = buffer;
                if let Ok(gcode) = str::from_utf8(&line.buffer[0..line.index]) {
                    for line in gcode::parse(&gcode) {
                        match line.mnemonic() {
                            Mnemonic::General => {
                                match line.major_number() {
                                    0 | 1 => {
                                        // TODO: Handle the buffer being full
                                        if resources
                                            .MOVE_BUFFER
                                            .push(&Movement::LinearMove(LinearMove {
                                                x: line.value_for('x'),
                                                y: line.value_for('y'),
                                            }))
                                            .is_err()
                                        {
                                        } else {
                                            *resources.VIRT_LOCATION = Point {
                                                x: line.value_for('x').unwrap(),
                                                y: line.value_for('y').unwrap(),
                                            };
                                        }
                                    }
                                    // TODO: if path planning is going to be added arcs should be converted in to linear moves here
                                    2 => {
                                        if resources
                                            .MOVE_BUFFER
                                            .push(&Movement::ArcMove(ArcMove::new(
                                                resources.VIRT_LOCATION.x,
                                                resources.VIRT_LOCATION.y,
                                                line.value_for('x').unwrap(),
                                                line.value_for('y').unwrap(),
                                                line.value_for('i').unwrap(),
                                                line.value_for('j').unwrap(),
                                                ArcDirection::CW,
                                            )))
                                            .is_err()
                                        {
                                        } else {
                                            *resources.VIRT_LOCATION = Point {
                                                x: line.value_for('x').unwrap(),
                                                y: line.value_for('y').unwrap(),
                                            };
                                        }
                                    }
                                    3 => {
                                        if resources
                                            .MOVE_BUFFER
                                            .push(&Movement::ArcMove(ArcMove::new(
                                                resources.LOCATION.x,
                                                resources.LOCATION.y,
                                                line.value_for('x').unwrap(),
                                                line.value_for('y').unwrap(),
                                                line.value_for('i').unwrap(),
                                                line.value_for('j').unwrap(),
                                                ArcDirection::CCW,
                                            )))
                                            .is_err()
                                        {}
                                    }
                                    _ => {}
                                }
                            }
                            Mnemonic::Miscellaneous => match line.major_number() {
                                0 => {
                                    resources.MOVE_BUFFER.clear();
                                    resources.CURRENT.x = None;
                                    resources.CURRENT.y = None;
                                }
                                _ => {}
                            },
                            _ => {}
                        }
                    }
                }
            }
        }

        // check if current move is done
        if resources.CURRENT.x.is_none() && resources.CURRENT.y.is_none() {
            // if it is try to grab a move off the buffer
            if let Some(new_move) = resources.MOVE_BUFFER.next() {
                match new_move.x {
                    // if there's an x movement set the direction and make the movement positive
                    Some(x) => {
                        let next = x - resources.LOCATION.x;
                        if next > 0.0 {
                            resources.STEPPER_X.direction(Direction::CW);
                        } else {
                            resources.STEPPER_X.direction(Direction::CCW);
                        }
                        resources.CURRENT.x = Some((next).abs());
                        resources.TIMER_X.listen(Event::Update);
                    }
                    // if there isn't a move disable the stepper
                    None => {
                        resources.CURRENT.x = None;
                        resources.TIMER_X.unlisten(Event::Update);
                    }
                }
                match new_move.y {
                    // if there's an y movement set the direction and make the movement positive
                    Some(y) => {
                        let next = y - resources.LOCATION.y;
                        if next > 0.0 {
                            resources.STEPPER_Y.direction(Direction::CW);
                        } else {
                            resources.STEPPER_Y.direction(Direction::CCW);
                        }
                        resources.CURRENT.y = Some((next).abs());
                        resources.TIMER_Y.listen(Event::Update);
                    }
                    // if there isn't a move disable the stepper
                    None => {
                        resources.CURRENT.y = None;
                        resources.TIMER_Y.unlisten(Event::Update);
                    }
                }

                match (resources.CURRENT.x, resources.CURRENT.y) {
                    // TODO: test why these aren't matching up on longer moves
                    // TODO: handle x or y being zero
                    // calculate the stepper speeds so that x and y movement end at the same time
                    // (linear interpolation)
                    (Some(_x), None) => {
                        resources.TIMER_X.start(MAX_FREQ_X.hz());
                    }
                    (None, Some(_y)) => {
                        resources.TIMER_Y.start(MAX_FREQ_Y.hz());
                    }
                    (Some(x), Some(y)) => {
                        if x > y {
                            let freq = MAX_FREQ_Y / (x / y) as u32 + 1;
                            resources.TIMER_X.start(MAX_FREQ_X.hz());
                            resources.TIMER_Y.start(freq.hz());
                        } else {
                            let freq = MAX_FREQ_X / (y / x) as u32 + 1;
                            resources.TIMER_X.start(freq.hz());
                            resources.TIMER_Y.start(MAX_FREQ_Y.hz());
                        }
                    }
                    _ => {}
                }
            }
        }

        // send performance info once a second
        // TODO: figure out a good interval, 1 second may be too long to be useful
        if resources.LAST_UPDATE.elapsed() > 32_000_000 {
            *resources.LAST_UPDATE = resources.MONO.now();
            // write the cpu monitoring data out
            let mut data = [0; 12];
            let mut buf: [u8; 13] = [0; 13];
            LE::write_u32(&mut data[0..4], *resources.SLEEP);
            LE::write_f32(&mut data[4..8], resources.LOCATION.x);
            LE::write_f32(&mut data[8..12], resources.LOCATION.y);
            cobs::encode(&data, &mut buf);
            *resources.SLEEP = 0;
            for byte in &buf {
                block!(resources.TX.write(*byte)).ok();
            }
        }
    }

    #[interrupt(resources=[CURRENT,LOCATION, STEPPER_X, TIMER_X])]
    fn TIM2() {
        if let Some(x) = resources.CURRENT.x {
            if x > 0.0 {
                resources.STEPPER_X.step();
                resources.CURRENT.x = Some(x - X_STEP_SIZE);
                match resources.STEPPER_X.direction {
                    Direction::CW => resources.LOCATION.x += X_STEP_SIZE,
                    _ => resources.LOCATION.x -= X_STEP_SIZE,
                }
            } else {
                resources.CURRENT.x = None;
            }
        } else {
            resources.STEPPER_X.disable();
        }

        if resources.TIMER_X.wait().is_err() {
            //interrupts probably disabled
        }
    }

    #[interrupt(resources=[CURRENT,LOCATION, STEPPER_Y, TIMER_Y])]
    fn TIM3() {
        if let Some(y) = resources.CURRENT.y {
            if y > 0.0 {
                resources.STEPPER_Y.step();
                resources.CURRENT.y = Some(y - Y_STEP_SIZE);
                match resources.STEPPER_Y.direction {
                    Direction::CW => resources.LOCATION.y += Y_STEP_SIZE,
                    _ => resources.LOCATION.y -= Y_STEP_SIZE,
                }
            } else {
                resources.CURRENT.y = None;
            }
        } else {
            resources.STEPPER_Y.disable();
        }

        if resources.TIMER_Y.wait().is_err() {
            //interrupts probably disabled
        }
    }

    #[interrupt(resources=[RX,RX_BUF,TX])]
    fn USART1() {
        // Read each character from serial as it comes in
        match resources.RX.read() {
            Ok(c) => {
                // TODO: handle buffer being full
                if resources.RX_BUF.push(c).is_ok() {}
            }
            Err(e) => {
                match e {
                    nb::Error::WouldBlock => {
                        for c in b"blocking\r\n" {
                            block!(resources.TX.write(*c)).ok();
                        }
                    }
                    // currently no way to easily clear the overrun flag, if you hit this
                    // it'll be stuck here
                    nb::Error::Other(stm32f103xx_hal::serial::Error::Overrun) => {
                        for c in b"overrun error\r\n" {
                            block!(resources.TX.write(*c)).ok();
                        }
                    }
                    _ => {
                        for c in b"other error\r\n" {
                            block!(resources.TX.write(*c)).ok();
                        }
                    }
                }
            }
        }
    }
};
