//! zen garden sandbox
//! Codes currently supported
//! G0/G1 linear movement
//! G2/G3 arc movement
//! G28 home (currently homes X and Y always)
//! M0 emergency stop (also clears move buffer)
//!
//! only uses absolute positioning for now

//#![deny(unsafe_code)]
//#![deny(warnings)]
#![no_main]
#![no_std]
#![feature(core_intrinsics)]

extern crate panic_abort;

use core::intrinsics::powif32;
use gcode::{buffers::DefaultBuffers, Mnemonic, Nop, Parser};
use nb::block;
use rtic::{
    app,
    cyccnt::{Instant, U32Ext},
};
// atan2 and sqrt
use micromath::F32Ext;
use rtic::export::wfi;
use stepper_driver::{ic::ULN2003, Direction, Stepper};
use stm32f1::stm32f103::Interrupt;
use stm32f1xx_hal::{
    gpio::{
        gpioa::{PA1, PA2, PA3, PA4},
        gpiob::{PB5, PB6, PB7, PB8},
        Output, PushPull,
    },
    prelude::*,
    serial::{Config, Rx, Serial, Tx},
    timer::{CountDownTimer, Event, Timer},
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
const MAX_SPEED: f32 = 0.4;
const MAX_FREQ_X: u32 = (MAX_SPEED / X_STEP_SIZE) as u32;
const MAX_FREQ_Y: u32 = (MAX_SPEED / Y_STEP_SIZE) as u32;
// Length of linear segment for arcs
const MM_PER_ARC_SEGMENT: f32 = 1.0;

// Size of buffer for received serial data
// max length of gcode line is supposed to be 256 and we need to hold a min of 2 lines
// TODO: should probably increase this since utf-8 is not 1 byte per character, seems to be working well though
const RX_SZ: usize = 512;

// number of moves to buffer
// TODO: tweak this after testing with real patterns
const MOVE_SZ: usize = 100;

fn sqr(x: f32) -> f32 {
    unsafe { powif32(x, 2) }
}

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

    pub fn len(&self) -> usize {
        self.tail - self.head
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
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

    pub fn clear(&mut self) {
        self.tail = self.head;
    }
}

impl Iterator for MoveBuffer {
    type Item = LinearMove;

    fn next(&mut self) -> Option<LinearMove> {
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

// new is const so you can make static LinearMove variables
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
        let radius = (sqr(curr_x - center_x) + sqr(curr_y - center_y)).sqrt();
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
        let cos_t = 1.0 - 0.5 * sqr(sin_t); // Small angle approximation

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
type TX = Tx<stm32f1xx_hal::pac::USART1>;
type RX = Rx<stm32f1xx_hal::pac::USART1>;

// the first stepper is connected to a ULN2003 on pins PA1-4
#[allow(non_camel_case_types)]
type STEPPER_X = Stepper<
    PA1<Output<PushPull>>,
    PA2<Output<PushPull>>,
    PA3<Output<PushPull>>,
    PA4<Output<PushPull>>,
    ULN2003,
    core::convert::Infallible,
>;

// the second stepper is connected to a ULN2003 on pins PB5-8
#[allow(non_camel_case_types)]
type STEPPER_Y = Stepper<
    PB5<Output<PushPull>>,
    PB6<Output<PushPull>>,
    PB7<Output<PushPull>>,
    PB8<Output<PushPull>>,
    ULN2003,
    core::convert::Infallible,
>;

#[app(device = stm32f1xx_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        #[init(LinearMove::new())]
        CURRENT: LinearMove,
        #[init(Buffer::new())]
        RX_BUF: Buffer,
        #[init(Buffer::new())]
        CMD_BUF: Buffer,
        #[init(MoveBuffer::new())]
        MOVE_BUFFER: MoveBuffer,
        #[init(Point { x: 0.0, y: 0.0 })]
        LOCATION: Point,
        #[init(Point { x: 0.0, y: 0.0 })]
        VIRT_LOCATION: Point,
        STEPPER_X: STEPPER_X,
        STEPPER_Y: STEPPER_Y,
        TIMER_X: CountDownTimer<stm32f1xx_hal::pac::TIM2>,
        TIMER_Y: CountDownTimer<stm32f1xx_hal::pac::TIM3>,
        TX: TX,
        RX: RX,
        EXTI: stm32f1xx_hal::pac::EXTI,
        #[init(0)]
        SLEEP: u32,
    }

    #[init(schedule = [process, perf])]
    fn init(cx: init::Context) -> init::LateResources {
        let device = cx.device;
        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();
        let mut afio = device.AFIO.constrain(&mut rcc.apb2);
        let exti = device.EXTI;

        // TODO: test performance and see what this needs to actually be set to
        // set the system clock to max for this chip
        let clocks = rcc
            .cfgr
            .sysclk(64.mhz())
            .pclk1(32.mhz())
            .freeze(&mut flash.acr);

        // These timers are used to control the step speed of the steppers
        // the initial update freq doesn't matter since they won't start until a move is received
        // just don't set to 0hz or you get a divide by zero error
        let tim2 = Timer::tim2(device.TIM2, &clocks, &mut rcc.apb1).start_count_down(1.hz());
        let tim3 = Timer::tim3(device.TIM3, &clocks, &mut rcc.apb1).start_count_down(1.hz());

        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = device.GPIOB.split(&mut rcc.apb2);

        // configure PB0 and PB1 for limit switches on X and Y axis
        gpiob.pb0.into_pull_up_input(&mut gpiob.crl);
        gpiob.pb1.into_pull_up_input(&mut gpiob.crl);

        // configure interrupts, PB0 to EXTI0, PB1 to EXTI1 for limit switches
        // enable interrupt mask for line 0 and 1
        exti.imr.write(|w| {
            w.mr0().set_bit();
            w.mr1().set_bit()
        });

        // set to falling edge triggering
        exti.ftsr.write(|w| {
            w.tr0().set_bit();
            w.tr1().set_bit()
        });

        // set exti0 and 1 to gpio bank B
        // TODO: submit patch to stm32f1 crate to make this call safe
        afio.exticr1.exticr1().write(|w| unsafe {
            w.exti0().bits(1);
            w.exti1().bits(1)
        });

        // SERIAL pins for USART 1
        let pa9 = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let pa10 = gpioa.pa10;

        let mut serial = Serial::usart1(
            device.USART1,
            (pa9, pa10),
            &mut afio.mapr,
            Config {
                baudrate: 9_600.bps(),
                ..Default::default()
            },
            clocks,
            &mut rcc.apb2,
        );

        // Enable RX interrupt
        serial.listen(stm32f1xx_hal::serial::Event::Rxne);

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
        for c in b"\r\ninitialized\r\n" {
            block!(tx.write(*c)).ok();
        }

        // schedule tasks to process gcode and send out performance data
        // TODO: tweak how often this runs
        cx.schedule
            .process(Instant::now() + 1_000_000.cycles())
            .unwrap();
        cx.schedule
            .perf(Instant::now() + 64_000_000.cycles())
            .unwrap();

        // pend all used interrupts
        rtic::pend(Interrupt::USART1);
        rtic::pend(Interrupt::EXTI0);
        rtic::pend(Interrupt::EXTI1);
        rtic::pend(Interrupt::TIM2);
        rtic::pend(Interrupt::TIM3);

        init::LateResources {
            STEPPER_X: stepper_x,
            STEPPER_Y: stepper_y,
            TIMER_X: tim2,
            TIMER_Y: tim3,
            TX: tx,
            RX: rx,
            EXTI: exti,
        }
    }

    #[idle(resources = [SLEEP])]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            // record when this loop starts
            let before = Instant::now();
            // wait for an interrupt (sleep)
            wfi();
            // after interrupt is fired add sleep time to the sleep tracker
            cx.resources
                .SLEEP
                .lock(|sleep| *sleep += before.elapsed().as_cycles());
        }
    }

    #[task(schedule = [process], resources = [CURRENT, RX_BUF, CMD_BUF, MOVE_BUFFER, LOCATION, VIRT_LOCATION, SLEEP, STEPPER_X, STEPPER_Y, TX, TIMER_X, TIMER_Y])]
    fn process(mut cx: process::Context) {
        // check if any commands have been received, if so process the gcode
        if let Some(data) = cx.resources.RX_BUF.lock(Buffer::read) {
            // add the data to the command buffer
            for c in &data.buffer[0..data.index] {
                // TODO: handle error here
                if cx.resources.CMD_BUF.push(*c).is_err() {}
            }
            // if any complete lines have been received, process them
            while let Some((line, buffer)) = cx.resources.CMD_BUF.split() {
                *cx.resources.CMD_BUF = buffer;
                if let Ok(gcode) = str::from_utf8(&line.buffer[0..line.index]) {
                    let parser: Parser<Nop, DefaultBuffers> = Parser::new(gcode, Nop);
                    for lines in parser {
                        for line in lines.gcodes() {
                            match line.mnemonic() {
                                Mnemonic::General => {
                                    match line.major_number() {
                                        0 | 1 => {
                                            // TODO: Handle the buffer being full
                                            if cx
                                                .resources
                                                .MOVE_BUFFER
                                                .push(&Movement::LinearMove(LinearMove {
                                                    x: line.value_for('x'),
                                                    y: line.value_for('y'),
                                                }))
                                                .is_err()
                                            {
                                            } else {
                                                let x = if let Some(x) = line.value_for('x') {
                                                    x
                                                } else {
                                                    cx.resources.VIRT_LOCATION.x
                                                };
                                                let y = if let Some(y) = line.value_for('y') {
                                                    y
                                                } else {
                                                    cx.resources.VIRT_LOCATION.y
                                                };
                                                *cx.resources.VIRT_LOCATION = Point { x, y };
                                            }
                                        }
                                        // TODO: if path planning is going to be added arcs should be converted in to linear moves here
                                        2 => {
                                            if cx
                                                .resources
                                                .MOVE_BUFFER
                                                .push(&Movement::ArcMove(ArcMove::new(
                                                    cx.resources.VIRT_LOCATION.x,
                                                    cx.resources.VIRT_LOCATION.y,
                                                    line.value_for('x').unwrap(),
                                                    line.value_for('y').unwrap(),
                                                    line.value_for('i').unwrap(),
                                                    line.value_for('j').unwrap(),
                                                    ArcDirection::CW,
                                                )))
                                                .is_err()
                                            {
                                            } else {
                                                let x = if let Some(x) = line.value_for('x') {
                                                    x
                                                } else {
                                                    cx.resources.VIRT_LOCATION.x
                                                };
                                                let y = if let Some(y) = line.value_for('y') {
                                                    y
                                                } else {
                                                    cx.resources.VIRT_LOCATION.y
                                                };
                                                *cx.resources.VIRT_LOCATION = Point { x, y };
                                            }
                                        }
                                        3 => {
                                            if cx
                                                .resources
                                                .MOVE_BUFFER
                                                .push(&Movement::ArcMove(ArcMove::new(
                                                    cx.resources.LOCATION.x,
                                                    cx.resources.LOCATION.y,
                                                    line.value_for('x').unwrap(),
                                                    line.value_for('y').unwrap(),
                                                    line.value_for('i').unwrap(),
                                                    line.value_for('j').unwrap(),
                                                    ArcDirection::CCW,
                                                )))
                                                .is_err()
                                            {
                                            }
                                        }
                                        28 => {
                                            // TODO: add support for homing x or y individually
                                            if cx
                                                .resources
                                                .MOVE_BUFFER
                                                .push(&Movement::LinearMove(LinearMove {
                                                    x: Some(-100_000.0),
                                                    y: Some(-100_000.0),
                                                }))
                                                .is_err()
                                            {
                                            }
                                        }
                                        _ => {}
                                    }
                                }
                                Mnemonic::Miscellaneous => match line.major_number() {
                                    0 => {
                                        cx.resources.MOVE_BUFFER.clear();
                                        cx.resources.CURRENT.x = None;
                                        cx.resources.CURRENT.y = None;
                                    }
                                    _ => {}
                                },
                                _ => {}
                            }
                        }
                    }
                }
            }
        }

        // check if current move is done
        if cx.resources.CURRENT.x.is_none() && cx.resources.CURRENT.y.is_none() {
            // if it is try to grab a move off the buffer
            if let Some(new_move) = cx.resources.MOVE_BUFFER.next() {
                match new_move.x {
                    // if there's an x movement set the direction and make the movement positive
                    Some(x) => {
                        let next = x - cx.resources.LOCATION.x;
                        if next > 0.0 {
                            cx.resources.STEPPER_X.direction(Direction::CW);
                        } else {
                            cx.resources.STEPPER_X.direction(Direction::CCW);
                        }
                        cx.resources.CURRENT.x = Some((next).abs());
                        cx.resources.TIMER_X.listen(Event::Update);
                    }
                    // if there isn't a move disable the stepper
                    None => {
                        cx.resources.TIMER_X.unlisten(Event::Update);
                    }
                }
                match new_move.y {
                    // if there's a y movement set the direction and make the movement positive
                    Some(y) => {
                        let next = y - cx.resources.LOCATION.y;
                        if next > 0.0 {
                            cx.resources.STEPPER_Y.direction(Direction::CW);
                        } else {
                            cx.resources.STEPPER_Y.direction(Direction::CCW);
                        }
                        cx.resources.CURRENT.y = Some((next).abs());
                        cx.resources.TIMER_Y.listen(Event::Update);
                    }
                    // if there isn't a move disable the stepper
                    None => {
                        cx.resources.TIMER_Y.unlisten(Event::Update);
                    }
                }

                match (cx.resources.CURRENT.x, cx.resources.CURRENT.y) {
                    // TODO: test why these aren't matching up on longer moves
                    // TODO: handle x or y being zero
                    // calculate the stepper speeds so that x and y movement end at the same time
                    // (linear interpolation)
                    (Some(_x), None) => {
                        cx.resources.TIMER_X.start(MAX_FREQ_X.hz());
                    }
                    (None, Some(_y)) => {
                        cx.resources.TIMER_Y.start(MAX_FREQ_Y.hz());
                    }
                    (Some(x), Some(y)) => {
                        if x > y {
                            let freq = (MAX_FREQ_Y / (x / y) as u32) + 1;
                            cx.resources.TIMER_X.start(MAX_FREQ_X.hz());
                            cx.resources.TIMER_Y.start(freq.hz());
                        } else {
                            let freq = (MAX_FREQ_X / (y / x) as u32) + 1;
                            cx.resources.TIMER_X.start(freq.hz());
                            cx.resources.TIMER_Y.start(MAX_FREQ_Y.hz());
                        }
                    }
                    _ => {}
                }
            }
        }
        // TODO: tweak how often this runs
        cx.schedule
            .process(cx.scheduled + 1_000_000.cycles())
            .unwrap();
    }

    #[task(schedule = [perf], resources = [SLEEP, TX, LOCATION])]
    fn perf(mut cx: perf::Context) {
        // send performance info once a second
        let mut data = [0; 12];
        let mut buf: [u8; 13] = [0; 13];
        LE::write_u32(&mut data[0..4], *cx.resources.SLEEP);
        LE::write_f32(&mut data[4..8], cx.resources.LOCATION.x);
        LE::write_f32(&mut data[8..12], cx.resources.LOCATION.y);
        cobs::encode(&data, &mut buf);
        *cx.resources.SLEEP = 0;
        for byte in &buf {
            cx.resources.TX.lock(|tx| block!(tx.write(*byte)).ok());
        }
        cx.schedule
            .perf(cx.scheduled + 64_000_000.cycles())
            .unwrap();
    }

    // Timer to handle X stepper movement, speed is set in the process software task
    #[task(binds=TIM2, resources=[CURRENT,LOCATION, STEPPER_X, TIMER_X, TX])]
    fn TIM2(cx: TIM2::Context) {
        if let Some(x) = cx.resources.CURRENT.x {
            if x > 0.0 {
                cx.resources.STEPPER_X.step().unwrap();
                cx.resources.CURRENT.x = Some(x - X_STEP_SIZE);
                match cx.resources.STEPPER_X.direction {
                    Direction::CW => cx.resources.LOCATION.x += X_STEP_SIZE,
                    _ => cx.resources.LOCATION.x -= X_STEP_SIZE,
                }
            } else {
                cx.resources.CURRENT.x = None;
            }
        } else {
            cx.resources.STEPPER_X.disable().unwrap();
        }

        if cx.resources.TIMER_X.wait().is_err() {
            //interrupts probably disabled
        }
    }

    // Timer to handle Y stepper movement, speed is set in the process software task
    #[task(binds=TIM3, resources=[CURRENT,LOCATION, STEPPER_Y, TIMER_Y])]
    fn TIM3(cx: TIM3::Context) {
        if let Some(y) = cx.resources.CURRENT.y {
            if y > 0.0 {
                cx.resources.STEPPER_Y.step().unwrap();
                cx.resources.CURRENT.y = Some(y - Y_STEP_SIZE);
                match cx.resources.STEPPER_Y.direction {
                    Direction::CW => cx.resources.LOCATION.y += Y_STEP_SIZE,
                    _ => cx.resources.LOCATION.y -= Y_STEP_SIZE,
                }
            } else {
                cx.resources.CURRENT.y = None;
            }
        } else {
            cx.resources.STEPPER_Y.disable().unwrap();
        }

        if cx.resources.TIMER_Y.wait().is_err() {
            //interrupts probably disabled
        }
    }

    // Interrupt handler for serial receive, needs to be high priority or the receive buffer overruns
    #[task(binds=USART1, priority = 2, resources=[RX,RX_BUF,TX])]
    fn USART1(cx: USART1::Context) {
        // Read each character from serial as it comes in and add it to the rx buffer
        match cx.resources.RX.read() {
            Ok(c) => {
                // TODO: handle buffer being full
                if cx.resources.RX_BUF.push(c).is_ok() {}
            }
            Err(e) => {
                match e {
                    // no data available
                    nb::Error::WouldBlock => {
                        /*
                        for c in b"blocking\r\n" {
                            block!(resources.TX.write(*c)).ok();
                        }
                        */
                    }
                    nb::Error::Other(stm32f1xx_hal::serial::Error::Overrun) => {
                        for c in b"rx buffer overrun error\r\n" {
                            block!(cx.resources.TX.write(*c)).ok();
                        }
                    }
                    _ => {
                        for c in b"other error\r\n" {
                            block!(cx.resources.TX.write(*c)).ok();
                        }
                    }
                }
            }
        }
    }

    // X axis limit switch interrupt handler
    #[task(binds=EXTI0, resources=[EXTI, LOCATION, VIRT_LOCATION, CURRENT])]
    fn EXTI0(cx: EXTI0::Context) {
        cx.resources.CURRENT.x = None;
        cx.resources.LOCATION.x = 0.0;
        cx.resources.VIRT_LOCATION.x = 0.0;
        // clear the interrupt pending bit or the interrupt will keep firing
        // TODO: see if there's a way to clear this without having an instance of the EXTI registers
        // reference https://github.com/rust-embedded/cortex-m/pull/138
        cx.resources.EXTI.pr.write(|w| w.pr0().set_bit());
    }

    // Y axis limit switch interrupt handler
    #[task(binds=EXTI1, resources=[EXTI, LOCATION, VIRT_LOCATION, CURRENT])]
    fn EXTI1(cx: EXTI1::Context) {
        cx.resources.CURRENT.y = None;
        cx.resources.LOCATION.y = 0.0;
        cx.resources.VIRT_LOCATION.y = 0.0;
        // clear the interrupt pending bit or the interrupt will keep firing
        cx.resources.EXTI.pr.write(|w| w.pr1().set_bit());
    }

    // spare interrupt used for scheduling software tasks
    extern "C" {
        fn USART2();
    }
};
