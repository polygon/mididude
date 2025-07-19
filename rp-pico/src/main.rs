//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]
#![feature(iter_next_chunk)]

use bsp::entry;
use cortex_m::register::control::Control;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::{delay::DelayNs, digital::OutputPin, i2c::I2c, pwm::SetDutyCycle};
use panic_probe as _;
use usb_device::{class_prelude::*, prelude::*};

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico::{
    self as bsp,
    hal::{
        fugit::HertzU32,
        gpio::{bank0::Gpio26, FunctionSio, PullNone, SioInput},
    },
};
// use sparkfun_pro_micro_rp2040 as bsp;

use usbd_midi::{
    message::{Channel, ControlFunction, FromClamped, Message, Note, U7},
    UsbMidiClass, UsbMidiEventPacket, UsbMidiPacketReader,
};

use arrayvec::ArrayVec;
use bsp::hal::{
    adc::{Adc, AdcPin},
    clocks::{init_clocks_and_plls, Clock},
    i2c::I2C,
    pac, pwm,
    sio::Sio,
    usb,
    watchdog::Watchdog,
    Timer,
};
use embedded_hal_0_2::adc::OneShot;

#[derive(Debug)]
struct KnobTracker {
    pub rotations: i32,
    pub last_val: u16,
    pub invert: bool,
}

impl KnobTracker {
    pub fn new(init: u16, invert: bool) -> Self {
        KnobTracker {
            rotations: 0,
            last_val: init,
            invert,
        }
    }

    pub fn update(&mut self, val: u16) {
        let diff = val as i32 - self.last_val as i32;
        if diff > 511 {
            // We went from low value to high value, rotation needs to go -1 in order to compensate
            self.rotations -= 1;
        } else if diff < -511 {
            // Went went from high value to low value, rotation needs to go +1 in order to compensate
            self.rotations += 1;
        }
        self.last_val = val;
    }

    pub fn value(&self) -> i32 {
        let val = self.rotations * 1024 + self.last_val as i32;
        if self.invert {
            -val
        } else {
            val
        }
    }
}

impl Format for KnobTracker {
    fn format(&self, fmt: Formatter) {
        crate::write!(fmt, "{}", self.value());
    }
}

#[derive(Clone)]
struct Parameter {
    last_knob_val: i32,
    range: i32,
    i_min: i32,
    i_max: i32,
    pub cc: ControlFunction,
    pub channel: Channel,
}

impl Parameter {
    pub fn new(cc: ControlFunction, channel: Channel) -> Self {
        Self {
            last_knob_val: 0,
            range: 1024,
            i_min: 0,
            i_max: 1024,
            cc,
            channel,
        }
    }

    pub fn connect_knob(&mut self, val: i32) {
        let param_val = self.value();
        self.last_knob_val = val;
        self.recalculate_limits(param_val);
    }

    pub fn set_value(&mut self, param_val: i32) {
        self.recalculate_limits(param_val);
    }

    pub fn update(&mut self, knob_val: i32) {
        self.last_knob_val = knob_val;
        if knob_val < self.i_min {
            self.recalculate_limits(0);
        } else if knob_val > self.i_max {
            self.recalculate_limits(16383);
        }
    }

    pub fn value(&self) -> i32 {
        ((self.last_knob_val - self.i_min) * 16384) / self.range
    }

    fn recalculate_limits(&mut self, param_val: i32) {
        self.i_min = self.last_knob_val - ((param_val * self.range) / 16384);
        self.i_max = self.i_min + self.range;
    }
}

impl Format for Parameter {
    fn format(&self, fmt: Formatter) {
        crate::write!(fmt, "{}", self.value());
    }
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut midi = usbd_midi::UsbMidiClass::new(&usb_bus, 1, 1).unwrap();

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x5e4))
        .device_class(1)
        .device_sub_class(0)
        .strings(&[StringDescriptors::default()
            .manufacturer("Matelab")
            .product("Mididude")
            .serial_number("17b")])
        .unwrap()
        .build();

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    //
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead.
    // One way to do that is by using [embassy](https://github.com/embassy-rs/embassy/blob/main/examples/rp/src/bin/wifi_blinky.rs)
    //
    // If you have a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here. Don't forget adding an appropriate resistor
    // in series with the LED.
    //let mut led_pin = pins.led.into_push_pull_output();

    let mut i2c0 = I2C::i2c0(
        pac.I2C0,
        pins.gpio20.reconfigure(),
        pins.gpio21.reconfigure(),
        HertzU32::kHz(250),
        &mut pac.RESETS,
        &clocks.system_clock,
    );
    let mut i2c1 = I2C::i2c1(
        pac.I2C1,
        pins.gpio18.reconfigure(),
        pins.gpio19.reconfigure(),
        HertzU32::kHz(250),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let mut pwm_slices = pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let pwm = &mut pwm_slices.pwm4;
    pwm.set_ph_correct();
    pwm.enable();

    let channel = &mut pwm.channel_b;
    channel.output_to(pins.led);

    let mut buf0 = [0; 8];
    let mut buf1 = [0; 8];
    let mut warmup = 0;
    loop {
        info!("I2C Read");
        match (
            i2c0.read(0x10 as u8, &mut buf0),
            i2c1.read(0x10 as u8, &mut buf1),
        ) {
            (Ok(_), Ok(_)) => warmup += 1,
            (a, b) => info!("I2C Read {:?}, {:?}", a.is_ok(), b.is_ok()),
        };
        if warmup > 100 {
            break;
        }
    }

    let mut it = buf0.chunks(2).chain(buf1.chunks(2));
    let mut vals = it.map(|chunk| (chunk[0] as u16) + ((chunk[1] as u16) << 8));
    let mut knobs: ArrayVec<KnobTracker, 8> = ArrayVec::new();
    for val in vals.take(8) {
        knobs.push(KnobTracker::new(val, false));
    }
    // TODO: Evil hack for build issue :)
    knobs[2].invert = true;

    let order = [4 as usize, 5, 0, 1, 7, 6, 3, 2];

    let mut params: ArrayVec<Parameter, 8> = ArrayVec::new();
    let mut valss: ArrayVec<i32, 8> = ArrayVec::new();
    for (idx, knob) in knobs.iter().enumerate() {
        params.push(Parameter::new(
            ControlFunction(U7::from_clamped(9 + idx as u8)),
            Channel::Channel5,
        ));
        valss.push(0);
        params
            .last_mut()
            .unwrap()
            .connect_knob(knobs[order[idx]].value());
    }

    let mut queue: ArrayVec<Message, 16> = ArrayVec::new();

    loop {
        for msg in queue.into_iter() {
            midi.send_packet(msg.into_packet(usbd_midi::CableNumber::Cable0))
                .unwrap_or(0);

            while usb_dev.poll(&mut [&mut midi]) {
                let mut buffer = [0; 64];

                if let Ok(size) = midi.read(&mut buffer) {
                    let mut reader = UsbMidiPacketReader::new(&buffer, size);
                    for msg in reader.into_iter() {
                        match msg {
                            Ok(msg) => {
                                let msg = Message::try_from(&msg).unwrap();
                                match msg {
                                    /* Message::ControlChange(ch, func, val) => {
                                        info!(
                                            "CC: {}, {}, {}",
                                            ch as u8,
                                            u8::from(func.0),
                                            u8::from(val)
                                        );
                                    }*/
                                    _ => (),
                                };
                            }
                            Err(_) => (),
                        }
                    }
                }
            }
        }
        queue = ArrayVec::new();

        if let (Ok(_), Ok(_)) = (
            i2c0.read(0x10 as u8, &mut buf0),
            i2c1.read(0x10 as u8, &mut buf1),
        ) {
            let it = buf0.chunks(2).chain(buf1.chunks(2));
            let vals = it.map(|chunk| (chunk[0] as u16) + ((chunk[1] as u16) << 8));
            for (i, val) in vals.enumerate() {
                valss[i] = val as i32;
                knobs[i].update(val);
                params[i].update(knobs[order[i]].value());
            }
        };
        //info!("vals: {:05}", valss[..]);
        //info!("knbs: {:05}", knobs[..]);
        info!("prms: {:05}", params[..]);
        for param in params.iter() {
            let cc_hi = param.cc.clone();
            let cc_lo = ControlFunction(U7::from_clamped(u8::from(cc_hi.clone().0) + 32));
            let msg_hi = Message::ControlChange(
                param.channel,
                cc_hi,
                U7::from_clamped((param.value() >> 7) as u8),
            );
            let msg_lo = Message::ControlChange(
                param.channel,
                cc_lo,
                U7::from_clamped((param.value() & 0x7f) as u8),
            );
            queue.push(msg_hi);
            queue.push(msg_lo);
        }

        //info!("{:?}, {:?}", buf0, buf1);

        /*if now as i64 - t_last_adc as i64 > 1_000 {
            t_last_adc = now;
            let value = <Adc as OneShot<
                Adc,
                i32,
                AdcPin<rp_pico::hal::gpio::Pin<Gpio26, FunctionSio<SioInput>, PullNone>>,
            >>::read(&mut adc, &mut adc_pin)
            .unwrap();
            info!("ADC Value: {}", value);
            channel.set_duty_cycle(16 * value as u16).unwrap();
            let message = MD::message::Message::ControlChange(
                MD::channel::Channel::Channel1,
                MD::message::control_function::ControlFunction::GENERAL_PURPOSE_CONTROLLER_1_16,
                U7::from_clamped((value / 32) as u8),
            );
            midi.send_message(UMD::usb_midi_event_packet::UsbMidiEventPacket {
                cable_number: UMD::cable_number::CableNumber::Cable0,
                message,
            })
            .unwrap_or(0);
        }*/
    }
}

// End of file
