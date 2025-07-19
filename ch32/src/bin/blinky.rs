#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]
#![feature(iterator_try_collect)]

use ch32_hal::adc::{Adc, AnyAdcChannel};
use ch32_hal::bind_interrupts;
use ch32_hal::peripherals::ADC1;
use ch32_hal::{self as hal, adc::AdcChannel};
use core::fmt::Write;
use core::task::Poll;
use embedded_graphics::primitives::ellipse;
use hal::usart::UartTx;

use mididude_adc::i2c_device::{Command, Config, I2cSlave};

bind_interrupts!(struct Irqs {
    I2C1_EV => mididude_adc::i2c_device::EventInterruptHandler<ch32_hal::peripherals::I2C1>;
    I2C1_ER => mididude_adc::i2c_device::ErrorInterruptHandler<ch32_hal::peripherals::I2C1>;
});

struct InfPoti {
    in0: AnyAdcChannel<ADC1>,
    in1: AnyAdcChannel<ADC1>,
}

impl InfPoti {
    pub fn measure(&mut self, adc: &mut Adc<'_, ADC1>) -> (i16, i16) {
        let a1 = adc.convert(&mut self.in0, hal::adc::SampleTime::CYCLES73) & 0x0fff;
        let a2 = adc.convert(&mut self.in1, hal::adc::SampleTime::CYCLES73) & 0x0fff;
        (a1 as i16, a2 as i16)
    }
}

struct Converter {
    cutoff: i16,
    transition: i16,
    wrap_range: i16,
}

impl Default for Converter {
    fn default() -> Self {
        Self {
            cutoff: 128,
            transition: 16,
            wrap_range: 32,
        }
    }
}

impl Converter {
    pub fn tri_inv(&self, val: i16, offset: i16) -> (i16, i16) {
        let val = val.clamp(0, 1023);
        let p1 = val / 2;
        let p2 = 1024 - p1;
        return (
            (p1 + offset).rem_euclid(1024),
            (p2 + offset).rem_euclid(1024),
        );
    }

    pub fn tri_weight(&self, val: i16) -> f32 {
        let val = val.clamp(0, 1023);
        let diff = val.min(1024 - val);
        if diff < self.cutoff {
            0.0
        } else if diff < self.cutoff + self.transition {
            1.0 - (diff - self.cutoff) as f32 / self.transition as f32
        } else {
            1.0
        }
    }

    pub fn wrap(&self, v: (i16, i16)) -> (i16, i16) {
        if (v.0 <= self.wrap_range) && (v.1 >= 1024 - self.wrap_range) {
            (v.0, v.1 - 1024)
        } else if (v.1 <= self.wrap_range) && (v.0 >= 1024 - self.wrap_range) {
            (v.0 - 1024, v.1)
        } else {
            v
        }
    }

    pub fn convert(&self, v: (i16, i16)) -> i16 {
        let p1 = self.tri_inv(v.0, 0);
        let w_a = self.tri_weight(v.0);
        let p2 = self.tri_inv(v.1, -256);
        let w_b = self.tri_weight(v.1);
        let diffs = [
            (p1.0 - p2.0).abs(),
            (p1.0 - p2.1).abs(),
            (p1.1 - p2.0).abs(),
            (p1.1 - p2.1).abs(),
        ];
        let (pos_a, pos_b) =
            if (diffs[0] < diffs[1]) && (diffs[0] < diffs[2]) && (diffs[0] < diffs[3]) {
                (p1.0, p2.0)
            } else if (diffs[1] < diffs[2]) && (diffs[1] < diffs[3]) {
                (p1.0, p2.1)
            } else if (diffs[2] < diffs[3]) {
                (p1.1, p2.0)
            } else {
                (p1.1, p2.1)
            };

        let final_pos =
            (((pos_a as f32 * w_a + pos_b as f32 * w_b) / (w_a + w_b)) as i16).rem_euclid(1024);

        final_pos
    }

    pub fn convert_print(&self, v: (i16, i16), uart: &mut impl Write) -> i16 {
        let p1 = self.tri_inv(v.0, 0);
        let w_a = self.tri_weight(v.0);
        let p2 = self.tri_inv(v.1, -256);
        let w_b = self.tri_weight(v.1);
        let diffs = [
            (p1.0 - p2.0).abs(),
            (p1.0 - p2.1).abs(),
            (p1.1 - p2.0).abs(),
            (p1.1 - p2.1).abs(),
        ];
        let (pos_a, pos_b) =
            if (diffs[0] < diffs[1]) && (diffs[0] < diffs[2]) && (diffs[0] < diffs[3]) {
                (p1.0, p2.0)
            } else if (diffs[1] < diffs[2]) && (diffs[1] < diffs[3]) {
                (p1.0, p2.1)
            } else if (diffs[2] < diffs[3]) {
                (p1.1, p2.0)
            } else {
                (p1.1, p2.1)
            };

        let final_pos =
            (((pos_a as f32 * w_a + pos_b as f32 * w_b) / (w_a + w_b)) as i16).rem_euclid(1024);
        writeln!(
            uart,
            //":: {:?} :: {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}\r",
            //final_pos, v.0, v.1, p1, p2, diff_a, diff_b, pos_a, pos_b
            "{:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}, {:?}\r",
            v.0, v.1, final_pos, p1.0, p1.1, p2.0, p2.1, pos_a, pos_b
        )
        .unwrap();

        final_pos
    }
}

#[qingke_rt::entry]
fn main() -> ! {
    //hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI;
    let p = hal::init(config);

    //let mut uart = UartTx::new_blocking(p.USART1, p.PC0, Default::default()).unwrap();

    let mut config = Config::default();
    config.addr = 0x10;
    config.general_call = false;

    let i2c = I2cSlave::new(p.I2C1, config, p.PC1, p.PC2, Irqs);

    let mut adc = hal::adc::Adc::new(p.ADC1, Default::default());

    let mut potis = [
        InfPoti {
            in0: p.PA2.degrade_adc(),
            in1: p.PA1.degrade_adc(),
        },
        InfPoti {
            in0: p.PC4.degrade_adc(),
            in1: p.PD2.degrade_adc(),
        },
        InfPoti {
            in0: p.PD3.degrade_adc(),
            in1: p.PD5.degrade_adc(),
        },
        InfPoti {
            in0: p.PD6.degrade_adc(),
            in1: p.PD4.degrade_adc(),
        },
    ];

    let converter = Converter::default();

    loop {
        let mut listen = i2c.listen();
        loop {
            if let Poll::Ready(res) = listen() {
                match res {
                    Ok(Command::Read) => {
                        let positions = [
                            converter.convert(potis[0].measure(&mut adc)),
                            converter.convert(potis[1].measure(&mut adc)),
                            converter.convert(potis[2].measure(&mut adc)),
                            converter.convert(potis[3].measure(&mut adc)),
                        ];

                        let mut val: [u8; 8] = [0; 8];
                        val[..2].copy_from_slice(&positions[0].to_le_bytes());
                        val[2..4].copy_from_slice(&positions[1].to_le_bytes());
                        val[4..6].copy_from_slice(&positions[2].to_le_bytes());
                        val[6..].copy_from_slice(&positions[3].to_le_bytes());
                        let mut respond = i2c.respond_to_read(&val, Some(0));
                        loop {
                            if let Poll::Ready(res) = respond() {
                                match res {
                                    /*Ok(_) => (),
                                    Err(e) => writeln!(uart, "Error: {:?}\r", e).unwrap(),*/
                                    _ => (),
                                }
                                break;
                            }
                        }
                    }
                    //Ok(Command::Write(_, buf)) => val = buf[0],
                    //Ok(a) => (), //writeln!(uart, "Ok: {:?}\r", a).unwrap(),
                    /*Ok(Command::Write(_, _)) => (),
                    Ok(a) => writeln!(uart, "I2C: {:?}\r", a).unwrap(),
                    Err(e) => writeln!(uart, "Error: {:?}\r", e).unwrap(),*/
                    _ => (),
                }
                break;
            } else {
                // Timeout
                //writeln!(uart, "Timeout\r").unwrap();
            }
        }
    }
}

use core::panic::PanicInfo;

#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
