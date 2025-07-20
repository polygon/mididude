use ch32_hal::pac::gpio::vals::{Cnf, Mode};
use ch32_hal::Peripheral;
use ch32_hal::{interrupt, peripherals};
use core::marker::PhantomData;
use core::task::Poll;

// I2C error
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
// TODO: Reinstate eventually
//#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(defmt::Format)]
#[non_exhaustive]
pub enum Error {
    /// I2C abort with error
    Abort,
    /// User passed in a response buffer that was 0 length
    InvalidResponseBufferLength,
    /// The response buffer length was too short to contain the message
    ///
    /// The length parameter will always be the length of the buffer, and is
    /// provided as a convenience for matching alongside `Command::Write`.
    /// Second returned value is total size of this write
    PartialWrite(usize, usize),
    /// The response buffer length was too short to contain the message
    ///
    /// The length parameter will always be the length of the buffer, and is
    /// provided as a convenience for matching alongside `Command::GeneralCall`.
    PartialGeneralCall(usize),
}

/// Received command
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
// TODO: Reinstate eventually
//#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(defmt::Format)]
pub enum Command {
    /// General Call
    GeneralCall(usize),
    /// Read
    Read,
    /// Write+read
    WriteRead(usize, [u8; 8]),
    /// Write
    Write(usize, [u8; 8]),
}

/// Possible responses to responding to a read
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
// TODO: Reinstate eventually
//#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(defmt::Format)]
pub enum ReadStatus {
    /// Transaction Complete, controller naked our last byte
    Done,
    /// Transaction Incomplete, controller trying to read more bytes than were provided
    NeedMoreBytes,
    /// Transaction Complere, but controller stopped reading bytes before we ran out
    LeftoverBytes(u16),
}

/// Slave Configuration
#[non_exhaustive]
#[derive(Copy, Clone)]
// TODO: Reinstate eventually
//#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(defmt::Format)]
pub struct Config {
    /// Target Address
    pub addr: u8,
    /// Control if the peripheral should ack to and report general calls.
    pub general_call: bool,
    // Frequency
    pub freq: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            addr: 0x55,
            general_call: true,
            freq: 48000000,
        }
    }
}

/// I2CSlave driver.
pub struct I2cSlave<'d, T: Instance> {
    phantom: PhantomData<&'d mut T>,
    config: Config,
}

#[derive(Debug, Clone)]
enum TransactionState {
    Idle,
    Receiving,
}

impl<'d, T: Instance> I2cSlave<'d, T> {
    /// Create a new instance.
    pub fn new<const REMAP: u8>(
        _peri: impl Peripheral<P = T> + 'd,
        config: Config,
        sda_pin: impl SdaPin<T, REMAP>,
        scl_pin: impl SclPin<T, REMAP>,
    ) -> Self {
        assert!(config.addr != 0);

        // Configure pin mapping
        let afio = ch32_hal::pac::AFIO;
        afio.pcfr1().modify(|r| {
            r.set_i2c1_rm(REMAP & 0x1 > 0);
            r.set_i2c1_rm1(REMAP & 0x2 > 0);
        });

        // Configure Pins
        ch32_hal::pac::GPIO(sda_pin.port() as usize)
            .cfglr()
            .modify(|r| {
                r.set_mode(sda_pin.pin() as usize, Mode::OUTPUT_50MHZ);
                r.set_cnf(sda_pin.pin() as usize, Cnf::AF_OPEN_DRAIN_OUT);
            });
        ch32_hal::pac::GPIO(scl_pin.port() as usize)
            .cfglr()
            .modify(|r| {
                r.set_mode(scl_pin.pin() as usize, Mode::OUTPUT_50MHZ);
                r.set_cnf(scl_pin.pin() as usize, Cnf::AF_OPEN_DRAIN_OUT);
            });

        // Enable clock and reset module
        let rcc = &ch32_hal::pac::RCC;
        rcc.apb1pcenr().modify(|r| {
            r.set_i2c1en(true);
        });
        rcc.apb1prstr().write(|r| r.set_i2c1rst(true));
        rcc.apb1prstr().write(|r| r.set_i2c1rst(false));

        let ret = Self {
            phantom: PhantomData,
            config,
        };

        ret.reset();

        /*T::EventInterrupt::unpend();
        T::ErrorInterrupt::unpend();
        unsafe {
            T::EventInterrupt::enable();
            T::ErrorInterrupt::enable();
        }*/

        ret
    }

    pub fn reset(&self) {
        let i2c = T::regs();

        // Disable I2C
        i2c.ctlr1().modify(|w| w.set_pe(false));

        // soft reset
        i2c.ctlr1().modify(|w| w.set_swrst(true));
        i2c.ctlr1().modify(|w| w.set_swrst(false));

        // Set address (currently 7 bit only), currently no general call
        i2c.oaddr1().modify(|r| {
            r.set_add7_1(self.config.addr);
            r.set_addmode(self.config.general_call);
        });

        // Set clock freq
        let freq_set = (self.config.freq / 1000000) as u8;
        assert!(freq_set >= 8);
        assert!(freq_set <= 48);
        i2c.ctlr2().modify(|r| {
            r.set_freq((self.config.freq / 1000000) as u8);
        });

        i2c.ctlr1().modify(|r| {
            r.set_pe(true);
        });

        // Enable ACK
        i2c.ctlr1().modify(|r| {
            r.set_ack(true);
        });
    }

    // #[inline(always)]
    // pub fn get_state(&self) ->

    #[inline(always)] // pretty sure this should always be inlined
    #[allow(unused)]
    fn enable_interrupts() -> () {
        T::regs().ctlr2().modify(|w| {
            w.set_iterren(true);
            w.set_itevten(true);
        });
    }

    pub fn listen(&self) -> impl FnMut() -> Poll<Result<Command, Error>> + '_ {
        let i2c = T::regs();

        let mut rx_len = 0;
        let mut state = TransactionState::Idle;
        let mut buf = [0; 8];
        let ret = move || {
            loop {
                let star1 = i2c.star1().read();
                match state {
                    TransactionState::Idle => {
                        if star1.berr() || star1.af() {
                            i2c.star1().modify(|r| {
                                r.set_berr(false);
                                r.set_af(false);
                            });
                            return Poll::Ready(Err(Error::Abort));
                        }

                        if star1.addr() {
                            if star1.tx_e() && !star1.rx_ne() {
                                // We are transmitting, ADDR ACK needs to be done in response
                                return Poll::Ready(Ok(Command::Read));
                            } else {
                                // We are receiving, acknowledge ADDR match and change state
                                // Acknowledge ADDR match
                                i2c.star1().read();
                                i2c.star2().read();
                                state = TransactionState::Receiving;
                                continue;
                            }
                        }

                        if star1.stopf() {
                            // Clear STOPF bit
                            let val = i2c.ctlr1().read();
                            i2c.star1().read();
                            i2c.ctlr1().write_value(val);
                            continue;
                        }

                        break Poll::Pending;
                    }
                    TransactionState::Receiving => {
                        if star1.rx_ne() {
                            // Another byte available
                            let new_byte = i2c.datar().read().datar();
                            if rx_len < buf.len() {
                                buf[rx_len] = new_byte;
                            }
                            rx_len += 1;

                            continue;
                        }

                        if star1.stopf() {
                            // Clear stop
                            i2c.ctlr1().write_value(i2c.ctlr1().read());
                            if rx_len > buf.len() {
                                return Poll::Ready(Err(Error::PartialWrite(buf.len(), rx_len)));
                            } else {
                                return Poll::Ready(Ok(Command::Write(rx_len, buf)));
                            }
                        }

                        if star1.tx_e() && star1.addr() {
                            if rx_len > buf.len() {
                                return Poll::Ready(Err(Error::PartialWrite(buf.len(), rx_len)));
                            } else {
                                return Poll::Ready(Ok(Command::Write(buf.len(), buf)));
                            }
                        }

                        if star1.addr() {
                            // This should not happen, possibly restart
                            //writeln!(uart, "BUG: addr seen while rx\r");
                            return Poll::Ready(Ok(Command::Write(rx_len, buf)));
                        }

                        break Poll::Pending;
                    }
                }
            }
        };
        ret
    }

    pub fn respond_to_read(
        &self,
        in_buf: &[u8],
        fill: Option<u8>,
    ) -> impl FnMut() -> Poll<Result<ReadStatus, Error>> + '_ {
        if in_buf.len() > 8 {
            panic!();
        }

        let i2c = T::regs();

        let mut tx_pos: usize = 0;
        let mut buf = [0; 16];
        buf[..in_buf.len()].copy_from_slice(in_buf);
        let len = in_buf.len();

        let ret = move || {
            loop {
                let star1 = i2c.star1().read();
                if star1.tx_e() {
                    // Remote device expects a data byte
                    if tx_pos < len || fill.is_some() {
                        let byte = if tx_pos >= len {
                            fill.unwrap()
                        } else {
                            buf[tx_pos]
                        };
                        // We still have unsent data
                        i2c.datar().write(|r| r.set_datar(byte));
                        tx_pos += 1;

                        if star1.addr() {
                            // This might be the first byte, clear ADDR to start transmitting
                            i2c.star1().read();
                            i2c.star2().read();
                        }
                        continue;
                    } else {
                        // Remove device wants more data, but whole buffer is sent
                        return Poll::Ready(Ok(ReadStatus::NeedMoreBytes));
                    }
                } else {
                    if star1.stopf() || star1.af() {
                        if star1.stopf() {
                            i2c.ctlr1().write_value(i2c.ctlr1().read());
                        }
                        if star1.af() {
                            i2c.star1().modify(|r| r.set_af(false));
                        }
                        // Remote device stopped reception
                        if tx_pos == buf.len() {
                            return Poll::Ready(Ok(ReadStatus::Done));
                        } else {
                            return Poll::Ready(Ok(ReadStatus::LeftoverBytes(
                                (buf.len() - tx_pos) as u16,
                            )));
                        }
                    }
                }
                break Poll::Pending;
            }
        };
        ret
    }
}

trait SealedInstance: ch32_hal::RccPeripheral + ch32_hal::RemapPeripheral {
    fn regs() -> ch32_hal::pac::i2c::I2c;
}

impl SealedInstance for peripherals::I2C1 {
    fn regs() -> ch32_hal::pac::i2c::I2c {
        ch32_hal::pac::I2C1
    }
}

impl Instance for peripherals::I2C1 {
    type EventInterrupt = ch32_hal::interrupt::typelevel::I2C1_EV;
    type ErrorInterrupt = ch32_hal::interrupt::typelevel::I2C1_ER;
}

/// I2C peripheral instance
#[allow(private_bounds)]
pub trait Instance: SealedInstance + 'static {
    /// Event interrupt for this instance
    type EventInterrupt: interrupt::typelevel::Interrupt;
    /// Error interrupt for this instance
    type ErrorInterrupt: interrupt::typelevel::Interrupt;
}

pub trait SdaPin<T: Instance, const REMAP: u8 = 0>: ch32_hal::gpio::Pin {}
pub trait SclPin<T: Instance, const REMAP: u8 = 0>: ch32_hal::gpio::Pin {}
impl SdaPin<ch32_hal::peripherals::I2C1, 0> for ch32_hal::peripherals::PC1 {}
impl SclPin<ch32_hal::peripherals::I2C1, 0> for ch32_hal::peripherals::PC2 {}
impl SdaPin<ch32_hal::peripherals::I2C1, 1> for ch32_hal::peripherals::PD0 {}
impl SclPin<ch32_hal::peripherals::I2C1, 1> for ch32_hal::peripherals::PD1 {}
impl SdaPin<ch32_hal::peripherals::I2C1, 2> for ch32_hal::peripherals::PC6 {}
impl SclPin<ch32_hal::peripherals::I2C1, 2> for ch32_hal::peripherals::PC5 {}
