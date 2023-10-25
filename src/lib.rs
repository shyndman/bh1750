#![no_std]

use embedded_hal_async::i2c::{AddressMode, I2c, SevenBitAddress};

pub const DEFAULT_ADDRESS_ADAFRUIT: SevenBitAddress = 0x08;

#[derive(Debug, Copy, Clone)]
#[allow(non_camel_case_types)]
enum Instruction {
    POWER_DOWN = 0b0000_0000,
    POWER_ON = 0b0000_0001,
    RESET = 0b0000_0111,
}

#[allow(non_camel_case_types)]
pub enum OneTimeMeasurement {
    HIHGT_RES = 0b0010_0000,  // 1    lx resolution
    HIHGT_RES2 = 0b0010_0001, // 0.5  lx resolution
    LOW_RES = 0b0010_0011,    // 4    lx resolution
}

#[allow(non_camel_case_types)]
pub enum ContinuesMeasurement {
    HIHGT_RES = 0b0001_0000,  // 1    lx resolution
    HIHGT_RES2 = 0b0001_0001, // 0.5  lx resolution
    LOW_RES = 0b0001_0011,    // 4    lx resolution
}
pub struct BH1750<I2C, A, DELAY> {
    com: I2C,
    delay: DELAY,
    address: A,
}

impl<I2C, DELAY, E> BH1750<I2C, SevenBitAddress, DELAY>
where
    I2C: I2c<SevenBitAddress, Error = E>,
    DELAY: embedded_hal_async::delay::DelayUs,
{
    /// Create new BH1750 driver
    pub fn new(i2c: I2C, delay: DELAY) -> Result<Self, E> {
        Self::new_with_address(i2c, delay, DEFAULT_ADDRESS_ADAFRUIT)
    }
}

impl<I2C, A, DELAY, E> BH1750<I2C, A, DELAY>
where
    A: AddressMode + Copy,
    I2C: I2c<A, Error = E>,
    DELAY: embedded_hal_async::delay::DelayUs,
{
    /// Create new BH1750 driver
    pub fn new_with_address(i2c: I2C, delay: DELAY, address: A) -> Result<Self, E> {
        // AddressMode;
        let chip = BH1750 {
            com: i2c,
            delay,
            address,
        };
        Ok(chip)
    }

    pub async fn light_one_shot(&mut self, mode: OneTimeMeasurement) -> u32 {
        let delay = match mode {
            OneTimeMeasurement::HIHGT_RES => 140,
            OneTimeMeasurement::HIHGT_RES2 => 160,
            OneTimeMeasurement::LOW_RES => 18,
        };
        let command = mode as u8;
        self.send_instruction(command).await;
        self.delay.delay_ms(delay).await;
        raw_to_lx(self.resive_answer(command).await)
    }

    pub async fn start_measurement(&mut self, mode: ContinuesMeasurement) {
        let command = mode as u8;
        self.send_instruction(command).await;
    }

    pub async fn reset(&mut self) {
        self.send_instruction(Instruction::RESET as u8).await;
    }

    pub async fn power_down(&mut self) {
        self.send_instruction(Instruction::POWER_DOWN as u8).await;
    }

    pub async fn power_on(&mut self) {
        self.send_instruction(Instruction::POWER_ON as u8).await;
    }

    pub async fn get_measurement(&mut self, mode: ContinuesMeasurement) -> u32 {
        let delay = match mode {
            ContinuesMeasurement::HIHGT_RES => 120,
            ContinuesMeasurement::HIHGT_RES2 => 120,
            ContinuesMeasurement::LOW_RES => 16,
        };
        let command = mode as u8;
        self.delay.delay_ms(delay).await;
        raw_to_lx(self.resive_answer(command).await)
    }

    async fn send_instruction(&mut self, instr: u8) {
        let mut buffer = [0];
        let _ = self.com.write_read(self.address, &[instr], &mut buffer);
    }

    async fn resive_answer(&mut self, instr: u8) -> u16 {
        let mut data: [u8; 2] = [0; 2];
        let _ = self.com.write_read(self.address, &[instr], &mut data);
        let raw_answer: u16 = ((data[0] as u16) << 8) | data[1] as u16;
        raw_answer
    }
}

fn raw_to_lx(raw: u16) -> u32 {
    (raw as u32) * 12 / 10
}
