use embassy_stm32::pac::common::Read;
use embedded_hal::spi::Operation;
use embedded_hal_async::spi::SpiDevice;

trait Register: Sized {
    type Error;
    const REGNUM: u8;
    const SIZE: usize;

    fn encode(&self) -> [u8; Self::SIZE];
    fn decode(v: [u8; Self::SIZE]) -> Result<Self, Self::Error>;
}

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum VideoStandard {
    NTSC = 0,
    PAL = 1,
}

impl TryFrom<u8> for VideoStandard {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value & 1 {
            0 => Ok(Self::NTSC),
            1 => Ok(Self::PAL),
            _ => Err(())
        }
    }
}

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum SyncSelectMode {
    AutoSync = 0b00,
    External = 0b10,
    Internal = 0b11,
}

impl TryFrom<u8> for SyncSelectMode {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value & 3 {
            0b00 | 0b01 => Ok(Self::AutoSync),
            0b10 => Ok(Self::External),
            0b11 => Ok(Self::Internal),
            _ => Err(())
        }
    }
}

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum OnOff {
    Off = 0,
    On = 1,
}

impl TryFrom<u8> for OnOff {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Off),
            1 => Ok(Self::On),
            _ => Err(())
        }
    }
}

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum EnableVSync {
    Now = 0,
    NextVSync = 1,
}

impl TryFrom<u8> for EnableVSync {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value & 1 {
            0 => Ok(Self::Now),
            1 => Ok(Self::NextVSync),
            _ => Err(())
        }
    }
}

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum SoftwareReset {
    Running = 0,
    Reseting = 1,
}

impl TryFrom<u8> for SoftwareReset {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value & 1 {
            0 => Ok(Self::Running),
            1 => Ok(Self::Reseting),
            _ => Err(())
        }
    }
}

#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum EnableDisable {
    Enable = 0,
    Disable = 1,
}

impl TryFrom<u8> for EnableDisable {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value & 1 {
            0 => Ok(Self::Enable),
            1 => Ok(Self::Disable),
            _ => Err(())
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct VideoMode0 {
    pub video_standard: VideoStandard,
    pub sync_select_mode: SyncSelectMode,
    pub enable_osd_display: OnOff,
    pub enable_vsync: EnableVSync,
    pub software_reset: SoftwareReset,
    pub video_buffer_enable: EnableDisable
}

impl Register for VideoMode0 {
    type Error = ();
    const REGNUM: u8 = 0;
    const SIZE: usize = 1;

    fn encode(&self) -> [u8; Self::SIZE] {
        [
            ((self.video_standard as u8) << 6) |
            ((self.sync_select_mode as u8) << 4) |
            ((self.enable_osd_display as u8) << 3) |
            ((self.enable_vsync as u8) << 2) |
            ((self.software_reset as u8) << 1) |
            ((self.video_buffer_enable as u8) << 0)
        ]
    }

    fn decode(v: [u8; Self::SIZE]) -> Result<Self, Self::Error> {
        Ok(Self {
            video_standard: (v[0] >> 6).try_into()?,
            sync_select_mode: (v[0] >> 4).try_into()?,
            enable_osd_display: (v[0] >> 3).try_into()?,
            enable_vsync: (v[0] >> 2).try_into()?,
            software_reset: (v[0] >> 1).try_into()?,
            video_buffer_enable: (v[0] >> 0).try_into()?,
        })
    }
}

enum ReadRegisterError<Spi: SpiDevice, Reg: Register> {
    Spi(Spi::Error),
    Decode(Reg::Error)
}

struct MAX7456<SpiDev> {
    spidev: SpiDev,
}

impl<SpiDev: SpiDevice> MAX7456<SpiDev> {
    pub fn new(spidev: SpiDev) -> Self {
        Self { spidev }
    }

    pub async fn read_register<Reg: Register>(&mut self) -> Result<Reg, ReadRegisterError<SpiDev, Reg>>
            where [(); Reg::SIZE]: {
        let mut bytes = [0u8; Reg::SIZE];
        self.spidev.transaction(&mut [
            Operation::Write(&[Reg::REGNUM | 0x80]),
            Operation::Read(&mut bytes),
        ]).await.map_err(ReadRegisterError::Spi)?;
        Ok(Reg::decode(bytes).map_err(ReadRegisterError::Decode)?)
    }

    pub async fn write_register<Reg: Register>(&mut self, reg: Reg) -> Result<(), SpiDev::Error>
            where [(); Reg::SIZE]: {
                let bytes = reg.encode();
                self.spidev.transaction(&mut [
                    Operation::Write(&[Reg::REGNUM | 0x00]),
                    Operation::Write(&bytes),
                ]).await?;
                Ok(())

        }
}
