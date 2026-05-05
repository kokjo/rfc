use core::{convert::Infallible, fmt::{Write, write}, intrinsics::unreachable};

use embassy_time::Timer;
use embedded_hal::spi::Operation;
use embedded_hal_async::spi::SpiDevice;

use crate::{Stm32SpiDevice, control::Switch2, util::scale};

pub trait Register: Sized {
    type Error;
    const REGNUM: u8;
    const SIZE: usize;

    fn encode(&self) -> [u8; Self::SIZE];
    fn decode(v: [u8; Self::SIZE]) -> Result<Self, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
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
            _ => Err(()),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
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
            _ => Err(()),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
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
            _ => Err(()),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
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
            _ => Err(()),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
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
            _ => Err(()),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
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
            _ => Err(()),
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
    pub video_buffer_enable: EnableDisable,
}

impl Register for VideoMode0 {
    type Error = ();
    const REGNUM: u8 = 0;
    const SIZE: usize = 1;

    fn encode(&self) -> [u8; Self::SIZE] {
        [((self.video_standard as u8) << 6)
            | ((self.sync_select_mode as u8) << 4)
            | ((self.enable_osd_display as u8) << 3)
            | ((self.enable_vsync as u8) << 2)
            | ((self.software_reset as u8) << 1)
            | (self.video_buffer_enable as u8)]
    }

    fn decode(v: [u8; Self::SIZE]) -> Result<Self, Self::Error> {
        Ok(Self {
            video_standard: (v[0] >> 6).try_into()?,
            sync_select_mode: (v[0] >> 4).try_into()?,
            enable_osd_display: (v[0] >> 3).try_into()?,
            enable_vsync: (v[0] >> 2).try_into()?,
            software_reset: (v[0] >> 1).try_into()?,
            video_buffer_enable: v[0].try_into()?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
enum ResetMode {
    Running = 0,
    PowerOnReset = 1,
}

impl TryFrom<u8> for ResetMode {
    type Error = !;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value & 1 {
            1 => Ok(ResetMode::PowerOnReset),
            0 => Ok(ResetMode::Running),
            _ => unreachable!()
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct Status {
    reset_mode: ResetMode,
    character_memory_unavailable: bool,
    vsync: bool,
    hsync: bool,
    loss_of_sync: bool,
    ntsc_detect: bool,
    pal_detect: bool,
}

impl Register for Status {
    type Error = ();

    const REGNUM: u8 = 0xa0;

    const SIZE: usize = 1;

    fn encode(&self) -> [u8; Self::SIZE] {
        [
            ((self.reset_mode as u8) << 6) |
            ((self.character_memory_unavailable as u8) << 5) |
            ((self.vsync as u8) << 4) | 
            ((self.hsync as u8) << 3) | 
            ((self.loss_of_sync as u8) << 2) | 
            ((self.ntsc_detect as u8) << 1) | 
            (self.pal_detect as u8)
        ]
    }

    fn decode(v: [u8; Self::SIZE]) -> Result<Self, Self::Error> {
        Ok(Self {
            reset_mode: ((v[0] >> 6) & 1).try_into().into_ok(),
            character_memory_unavailable: ((v[0] >> 5) & 1) == 1,
            vsync: ((v[0] >> 4) & 1) == 1,
            hsync: ((v[0] >> 3) & 1) == 1,
            loss_of_sync: ((v[0] >> 2) & 1) == 1, 
            ntsc_detect: ((v[0] >> 1) & 1) == 1,
            pal_detect: (v[0] & 1) == 1,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]    
struct DMAL(u8);

impl Register for DMAL {
    type Error = !;

    const REGNUM: u8 = 0x06;

    const SIZE: usize = 0x01;

    fn encode(&self) -> [u8; Self::SIZE] {
        [self.0]
    }

    fn decode(v: [u8; Self::SIZE]) -> Result<Self, Self::Error> {
        Ok(Self(v[0]))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]    
#[repr(u8)]
enum OperationMode {
    Mode16Bit = 0,
    Mode8Bit = 1
}

impl TryFrom<u8> for OperationMode {
    type Error = !;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value & 1 {
            0 => Ok(OperationMode::Mode16Bit),
            1 => Ok(OperationMode::Mode8Bit),
            _ => unreachable!()
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]    
struct DisplayMemoryMode {
    operation_mode: OperationMode
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]    
#[repr(u8)]
pub enum ByteSelect {
    Address = 0,
    Attribute = 1,
}

impl TryFrom<u8> for ByteSelect {
    type Error = !;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value & 1 {
            0 => Ok(ByteSelect::Address),
            1 => Ok(ByteSelect::Attribute),
            _ => unreachable!()
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]    
struct DMAH {
    byte_select: ByteSelect,
    msb: bool,
}

impl Register for DMAH {
    type Error = !;

    const REGNUM: u8 = 0x05;

    const SIZE: usize = 1;

    fn encode(&self) -> [u8; Self::SIZE] {
        [
            ((self.byte_select as u8) << 1) | 
            (self.msb as u8)
        ]
    }

    fn decode(v: [u8; Self::SIZE]) -> Result<Self, Self::Error> {
        Ok(Self {
            byte_select: ((v[0] >> 1) & 1).try_into().into_ok(),
            msb: (v[0] & 1) == 1,
        })
    }
}


#[derive(Debug, Clone)]
pub enum ReadRegisterError<Spi: SpiDevice, Reg: Register> {
    Spi(Spi::Error),
    Decode(Reg::Error),
}

pub struct MAX7456<SpiDev> {
    spidev: SpiDev,
}

impl<SpiDev: SpiDevice> MAX7456<SpiDev> {
    pub fn new(spidev: SpiDev) -> Self {
        Self { spidev }
    }

    pub async fn reset(&mut self) -> Result<(), ()> {
        // Wait for power on reset to finish
        loop {
            let status = self.read_register::<Status>().await.map_err(|_| ())?;
            if status.reset_mode == ResetMode::Running {
                break
            }
        }

        // Ensure that we are not already resetting
        let mut vm0 = loop {
            let vm0 = self.read_register::<VideoMode0>().await.map_err(|_| ())?;
            if vm0.software_reset == SoftwareReset::Running {
                break vm0;
            }
        };

        // Trigger software reset
        vm0.software_reset = SoftwareReset::Reseting;
        self.write_register(vm0).await.map_err(|_| ())?;


        // Wait until reset is finished
        loop {
            let vm0 = self.read_register::<VideoMode0>().await.map_err(|_| ())?;
            if vm0.software_reset == SoftwareReset::Running {
                break vm0;
            }
        };

        Ok(())
    }

    pub async fn read_register<Reg: Register>(
        &mut self,
    ) -> Result<Reg, ReadRegisterError<SpiDev, Reg>>
    where
        [(); Reg::SIZE]:,
    {
        let mut bytes = [0u8; Reg::SIZE];
        self.spidev
            .transaction(&mut [
                Operation::Write(&[Reg::REGNUM | 0x80]),
                Operation::Read(&mut bytes),
            ])
            .await
            .map_err(ReadRegisterError::Spi)?;
        Reg::decode(bytes).map_err(ReadRegisterError::Decode)
    }

    pub async fn write_dma(&mut self, address: u16, byte_select: ByteSelect) -> Result<(), SpiDev::Error> {
        let dmah = DMAH {
            byte_select,
            msb: ((address >> 8) & 1) == 1,
        };
        let dmal = DMAL((address & 0xff) as u8);

        self.write_register(dmah).await?;
        self.write_register(dmal).await?;
        Ok(())
    }

    pub async fn write_dmdi(&mut self, data: &[u8]) -> Result<(), SpiDev::Error> {
        self.spidev.transaction(&mut [
            Operation::Write(&[0x07]),
            Operation::Write(data),
            // Operation::Write(&[0xff, 0xff]),
        ]).await
    }

    pub async fn write_register<Reg: Register>(&mut self, reg: Reg) -> Result<(), SpiDev::Error>
    where
        [(); Reg::SIZE]:,
    {
        let bytes = reg.encode();
        self.spidev
            .transaction(&mut [Operation::Write(&[Reg::REGNUM]), Operation::Write(&bytes)])
            .await?;
        Ok(())
    }

    pub async fn modify_register<Reg: Register, F: FnOnce(&mut Reg)>(&mut self, f: F) -> Result<(), ReadRegisterError<SpiDev, Reg>>
    where 
        [(); Reg::SIZE]:,
    {
        let mut reg = self.read_register().await?;
        f(&mut reg);
        self.write_register(reg).await.map_err(ReadRegisterError::Spi)?;
        Ok(())
    }
}

#[embassy_executor::task]
pub async fn osd_task(
    spidev: Stm32SpiDevice,
) {
    let mut max7456 = MAX7456::new(spidev);
    max7456.reset().await.expect("OSD: Could not reset MAX7465");
    max7456.modify_register::<VideoMode0, _>(|vm0| vm0.enable_osd_display = OnOff::On).await.map_err(|_| ()).expect("OSD: Failed to enable OSD in VM0");
    let mut display = UpdateBuffer::new();
    loop {
        let armed = match crate::CONTROL.left_button.try_get() {
            Some(Switch2::Off) => "0",
            Some(Switch2::On) => "1",
            None => "U",
        };

        let _ = display.write_fmt(1, 2, |w| write!(w, "ARMED: {armed}"));

        let (gyro_pit, gyro_rol , gyro_yaw) = crate::GYRO.read();
        let _ = display.write_fmt(2, 2, |w| write!(w, "GY P: {gyro_pit:4.1}"));
        let _ = display.write_fmt(3, 2, |w| write!(w, "GY R: {gyro_rol:4.1}"));
        let _ = display.write_fmt(4, 2, |w| write!(w, "GY Y: {gyro_yaw:4.1}"));

        if let Some((gimbals)) = crate::CONTROL.gimbals.try_get() {
            let thr = scale(gimbals.thr as f32, 174.0, 1811.0, 0.0, 100.0);
            let pit = scale(gimbals.pit as f32, 174.0, 1811.0, -180.0, 180.0);
            let rol = scale(gimbals.rol as f32, 174.0, 1811.0, -180.0, 180.0);
            let yaw = scale(gimbals.yaw as f32, 174.0, 1811.0, 270.0, 270.0);
            let _ = display.write_fmt(4, 2, |w| write!(w, "RC P: {pit:4.1}"));
            let _ = display.write_fmt(5, 2, |w| write!(w, "RC R: {rol:4.1}"));
            let _ = display.write_fmt(6, 2, |w| write!(w, "RC Y: {yaw:4.1}"));
            let _ = display.write_fmt(7, 2, |w| write!(w, "THR: {thr:4.1}"));
        }

        for (addr, byte) in display.updates() {
            let _ = max7456.write_dma(addr as u16, ByteSelect::Address).await;
            let _ = max7456.write_dmdi(&[byte]).await;
        }

        display.swap_and_clear();

        Timer::after_millis(100).await;
    }
}

struct SliceWriter<'a>{
    content: &'a mut [u8],
    offset: usize
}

impl<'a> SliceWriter<'a> {
    pub fn new(content: &'a mut [u8]) -> Self {
        Self {
            content,
            offset: 0
        }
    }
}

impl<'a> Write for SliceWriter<'a> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for b in s.bytes() {
            if let Some(ptr) = self.content.get_mut(self.offset) {
                *ptr = b;
            }
            self.offset += 1;
        }
        Ok(())
    }
}


#[derive(Debug, Clone, Copy)]
struct DisplayMemory([u8; 30 * 16]);

impl DisplayMemory {
    pub const fn new() -> Self {
        Self([0x20u8; 30 * 16])
    }

    pub fn write_fmt<F : FnOnce(&mut SliceWriter) -> core::fmt::Result>(&mut self, row: u16, col: u16, f: F) -> core::fmt::Result {
        let mut writer = SliceWriter::new(&mut self.0[(30*row + col) as usize..]);
        f(&mut writer)
    }
}

impl Default for DisplayMemory {
    fn default() -> Self {
        Self::new()
    }
}

struct UpdateBuffer([DisplayMemory; 2]);

impl UpdateBuffer {
    pub const fn new() -> Self {
        UpdateBuffer([DisplayMemory::new(); 2])
    }

    pub fn updates(&mut self) -> impl Iterator<Item = (usize, u8)> {
        self.0[0].0.iter().enumerate().zip(self.0[1].0.iter()).filter_map(|((i, a), b)| (a != b).then_some((i, *b)))
    }

    pub fn swap_and_clear(&mut self) {
        self.0[0] = self.0[1];
        self.0[1] = DisplayMemory::new();
    }

    pub fn write_fmt<F: FnOnce(&mut SliceWriter) -> core::fmt::Result>(&mut self, row: u16, col: u16, f: F) -> core::fmt::Result {
        self.0[1].write_fmt(row, col, f)
    }
}