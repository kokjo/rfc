use core::{convert::Infallible, sync::atomic::{AtomicBool, Ordering}};

use embassy_stm32::{exti::ExtiInput, mode::Async};
use embassy_time::{Duration, Ticker, Timer};
use embedded_hal::spi::Operation;
use embedded_hal_async::spi::SpiDevice;

use crate::{Stm32SpiDevice, util::{self, AtomicF32, Average, RateLimter, rotate}};

#[derive(Debug)]
pub struct AtomicGyro {
    pit: AtomicF32,
    rol: AtomicF32,
    yaw: AtomicF32,
}

pub trait Gyro {
    type Error;
    async fn gyro_read(&mut self) -> Result<[f32; 3], Self::Error>;
}

impl AtomicGyro {
    pub const fn new() -> Self {
        Self {
            pit: AtomicF32::new(0.0),
            rol: AtomicF32::new(0.0),
            yaw: AtomicF32::new(0.0),
        }
    }

    pub fn read(&self) -> (f32, f32, f32) {
        (
            self.pit.load(Ordering::Relaxed),
            self.rol.load(Ordering::Relaxed),
            self.yaw.load(Ordering::Relaxed),
        )
    }
}

impl Gyro for &AtomicGyro {
    type Error = Infallible;

    async fn gyro_read(&mut self) -> Result<[f32; 3], Self::Error> {
        Ok([
            self.pit.load(Ordering::Relaxed),
            self.rol.load(Ordering::Relaxed),
            self.yaw.load(Ordering::Relaxed),
        ])
    }
}

pub struct WatchGyro<const N: usize>(util::watch::Watch<[f32;3], N>);

impl<const N: usize> WatchGyro<N> {
    pub fn receiver(&'static self) -> Option<WatchGyroReceiver<N>> {
        self.0.receiver().map(WatchGyroReceiver)
    }
}

pub struct WatchGyroReceiver<const N: usize>(util::watch::Recviver<[f32; 3], N>);

impl<const N: usize> Gyro for WatchGyroReceiver<N> {
    type Error = Infallible;

    async fn gyro_read(&mut self) -> Result<[f32; 3], Self::Error> {
        Ok(self.0.changed().await)
    }
}

pub struct TransformGyro<G> {
    inner: G,
    transform: [[f32; 4]; 3],
}

impl<G> TransformGyro<G> {
    pub const fn new(inner: G, transform: [[f32; 4]; 3]) -> Self {
        Self {
            inner,
            transform,
        }
    }

    pub fn set_bias(&mut self, bias: [f32; 3]) {
        self.transform[0][3] = bias[0];
        self.transform[1][3] = bias[1];
        self.transform[2][3] = bias[2];
    }
}

impl<G: Gyro> Gyro for TransformGyro<G> {
    type Error = G::Error;

    async fn gyro_read(&mut self) -> Result<[f32; 3], Self::Error> {
        let data = self.inner.gyro_read().await?;
        Ok(core::array::from_fn(|i| {
            let row = self.transform[i];
            data[0]*row[0] + data[1]*row[1] + data[2]*row[2] + row[3]
        }))
    }
}

pub struct Accel {
    x: AtomicF32,
    y: AtomicF32,
    z: AtomicF32,
}

impl Accel {
    pub const fn new() -> Self {
        Self {
            x: AtomicF32::new(0.0),
            y: AtomicF32::new(0.0),
            z: AtomicF32::new(0.0),
        }
    }

    pub fn read(&self) -> (f32, f32, f32) {
        (
            self.x.load(Ordering::Relaxed),
            self.y.load(Ordering::Relaxed),
            self.z.load(Ordering::Relaxed),
        )
    }
}

#[embassy_executor::task]
pub async fn gyro_task(spidev: Stm32SpiDevice, _intr: ExtiInput<'static, Async>, cal_gyro_request: &'static AtomicBool, gyro: &'static AtomicGyro, accel: &'static Accel) {
    let mut icm = TransformGyro::new(
        ICM42688P::new(spidev).await.unwrap(),
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
        ]
    );

    let mut cal_average: Average<[f32; 3]> = Average::new();
    let mut cal_running = false;
    
    let mut gyro_err_rl = RateLimter::new(Duration::from_millis(100));
    let mut gyro_info_rl = RateLimter::new(Duration::from_millis(100));
    let mut accel_err_rl = RateLimter::new(Duration::from_millis(100));

    let mut ticker = Ticker::every(Duration::from_hz(8000));
    loop {
        if cal_gyro_request.load(Ordering::Relaxed) && !cal_running {
            log::info!("Gyro: Calibrating...");
            cal_average = Average::new();
            icm.set_bias([0.0, 0.0, 0.0]);
            cal_running = true;
        }

        match icm.gyro_read().await {
            Ok(dps) => {
                if cal_running {
                    if cal_average.count() < 1024 {
                        let avg = cal_average.update(dps);
                        if gyro_info_rl.check() {
                            log::info!("GYRO: Calibration average: {:4.1} {:4.1} {:4.1}", avg[0], avg[1], avg[2]);
                        }
                    } else {
                        cal_running = false;
                        cal_gyro_request.store(false, Ordering::Relaxed);
                        let avg = cal_average.average();
                        icm.set_bias(avg.map(|v| -v));
                        log::info!("Gyro: Calibrated: {:4.1} {:4.1} {:4.1}", avg[0], avg[1], avg[2]);
                    }
                }

                let [pit, rol, yaw] = dps;
                let (pit, rol) = rotate(pit, rol, -45.0 * (core::f32::consts::PI / 180.0));
                let pit = -pit;
                gyro.pit.store(pit, Ordering::Relaxed);
                gyro.rol.store(rol, Ordering::Relaxed);
                gyro.yaw.store(yaw, Ordering::Relaxed);
            },
            Err(err) => if gyro_err_rl.check() {
                log::warn!("Gyro: Failed to read: {err:?}");
            }
        }

        match icm.inner.read_accel_raw().await {
            Ok(raw) => {
                let raw = raw.map(|v| (v as f32) / 2048.0);
                accel.x.store(raw[0], Ordering::Relaxed);
                accel.y.store(raw[1], Ordering::Relaxed);
                accel.z.store(raw[2], Ordering::Relaxed);
            },

            Err(err) => if  accel_err_rl.check() {
                log::warn!("Accel: Failed to read: {err:?}");
            }
        }

        ticker.next().await
    }
}

struct ICM42688P<SpiDev> {
    spidev: SpiDev,
}

impl<SpiDev: SpiDevice> ICM42688P<SpiDev> {
    pub async fn new(spidev: SpiDev) -> Result<Self, SpiDev::Error> {
        let mut this = Self{ spidev };
        this.write_reg(0x11, &[0x01]).await?;
        Timer::after_millis(10).await;
        this.write_reg(0x76, &[0x00]).await?;
        this.write_reg(0x4e, &[0x0f]).await?;
        Ok(this)
    }

    pub async fn read_reg(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), SpiDev::Error> {
        self.spidev.transaction(&mut [
            Operation::Write(&[address | 0x80]),
            Operation::Read(buffer)
        ]).await
    }

    pub async fn write_reg(&mut self, address: u8, buffer: &[u8]) -> Result<(), SpiDev::Error> {
        self.spidev.transaction(&mut [
            Operation::Write(&[address]),
            Operation::Write(buffer)
        ]).await
    }

    pub async fn read_gyro_bytes(&mut self) -> Result<[u8; 6], SpiDev::Error> {
        let mut bytes = [0u8; 6];
        self.read_reg(0x25, &mut bytes).await?;
        Ok(bytes)
    }

    pub async fn read_gyro_raw(&mut self) -> Result<[i16; 3], SpiDev::Error> {
        let bytes = self.read_gyro_bytes().await?;
        Ok([
            i16::from_be_bytes([bytes[0], bytes[1]]),
            i16::from_be_bytes([bytes[2], bytes[3]]),
            i16::from_be_bytes([bytes[4], bytes[5]]),
        ])
    }

    pub async fn read_accel_bytes(&mut self) -> Result<[u8; 6], SpiDev::Error> {
        let mut bytes = [0u8; 6];
        self.read_reg(0x1f, &mut bytes).await?;
        Ok(bytes)
    }

    pub async fn read_accel_raw(&mut self) -> Result<[i16; 3], SpiDev::Error> {
        let bytes = self.read_accel_bytes().await?;
        Ok([
            i16::from_be_bytes([bytes[0], bytes[1]]),
            i16::from_be_bytes([bytes[2], bytes[3]]),
            i16::from_be_bytes([bytes[4], bytes[5]]),
        ])
    }
}

impl<SpiDev: SpiDevice> Gyro for ICM42688P<SpiDev> {
    type Error = SpiDev::Error;

    async fn gyro_read(&mut self) -> Result<[f32; 3], Self::Error> {
        Ok(self.read_gyro_raw().await?.map(|v| (v as f32)/16.4))
    }
}