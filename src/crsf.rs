use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, Ordering};

use embassy_stm32::{mode::Async, usart::Uart};
use embassy_time::Duration;

use crate::util::RateLimter;

#[derive(Debug)]
pub enum CRSFMessage {
    RcChannels([u16; 16]),
    Unknown(u8),
}

#[derive(Debug)]
pub struct RcCtrl {
    pub thr: AtomicU16,
    pub pit: AtomicU16,
    pub rol: AtomicU16,
    pub yaw: AtomicU16,
    pub arm: AtomicBool,
    pub btn: AtomicBool,
    pub frames: AtomicU32,
}

impl RcCtrl {
    pub const fn new() -> Self {
        Self {
            thr: AtomicU16::new(0),
            pit: AtomicU16::new(0),
            rol: AtomicU16::new(0),
            yaw: AtomicU16::new(0),
            arm: AtomicBool::new(false),
            btn: AtomicBool::new(false),
            frames: AtomicU32::new(0),
        }
    }

    pub fn read_rc(&self) -> (u16, u16, u16, u16) {
        (
            self.thr.load(Ordering::Relaxed),
            self.pit.load(Ordering::Relaxed),
            self.rol.load(Ordering::Relaxed),
            self.yaw.load(Ordering::Relaxed),
        )
    }

    pub fn armed(&self) -> bool {
        self.arm.load(Ordering::Relaxed)
    }

    pub fn frames(&self) -> u32 {
        self.frames.load(Ordering::Relaxed)
    }
}

#[derive(Debug, Clone)]
struct CSRFFramer {
    buf: [u8; 64],
    idx: usize,
}

impl Default for CSRFFramer {
    fn default() -> Self {
        Self {
            buf: [0u8; 64],
            idx: Default::default(),
        }
    }
}

impl CSRFFramer {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, byte: u8) -> Option<CRSFMessage> {
        if self.buf.as_slice().len() <= self.idx {
            self.idx = 0;
        }

        if self.idx == 0 && byte != 0xc8 {
            return None;
        }

        self.buf[self.idx] = byte;
        self.idx += 1;

        if self.idx < 3 {
            return None;
        }

        if self.idx < (self.buf[1] as usize) + 2 {
            return None;
        }

        self.parse_frame()
    }

    pub fn parse_frame(&mut self) -> Option<CRSFMessage> {
        let frame = &self.buf[..self.idx];
        self.idx = 0;

        // log::info!("{frame:02x?}");

        match frame.get(2)?.clone() {
            0x16 => {
                let channels = core::array::try_from_fn(|i| {
                    let bit = 11 * i;
                    let byte = bit >> 3;
                    let shift = bit & 7;
                    let b0 = frame.get(3 + byte + 0)?.clone() as u32;
                    let b1 = frame.get(3 + byte + 1)?.clone() as u32;
                    let b2 = frame.get(3 + byte + 2)?.clone() as u32;
                    let word = (b0 << 0) | (b1 << 8) | (b2 << 16);
                    Some(((word >> shift) & 0x7ff) as u16)
                })?;

                Some(CRSFMessage::RcChannels(channels))
            }
            typ => Some(CRSFMessage::Unknown(typ)),
        }
    }
}

#[embassy_executor::task]
pub async fn crsf_rx_task(mut uart: Uart<'static, Async>, ctrl: &'static RcCtrl) {
    let mut framer = CSRFFramer::new();

    let mut err_rl = RateLimter::new(Duration::from_millis(100));

    loop {
        let mut buffer = [0u8; 64];

        match uart.read_until_idle(&mut buffer).await {
            Ok(n) => {
                for &byte in &buffer[..n] {
                    // if dbg_rl.check() {
                    //     let buf = &framer.buf[..framer.idx];
                    //     log::info!("{bytecnt:08x}: {byte:02x} {buf:02x?}");
                    // }

                    if let Some(message) = framer.push(byte) {
                        handle_message(&message, ctrl);
                    }
                }
            }
            Err(err) => {
                if err_rl.check() {
                    log::warn!("CSRF: {err:?}");
                }
            }
        }
    }
}

pub fn handle_message(message: &CRSFMessage, ctrl: &RcCtrl) {
    match message {
        CRSFMessage::RcChannels(chans) => {
            ctrl.thr.store(chans[2], Ordering::Relaxed);
            ctrl.pit.store(chans[1], Ordering::Relaxed);
            ctrl.rol.store(chans[0], Ordering::Relaxed);
            ctrl.yaw.store(chans[3], Ordering::Relaxed);
            ctrl.arm.store(chans[4] > 1024, Ordering::Relaxed);
            ctrl.btn.store(chans[8] > 1024, Ordering::Relaxed);
            ctrl.frames.fetch_add(1, Ordering::Relaxed);
        }
        _message => {
            // if last_msg.elapsed().as_millis() > 100 {
            //     log::info!("CSRF: {message:x?}");
            //     last_msg = Instant::now();
            // }
        }
    }
}

pub fn crc8_dvb_s2(data: &[u8]) -> u8 {
    let mut crc = 0u8;

    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            crc = if (crc & 0x80) != 0 {
                (crc << 1) ^ 0xD5
            } else {
                crc << 1
            };
        }
    }

    crc
}
