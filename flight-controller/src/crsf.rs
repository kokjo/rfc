use embassy_stm32::{mode::Async, usart::Uart};
use embassy_time::Duration;

use crate::util::RateLimter;

#[derive(Debug)]
pub enum CRSFMessage {
    RcChannels([u16; 16]),
    Unknown(u8),
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

        #[allow(clippy::identity_op)]
        match *frame.get(2)? {
            0x16 => {
                let channels = core::array::try_from_fn(|i| {
                    let bit = 11 * i;
                    let byte = bit >> 3;
                    let shift = bit & 7;
                    let b0 = *frame.get(3 + byte + 0)? as u32;
                    let b1 = *frame.get(3 + byte + 1)? as u32;
                    let b2 = *frame.get(3 + byte + 2)? as u32;
                    let word = b0 | (b1 << 8) | (b2 << 16);
                    Some(((word >> shift) & 0x7ff) as u16)
                })?;

                Some(CRSFMessage::RcChannels(channels))
            }
            typ => Some(CRSFMessage::Unknown(typ)),
        }
    }
}

#[embassy_executor::task]
pub async fn crsf_rx_task(mut uart: Uart<'static, Async>, channels: crate::ChannelsSender) {
    let mut framer = CSRFFramer::new();

    let mut err_rl = RateLimter::new(Duration::from_millis(100));
    let mut channels_rl = RateLimter::new(Duration::from_secs(1));

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
                        match message {
                            CRSFMessage::RcChannels(chans) => {
                                if channels_rl.check() {
                                    log::info!("Channels: {chans:?}");
                                }
                                channels.send(chans);
                            }
                            _ => (),
                        }
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
