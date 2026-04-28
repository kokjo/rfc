use core::sync::atomic::{AtomicBool, AtomicU16, Ordering};

use dshot_frame::{Command, Frame, NormalDshot};
use embassy_stm32::{Peri, peripherals::{DMA2_CH1, TIM3}, timer::{Channel, simple_pwm::SimplePwm}};
use embassy_time::{Duration, Ticker, Timer};
use heapless::Vec;

use crate::Irqs;

pub struct Motors {
    pub speeds: [AtomicU16; 4],
    pub armed: AtomicBool,
}

impl Motors {
    pub const fn new() -> Self {
        Self {
            speeds: [AtomicU16::new(0), AtomicU16::new(0), AtomicU16::new(0), AtomicU16::new(0)],
            armed: AtomicBool::new(false),
        }
    }

    pub fn armed(&self) -> bool {
        self.armed.load(Ordering::Relaxed)
    }

    pub fn speeds(&self) -> [u16; 4] {
        core::array::from_fn(|i| self.speeds[i].load(Ordering::Relaxed))
    }
}

const CHANNEL_TO_MOTOR: [usize; 4] = [2, 3, 0, 1];

#[embassy_executor::task]
pub async fn dshot_task(mut pwm: SimplePwm<'static, TIM3>, mut dma: Peri<'static, DMA2_CH1>, motors: &'static Motors) {
    let mut was_armed = false;
    let mut ticker = Ticker::every(Duration::from_hz(8000));
    loop {
        let frames = {
            let armed = motors.armed();
            let frames: Vec<(usize, u64, [Frame::<NormalDshot>; 4]), 4> = match (was_armed, armed) {
                (false, true) => {
                    // Arming
                    Vec::from_slice(&[
                        (10, 0, [Frame::<NormalDshot>::command(Command::SpinDirectionNormal, false); 4]),
                        (500, 0, [Frame::<NormalDshot>::new(0, false).unwrap(); 4]),
                    ]).unwrap()
                },
                (true, true) => {
                    // Armed
                    let speeds = motors.speeds();
                    let speeds = CHANNEL_TO_MOTOR.map(|i| speeds[CHANNEL_TO_MOTOR[i]]);
                    Vec::from_slice(&[
                        (1, 0, speeds.map(|speed| Frame::<NormalDshot>::new(speed.min(1999), false).unwrap()))
                    ]).unwrap()
                },
                (true, false) => {
                    // Disarming
                    Vec::from_slice(&[
                        (10, 0, [Frame::<NormalDshot>::command(Command::MotorStop, false); 4]),
                        (10, 0, [Frame::<NormalDshot>::new(0, false).unwrap(); 4]),
                    ]).unwrap()
                }
                (false, false) => {
                    // Disarmed
                    Vec::from_slice(&[
                        (1, 0, [Frame::<NormalDshot>::new(0, false).unwrap(); 4]),
                    ]).unwrap()
                }
            };
            was_armed = armed;
            frames
        };

        for &(repeats, sleep, frames) in frames.iter() {
            let duty_cycles = frames.map(|frame| frame.duty_cycles(pwm.max_duty_cycle() as u16));
            // 68 == 4*16 + 4
            let duty_cycles: [u16; 68] = core::array::from_fn(|i| duty_cycles[i % 4][i / 4]);
            for _ in 0..repeats {
                pwm.waveform_up_multi_channel(dma.reborrow(), Irqs, Channel::Ch1, Channel::Ch4, &duty_cycles).await;
                
                Timer::after_millis(sleep).await;
                ticker.next().await;
            }
        }
    }
}