use core::{range::Range, sync::atomic::{AtomicU16, Ordering}};

use embassy_stm32::{peripherals::TIM4, timer::{low_level::OutputPolarity, simple_pwm::SimplePwmChannel}};
use embassy_time::{Duration, Ticker};

#[embassy_executor::task]
pub async fn pwm_task(mut channel: SimplePwmChannel<'static, TIM4>, range: Range<u16>, ctrl: &'static AtomicU16) {
    channel.set_polarity(OutputPolarity::ActiveLow);
    channel.enable();
    let mut ticker = Ticker::every(Duration::from_millis(1));
    let denom = range.end - range.start;
    loop {
        let value = ctrl.load(Ordering::Relaxed);
        let value = value.max(range.start).min(range.end) - range.start;
        channel.set_duty_cycle_fraction(value as u32, denom as u32);
        ticker.next().await;
    }
}