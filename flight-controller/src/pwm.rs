use core::range::Range;

use embassy_stm32::{
    peripherals::TIM4,
    timer::{low_level::OutputPolarity, simple_pwm::SimplePwmChannel},
};

use crate::control::Gimbals;

#[embassy_executor::task]
pub async fn pwm_task(
    mut channel: SimplePwmChannel<'static, TIM4>,
    range: Range<u16>,
    mut gimbals: crate::util::watch::Recviver<Gimbals, 4>,
) {
    channel.set_polarity(OutputPolarity::ActiveLow);
    channel.enable();
    let denom = range.end - range.start;
    loop {
        let gimbals = gimbals.changed().await;
        let value = gimbals.thr.max(range.start).min(range.end) - range.start;
        channel.set_duty_cycle_fraction(value as u32, denom as u32);
    }
}
