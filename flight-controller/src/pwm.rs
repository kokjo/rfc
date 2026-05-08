use embassy_stm32::{
    peripherals::TIM4,
    timer::{low_level::OutputPolarity, simple_pwm::SimplePwmChannel},
};

#[embassy_executor::task]
pub async fn pwm_task(mut channel: SimplePwmChannel<'static, TIM4>) {
    let mut gimbals = crate::CONTROL.gimbals.receiver().unwrap();
    channel.set_polarity(OutputPolarity::ActiveLow);
    channel.enable();
    let denom = 2000.0f32;
    loop {
        let gimbals = gimbals.changed().await;
        let value = gimbals.thr;
        channel.set_duty_cycle_fraction(value as u32, denom as u32);
    }
}
