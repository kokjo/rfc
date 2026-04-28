use core::sync::atomic::{AtomicBool, Ordering};

use embassy_stm32::gpio::Output;
use embassy_time::{Duration, Ticker};


#[embassy_executor::task(pool_size=2)]
pub async fn led_task(mut pin: Output<'static>, ctrl: &'static AtomicBool) {
    let mut ticker = Ticker::every(Duration::from_millis(10));
    loop {
        ticker.next().await;
        pin.set_level((!ctrl.load(Ordering::Relaxed)).into());
    }
}