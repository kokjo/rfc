use core::sync::atomic::{AtomicBool, Ordering};

use embassy_stm32::gpio::{Level, Output};
use embassy_time::{Duration, Ticker};

use crate::{SystemEventsSubscriber, control::Switch2};

#[embassy_executor::task(pool_size = 2)]
pub async fn led_task(mut pin: Output<'static>, mut events: SystemEventsSubscriber) {
    loop {
        match events.next_message_pure().await {
            crate::SystemEvents::Armed => pin.set_low(),
            crate::SystemEvents::Disarmed => pin.set_high(),
            _ => (),
        }
    }
}
