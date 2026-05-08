use embassy_stm32::gpio::Output;

#[embassy_executor::task(pool_size = 2)]
pub async fn led_task(mut pin: Output<'static>) {
    let mut events = crate::EVENTS.subscriber().unwrap();
    loop {
        match events.next_message_pure().await {
            crate::SystemEvents::Armed => pin.set_low(),
            crate::SystemEvents::Disarmed => pin.set_high(),
            _ => (),
        }
    }
}
