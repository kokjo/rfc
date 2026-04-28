use embassy_stm32::{exti::ExtiInput, mode::Async};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Ticker, Timer};

use crate::crsf::RcCtrl;

#[embassy_executor::task]
pub async fn boot_btn_task(mut btn: ExtiInput<'static, Async>) {
    loop {
        btn.wait_for_high().await;
        log::info!("BOOT button pressed, resetting!");
        Timer::after_millis(10).await;
        cortex_m::peripheral::SCB::sys_reset();
    }
}

// #[embassy_executor::task]
// pub async fn rc_btn_task(ctrl: &'static RcCtrl) {
//     let mut ticker = Ticker::every(Duration::from_millis(10));
//     loop {
//         if ctrl.btn.load(core::sync::atomic::Ordering::Relaxed) {
//             log::info!("RC button pushed entering DFU bootloader");
//             Timer::after_millis(100).await;
//             reset_to_bootloader();
//         }
//         ticker.next().await;
//     }
// }

// #[unsafe(link_section = ".noinit")]
// static mut BOOTLOADER_FLAG: u32 = 0;

// pub fn reset_to_bootloader() {
//     unsafe { BOOTLOADER_FLAG = 0xdeadbeed; }
//     cortex_m::peripheral::SCB::sys_reset();
// }

// pub fn check_enter_bootloader() {
//     if unsafe { BOOTLOADER_FLAG } == 0xdeadbeef {
//         unsafe { BOOTLOADER_FLAG = 0; }
//         log::info!("Should have jumepd tp bootloader");
//         // jump_to_dfu_bootloader();
//     }
// }

// pub fn jump_to_dfu_bootloader() {
//     unsafe {
//         cortex_m::interrupt::disable();

//         let p = cortex_m::Peripherals::steal();

//         p.SYST.csr.write(0);

//         for i in 0..16 {
//             // Interrupt Clear-Enabled
//             p.NVIC.icer[i].write(0xffffffff);
//             // Interrupt Clear-Pending
//             p.NVIC.icpr[i].write(0xffffffff);
//         }

//         cortex_m::interrupt::enable();
//         cortex_m::asm::bootload(0x1FFF0000 as *const u32);
//     }
// }
