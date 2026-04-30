#![no_std]
#![no_main]
use core::cell::RefCell;

use embassy_boot_stm32::{AlignedBuffer, BlockingFirmwareState, BlockingFirmwareUpdater, FirmwareState, FirmwareUpdaterConfig, State};
use embassy_embedded_hal::flash::ConcatFlash;
use embassy_stm32::{bind_interrupts, flash::{BANK1_REGION, Flash}};
use embassy_sync::blocking_mutex::Mutex;
use embassy_usb::Builder;
use embassy_usb_dfu::{ResetImmediate, consts::DfuAttributes, new_state, usb_dfu};

use defmt as _;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USB_LP => embassy_stm32::usb::InterruptHandler<embassy_stm32::peripherals::USB>;
});

#[cortex_m_rt::entry]
unsafe fn main() -> ! {
    let p = embassy_stm32::init(Default::default());

    for _ in 0..10000000 {
        cortex_m::asm::nop();
    }

    defmt::info!("Booting!");

    let layout = Flash::new_blocking(p.FLASH).into_blocking_regions();
    let whole_flash = ConcatFlash::new(layout.bank1_region, layout.bank2_region);
    let flash = Mutex::new(RefCell::new(whole_flash));
    let fw_config = FirmwareUpdaterConfig::from_linkerfile_blocking(&flash, &flash);
    let fw_offset = fw_config.dfu.offset();

    let mut state_buf = AlignedBuffer([0u8; 8]);
    let mut firmware_state = BlockingFirmwareState::from_config(fw_config, state_buf.as_mut());

    let state = firmware_state.get_state().unwrap_or(State::DfuDetach);
    defmt::info!("Firmware state: {:?}", state);

    if state != State::Boot {
    // if true {
        let driver = embassy_stm32::usb::Driver::new(p.USB, Irqs, p.PA12, p.PA11);
        defmt::info!("Starting DFU USB device");

        let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
        config.manufacturer = Some("RFC");
        config.product = Some("Bootloader");
        config.serial_number = Some(embassy_stm32::uid::uid_hex());
        
        let mut write_buffer = AlignedBuffer([0u8; 8]);
        let fw_config = FirmwareUpdaterConfig::from_linkerfile_blocking(&flash, &flash);
        let updater = BlockingFirmwareUpdater::new(fw_config, write_buffer.as_mut());

        let mut config_descriptor = [0u8; 256];
        let mut bos_descriptor = [0u8; 256];
        let mut msos_descriptor = [0u8; 256];
        let mut control_buf = [0u8; 4096];

        let mut builder = Builder::new(driver, config, &mut config_descriptor, &mut bos_descriptor, &mut msos_descriptor, &mut control_buf);
        let mut state = new_state(updater, DfuAttributes::CAN_DOWNLOAD, ResetImmediate);
        usb_dfu::<_, _, _, _, 4096>(&mut builder, &mut state, |_| {});

        let mut device = builder.build();
        defmt::info!("DFU USB running!");
        embassy_futures::block_on(device.run());
    }

    let firmware_address = BANK1_REGION.base() + fw_offset;
    defmt::info!("Jumping to firmware: {:x}", firmware_address);
    unsafe { cortex_m::asm::bootload((firmware_address) as *const u32) };
    
    // let config = BootLoaderConfig::from_linkerfile_blocking(&flash, &flash, &flash);
    // let active_offset = config.active.offset();
    // let bl = BootLoader::prepare::<_, _, _, 2048>(config);

    // unsafe {
    //     bl.load(BANK1_REGION.base() + active_offset)
    // }
}