use core::cell::RefCell;

use embassy_boot::{AlignedBuffer, BlockingFirmwareState, FirmwareUpdaterConfig};
use embassy_embedded_hal::flash::{ConcatFlash, partition::BlockingPartition};
use embassy_executor::Spawner;
use embassy_stm32::{Peri, flash::{Bank1Region, Bank2Region, Blocking, Flash, WRITE_SIZE}, peripherals::{FLASH, USB}, usb::{DmPin, DpPin}};
use embassy_sync::blocking_mutex::{Mutex, raw::NoopRawMutex};
use embassy_time::Duration;
use embassy_usb_dfu::{application::{DfuState, Handler}, consts::DfuAttributes};
use static_cell::StaticCell;

type UsbDriver = embassy_stm32::usb::Driver<'static, USB>;
type UsbDevice = embassy_usb::UsbDevice<'static, UsbDriver>;

static CONFIG_DESCRIPTOR_BUF: StaticCell<[u8; 512]> = StaticCell::new();
static BOS_DESCRIPTOR_BUF: StaticCell<[u8; 256]> = StaticCell::new();
static MSOS_DESCRIPTOR_BUF: StaticCell<[u8; 256]> = StaticCell::new();
static CONTROL_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
static ACM_STATE: StaticCell<embassy_usb::class::cdc_acm::State> = StaticCell::new();

type WholeFlash = ConcatFlash<Bank1Region<'static, Blocking>, Bank2Region<'static, Blocking>>;
type OurFlash = Mutex<NoopRawMutex, RefCell<WholeFlash>>;
type Partition = BlockingPartition<'static, NoopRawMutex, WholeFlash>;
static FLASH_CELL: StaticCell<OurFlash> = StaticCell::new();

struct DfuHandler<'d, FLASH: embedded_storage::nor_flash::NorFlash> {
    firmware_state: BlockingFirmwareState<'d, FLASH>
}

impl<'d, FLASH: embedded_storage::nor_flash::NorFlash> Handler for DfuHandler<'d, FLASH> {
    fn enter_dfu(&mut self) {
        self.firmware_state.mark_dfu().expect("Failed to make boot state to enter DFU");
        cortex_m::peripheral::SCB::sys_reset();
    }
}

static FIRMWARE_STATE_BUFFER: StaticCell<AlignedBuffer<8>> = StaticCell::new();
static DFU_STATE: StaticCell<DfuState<DfuHandler<Partition>>> = StaticCell::new();

pub fn start_usb_device(
    spawner: &Spawner,
    usb: Peri<'static, USB>,
    flash: Peri<'static, FLASH>,
    dp: Peri<'static, impl DpPin<USB>>,
    dm: Peri<'static, impl DmPin<USB>>,
) {
    let driver = embassy_stm32::usb::Driver::new(usb, crate::Irqs, dp, dm);
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("YOLO");
    config.product = Some("Flight Controller");
    config.serial_number = Some(embassy_stm32::uid::uid_hex());
    config.max_power = 100;
    config.max_packet_size_0 = 64;
    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        CONFIG_DESCRIPTOR_BUF.init([0u8; _]),
        BOS_DESCRIPTOR_BUF.init([0u8; _]),
        MSOS_DESCRIPTOR_BUF.init([0u8; _]),
        CONTROL_BUF.init([0u8; _]),
    );
    let acm_state = ACM_STATE.init(embassy_usb::class::cdc_acm::State::new());
    let acm_class = embassy_usb::class::cdc_acm::CdcAcmClass::new(&mut builder, acm_state, 64);

    // let layout = Flash::new_blocking(flash).into_blocking_regions();
    // let whole_flash: WholeFlash = ConcatFlash::new(layout.bank1_region, layout.bank2_region);
    // let flash: &mut OurFlash = FLASH_CELL.init(Mutex::new(RefCell::new(whole_flash)));

    // let updater_config = FirmwareUpdaterConfig::from_linkerfile_blocking(flash, flash);
    // let firmware_state_buffer = FIRMWARE_STATE_BUFFER.init(AlignedBuffer([0u8; WRITE_SIZE]));
    // let mut firmware_state = BlockingFirmwareState::from_config(updater_config, firmware_state_buffer.as_mut());
    // firmware_state.mark_booted().expect("Failed to mark boot state as booted");

    // let dfu_handler = DfuHandler { firmware_state };
    // let dfu_state = DFU_STATE.init(DfuState::new(dfu_handler, DfuAttributes::CAN_DOWNLOAD, Duration::from_secs(2)));
    // embassy_usb_dfu::application::usb_dfu(&mut builder, dfu_state, |_| {});

    let device = builder.build();

    spawner.spawn(usb_logger_task(acm_class).unwrap());
    spawner.spawn(usb_device_task(device).unwrap());
}

#[embassy_executor::task]
async fn usb_logger_task(class: embassy_usb::class::cdc_acm::CdcAcmClass<'static, UsbDriver>) {
    embassy_usb_logger::with_class!(1024, log::LevelFilter::Info, class).await
}

#[embassy_executor::task]
async fn usb_device_task(mut device: UsbDevice) {
    device.run().await
}