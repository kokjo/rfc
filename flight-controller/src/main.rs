#![no_std]
#![no_main]
#![feature(array_try_from_fn)]
#![allow(internal_features)]
#![feature(core_intrinsics)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![feature(unwrap_infallible)]
#![feature(never_type)]

use core::sync::atomic::AtomicBool;

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, OutputType, Pull, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::spi::mode::Master;
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::usart::Uart;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::{bind_interrupts, usart};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::pubsub::PubSubBehavior;
use embassy_time::{Duration, Ticker, Timer};
use static_cell::StaticCell;
use crate::crsf::RcCtrl;
use crate::dshot::Motors;
use crate::gyro::{Accel, AtomicGyro};

use {defmt_rtt as _, panic_probe as _};

pub mod btn;
pub mod crsf;
pub mod dshot;
pub mod gyro;
pub mod led;
pub mod osd;
pub mod pids;
pub mod pwm;
pub mod util;
pub mod usb;

bind_interrupts!(struct Irqs {
    USB_LP => embassy_stm32::usb::InterruptHandler<embassy_stm32::peripherals::USB>;
    USART3 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART3>;
    DMA1_CHANNEL1 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA1_CH1>;
    DMA1_CHANNEL2 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA1_CH2>;
    DMA1_CHANNEL3 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA1_CH3>;
    DMA1_CHANNEL4 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA1_CH4>;
    EXTI9_5 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI9_5>;
    DMA2_CHANNEL1 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA2_CH1>;
    EXTI15_10 => embassy_stm32::exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI15_10>;
    FLASH => embassy_stm32::flash::InterruptHandler;
});

#[derive(Debug, Clone)]
pub enum SystemEvents {
    Booted,
    Armed,
    Disarmed,
    GyroCalibrated,
    AccelCalibrated,
}

static EVENTS: util::pubsub::PubSub<SystemEvents, 10, 10, 10> = util::pubsub::PubSub::new();

static CONTROLS: RcCtrl = RcCtrl::new();
static GYRO: AtomicGyro = AtomicGyro::new();
static GYRO_CAL: AtomicBool = AtomicBool::new(false);
pub static ACCEL: Accel = Accel::new();
static MOTORS: Motors = Motors::new();

pub type Stm32SpiBus = Mutex<NoopRawMutex, Spi<'static, Async, Master>>;
pub type Stm32SpiDevice =
    SpiDevice<'static, NoopRawMutex, Spi<'static, Async, Master>, Output<'static>>;

static SPI1_BUS: StaticCell<Stm32SpiBus> = StaticCell::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = {
        use embassy_stm32::rcc::*;
        // Main system clock at 170 MHz
        let mut config = embassy_stm32::Config::default();
        config.rcc.hsi = true;
        config.rcc.pll = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL85,
            divp: None,
            divq: None,
            divr: Some(PllRDiv::DIV2),
        });
        config.rcc.sys = Sysclk::PLL1_R;
        embassy_stm32::init(config)
    };

    usb::start_usb_device(&spawner, p.USB, p.FLASH, p.PA12, p.PA11);

    spawner.spawn(btn::boot_btn_task(ExtiInput::new(p.PB8, p.EXTI8, Pull::None, Irqs)).unwrap());
    spawner
        .spawn(led::led_task(Output::new(p.PC4, Level::High, Speed::High), &CONTROLS.arm).unwrap());

    let pwm = {
        let led0 = PwmPin::new(p.PB6, OutputType::PushPull);
        SimplePwm::new(
            p.TIM4,
            Some(led0),
            None,
            None,
            None,
            Hertz::khz(100),
            Default::default(),
        )
    };
    let channels = pwm.split();
    spawner.spawn(pwm::pwm_task(channels.ch1, (200..1800).into(), &CONTROLS.thr).unwrap());

    let usart3 = {
        let mut config = usart::Config::default();
        config.baudrate = 420000;
        config.stop_bits = usart::StopBits::STOP1;
        config.data_bits = usart::DataBits::DataBits8;
        config.detect_previous_overrun = false;
        Uart::new(
            p.USART3, p.PB11, p.PB10, p.DMA1_CH1, p.DMA1_CH2, Irqs, config,
        )
        .unwrap()
    };
    spawner.spawn(crsf::crsf_rx_task(usart3, &CONTROLS).unwrap());

    let spi1_bus = SPI1_BUS.init({
        let mut config = spi::Config::default();
        config.frequency = Hertz::mhz(20);
        let spi = Spi::new(
            p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA1_CH3, p.DMA1_CH4, Irqs, config,
        );
        Mutex::new(spi)
    });

    let gyro_device = SpiDevice::new(spi1_bus, Output::new(p.PC14, Level::High, Speed::High));
    let gyro_intr = ExtiInput::new(p.PC15, p.EXTI15, Pull::None, Irqs);

    spawner.spawn(gyro::gyro_task(gyro_device, gyro_intr, &GYRO_CAL, &GYRO, &ACCEL).unwrap());

    let motor_pwm = {
        let ch1 = PwmPin::new(p.PC6, OutputType::PushPull);
        let ch2 = PwmPin::new(p.PA4, OutputType::PushPull);
        let ch3 = PwmPin::new(p.PB0, OutputType::PushPull);
        let ch4 = PwmPin::new(p.PB1, OutputType::PushPull);

        SimplePwm::new(
            p.TIM3,
            Some(ch1),
            Some(ch2),
            Some(ch3),
            Some(ch4),
            Hertz::khz(600),
            CountingMode::EdgeAlignedUp,
        )
    };
    spawner.spawn(dshot::dshot_task(motor_pwm, p.DMA2_CH1, &MOTORS).unwrap());
    spawner.spawn(pids::pids_task(&CONTROLS, &GYRO, &MOTORS).unwrap());
    // spawner.spawn(btn::rc_btn_task(&CONTROLS).unwrap());

    log::info!("System booted!");

    EVENTS.publish_immediate(SystemEvents::Booted);

    Timer::after_secs(3).await;
    GYRO_CAL.store(true, core::sync::atomic::Ordering::Relaxed);
    // check_enter_bootloader();

    let mut wdog = IndependentWatchdog::new(p.IWDG, 1000 * 1000);
    let mut wdog_ticker = Ticker::every(Duration::from_millis(100));
    wdog.unleash();
    loop {
        wdog.pet();
        wdog_ticker.next().await;
    }
}



// #define USE_ACC
// #define USE_GYRO
// #define USE_ACC_SPI_ICM42688P
// #define USE_GYRO_SPI_ICM42688P
// #define USE_ACCGYRO_LSM6DSK320X
// #define USE_FLASH
// #define USE_FLASH_M25P16
// #define USE_MAX7456

// #define BEEPER_PIN PA8
// #define MOTOR1_PIN PB0
// #define MOTOR2_PIN PB1
// #define MOTOR3_PIN PC6
// #define MOTOR4_PIN PA4
// #define LED_STRIP_PIN PB2
// #define UART1_TX_PIN PA9
// #define UART2_TX_PIN PA2
// #define UART3_TX_PIN PB10
// #define UART4_TX_PIN PC10
// #define UART1_RX_PIN PA10
// #define UART2_RX_PIN PA3
// #define UART3_RX_PIN PB11
// #define UART4_RX_PIN PC11
// #define I2C1_SCL_PIN PA15
// #define I2C1_SDA_PIN PB7
// #define LED0_PIN PB6
// #define LED1_PIN PC4
// #define SPI1_SCK_PIN PA5
// #define SPI2_SCK_PIN PB13
// #define SPI3_SCK_PIN PB3
// #define SPI1_SDI_PIN PA6
// #define SPI2_SDI_PIN PB14
// #define SPI3_SDI_PIN PB4
// #define SPI1_SDO_PIN PA7
// #define SPI2_SDO_PIN PB15
// #define SPI3_SDO_PIN PB5
// #define ADC_VBAT_PIN PA0
// #define ADC_CURR_PIN PA1
// #define FLASH_CS_PIN PC13
// #define MAX7456_SPI_CS_PIN PB12
// #define GYRO_1_EXTI_PIN PC15
// #define GYRO_1_CS_PIN PC14

// #define TIMER_PIN_MAPPING \
//     TIMER_PIN_MAP( 0, PB2, 1, 1 ) \
//     TIMER_PIN_MAP( 1, PB0, 1, 2 ) \
//     TIMER_PIN_MAP( 2, PB1, 1, 3 ) \
//     TIMER_PIN_MAP( 3, PC6, 1, 4 ) \
//     TIMER_PIN_MAP( 4, PA4, 1, 5 )

// #define ADC1_DMA_OPT 6

// #define VTX_SMARTAUDIO_UART SERIAL_PORT_USART2
// #define SERIALRX_UART SERIAL_PORT_USART3

// #define BARO_I2C_INSTANCE	I2CDEV_1
// #define MAG_I2C_INSTANCE	I2CDEV_1
// #define SERIALRX_PROVIDER	SERIALRX_CRSF
// #define DEFAULT_BLACKBOX_DEVICE BLACKBOX_DEVICE_FLASH
// #define DEFAULT_DSHOT_BURST DSHOT_DMAR_OFF
// #define DEFAULT_DSHOT_BITBANG DSHOT_BITBANG_ON
// #define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
// #define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
// #define DEFAULT_CURRENT_METER_SCALE 750
// #define BEEPER_INVERTED
// #define SYSTEM_HSE_MHZ 8
// #define MAX7456_SPI_INSTANCE SPI2
// #define FLASH_SPI_INSTANCE SPI3
// #define GYRO_1_SPI_INSTANCE SPI1
// #define GYRO_1_ALIGN CW180_DEG
