use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, Ordering};

use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Timer, with_timeout};

use crate::{
    SystemEvents,
    util::{Edge, EdgeDetect, watch},
};

#[derive(Debug)]
pub struct RcCtrl {
    pub thr: AtomicU16,
    pub pit: AtomicU16,
    pub rol: AtomicU16,
    pub yaw: AtomicU16,
    pub arm: AtomicBool,
    pub btn: AtomicBool,
    pub frames: AtomicU32,
}

#[derive(Debug, Clone, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Gimbals {
    pub thr: u16,
    pub pit: u16,
    pub rol: u16,
    pub yaw: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Button {
    Pressed,
    Unpressed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Switch2 {
    Off,
    On,
}

impl From<u16> for Switch2 {
    fn from(value: u16) -> Self {
        if value < 993 { Self::Off } else { Self::On }
    }
}

impl From<Switch2> for bool {
    fn from(value: Switch2) -> Self {
        match value {
            Switch2::Off => false,
            Switch2::On => true,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Switch3 {
    Low,
    Mid,
    High,
}

impl From<u16> for Switch3 {
    fn from(value: u16) -> Self {
        if value < 600 {
            Self::Low
        } else if value < 1200 {
            Self::Mid
        } else {
            Self::High
        }
    }
}

pub struct Control {
    pub gimbals: watch::Watch<Gimbals, 4>,
    pub left_button: watch::Watch<Switch2, 2>,
    pub right_button: watch::Watch<Switch2, 2>,
    pub left_switch: watch::Watch<Switch3, 2>,
    pub right_switch: watch::Watch<Switch3, 2>,
    pub frames: AtomicU32,
}

impl Control {
    pub const fn new() -> Self {
        Self {
            gimbals: watch::Watch::new(),
            left_button: watch::Watch::new(),
            right_button: watch::Watch::new(),
            left_switch: watch::Watch::new(),
            right_switch: watch::Watch::new(),
            frames: AtomicU32::new(0),
        }
    }
}

#[embassy_executor::task]
pub async fn control_task() {
    let events = crate::EVENTS
        .publisher()
        .expect("Control loop: Faield to get event publisher");
    let mut channels = crate::CHANNELS
        .receiver()
        .expect("Control loop: Failed to get channels receiver");
    let gimbal_sender = crate::CONTROL.gimbals.sender();
    let left_btn_sender = crate::CONTROL.left_button.sender();
    let left_sw_sender = crate::CONTROL.left_switch.sender();
    let right_sw_sender = crate::CONTROL.left_switch.sender();
    let right_btn_sender = crate::CONTROL.right_button.sender();
    let mut arm_edge_detect = EdgeDetect::new();
    loop {
        if let Ok(chans) = with_timeout(Duration::from_secs(1), channels.changed()).await {
            let gimbals = Gimbals {
                thr: chans[2],
                pit: chans[1],
                rol: chans[0],
                yaw: chans[3],
            };
            gimbal_sender.send(gimbals);

            let left_btn = chans[4].into();
            left_btn_sender.send_if_modified(|value| update_if_different(value, left_btn));
            match arm_edge_detect.update(left_btn) {
                Some(Edge::Rising) => events.publish(SystemEvents::Armed).await,
                Some(Edge::Falling) => events.publish(SystemEvents::Disarmed).await,
                _ => (),
            };
            left_sw_sender.send_if_modified(|value| update_if_different(value, chans[5].into()));
            right_sw_sender.send_if_modified(|value| update_if_different(value, chans[6].into()));
            right_btn_sender.send_if_modified(|value| update_if_different(value, chans[7].into()));
        } else {
            gimbal_sender.clear();
            events.publish(SystemEvents::Disarmed).await;
        }
    }
}

pub fn update_if_different<T: PartialEq>(value: &mut Option<T>, update: T) -> bool {
    let update = Some(update);
    if value == &update {
        return false;
    }
    *value = update;
    true
}
