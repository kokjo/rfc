use core::{f32, sync::atomic::Ordering};

use embassy_futures::select::{Either, select};
use embassy_sync::pubsub::WaitResult;
use embassy_time::{Duration, Ticker};

use crate::{
    ACCEL, SystemEvents,
    control::Gimbals,
    dshot::Motors,
    gyro::{AtomicGyro, Gyro},
    util::{Average, RateLimter, constrain, deadband, scale},
};

struct Pid {
    kp: f32,
    ki: f32,
    kd: f32,
    last_error: f32,
    integral: f32,
}

impl Pid {
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            last_error: 0.0,
            integral: 0.0,
        }
    }

    pub fn update(&mut self, sp: f32, pv: f32, dt: f32) -> f32 {
        let error = sp - pv;
        self.integral += error * dt;
        let derivative = (error - self.last_error) / dt;
        self.last_error = error;
        self.kp * error + self.ki * self.integral + self.kd * derivative
    }

    pub fn reset(&mut self) {
        self.last_error = 0.0;
        self.integral = 0.0;
    }
}

struct PidParams {
    kp: f32,
    ki: f32,
    kd: f32,
}

impl PidParams {
    pub const fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self { kp, ki, kd }
    }
}

static PID_RP: PidParams = PidParams::new(2.5, 1.0, 0.005);
static PID_Y: PidParams = PidParams::new(2.5, 1.0, 0.005);

#[embassy_executor::task]
pub async fn pids_task(
    mut gimbals: crate::util::watch::AnonReceiver<Gimbals, 4>,
    mut events: crate::SystemEventsSubscriber,
    mut gyro: &'static AtomicGyro,
    motors: &'static Motors,
) {
    let timing = Duration::from_hz(8000);
    let mut ticker = Ticker::every(timing);
    let mut tick_rl = RateLimter::new(Duration::from_millis(250));

    let mut pit_pid = Pid::new(PID_RP.kp, PID_RP.ki, PID_RP.kd);
    let mut rol_pid = Pid::new(PID_RP.kp, PID_RP.ki, PID_RP.kd);
    let mut yaw_pid = Pid::new(PID_Y.kp, PID_Y.ki, PID_Y.kd);

    let mut gyro_avg: Average<[f32; 3]> = Average::new();
    let mut accel_avg: Average<[f32; 3]> = Average::new();
    let mut motor_avg: Average<[f32; 4]> = Average::new();

    let mut armed = false;
    let mut gyro_calibrated = false;
    let mut accel_calibrated = false;

    loop {
        match select(ticker.next(), events.next_message()).await {
            Either::First(()) => {
                if !armed || !gyro_calibrated || !accel_calibrated {
                    if tick_rl.check() {
                        log::info!(
                            "armed: {armed:?}, gyro: {gyro_calibrated:?}, accel: {accel_calibrated:?}"
                        );
                    }
                    motors.armed.store(false, Ordering::Relaxed);
                    continue;
                }
            }
            Either::Second(WaitResult::Lagged(skipped)) => {
                log::warn!("Pid loop missed {skipped} system events, lag!?");
                continue;
            }
            Either::Second(WaitResult::Message(msg)) => {
                match msg {
                    SystemEvents::Armed => {
                        log::info!("Pid loop arming, calibrating gyro/accel, reseting pids");
                        crate::GYRO_CAL.store(true, Ordering::Relaxed);
                        motors.armed.store(false, Ordering::Relaxed);
                        pit_pid.reset();
                        rol_pid.reset();
                        yaw_pid.reset();

                        armed = true;
                        gyro_calibrated = false;
                    }
                    SystemEvents::GyroCalibrated => {
                        gyro_calibrated = true;
                    }
                    SystemEvents::AccelCalibrated => {
                        accel_calibrated = true;
                    }
                    SystemEvents::Disarmed => {
                        armed = false;
                    }
                    _ => (),
                }
                continue;
            }
        }

        let gimbals = gimbals.try_get().unwrap_or_default();
        let (rc_thr_raw, rc_pit_raw, rc_rol_raw, rc_yaw_raw) =
            (gimbals.thr, gimbals.pit, gimbals.rol, gimbals.yaw);

        let rc_thr = scale(rc_thr_raw as f32, 174.0, 1811.0, 0.0, 1500.0);
        let rc_pit = scale(rc_pit_raw as f32, 174.0, 1811.0, -180.0, 180.0);
        let rc_rol = scale(rc_rol_raw as f32, 174.0, 1811.0, -180.0, 180.0);
        let rc_yaw = -scale(rc_yaw_raw as f32, 174.0, 1811.0, -270.0, 270.0);

        let rc_thr = if rc_thr < 10.0 { 0.0 } else { rc_thr };
        let rc_pit = deadband(rc_pit, 0.0, 1.0);
        let rc_rol = deadband(rc_rol, 0.0, 1.0);
        let rc_yaw = deadband(rc_yaw, 0.0, 1.0);

        let gyro_data = gyro.gyro_read().await.into_ok();
        gyro_avg.update(gyro_data);

        let accel_data = ACCEL.read();
        accel_avg.update([accel_data.0, accel_data.1, accel_data.2]);

        let [gy_pit, gy_rol, gy_yaw] = gyro_data;

        let dt = (timing.as_ticks() as f32) / (embassy_time::TICK_HZ as f32);

        let pit_cv = pit_pid.update(rc_pit, gy_pit, dt);
        let rol_cv = rol_pid.update(rc_rol, gy_rol, dt);
        let yaw_cv = yaw_pid.update(rc_yaw, gy_yaw, dt);

        let mixer: [[f32; 4]; 4] = [
            [1.0, 1.0, 1.0, 1.0],
            [1.0, -1.0, 1.0, -1.0],
            [1.0, 1.0, -1.0, -1.0],
            [1.0, -1.0, -1.0, 1.0],
        ];

        let speeds =
            mixer.map(|mix| mix[0] * rc_thr + mix[1] * pit_cv + mix[2] * rol_cv + mix[3] * yaw_cv);
        motor_avg.update(speeds);

        motors.speeds.iter().enumerate().for_each(|(i, m)| {
            m.store(constrain(speeds[i], 50.0, 1900.0) as u16, Ordering::Relaxed)
        });
        motors.armed.store(armed, Ordering::Relaxed);

        if tick_rl.check() {
            let armed = if armed { "A" } else { "D" };

            let [m0, m1, m2, m3] = motor_avg.average();
            motor_avg = Average::new();

            let [gy_avg_pit, gy_avg_rol, gy_avg_yaw] = gyro_avg.average();
            gyro_avg = Average::new();

            let [acc_avg_x, acc_avg_y, acc_avg_z] = accel_avg.average();
            accel_avg = Average::new();

            let (roll, pitch) = tilt(acc_avg_x, acc_avg_y, acc_avg_z);
            let roll = scale(roll, -f32::consts::PI, f32::consts::PI, -180.0, 180.0);
            let pitch = scale(pitch, -f32::consts::PI, f32::consts::PI, -180.0, 180.0);

            #[rustfmt::skip]
            log::info!(
                "{} RC {:4.1}\t{:4.1}\t{:4.1}\t{:4.1}\tGYRO {:4.1}\t{:4.1}\t{:4.1}\tACCEL {:4.1}\t{:4.1}\t{:4.1}\tANGLE {:3.3}\t{:3.3}\tMOTORS {:4.1}\t{:4.1}\t{:4.1}\t{:4.1} DT {:1.8}",
                armed,
                rc_thr, rc_pit, rc_rol, rc_yaw,
                gy_avg_pit, gy_avg_rol, gy_avg_yaw,
                acc_avg_x, acc_avg_y, acc_avg_z,
                pitch, roll,
                m0, m1, m2, m3,
                dt
            );
        }
    }
}

fn fast_atan2(y: f32, x: f32) -> f32 {
    const PI: f32 = core::f32::consts::PI;
    const PI_2: f32 = core::f32::consts::FRAC_PI_2;

    if x == 0.0 {
        return if y > 0.0 { PI_2 } else { -PI_2 };
    }

    let z = y / x;
    let atan = z / (1.0 + 0.28 * z * z);

    if x > 0.0 {
        atan
    } else if y >= 0.0 {
        atan + PI
    } else {
        atan - PI
    }
}

fn tilt(ax: f32, ay: f32, az: f32) -> (f32, f32) {
    let roll = fast_atan2(ay, az);
    let pitch = fast_atan2(-ax, core::intrinsics::sqrtf32(ay * ay + az * az));
    (roll, pitch)
}
