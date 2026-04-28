use core::{ops::{Add, Div, Sub}, sync::atomic::{AtomicU32, Ordering}};

use embassy_time::{Duration, Instant};

pub struct RateLimter {
    last: Instant,
    rate: Duration,
}

impl RateLimter {
    pub fn new(rate: Duration) -> Self {
        Self{
            last: Instant::now(),
            rate: rate,
        }
    }

    pub fn check(&mut self) -> bool {
        if self.last.elapsed() > self.rate {
            self.last = Instant::now();
            true
        } else {
            false
        }
    }
}

#[derive(Debug)]
pub struct AtomicF32(AtomicU32);

impl AtomicF32 {
    pub const fn new(v: f32) -> Self {
        Self(AtomicU32::new(v.to_bits()))
    }

    pub fn load(&self, order: Ordering) -> f32 {
        f32::from_bits(self.0.load(order))
    }

    pub fn store(&self, v: f32, order: Ordering) {
        self.0.store(v.to_bits(), order);
    }
}

pub fn constrain(v: f32, min: f32, max: f32) -> f32 {
    v.min(max).max(min)
}

pub fn scale(v: f32, min: f32, max: f32, lo: f32, hi: f32) -> f32 {
    (constrain(v, min, max) - min) * ((hi - lo) / (max - min)) + lo
}

pub fn deadband(v: f32, mid: f32, band: f32) -> f32 {
    if (v - mid).abs() < band {
        mid
    } else {
        v
    }
}

pub fn rotate(x: f32, y: f32, angle: f32) -> (f32, f32) {
    use core::intrinsics::{cosf32, sinf32};
    (
        x * cosf32(angle) - y * sinf32(angle),
        x * sinf32(angle) + y * cosf32(angle)
    )
}

type RawMutex = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

pub mod watch {
    use super::RawMutex;
    pub type Watch<T, const N: usize> = embassy_sync::watch::Watch<RawMutex, T, N>;
    pub type Recviver<T, const N: usize> = embassy_sync::watch::Receiver<'static, RawMutex, T, N>;
    pub type AnonReceiver<T, const N: usize> = embassy_sync::watch::AnonReceiver<'static, RawMutex, T, N>;
    pub type DynReceiver<T> = embassy_sync::watch::DynReceiver<'static, T>;
    pub type DynAnonReceiver<T> = embassy_sync::watch::DynAnonReceiver<'static, T>;
    pub type Sender<T, const N: usize> = embassy_sync::watch::Sender<'static, RawMutex, T, N>;
    pub type DynSender<T, const N: usize> = embassy_sync::watch::DynSender<'static, T>;
}

pub mod pubsub {
    use super::RawMutex;
    pub type PubSub<T, const CAP: usize, const SUBS: usize, const PUBS: usize> = embassy_sync::pubsub::PubSubChannel<RawMutex, T, CAP, SUBS, PUBS>;
    pub type Publisher<T, const CAP: usize, const SUBS: usize, const PUBS: usize> = embassy_sync::pubsub::Publisher<'static, RawMutex, T, CAP, SUBS, PUBS>;
    pub type Subscriber<T, const CAP: usize, const SUBS: usize, const PUBS: usize> = embassy_sync::pubsub::Subscriber<'static, RawMutex, T, CAP, SUBS, PUBS>;
}

pub mod globalvalue {
    use embassy_sync::rwlock::RwLock;

    use super::RawMutex;

    pub struct GlobalValue<T: ?Sized>(RwLock<RawMutex, T>);

    impl<T: Clone> GlobalValue<T> {
        pub const fn new(v: T) -> Self {
            Self(RwLock::new(v))
        }

        pub async fn get(&self) -> T {
            self.0.read().await.clone()
        }

        pub async fn put(&self, v: T) {
            *self.0.write().await = v;
        }

        pub async fn swap(&self, v: T) -> T {
            let mut locked = self.0.write().await;
            let old = locked.clone();
            *locked = v;
            old
        }
    }
}

pub trait Zero {
    const ZERO: Self;
}

impl Zero for f32 {
    const ZERO: Self = 0.0;
}

impl<T: Zero, const N: usize> Zero for [T; N] {
    const ZERO: Self = [T::ZERO; N];
}

pub trait Summable {
    fn add(self, rhs: Self) -> Self;
    fn sub(self, rhs: Self) -> Self;
}

impl Summable for f32 {
    fn add(self, rhs: Self) -> Self {
        self + rhs
    }

    fn sub(self, rhs: Self) -> Self {
        self - rhs
    }
}

impl<T: Summable + Copy, const N: usize> Summable for [T; N] {
    fn add(self, rhs: Self) -> Self {
        core::array::from_fn(|i| self[i].add(rhs[i]))
    }

    fn sub(self, rhs: Self) -> Self {
        core::array::from_fn(|i| self[i].sub(rhs[i]))
    }
}

pub trait Scaleable<T> {
    fn scale(self, scale: T) -> Self;
}

impl Scaleable<usize> for f32 {
    fn scale(self, scale: usize) -> Self {
        self / (scale as f32)
    }
}

impl<Scale: Copy, T: Scaleable<Scale>, const N: usize> Scaleable<Scale> for [T; N] {
    fn scale(self, scale: Scale) -> Self {
        self.map(|v| v.scale(scale))
    }
}

pub trait Averageable: Copy + Zero + Summable + Scaleable<usize> {}

impl Averageable for f32 { }
impl<T: Averageable, const N: usize> Averageable for [T; N] { }

pub struct MovingAverage<const N: usize, T> {
    i: usize,
    data: [T; N],
    sum: T,
}

impl<const N: usize, T: Averageable> MovingAverage<N, T> {
    pub const fn new() -> Self {
        Self {
            i: 0,
            data: [T::ZERO; N],
            sum: T::ZERO
        }
    }

    pub fn update(&mut self, new: T) -> T {
        let old = self.data[self.i];
        self.data[self.i] = new;
        self.sum = self.sum.sub(old).add(new);
        self.sum.scale(N)
    }
}

pub struct Average<T> {
    count: usize,
    sum: T
}

impl<T: Averageable> Average<T> {
    pub fn new() -> Self {
        Average {
            count: 0,
            sum: T::ZERO,
        }
    }

    pub fn update(&mut self, v: T) -> T {
        self.sum = self.sum.add(v);
        self.count += 1;
        self.sum.scale(self.count)
    }

    pub fn average(&self) -> T {
        if self.count == 0 {
            T::ZERO
        } else {
            self.sum.scale(self.count)
        }
    }

    pub fn count(&self) -> usize {
        self.count
    }
}