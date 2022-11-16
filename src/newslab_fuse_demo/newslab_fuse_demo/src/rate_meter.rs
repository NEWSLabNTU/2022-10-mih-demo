use futures::prelude::*;
use std::{
    sync::atomic::{AtomicUsize, Ordering::*},
    time::Duration,
};

pub struct RateMeter {
    interval: Duration,
    count: AtomicUsize,
}

impl RateMeter {
    pub fn new(interval: Duration) -> Self {
        Self {
            interval,
            count: AtomicUsize::new(0),
        }
    }

    pub fn new_secs() -> Self {
        Self::new(Duration::from_secs(1))
    }

    pub fn bump(&self) {
        self.count.fetch_add(1, SeqCst);
    }

    pub fn rate_stream(&self) -> impl Stream<Item = f64> + '_ {
        async_std::stream::interval(self.interval).map(|()| {
            let count = self.count.swap(0, SeqCst);
            count as f64 / self.interval.as_secs_f64()
        })
    }
}
