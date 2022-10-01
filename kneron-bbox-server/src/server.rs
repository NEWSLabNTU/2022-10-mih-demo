use crate::protocol::YoloResult;
use anyhow::Result;
use log::info;
use std::{
    io::prelude::*,
    mem,
    net::{TcpListener, ToSocketAddrs},
};

pub const DEFAULT_ADDR: &str = "0.0.0.0:8700";

#[derive(Debug)]
pub struct Server {
    listener: TcpListener,
}

impl Server {
    pub fn new() -> Result<Self> {
        Self::bind(DEFAULT_ADDR)
    }

    pub fn bind<A>(addrs: A) -> Result<Self>
    where
        A: ToSocketAddrs,
    {
        let listener = TcpListener::bind(addrs)?;
        Ok(Self { listener })
    }

    pub fn recv(&self) -> Result<YoloResult> {
        let (mut stream, addr) = self.listener.accept()?;
        info!("Connected from client {}", addr);

        let mut bytes = [0u8; mem::size_of::<YoloResult>()];
        stream.read_exact(&mut bytes)?;
        let result: YoloResult = unsafe { mem::transmute(bytes) };
        Ok(result)
    }
}

impl Iterator for Server {
    type Item = Result<YoloResult>;

    fn next(&mut self) -> Option<Self::Item> {
        Some(self.recv())
    }
}

impl<'a> Iterator for &'a Server {
    type Item = Result<YoloResult>;

    fn next(&mut self) -> Option<Self::Item> {
        Some(self.recv())
    }
}
