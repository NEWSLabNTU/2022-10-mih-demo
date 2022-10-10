use crate::protocol::YoloResult;
use anyhow::Result;
use async_std::net::{TcpListener, ToSocketAddrs};
use futures::prelude::*;
use log::info;
use std::mem;

pub const DEFAULT_ADDR: &str = "0.0.0.0:8700";

/// The async/.await server that listens to detection messages from
/// the Kneron camera.
#[derive(Debug)]
pub struct AsyncServer {
    listener: TcpListener,
}

impl AsyncServer {
    /// Starts the server that binds to the default address.
    pub async fn new() -> Result<Self> {
        Self::bind(DEFAULT_ADDR).await
    }

    /// Starts the server that binds to specified address.
    pub async fn bind<A>(addrs: A) -> Result<Self>
    where
        A: ToSocketAddrs,
    {
        let listener = TcpListener::bind(addrs).await?;
        Ok(Self { listener })
    }

    pub async fn recv(&self) -> Result<YoloResult> {
        let (mut stream, addr) = self.listener.accept().await?;
        info!("Connected from client {}", addr);

        let mut bytes = [0u8; mem::size_of::<YoloResult>()];
        stream.read_exact(&mut bytes).await?;
        let result: YoloResult = unsafe { mem::transmute(bytes) };
        Ok(result)
    }

    pub fn into_stream(self) -> impl Stream<Item = Result<YoloResult>> + Sync + Send {
        stream::try_unfold(self, |server| async move {
            let result = server.recv().await?;
            anyhow::Ok(Some((result, server)))
        })
    }
}
