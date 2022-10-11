use anyhow::Result;
use clap::Parser;
use futures::prelude::*;
use kneron_bbox_server::AsyncServer;
use std::net::SocketAddr;

pub const DEFAULT_ADDR: &str = "0.0.0.0:8700";

#[derive(Debug, Parser)]
struct Opts {
    #[clap(long, help = "Server bind address.")]
    pub addr: Option<SocketAddr>,
}

#[async_std::main]
async fn main() -> Result<()> {
    let opts = Opts::parse();

    let server = {
        let addr = opts.addr.unwrap_or_else(|| DEFAULT_ADDR.parse().unwrap());
        AsyncServer::bind(addr).await?
    };

    server
        .into_stream()
        .try_for_each(|result| async move {
            let box_count = result.box_count();
            eprintln!("Result Count : {}\n", box_count);

            result.boxes().iter().enumerate().for_each(|(idx, bbox)| {
                eprintln!(
                    "Result[{}] x1 {}, y1 {}, x2 {}, y2 {} score {} class_num {}\n",
                    idx, bbox.x1, bbox.y1, bbox.x2, bbox.y2, bbox.score, bbox.class_num,
                );
            });

            anyhow::Ok(())
        })
        .await?;

    Ok(())
}
