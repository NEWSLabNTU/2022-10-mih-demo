use anyhow::Result;
use clap::Parser;
use kneron_bbox_server::Server;
use std::net::SocketAddr;

pub const DEFAULT_ADDR: &str = "0.0.0.0:8700";

#[derive(Debug, Parser)]
struct Opts {
    #[clap(long, help = "Server bind address.")]
    pub addr: Option<SocketAddr>,
}

fn main() -> Result<()> {
    let opts = Opts::parse();

    let mut server = {
        let addr = opts.addr.unwrap_or_else(|| DEFAULT_ADDR.parse().unwrap());
        Server::bind(addr)?
    };

    server.try_for_each(|result| {
        let result = result?;
        let box_count = result.box_count();
        eprintln!("Result Count : {}\n", box_count);

        result.boxes().iter().enumerate().for_each(|(idx, bbox)| {
            eprintln!(
                "Result[{}] x1 {}, y1 {}, x2 {}, y2 {} score {} class_num {}\n",
                idx, bbox.x1, bbox.y1, bbox.x2, bbox.y2, bbox.score, bbox.class_num,
            );
        });

        anyhow::Ok(())
    })?;

    Ok(())
}
