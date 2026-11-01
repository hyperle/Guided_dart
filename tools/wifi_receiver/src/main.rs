mod encoder;
mod jpeg;
mod server;

use axum::body::Bytes;
use clap::Parser;
use std::{
    error::Error,
    net::{IpAddr, SocketAddr},
    path::PathBuf,
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicU64},
    },
    time::{Instant, SystemTime, UNIX_EPOCH},
};
use tokio::sync::watch;

#[derive(Parser)]
#[command(about = "Receive camera JPEGs, encode H.264/HLS and save fragmented MP4 with FFmpeg")]
struct Arguments {
    #[arg(long, default_value = "127.0.0.1")]
    bind: IpAddr,
    #[arg(long, default_value_t = 8080)]
    port: u16,
    #[arg(long, default_value_t = 15, value_parser = clap::value_parser!(u16).range(1..=120))]
    fps: u16,
    #[arg(long, default_value_t = 320, value_parser = clap::value_parser!(u16).range(2..=1920))]
    width: u16,
    #[arg(long, default_value_t = 240, value_parser = clap::value_parser!(u16).range(2..=1080))]
    height: u16,
    #[arg(long, env = "DART_WIFI_TOKEN", hide_env_values = true)]
    token: String,
    #[arg(long, default_value = "ffmpeg")]
    ffmpeg: String,
    /// A fresh session directory is created below this path; files are retained on exit.
    #[arg(long, default_value = "tools/wifi_receiver/output")]
    output: PathBuf,
}

pub struct Frame {
    jpeg: Bytes,
    received: Instant,
}
pub struct Shared {
    token: String,
    width: u16,
    height: u16,
    fps: u16,
    stream_id: String,
    directory: PathBuf,
    latest: watch::Sender<Option<Arc<Frame>>>,
    received: AtomicU64,
    encoded: AtomicU64,
    encoder_alive: AtomicBool,
    stop: watch::Receiver<bool>,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let args = Arguments::parse();
    if args.token.is_empty()
        || args.token.len() > 128
        || !args.token.bytes().all(|v| (33..127).contains(&v))
    {
        return Err("token must contain 1..128 printable non-space ASCII bytes".into());
    }
    if args.width % 2 != 0 || args.height % 2 != 0 {
        return Err("H.264 yuv420p output requires even width and height".into());
    }
    let stamp = SystemTime::now().duration_since(UNIX_EPOCH)?.as_millis();
    let stream_id = format!("{stamp}-{}", std::process::id());
    let directory = args.output.join(&stream_id);
    std::fs::create_dir_all(&args.output)?;
    std::fs::create_dir(&directory)?;
    let directory = std::fs::canonicalize(directory)?;
    let listener = tokio::net::TcpListener::bind(SocketAddr::new(args.bind, args.port)).await?;
    let (latest, _) = watch::channel(None);
    let (stop, stop_rx) = watch::channel(false);
    let shared = Arc::new(Shared {
        token: args.token,
        width: args.width,
        height: args.height,
        fps: args.fps,
        stream_id,
        directory: directory.clone(),
        latest,
        received: AtomicU64::new(0),
        encoded: AtomicU64::new(0),
        encoder_alive: AtomicBool::new(true),
        stop: stop_rx.clone(),
    });
    let child = encoder::spawn(&args.ffmpeg, &directory, args.fps)?;
    let mut encoding = tokio::spawn(encoder::run(child, shared.clone(), stop_rx.clone()));
    let app = server::router(shared, directory.clone());
    let address = listener.local_addr()?;
    let mut serving = tokio::spawn(async move {
        let mut stopped = stop_rx;
        axum::serve(listener, app)
            .with_graceful_shutdown(async move {
                let _ = stopped.changed().await;
            })
            .await
    });
    eprintln!(
        "level=INFO module=wifi_receiver op=start address={address} frame={}x{} fps={} output={}",
        args.width,
        args.height,
        args.fps,
        directory.display()
    );
    let (encoder_done, server_done, result) = tokio::select! {
        result = &mut encoding => (true, false, result.map_err(std::io::Error::other).and_then(|v| v)),
        result = &mut serving => (false, true, result.map_err(std::io::Error::other).and_then(|v| v)),
        result = shutdown_signal() => (false, false, result),
    };
    stop.send_replace(true);
    let server_result = if !server_done {
        match tokio::time::timeout(std::time::Duration::from_secs(12), &mut serving).await {
            Ok(result) => result.map_err(std::io::Error::other).and_then(|v| v),
            Err(_) => {
                serving.abort();
                Err(std::io::Error::other("HTTP shutdown timed out"))
            }
        }
    } else {
        Ok(())
    };
    let encoder_result = if !encoder_done {
        encoding
            .await
            .map_err(std::io::Error::other)
            .and_then(|v| v)
    } else {
        Ok(())
    };
    eprintln!(
        "level=INFO module=wifi_receiver op=stop recording={}",
        directory.join("recording.mp4").display()
    );
    result?;
    server_result?;
    encoder_result?;
    Ok(())
}

async fn shutdown_signal() -> std::io::Result<()> {
    #[cfg(unix)]
    {
        let mut terminate =
            tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())?;
        tokio::select! { result = tokio::signal::ctrl_c() => result, _ = terminate.recv() => Ok(()) }
    }
    #[cfg(not(unix))]
    tokio::signal::ctrl_c().await
}
