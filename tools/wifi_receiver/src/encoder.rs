use crate::Shared;
use std::{io, path::Path, process::Stdio, sync::Arc, sync::atomic::Ordering, time::Duration};
use tokio::{
    io::AsyncWriteExt,
    process::{Child, Command},
    sync::watch,
    time,
};

pub fn spawn(ffmpeg: &str, directory: &Path, fps: u16) -> io::Result<Child> {
    Command::new(ffmpeg)
        .current_dir(directory)
        .args(["-hide_banner", "-loglevel", "warning", "-nostdin", "-n", "-f", "image2pipe",
            "-framerate", &fps.to_string(), "-vcodec", "mjpeg", "-probesize", "32",
            "-analyzeduration", "0", "-i", "pipe:0", "-map", "0:v:0", "-an",
            "-c:v", "libx264", "-preset", "veryfast", "-tune", "zerolatency",
            "-pix_fmt", "yuv420p", "-flags", "+global_header", "-g", &fps.to_string(),
            "-keyint_min", &fps.to_string(), "-sc_threshold", "0", "-f", "tee",
            "[f=hls:hls_time=1:hls_list_size=6:hls_flags=delete_segments+temp_file:hls_segment_filename=segment-%06d.ts]index.m3u8|[f=mp4:movflags=+frag_keyframe+empty_moov]recording.mp4"])
        .stdin(Stdio::piped())
        .stdout(Stdio::null())
        .stderr(Stdio::inherit())
        .kill_on_drop(true)
        .spawn()
}

pub async fn run(
    mut child: Child,
    shared: Arc<Shared>,
    mut stop: watch::Receiver<bool>,
) -> io::Result<()> {
    let mut input = child
        .stdin
        .take()
        .ok_or_else(|| io::Error::other("FFmpeg stdin unavailable"))?;
    let latest = shared.latest.subscribe();
    let mut clock = time::interval(Duration::from_secs_f64(1.0 / f64::from(shared.fps)));
    clock.set_missed_tick_behavior(time::MissedTickBehavior::Skip);
    let result = async {
        loop {
            tokio::select! {
                _ = stop.changed() => break,
                _ = clock.tick() => {}
            }
            if let Some(status) = child.try_wait()? {
                return Err(io::Error::other(format!("FFmpeg exited: {status}")));
            }
            let frame = latest.borrow().clone();
            if let Some(frame) = frame {
                // Repeat only recent frames at output FPS. Never build a queue
                // during network stalls or replay a burst to catch up.
                if frame.received.elapsed() > Duration::from_secs(2) {
                    continue;
                }
                time::timeout(Duration::from_secs(2), input.write_all(&frame.jpeg)).await??;
                shared.encoded.fetch_add(1, Ordering::Relaxed);
            }
        }
        Ok(())
    }
    .await;
    shared.encoder_alive.store(false, Ordering::Release);
    drop(input); // EOF lets FFmpeg finish both the HLS playlist and MP4.
    let finished = time::timeout(Duration::from_secs(5), child.wait()).await;
    match finished {
        Ok(Ok(status))
            if status.success()
                || (result.is_ok()
                    && shared.encoded.load(Ordering::Relaxed) == 0
                    && *stop.borrow()) =>
        {
            result
        }
        Ok(Ok(status)) => result.and(Err(io::Error::other(format!("FFmpeg exited: {status}")))),
        Ok(Err(error)) => Err(error),
        Err(_) => {
            child.kill().await?;
            Err(io::Error::new(
                io::ErrorKind::TimedOut,
                "FFmpeg shutdown timed out",
            ))
        }
    }
}
