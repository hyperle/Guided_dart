use crate::{Frame, Shared, jpeg};
use axum::{
    Json, Router,
    body::Bytes,
    extract::{
        DefaultBodyLimit, State, WebSocketUpgrade,
        ws::{Message, WebSocket},
    },
    http::{HeaderMap, HeaderValue, StatusCode, header},
    response::{Html, IntoResponse, Response},
    routing::{get, post},
};
use serde::Deserialize;
use serde_json::json;
use std::{
    path::PathBuf,
    sync::{Arc, atomic::Ordering},
    time::Instant,
};
use tokio::time::{Duration, timeout};
use tower_http::{services::ServeDir, set_header::SetResponseHeaderLayer, timeout::TimeoutLayer};

const MAX_JPEG: usize = 2 * 1024 * 1024;

pub fn router(shared: Arc<Shared>, directory: PathBuf) -> Router {
    Router::new()
        .route(
            "/",
            get(|| async { Html(include_str!("../../wifi/viewer.html")) }),
        )
        .route(
            "/assets/hls-1.5.18.min.js",
            get(|| async {
                (
                    [(header::CONTENT_TYPE, "text/javascript")],
                    include_str!("../../wifi/vendor/hls-1.5.18.min.js"),
                )
            }),
        )
        .route("/api/status", get(status))
        .route("/api/frame", post(upload))
        .route("/api/frame/ws", get(websocket))
        .route("/api/relay/ws", get(relay))
        .nest_service("/live", ServeDir::new(directory))
        .layer(DefaultBodyLimit::max(MAX_JPEG))
        .layer(TimeoutLayer::with_status_code(
            StatusCode::REQUEST_TIMEOUT,
            Duration::from_secs(10),
        ))
        .layer(SetResponseHeaderLayer::overriding(
            header::CACHE_CONTROL,
            HeaderValue::from_static("no-store"),
        ))
        .with_state(shared)
}

fn authorized(headers: &HeaderMap, token: &str) -> bool {
    headers
        .get(header::AUTHORIZATION)
        .and_then(|v| v.to_str().ok())
        .and_then(|v| v.strip_prefix("Bearer "))
        == Some(token)
}

fn submit(shared: &Shared, jpeg: Bytes) -> Result<(), StatusCode> {
    if !shared.encoder_alive.load(Ordering::Acquire) {
        return Err(StatusCode::SERVICE_UNAVAILABLE);
    }
    if jpeg.len() > MAX_JPEG || jpeg::dimensions(&jpeg) != Some((shared.width, shared.height)) {
        return Err(StatusCode::UNPROCESSABLE_ENTITY);
    }
    let frame = Arc::new(Frame {
        jpeg,
        received: Instant::now(),
    });
    shared.latest.send_replace(Some(frame));
    shared.received.fetch_add(1, Ordering::Relaxed);
    Ok(())
}

async fn upload(State(shared): State<Arc<Shared>>, headers: HeaderMap, data: Bytes) -> StatusCode {
    if !authorized(&headers, &shared.token) {
        return StatusCode::UNAUTHORIZED;
    }
    if headers
        .get(header::CONTENT_TYPE)
        .and_then(|v| v.to_str().ok())
        != Some("image/jpeg")
    {
        return StatusCode::UNSUPPORTED_MEDIA_TYPE;
    }
    match submit(&shared, data) {
        Ok(()) => StatusCode::NO_CONTENT,
        Err(status) => status,
    }
}

async fn status(State(shared): State<Arc<Shared>>) -> Json<serde_json::Value> {
    let age = shared
        .latest
        .borrow()
        .as_ref()
        .map(|frame| frame.received.elapsed().as_secs_f64());
    Json(json!({
        "received_frames": shared.received.load(Ordering::Relaxed),
        "encoded_frames": shared.encoded.load(Ordering::Relaxed),
        "online": age.is_some_and(|age| age < 3.0), "age_seconds": age,
        "width": shared.width, "height": shared.height, "fps": shared.fps,
        "hls_ready": tokio::fs::try_exists(shared.directory.join("index.m3u8")).await.unwrap_or(false),
        "stream_id": shared.stream_id, "error": if shared.encoder_alive.load(Ordering::Acquire) { "" } else { "FFmpeg stopped" },
        "recording": "/live/recording.mp4"
    }))
}

async fn websocket(
    State(shared): State<Arc<Shared>>,
    headers: HeaderMap,
    ws: WebSocketUpgrade,
) -> Response {
    if !authorized(&headers, &shared.token) {
        return StatusCode::UNAUTHORIZED.into_response();
    }
    ws.max_message_size(MAX_JPEG)
        .max_frame_size(MAX_JPEG)
        .on_upgrade(move |socket| receive(socket, shared, false))
}
async fn relay(State(shared): State<Arc<Shared>>, ws: WebSocketUpgrade) -> Response {
    ws.max_message_size(MAX_JPEG)
        .max_frame_size(MAX_JPEG)
        .on_upgrade(move |socket| receive(socket, shared, true))
}

#[derive(Deserialize)]
struct Authentication {
    token: String,
}

async fn receive(mut socket: WebSocket, shared: Arc<Shared>, browser_auth: bool) {
    let mut stop = shared.stop.clone();
    if browser_auth {
        let authenticated = match timeout(Duration::from_secs(5), socket.recv()).await {
            Ok(Some(Ok(Message::Text(text)))) if text.len() <= 512 => {
                serde_json::from_str::<Authentication>(&text)
                    .is_ok_and(|auth| auth.token == shared.token)
            }
            _ => false,
        };
        if !authenticated
            || socket
                .send(Message::Text("{\"ready\":true}".into()))
                .await
                .is_err()
        {
            return;
        }
    }
    let mut received = 0_u32;
    loop {
        let message = tokio::select! {
            _ = stop.changed() => break,
            message = timeout(Duration::from_secs(15), socket.recv()) => message,
        };
        match message {
            Ok(Some(Ok(Message::Binary(data)))) => {
                if submit(&shared, data).is_err() {
                    break;
                }
                let Some(next) = received.checked_add(1) else {
                    break;
                };
                received = next;
                if !matches!(
                    timeout(
                        Duration::from_secs(2),
                        socket.send(Message::Binary(Bytes::copy_from_slice(
                            &received.to_be_bytes()
                        )))
                    )
                    .await,
                    Ok(Ok(()))
                ) {
                    break;
                }
            }
            // Phone relay sends periodic bounded status reports; frames use the
            // same cumulative binary acknowledgment as the MicroPython client.
            Ok(Some(Ok(Message::Text(text)))) if browser_auth && text.len() <= 512 => {}
            Ok(Some(Ok(Message::Ping(_)))) | Ok(Some(Ok(Message::Pong(_)))) => {}
            _ => break,
        }
    }
    let _ = timeout(Duration::from_secs(1), socket.send(Message::Close(None))).await;
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn authorization_requires_exact_bearer_value() {
        let mut headers = HeaderMap::new();
        assert!(!authorized(&headers, "test-token"));
        headers.insert(
            header::AUTHORIZATION,
            HeaderValue::from_static("Bearer test-token"),
        );
        assert!(authorized(&headers, "test-token"));
        assert!(!authorized(&headers, "test-token-extra"));
    }
}
