use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use vicon_sdk::{Client, ViconError};

use crate::source::{PoseSource, RawSample};

/// Reads pose for a single subject/segment from a Vicon DataStream server.
pub struct ViconSource {
    client: Client,
    host: String,
    subject: String,
    segment: String,
    t0: Instant,
    last_warn: Option<Instant>,
}

impl ViconSource {
    pub fn connect(host: &str, subject: &str, segment: Option<&str>) -> Result<Self> {
        let client = Client::new();
        let _ = client.set_connection_timeout(10_000);

        client
            .connect(host)
            .with_context(|| format!("connecting to Vicon DataStream at {host}"))?;
        client
            .enable_segment_data()
            .context("enabling segment data")?;

        let segment = match segment {
            Some(s) => s.to_string(),
            None => client
                .get_subject_root_segment_name(subject)
                .with_context(|| {
                    format!(
                        "subject {subject:?} not found{}",
                        available_subjects(&client)
                    )
                })?,
        };

        let version = client.get_version();
        eprintln!(
            "vicon: connected to {host}, SDK {}.{}.{}.{}, subject {subject:?}, segment {segment:?}",
            version.major, version.minor, version.point, version.revision
        );

        Ok(Self {
            client,
            host: host.to_string(),
            subject: subject.to_string(),
            segment,
            t0: Instant::now(),
            last_warn: None,
        })
    }

    fn warn_throttled(&mut self, msg: impl std::fmt::Display) {
        let now = Instant::now();
        if self
            .last_warn
            .map(|t| now.duration_since(t) >= Duration::from_secs(1))
            .unwrap_or(true)
        {
            eprintln!("vicon: {msg}");
            self.last_warn = Some(now);
        }
    }

    fn reconnect(&mut self) {
        let _ = self.client.disconnect();
        match self.client.connect(&self.host) {
            Ok(()) => {
                let _ = self.client.enable_segment_data();
                eprintln!("vicon: reconnected to {}", self.host);
            }
            Err(e) => self.warn_throttled(format!("reconnect failed: {e}")),
        }
    }
}

fn available_subjects(client: &Client) -> String {
    let count = match client.get_subject_count() {
        Ok(c) => c,
        Err(_) => return String::new(),
    };
    let names: Vec<String> = (0..count)
        .filter_map(|i| client.get_subject_name(i).ok())
        .collect();
    if names.is_empty() {
        String::new()
    } else {
        format!(" (available subjects: {})", names.join(", "))
    }
}

impl PoseSource for ViconSource {
    fn next_sample(&mut self) -> Option<RawSample> {
        loop {
            match self.client.get_frame() {
                Ok(()) => {}
                Err(ViconError::NoFrame) => {
                    std::thread::sleep(Duration::from_millis(1));
                    continue;
                }
                Err(ViconError::NotConnected) | Err(ViconError::ClientConnectionFailed) => {
                    self.warn_throttled("connection lost, retrying");
                    std::thread::sleep(Duration::from_millis(500));
                    self.reconnect();
                    continue;
                }
                Err(e) => {
                    self.warn_throttled(format!("GetFrame: {e}"));
                    std::thread::sleep(Duration::from_millis(20));
                    continue;
                }
            }

            let now = self.t0.elapsed().as_secs_f64();
            match self
                .client
                .get_segment_global_translation(&self.subject, &self.segment)
            {
                Ok(tr) => {
                    let quat = self
                        .client
                        .get_segment_global_rotation_quaternion(&self.subject, &self.segment)
                        .ok()
                        .map(|q| q.quaternion);
                    return Some(RawSample {
                        t: now,
                        // Vicon reports millimetres
                        pos_m: [
                            tr.position[0] * 1e-3,
                            tr.position[1] * 1e-3,
                            tr.position[2] * 1e-3,
                        ],
                        quat_xyzw: quat,
                        occluded: tr.occluded,
                    });
                }
                Err(e) => {
                    self.warn_throttled(format!("segment read: {e}"));
                    return Some(RawSample::missing(now));
                }
            }
        }
    }
}
