use std::time::Duration;

/// Set of rules the drone follows
/// log_to_stdout:                  if true, prints to console every sent/received/dropped packet (default: false).
/// sleep_duration:                 interval of time the drone sleeps before forwarding a packet. (default: ZERO)
/// await_queued_packets_on_crash:  if true, when a crash command is received, waits for all received packets to be processed before crashing. (default: true)
/// filter_packets:                 if true, filters fragment packets according to the current filter (default: true).
/// send_nack_on_filtered_packet:   if true, when a packet is filtered and not passed through, sends back a NACK. might (and will) cause loops. (default: false)
/// quack:                          if true, quacks the message. (default: false)
pub struct DroneSettings {
    pub log_to_stdout: bool,
    pub sleep_duration: Duration,
    pub await_queued_packets_on_crash: bool,
    pub filter_packets: bool,
    pub send_nack_on_filtered_packet: bool,
    pub quack: bool
}

impl Default for DroneSettings {
    fn default() -> Self {
        DroneSettings {
            log_to_stdout: false,
            sleep_duration: Duration::ZERO,
            await_queued_packets_on_crash: true,
            filter_packets: true,
            send_nack_on_filtered_packet: false,
            quack: false
        }
    }
}