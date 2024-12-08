use std::time::Duration;
use wg_2024::controller::DroneCommand;
use wg_2024::network::NodeId;
use crate::packets_filter::FilterType;

pub enum RustableCommand {
    DroneCommand(DroneCommand),
    SettingCommand(SettingsCommand),
    FilterCommand(FilterCommand),
    Quack
}

pub enum SettingsCommand {
    LogToStdout(bool),
    SleepDuration(Duration),
    AwaitQueuedPacketsOnCrash(bool),
    FilterPackets(bool),
    SendNackOnFilteredPackets(bool)
}

pub enum FilterCommand {
    AddId(NodeId),
    RemoveId(NodeId),
    Clear,
    Set(Vec<NodeId>),
    SetType(FilterType),
}

impl From<DroneCommand> for RustableCommand {
    fn from(cmd: DroneCommand) -> Self{
        RustableCommand::DroneCommand(cmd)
    }
}