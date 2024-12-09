# Rustable Drone Handbook

A `RustableDrone` contains two main features:  
`DroneSettings` and `PacketFilter`.  
and a bunch of `RustableCommand`s to manage them.  

Dependency:  
`rustable-drone = {git = "https://github.com/SmL-Boi/RustableDroneLib.git"}`

## Behavior

The drone panics when:
- the index in the `routing_header` is invalid ( < 1 or > len )
- An error occurs and while trying to send a NACK back to the reversed route, the channel to the next node does not exist
  - eg: drone 3 drops a packet with route 1 -> 2 -> 3 -> 4, but a channel to 2 does not exist.
- An error is returned by the `send` method of a `Sender<>`
- Calling the `new` method and the pdr value is invalid ( < 0.0 or > 1.0 )

The drone does NOT panic when:
- A `DroneCommand` fails
  - if the setting `log_to_stdout` is set to true, prints to stderr a message
  - eg: `DroneCommand::RemoveSender()` but the specified `NodeId` is not an adjecent node  

Note: these behaviors might be subject to change in the future.

## DroneSettings

Is a struct that contains a set of rules that change the drone's behavior.  
By default, it makes the drone behave like specified in the protocol.  

The fields are the following:
- `log_to_stdout: bool`
  - if true, prints to console a message every time any `Packet` is received/sent/dropped/filtered, or an error occurs, or a command is received.
  - default value: **false**
- `sleep_duration: Duration`
  - interval of time the drone sleeps before forwarding a `Packet`.
  - default value: **Duration::ZERO**
- `await_queued_packets_on_crash: bool`
  - if true, when a crash command is received, the drone awaits for the received packets to be processed before crashing.
  - default value: **true**
- `filter_packets: bool`
  - if true, filters packets out based on the drone's `PacketFilter`.
  - default value: **true** 
- `send_nack_on_filtered_packet: bool`
  - if true, when a `Packet` is filtered out, sends back a NACK of type `Dropped`.
  - warning: may cause loops.
  - default value: **false**
- `quack: bool`
  - the drone becomes a duck and quacks every `MsgFragment`
  - default value: **false**


## PacketFilter

Filters `Packet`s of type `MsgFragment` based on the `NodeId` of the node they are coming from.  
Effectively, this is a 100% drop probability of packets coming from specified adjacent nodes.  

A `PacketFilter` contains an internal list of `NodeId`s and a `FilterType`.  

The filter type can be set as a `WhiteList`, which only allows packets from `NodeId`s in the list.  
or to a `BlackList`, which only allows packets from `NodeId`s ***NOT*** in the list.  

The default value is an empty blacklist, that effectively lets every packet pass through.  

Methods:  
- `add (&mut self, id: NodeId) -> ()`
  - adds a `NodeId` to the internal list. 
- `remove (&mut self, id: NodeId) -> ()`
  - removes a `NodeId` from the internal list.
- `is_allowed (&self, id: NodeId) -> bool`
  - return true if a `NodeId` is allowed through the filter.
- `clear (&mut self) -> ()`
  - clears the internal list. 
- `set (&mut self, list: Vec<NodeId>) -> ()`
  - sets the internal list
- `set_type (&mut self, t: FilterType) -> ()`
  - sets the filter type

Note: every method that should return a `Result` actually returns void and the error is ignored.  
ad example, removing an id that is not in the list or adding one that is already there does not throw error.



## RustableCommands

These are some commands that are useful to manage the drone through the Simulation Controller.  
Of course a group is not bound to implement them, as the default behavior of the drone is the one specified in the protocol.  

### SettingsCommands

`SettingsCommand`s are commands to manage a drone's settings.  
The enum's value will overwrite the one in the corresponding `DroneSettings` field 

- `LogToStdout (bool)`
- `SleepDuration (Duration)`
- `AwaitQueuedPacketsOnCrash (bool)`
- `FilterPackets (bool)`
- `SendNackOnFilteredPackets (bool)`

### FilterCommands

`FilterCommand`s are commands to manage a drone's filter.  
The enum's value will act as the corresponding `PacketFilter`s method.  

- `AddId(NodeId)`
- `RemoveId(NodeId)`
- `Clear`
- `Set(Vec<NodeId>)`
- `SetType(FilterType)`


### Quack

Quacks is a command that **toggles** the quacking of a drone.
