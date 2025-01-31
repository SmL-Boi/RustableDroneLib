use std::collections::HashMap;
use std::thread;
use crossbeam_channel::{select_biased, Receiver, RecvError, SendError, Sender};
use rand::{thread_rng, Rng};
use wg_2024::controller::{DroneCommand, DroneEvent};
use wg_2024::drone::{Drone};
use wg_2024::network::{NodeId, SourceRoutingHeader};
use wg_2024::packet::{Ack, FloodRequest, FloodResponse, Fragment, Nack, NackType, NodeType, Packet, PacketType};
use wg_2024::packet::NackType::{DestinationIsDrone, Dropped, ErrorInRouting, UnexpectedRecipient};
use wg_2024::packet::NodeType::*;
use wg_2024::packet::PacketType::MsgFragment;
use crate::controller_commands::{FilterCommand, RustableCommand, SettingsCommand};
use crate::drone_settings::DroneSettings;
use crate::packets_filter::PacketFilter;



pub struct RustableDrone {
    id: NodeId,
    controller_send: Sender<DroneEvent>,
    controller_recv: Receiver<DroneCommand>,
    packet_send: HashMap<NodeId, Sender<Packet>>,
    packet_recv: Receiver<Packet>,
    drop_rate: f32,
    pub settings: DroneSettings,
    pub filter: PacketFilter,
    flood_ids: Vec<u64>,
    has_to_crash: bool
}

impl Drone for RustableDrone {
    fn new(
        id: NodeId,
        controller_send: Sender<DroneEvent>,
        controller_recv: Receiver<DroneCommand>,
        packet_recv: Receiver<Packet>,
        packet_send: HashMap<NodeId, Sender<Packet>>,
        pdr: f32
    ) -> Self {
        if pdr < 0.0f32 || pdr > 1.0f32 {
            panic!("Invalid packet drop rate value")
        }

        Self {
            id,
            controller_send,
            controller_recv,
            packet_send,
            packet_recv,
            drop_rate: pdr,
            settings: DroneSettings::default(),
            filter: PacketFilter::default(),
            flood_ids: vec![],
            has_to_crash: false
        }
    }

    fn run(&mut self) {
        loop {
            if self.has_to_crash {
                if !self.settings.await_queued_packets_on_crash {
                    return
                }
                //send all remaining packets
                while let Ok(packet) = self.packet_recv.try_recv() {
                    self.packet_handler(Ok(packet));
                }
            } else {
                // listens to commands/packets pipes, prioritizing commands
                select_biased! {
                    recv(self.controller_recv) -> command => {
                        self.command_handler(command)
                    },
                     recv(self.packet_recv) -> packet => {
                        self.packet_handler(packet)
                    }
                }
            }
        }
    }
}


impl RustableDrone {
    /// Handles a Packet
    fn packet_handler(&mut self, packet: Result<Packet, RecvError>) {
        if packet.is_err() {
            panic!("Packet Error in RustableDrone: {}\n{}", self.id, packet.err().unwrap());
        }

        let packet: Packet = packet.unwrap();

        //indexing errors
        if packet.routing_header.hop_index < 1 && !matches!(&packet.pack_type, PacketType::FloodRequest(x)) {
            panic!("Indexing Error in RustableDrone {} receiving a Packet from node {}. hop_index is less than 1 in packet's header:\n{:?}", self.id, packet.routing_header.previous_hop().unwrap(), packet.routing_header);
        }
        if packet.routing_header.hop_index > packet.routing_header.hops.len() && !matches!(&packet.pack_type, PacketType::FloodRequest(x)) {
            panic!("Indexing Error in RustableDrone {} receiving a Packet from node {}. hop_index is bigger than hops len in packet's header:\n{:?}", self.id, packet.routing_header.previous_hop().unwrap(), packet);
        }

        match packet.clone().pack_type {
            PacketType::Nack(_nack) => self.nack_handler(&packet, _nack),
            PacketType::Ack(_ack) => self.ack_handler(&packet, _ack),
            PacketType::MsgFragment(_frag) => self.msg_fragment_handler(&packet, _frag),
            PacketType::FloodRequest(_req) => self.flood_req_handler(&packet, _req),
            PacketType::FloodResponse(_res) => self.flood_res_handler(&packet),
        }
    }

    fn msg_fragment_handler(&self, packet: &Packet, mut fragment: Fragment) {
        let from: NodeId = packet.routing_header.previous_hop().unwrap();

        //destination is drone
        if packet.routing_header.is_last_hop() {
            //sends destination is drone nack
            if self.packet_send.contains_key(&from) {
                if self.settings.log_to_stdout {
                    println!("RustableDrone {} encountered a DestinationIsDrone error while receiving a MsgFragment from node {}", self.id, from);
                }
                self.send_nack(from, packet, fragment.fragment_index, DestinationIsDrone);
            } else {
                panic!("RustableDrone {} encountered a DestinationIsDrone error but was incapable of sending a NACK back to node {}, as the channel does not exist", self.id, from);
            }
            return;
        }

        let to: NodeId = packet.routing_header.next_hop().unwrap();

        //drop probability
        if thread_rng().gen_bool(self.drop_rate as f64) {
            //sends dropped nack
            if self.packet_send.contains_key(&from) {
                if self.settings.log_to_stdout {
                    println!("RustableDrone {} dropped a MsgFragment received from node {} directed to node {}", self.id, from, to);
                }
                self.send_nack(from, packet, fragment.fragment_index, Dropped);
            } else {
                panic!("RustableDrone {} dropped a packet but was incapable of sending a NACK back to node {}, as the channel does not exist", self.id, from);
            }
            return;
        }

        //filter
        if self.settings.filter_packets && !self.filter.is_allowed(from) {
            if self.settings.log_to_stdout {
                println!("RustableDrone {} filtered a MsgFragment received from node {} directed to node {}", self.id, from, to)
            }
            if self.settings.send_nack_on_filtered_packet {
                //sends dropped nack
                if self.packet_send.contains_key(&from) {
                    self.send_nack(from, packet, fragment.fragment_index, Dropped);
                } else {
                    panic!("RustableDrone {} filtered a packet but was incapable of sending a NACK back to node {}, as the channel does not exist", self.id, from);
                }
            }
            return;
        }

        //routing error
        if !self.packet_send.contains_key(&to) {
            //sends routing error nack
            if self.packet_send.contains_key(&from) {
                if self.settings.log_to_stdout {
                    println!("RustableDrone {} encountered an ErrorInRouting while trying to forward a MsgFragment from node {} to node {}", self.id, from, to);
                }
                self.send_nack(from, packet, fragment.fragment_index, ErrorInRouting(to));
            } else {
                panic!("RustableDrone {} encountered an ErrorInRouting while trying to forward a MsgFragment to node {} but was incapable of sending a NACK back to node {}, as the channel does not exist", self.id, to, from);
            }
            return;
        }

        //unexpected recipient
        if packet.routing_header.hops[packet.routing_header.hop_index] != self.id {
            //sends destination is drone nack
            if self.packet_send.contains_key(&from) {
                if self.settings.log_to_stdout {
                    println!("RustableDrone {} encountered an UnexpectedRecipient error while trying to forward a MsgFragment from node {} to node {}", self.id, from, to);
                }
                self.send_nack(from, packet, fragment.fragment_index, UnexpectedRecipient(self.id));
            } else {
                panic!("RustableDrone {} encountered an UnexpectedRecipient error but was incapable of sending a NACK back to node {}, as the channel does not exist", self.id, from);
            }
            return;
        }

        //is quack
        if self.settings.quack {
            let bytes = "QUACK".as_bytes();
            let bytes_len = bytes.len();
            for i in 0..fragment.data.len() {
                fragment.data[i] = bytes[i%bytes_len];
            }
        }

        //sleeps
        if !self.settings.sleep_duration.is_zero() {
            thread::sleep(self.settings.sleep_duration);
        }

        //all good, propagate packet as it should be
        let mut header = packet.routing_header.clone();
        header.hop_index += 1;
        let res: Result<(), SendError<Packet>> = self.packet_send.get(&to).unwrap().send(
            Packet{
                routing_header: header,
                session_id: packet.session_id,
                pack_type: (MsgFragment(fragment)),
            }
        );

        if res.is_err() {
            panic!("{}", res.err().unwrap())
        }

        //log it
        if self.settings.log_to_stdout {
            println!("RustableDrone {} received a MsgFragment from node {} and forwarded it to node {}", self.id, from, to);
        }
    }

    fn nack_handler(&self, packet: &Packet, nack: Nack) {
        let from: NodeId = packet.routing_header.previous_hop().unwrap();

        //destination is drone
        if packet.routing_header.is_last_hop() {
            //sends destination is drone nack
            if self.packet_send.contains_key(&from) {
                if self.settings.log_to_stdout {
                    println!("RustableDrone {} encountered a DestinationIsDrone error while receiving a NACK from node {}", self.id, from);
                }
                self.send_nack_through_controller(packet, nack.fragment_index, DestinationIsDrone);
            } else {
                panic!("RustableDrone {} encountered a DestinationIsDrone error but was incapable of sending a NACK back to node {}, as the channel does not exist", self.id, from);
            }
            return;
        }

        let to: NodeId = packet.routing_header.next_hop().unwrap();

        //routing error
        if !self.packet_send.contains_key(&to) {
            //sends routing error nack
            if self.packet_send.contains_key(&from) {
                if self.settings.log_to_stdout {
                    println!("RustableDrone {} encountered an ErrorInRouting while trying to forward a NACK from node {} to node {}", self.id, from, to);
                }
                self.send_nack_through_controller(packet, nack.fragment_index, ErrorInRouting(to));
            } else {
                panic!("RustableDrone {} encountered an ErrorInRouting while trying to forward a NACK to node {} but was incapable of sending a NACK back to node {}, as the channel does not exist", self.id, to, from);
            }
            return;
        }

        //unexpected recipient
        if packet.routing_header.hops[packet.routing_header.hop_index] != self.id {
            //sends destination is drone nack
            if self.packet_send.contains_key(&from) {
                if self.settings.log_to_stdout {
                    println!("RustableDrone {} encountered an UnexpectedRecipient error while trying to forward a NACK from node {} to node {}", self.id, from, to);
                }
                self.send_nack_through_controller(packet, nack.fragment_index, UnexpectedRecipient(self.id));
            } else {
                panic!("RustableDrone {} encountered an UnexpectedRecipient error but was incapable of sending a NACK back to node {}, as the channel does not exist", self.id, from);
            }
            return;
        }

        //sleeps
        if !self.settings.sleep_duration.is_zero() {
            thread::sleep(self.settings.sleep_duration);
        }

        //all is good, propagate NACK normally
        let mut p = packet.clone();
        p.routing_header.hop_index += 1;
        let res: Result<(), SendError<Packet>> = self.packet_send.get(&to).unwrap().send(p);

        if res.is_err() {
            panic!("{}", res.err().unwrap())
        }

        //log it
        if self.settings.log_to_stdout {
            println!("RustableDrone {} received a NACK from node {} and forwarded it to node {}", self.id, from, to);
        }
    }

    fn ack_handler(&self, packet: &Packet, ack: Ack) {
        let from: NodeId = packet.routing_header.hops[packet.routing_header.hop_index - 1];

        //destination is drone
        if packet.routing_header.is_last_hop() {
            //sends destination is drone nack
            if self.packet_send.contains_key(&from) {
                if self.settings.log_to_stdout {
                    println!("RustableDrone {} encountered a DestinationIsDrone error while receiving an ACK from node {}", self.id, from);
                }
                self.send_nack_through_controller(packet, ack.fragment_index, DestinationIsDrone);
            } else {
                panic!("RustableDrone {} encountered a DestinationIsDrone error but was incapable of sending a NACK back to node {}, as the channel does not exist", self.id, from);
            }
            return;
        }

        let to: NodeId = packet.routing_header.hops[packet.routing_header.hop_index + 1];

        //routing error
        if !self.packet_send.contains_key(&to) {
            //sends routing error nack
            if self.packet_send.contains_key(&from) {
                if self.settings.log_to_stdout {
                    println!("RustableDrone {} encountered an ErrorInRouting while trying to forward an ACK from node {} to node {}", self.id, from, to);
                }
                self.send_nack_through_controller(packet, ack.fragment_index, ErrorInRouting(to));
            } else {
                panic!("RustableDrone {} encountered an ErrorInRouting while trying to forward an ACK to node {} but was incapable of sending a NACK back to node {}, as the channel does not exist", self.id, to, from);
            }
            return;
        }

        //unexpected recipient
        if packet.routing_header.hops[packet.routing_header.hop_index] != self.id {
            //sends destination is drone nack
            if self.packet_send.contains_key(&from) {
                if self.settings.log_to_stdout {
                    println!("RustableDrone {} encountered an UnexpectedRecipient error while trying to forward an ACK from node {} to node {}", self.id, from, to);
                }
                self.send_nack_through_controller(packet, ack.fragment_index, UnexpectedRecipient(self.id));
            } else {
                panic!("RustableDrone {} encountered an UnexpectedRecipient error but was incapable of sending a NACK back to node {}, as the channel does not exist", self.id, from);
            }
            return;
        }

        //sleeps
        if !self.settings.sleep_duration.is_zero() {
            thread::sleep(self.settings.sleep_duration);
        }

        //all is good, propagate NACK normally
        let mut p = packet.clone();
        p.routing_header.hop_index += 1;
        let res: Result<(), SendError<Packet>> = self.packet_send.get(&to).unwrap().send(p);

        if res.is_err() {
            panic!("{}", res.err().unwrap())
        }

        //log it
        if self.settings.log_to_stdout {
            println!("RustableDrone {} received an ACK from node {} and forwarded it to node {}", self.id, from, to);
        }
    }

    fn flood_req_handler(&mut self,  packet: &Packet, mut request: FloodRequest ) {
        //checks
        let from: NodeId = request.path_trace[request.path_trace.len() - 1 ].0;

        request.path_trace.push((self.id, Drone));
        if self.flood_ids.contains(&request.flood_id) { //already visited
            //send back a FloodResponse

            let mut rev_route: Vec<NodeId> = vec![];
            for i in (0..request.path_trace.len()).rev() {
                rev_route.push(request.path_trace[i].0);
            }

            //sleeps
            if !self.settings.sleep_duration.is_zero() {
                thread::sleep(self.settings.sleep_duration);
            }

            let res: Result<(), SendError<Packet>> = self.packet_send.get(&from).unwrap().send(
                Packet {
                    pack_type: PacketType::FloodResponse(FloodResponse { flood_id: request.flood_id, path_trace: request.path_trace }),
                    routing_header: SourceRoutingHeader{
                        hop_index: 1,
                        hops: rev_route,
                    },
                    session_id: packet.session_id,
                }
            );

            if res.is_err() {
                panic!("{}", res.err().unwrap())
            }

            //log it
            if self.settings.log_to_stdout {
                println!("RustableDrone {} sent a FloodResponse to node {} because it was already visited", self.id, from);
            }

        } else {
            if self.packet_send.len() == 1 { //no neighbors
                //send back a Floodresponse

                let mut rev_route: Vec<NodeId> = vec![];
                for i in (0..request.path_trace.len()).rev() {
                    rev_route.push(request.path_trace[i].0);
                }

                //sleeps
                if !self.settings.sleep_duration.is_zero() {
                    thread::sleep(self.settings.sleep_duration);
                }

                let res: Result<(), SendError<Packet>> = self.packet_send.get(&from).unwrap().send(
                    Packet {
                        pack_type: PacketType::FloodResponse(FloodResponse { flood_id: request.flood_id, path_trace: request.path_trace }),
                        routing_header: SourceRoutingHeader{
                            hop_index: 1,
                            hops: rev_route,
                        },
                        session_id: packet.session_id,
                    }
                );

                if res.is_err() {
                    panic!("{}", res.err().unwrap())
                }

                //log it
                if self.settings.log_to_stdout {
                    println!("RustableDrone {} sent a FloodResponse to node {} because it has no neighbors", self.id, from);
                }

            }else{ //all good forward floosrequest
                self.flood_ids.push(request.flood_id);

                //sleeps
                if !self.settings.sleep_duration.is_zero() {
                    thread::sleep(self.settings.sleep_duration);
                }

                //iterare i vicni e mandare la richiesta a tutti tranne che a quello da cui l'hai ricevuta
                for (key, value) in self.packet_send.iter().filter(|(k, _)| **k != from) {

                    let res: Result<(), SendError<Packet>> = self.packet_send.get(&key).unwrap().send(
                        Packet{
                            routing_header: Default::default(),
                            session_id: packet.session_id,
                            pack_type: PacketType::FloodRequest(request.clone()),
                        }
                    );

                    if res.is_err() {
                        panic!("{}", res.err().unwrap())
                    }

                    //log it
                    if self.settings.log_to_stdout {
                        println!("RustableDrone {} received a FloodRequest from node {} and forwarded it to node {}", self.id, from, key);
                    }

                }

            }


        }

    }

    fn flood_res_handler(&mut self, packet: &Packet) {
        let from: NodeId = packet.routing_header.hops[packet.routing_header.hop_index - 1];

        //destination is drone
        if packet.routing_header.hop_index == packet.routing_header.hops.len() {
            //sends destination is drone nack
            if self.packet_send.contains_key(&from) {
                if self.settings.log_to_stdout {
                    println!("RustableDrone {} encountered a DestinationIsDrone error while receiving a NACK from node {}", self.id, from);
                }
                self.send_nack_through_controller(packet, u64::MAX, DestinationIsDrone);
            } else {
                panic!("RustableDrone {} is the destination of the packet but was incapable of sending a NACK back to node {}, as the channel does not exist", self.id, from);
            }
            return;
        }

        let to: NodeId = packet.routing_header.hops[packet.routing_header.hop_index + 1];
        //let to: NodeId = packet.routing_header.next_hop().unwrap();


        //routing error
        if !self.packet_send.contains_key(&to) {
            //sends routing error nack
            if self.packet_send.contains_key(&from) {
                if self.settings.log_to_stdout {
                    println!("RustableDrone {} encountered an ErrorInRouting while trying to forward a NACK from node {} to node {}", self.id, from, to);
                }
                self.send_nack_through_controller(packet, u64::MAX, ErrorInRouting(to));
            } else {
                panic!("RustableDrone {} encountered an ErrorInRouting (to node {}) but was incapable of sending a NACK back to node {}, as the channel does not exist", self.id, to, from);
            }
            return;
        }

        //unexpected recipient
        if packet.routing_header.hops[packet.routing_header.hop_index] != self.id {
            //sends destination is drone nack
            if self.packet_send.contains_key(&from) {
                if self.settings.log_to_stdout {
                    println!("RustableDrone {} encountered an UnexpectedRecipient error while trying to forward a NACK from node {} to node {}", self.id, from, to);
                }
                self.send_nack_through_controller(packet, u64::MAX, UnexpectedRecipient(self.id));
            } else {
                panic!("RustableDrone {} encountered an UnexpectedRecipient error but was incapable of sending a NACK back to node {}, as the channel does not exist", self.id, from);
            }
            return;
        }


        //sleeps
        if !self.settings.sleep_duration.is_zero() {
            thread::sleep(self.settings.sleep_duration);
        }

        //allgood
        let mut p = packet.clone();
        p.routing_header.hop_index += 1;
        let res: Result<(), SendError<Packet>> = self.packet_send.get(&to).unwrap().send(p);

        if res.is_err() {
            panic!("{}", res.err().unwrap())
        }

        //log it
        if self.settings.log_to_stdout {
            println!("RustableDrone {} received a FloodResponse from node {} and forwarded it to node {}", self.id, from, to);
        }
    }

    /// Handles a DroneCommand
    fn command_handler(&mut self, command: Result<DroneCommand, RecvError>) {
        if command.is_err() {
            panic!("Command Error in drone: {}\n{}", self.id, command.err().unwrap());
        }

        match RustableCommand::from(command.unwrap()) {
            RustableCommand::DroneCommand(command) => {
                match command {
                    DroneCommand::AddSender(_id, _sender) => {
                        let id = self.id;
                        let res = self.add_channel(_id, _sender);
                        if res.is_err() {
                            eprintln!("DroneCommand AddSender failed in RustableDrone {}: {}", id, res.err().unwrap())
                        }
                    }
                    DroneCommand::RemoveSender(_id) => {
                        let id = self.id;
                        let res = self.remove_channel(_id);
                        if res.is_err() {
                            eprintln!("DroneCommand RemoveSender failed in RustableDrone {}: {}", id, res.err().unwrap())
                        }
                    }
                    DroneCommand::SetPacketDropRate(_pdr) => {
                        if _pdr < 0.0f32 || _pdr > 1.0f32 {
                            eprintln!("Invalid packet drop rate value")
                        } else {
                            self.drop_rate = _pdr;
                        }
                    }
                    DroneCommand::Crash => {
                        self.has_to_crash = true;
                    }
                }
            }
            RustableCommand::SettingCommand(command) => {
                match command {
                    SettingsCommand::LogToStdout(_val) => {
                        self.settings.log_to_stdout = _val;
                    }
                    SettingsCommand::SleepDuration(_duration) => {
                        self.settings.sleep_duration = _duration.clone();
                    }
                    SettingsCommand::AwaitQueuedPacketsOnCrash(_val) => {
                        self.settings.await_queued_packets_on_crash = _val;
                    }
                    SettingsCommand::FilterPackets(_val) => {
                        self.settings.filter_packets = _val;
                    }
                    SettingsCommand::SendNackOnFilteredPackets(_val) => {
                        self.settings.send_nack_on_filtered_packet = _val;
                    }
                }
            }
            RustableCommand::FilterCommand(command) => {
                match command {
                    FilterCommand::AddId(_id) => {
                        self.filter.add(_id);
                    }
                    FilterCommand::RemoveId(_id) => {
                        self.filter.remove(_id);
                    }
                    FilterCommand::Clear => {
                        self.filter.clear();
                    }
                    FilterCommand::Set(_vec) => {
                        self.filter.set(_vec);
                    }
                    FilterCommand::SetType(_type) => {
                        self.filter.set_type(_type);
                    }
                }
            }
            RustableCommand::Quack => {
                self.settings.quack = !self.settings.quack;
            }
        }
    }

    /// Adds a channel to the list of adjacent nodes' channels
    fn add_channel(&mut self, id: NodeId, sender: Sender<Packet>) -> Result<&str, &str> {
        if self.packet_send.contains_key(&id) {
            return Err("Channel to this NodeId already exists")
        }

        self.packet_send.insert(id, sender);
        Ok("Channel added successfully")
    }

    /// Removes the channel to the specified adjacent node
    fn remove_channel(&mut self, id: NodeId) -> Result<&str, &str> {
        if ! self.packet_send.contains_key(&id) {
            return Err("No adjacent node with specified NodeId")
        }

        self.packet_send.remove(&id);
        self.filter.remove(id);

        Ok("Channel removed successfully")
    }

    /// Sends a nack with specified type back to where the packet came from
    fn send_nack(&self, from: NodeId, nacked_packet: &Packet, fragment_index: u64, nack_type: NackType) {
        //sleeps
        if !self.settings.sleep_duration.is_zero() {
            thread::sleep(self.settings.sleep_duration);
        }

        let mut rev_header = nacked_packet.routing_header.get_reversed();
        rev_header.hop_index += 1;

        let res: Result<(), SendError<Packet>> = self.packet_send.get(&from).unwrap().send(
            Packet {
                pack_type: PacketType::Nack(Nack{
                    fragment_index,
                    nack_type,
                }),
                routing_header: rev_header,
                session_id: nacked_packet.session_id,
            }
        );

        if res.is_err() {
            panic!("{}", res.err().unwrap())
        }
    }

    /// Sends a nack with specified type back to where the packet came from, through the controller
    fn send_nack_through_controller(&self, nacked_packet: &Packet, fragment_index: u64, nack_type: NackType) {
        //sleeps
        if !self.settings.sleep_duration.is_zero() {
            thread::sleep(self.settings.sleep_duration);
        }

        let res: Result<(), SendError<DroneEvent>> = self.controller_send.send(DroneEvent::ControllerShortcut(
            Packet {
                pack_type: PacketType::Nack(Nack{
                    fragment_index,
                    nack_type
                }),
                routing_header: nacked_packet.routing_header.get_reversed(),
                session_id: nacked_packet.session_id
            }
        ));

        if res.is_err() {
            panic!("{}", res.err().unwrap())
        }
    }
}