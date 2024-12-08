use wg_2024::network::NodeId;
use crate::packets_filter::FilterType::{BlackList, WhiteList};

/// Filters packets of type [MsgFragment] based on the node they are coming from.
/// Effectively, this is a 100% drop probability on packets coming from specified adjacent drone IDs.
/// Can be set as a WhiteList (allows only packets from drones in the list).
/// or as a BlackList (allows packets from every drone not in the list).
/// Note that the filter is applied after the probability to drop the packet.
/// Default value is an empty BlackList (everything passes).
pub struct PacketFilter {
    list: Vec<NodeId>,
    filter_type: FilterType
}

pub enum FilterType {
    BlackList,
    WhiteList
}

impl Default for PacketFilter {
    fn default() -> Self {
        PacketFilter {
            list: vec![],
            filter_type: BlackList
        }
    }
}

impl PacketFilter {
    /// adds NodeId to the internal list
    pub fn add(&mut self, id: NodeId) {
        if !self.list.contains(&id) {
            self.list.push(id);
        }
    }

    /// removes a NodeId from the internal list
    pub fn remove(&mut self, id: NodeId) {
        let p = self.list.iter().position(|&x| x == id);
        if p.is_some() {
            self.list.remove(p.unwrap());
        }
    }

    /// returns true if a NodeId is allowed to send a packet, false if it isn't
    pub fn is_allowed(&self, id: NodeId) -> bool {
        match self.filter_type {
            BlackList => {
                !self.list.contains(&id)
            }
            WhiteList => {
                self.list.contains(&id)
            }
        }
    }

    /// clears the filter
    pub fn clear(&mut self) {
        self.list.clear();
    }

    /// set the internal list of node ids
    pub fn set(&mut self, list: Vec<NodeId>) {
        self.list = list.clone();
    }

    /// set the filter type
    pub fn set_type(&mut self, t: FilterType) {
        self.filter_type = t;
    }
}