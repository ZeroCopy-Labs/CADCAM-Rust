//! Session versions of formal inputs and the derived CAD/CAM snapshots.

use crate::gcode::GCodeProfile;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Inputs {
    pub document: u64,
    pub geometry: u64,
    pub properties: u64,
    pub config: u64,
    pub plan: u64,
}

#[derive(Default)]
pub struct DerivedState {
    pub inputs: Inputs,
    pub display: Option<(u64, u64)>,
    pub machining: Option<(u64, u64)>,
    pub topology: Option<u64>,
    pub analysis: Option<u64>,
    pub trajectory: Option<Inputs>,
    profile: Option<GCodeProfile>,
}

impl DerivedState {
    pub fn document_changed(&mut self, revision: u64, geometry: bool) {
        self.inputs.document = revision;
        self.inputs.properties += 1;
        if geometry {
            self.inputs.geometry += 1;
            self.inputs.plan += 1;
        }
        self.trajectory = None;
    }

    pub fn geometry_ready(&mut self) {
        self.topology = Some(self.inputs.geometry);
        self.analysis = Some(self.inputs.geometry);
        self.properties_ready();
    }

    pub fn properties_ready(&mut self) {
        let version = (self.inputs.geometry, self.inputs.properties);
        self.display = Some(version);
        self.machining = Some(version);
    }

    pub fn observe_profile(&mut self, profile: &GCodeProfile) {
        if self.profile.as_ref() != Some(profile) {
            self.inputs.config += 1;
            self.profile = Some(profile.clone());
            self.trajectory = None;
        }
    }

    pub fn plan_changed(&mut self) {
        self.inputs.plan += 1;
        self.trajectory = None;
    }

    pub fn sources_current(&self) -> bool {
        let version = Some((self.inputs.geometry, self.inputs.properties));
        self.display == version
            && self.machining == version
            && self.topology == Some(self.inputs.geometry)
            && self.analysis == Some(self.inputs.geometry)
    }

    pub fn trajectory_current(&self) -> bool {
        self.sources_current() && self.trajectory == Some(self.inputs)
    }
}
