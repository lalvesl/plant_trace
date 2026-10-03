//! plant-trace: acquisition, experiment orchestration and identification for
//! the steam-turbine characterisation rig.
//!
//! The CLI is a thin shell over these modules; they are a library so that the
//! integration tests can run the simulator in-process and drive it with the
//! same code the command line uses.
#![deny(missing_docs)]

pub mod analysis;
pub mod bode;
pub mod check;
pub mod csvout;
pub mod daq;
pub mod experiment;
#[cfg(feature = "gui")]
pub mod gui;
pub mod link;
pub mod runner;
pub mod sim;
