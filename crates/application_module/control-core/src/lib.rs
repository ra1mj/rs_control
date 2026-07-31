//! Robot-level controller contract proving the application-to-protocol dependency edge.

mod lease;

pub use lease::*;

use message_model::Header;

/// Input paired with protocol-module timing and quality metadata.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TimedInput<T> {
    /// Protocol-module metadata.
    pub header: Header,
    /// Algorithm input.
    pub value: T,
}

/// Synchronous robot-level controller called by a qualified control thread.
pub trait Controller {
    /// Input obtained through a protocol-module port.
    type Input;
    /// Output returned to a protocol-module command port.
    type Output;
    /// Controller-owned typed error.
    type Error;

    /// Computes one bounded controller step without accessing a hardware driver.
    fn step(&mut self, input: TimedInput<Self::Input>) -> Result<Self::Output, Self::Error>;
}
