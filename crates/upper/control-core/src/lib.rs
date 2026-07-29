//! Robot-level controller contract proving the upper-to-middle dependency edge.

use message_model::Header;

/// Input paired with middle-layer timing and quality metadata.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TimedInput<T> {
    /// Middle-layer metadata.
    pub header: Header,
    /// Algorithm input.
    pub value: T,
}

/// Synchronous robot-level controller called by a qualified control thread.
pub trait Controller {
    /// Input obtained through a middle-layer port.
    type Input;
    /// Output returned to a middle-layer command port.
    type Output;
    /// Controller-owned typed error.
    type Error;

    /// Computes one bounded controller step without accessing a lower driver.
    fn step(&mut self, input: TimedInput<Self::Input>) -> Result<Self::Output, Self::Error>;
}
