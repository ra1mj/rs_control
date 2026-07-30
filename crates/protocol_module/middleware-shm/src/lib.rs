//! Fixed-layout Linux shared-memory implementations of the middleware ports.

#[cfg(not(target_os = "linux"))]
compile_error!("middleware-shm layout v1 supports Linux only");
#[cfg(not(target_endian = "little"))]
compile_error!("middleware-shm layout v1 supports little-endian targets only");
#[cfg(not(all(target_has_atomic = "8", target_has_atomic = "64")))]
compile_error!("middleware-shm requires lock-free 8-bit and 64-bit atomics");

mod contract;
mod endpoint;
mod error;
mod layout;
mod mapping;
mod namespace;

pub use contract::{
    ChannelName, IdentityCodec, MemoryPolicy, ShmChannelConfig, ShmCodec, ShmPayload,
};
pub use endpoint::{
    ShmDiagnostics, ShmEventPublisher, ShmEventSubscriber, ShmLatestPublisher, ShmLatestSubscriber,
};
pub use error::{ContractError, EndpointRole, LayoutError, LayoutField, ShmError};
pub use namespace::ShmNamespace;
