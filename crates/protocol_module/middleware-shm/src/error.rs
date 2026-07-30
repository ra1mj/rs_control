//! Typed failures owned by the shared-memory boundary.

use std::{io, path::PathBuf};

use thiserror::Error;

/// Invalid startup contract supplied before a mapping is opened.
#[derive(Clone, Copy, Debug, Eq, Error, PartialEq)]
pub enum ContractError {
    /// Channel names must contain at least one byte.
    #[error("shared-memory channel name must not be empty")]
    EmptyName,
    /// Channel names have a fixed storage limit.
    #[error("shared-memory channel name is {actual} bytes; maximum is {maximum}")]
    NameTooLong {
        /// Supplied UTF-8 byte count.
        actual: usize,
        /// Maximum accepted byte count.
        maximum: usize,
    },
    /// Leading dots make names hidden/ambiguous in the private namespace.
    #[error("shared-memory channel name must not start with '.'")]
    LeadingDot,
    /// Consecutive dots are rejected rather than normalized.
    #[error("shared-memory channel name must not contain '..'")]
    Traversal,
    /// Only the strict filename-safe grammar is accepted.
    #[error("invalid shared-memory channel byte 0x{byte:02x} at index {index}")]
    InvalidNameByte {
        /// Zero-based byte index.
        index: usize,
        /// Rejected byte.
        byte: u8,
    },
    /// A channel must have bounded nonzero storage.
    #[error("shared-memory channel capacity must be nonzero")]
    ZeroCapacity,
    /// The reviewed layout has a fixed capacity ceiling.
    #[error("shared-memory channel capacity {actual} exceeds maximum {maximum}")]
    CapacityTooLarge {
        /// Supplied slot count.
        actual: u32,
        /// Maximum reviewed slot count.
        maximum: u32,
    },
    /// All-zero schema identity is reserved as unspecified.
    #[error("shared-memory payload schema ID must be nonzero")]
    ZeroSchemaId,
    /// Zero schema version is reserved as unspecified.
    #[error("shared-memory payload schema version must be nonzero")]
    ZeroSchemaVersion,
    /// Zero-sized values have no stable slot representation.
    #[error("zero-sized shared-memory payloads are unsupported")]
    ZeroSizedPayload,
    /// Payloads are intentionally bounded for the RT local path.
    #[error("shared-memory payload is {actual} bytes; maximum is {maximum}")]
    PayloadTooLarge {
        /// Supplied representation size.
        actual: usize,
        /// Maximum reviewed representation size.
        maximum: usize,
    },
    /// Layout v1 supports at most one cache line of alignment.
    #[error("shared-memory payload alignment {actual} exceeds maximum {maximum}")]
    PayloadAlignmentTooLarge {
        /// Supplied representation alignment.
        actual: usize,
        /// Maximum reviewed alignment.
        maximum: usize,
    },
}

/// One exact layout identity dimension.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LayoutField {
    /// Initialization-ready magic.
    Magic,
    /// Layout schema version.
    Version,
    /// Byte-order tag.
    Endian,
    /// Required atomic width.
    AtomicWidth,
    /// Latest-value versus ordered-event kind.
    ChannelKind,
    /// Slot count.
    Capacity,
    /// Stable representation schema ID.
    SchemaId,
    /// Stable representation schema version.
    SchemaVersion,
    /// Representation byte size.
    PayloadSize,
    /// Representation alignment.
    PayloadAlignment,
    /// Per-slot stride.
    SlotStride,
    /// Slot-area offset.
    SlotsOffset,
    /// Version-array offset.
    VersionsOffset,
    /// Exact mapped/file length.
    TotalLength,
}

/// Invalid or incompatible layout-v1 description.
#[derive(Clone, Copy, Debug, Eq, Error, PartialEq)]
pub enum LayoutError {
    /// The payload/channel startup contract was invalid.
    #[error(transparent)]
    Contract(#[from] ContractError),
    /// Latest-value seqlock delivery needs a multi-slot window.
    #[error("latest-value shared-memory capacity must be at least {minimum}; got {actual}")]
    LatestCapacityTooSmall {
        /// Supplied capacity.
        actual: u32,
        /// Required minimum.
        minimum: u32,
    },
    /// Checked layout size/offset arithmetic exceeded `usize`.
    #[error("shared-memory layout arithmetic overflow")]
    ArithmeticOverflow,
    /// An observed file/header identity did not match the requested contract.
    #[error("incompatible shared-memory layout field: {field:?}")]
    Incompatible {
        /// First field that failed exact validation.
        field: LayoutField,
    },
}

/// Exclusive endpoint capability guarded by one kernel file lock.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EndpointRole {
    /// The single channel writer.
    Publisher,
    /// The single channel reader.
    Subscriber,
}

/// Startup, attachment or explicit cleanup failure.
#[derive(Debug, Error)]
pub enum ShmError {
    /// The private namespace must be anchored, not process-CWD relative.
    #[error("shared-memory namespace path must be absolute: {path:?}")]
    NamespaceNotAbsolute {
        /// Rejected root.
        path: PathBuf,
    },
    /// The namespace root is not a real directory.
    #[error("shared-memory namespace is not a directory: {path:?}")]
    NamespaceNotDirectory {
        /// Rejected root.
        path: PathBuf,
    },
    /// The runtime root is required to be exactly owner-rwx.
    #[error("shared-memory namespace permissions must be 0o700; mode was {mode:#o}")]
    NamespacePermissions {
        /// Observed Unix permission bits.
        mode: u32,
    },
    /// A private runtime root must belong to the service UID.
    #[error("shared-memory namespace owner UID {observed} differs from process UID {expected}")]
    NamespaceOwner {
        /// Effective process UID derived from procfs ownership.
        expected: u32,
        /// Namespace directory owner.
        observed: u32,
    },
    /// Creation never truncates an existing region.
    #[error("shared-memory channel already exists: {name}")]
    AlreadyExists {
        /// Validated channel name.
        name: String,
    },
    /// Attach/removal requires a previously created region.
    #[error("shared-memory channel does not exist: {name}")]
    NotFound {
        /// Validated channel name.
        name: String,
    },
    /// A live mapping prevents initialization/removal.
    #[error("shared-memory region is in use: {name}")]
    RegionBusy {
        /// Validated channel name.
        name: String,
    },
    /// A live process already owns this endpoint role.
    #[error("shared-memory {role:?} endpoint is occupied: {name}")]
    EndpointOccupied {
        /// Validated channel name.
        name: String,
        /// Contended endpoint.
        role: EndpointRole,
    },
    /// The file length must equal the fully checked layout.
    #[error("shared-memory region length mismatch: expected {expected}, observed {observed}")]
    FileLength {
        /// Expected bytes.
        expected: u64,
        /// Metadata bytes.
        observed: u64,
    },
    /// Namespace/file metadata was not the expected Unix object type.
    #[error("shared-memory object is not a regular file: {path:?}")]
    NotRegularFile {
        /// Rejected object.
        path: PathBuf,
    },
    /// Memory locking was mandatory but unavailable.
    #[error("required shared-memory mlock failed")]
    MemoryLock {
        /// Kernel failure.
        #[source]
        source: io::Error,
    },
    /// Pure contract/layout rejection.
    #[error(transparent)]
    Layout(#[from] LayoutError),
    /// Filesystem, locking or mapping operation failed.
    #[error("shared-memory {operation} failed")]
    Io {
        /// Stable operation category.
        operation: &'static str,
        /// OS failure.
        #[source]
        source: io::Error,
    },
}
