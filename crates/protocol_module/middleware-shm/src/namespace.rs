//! Private namespace, fixed files and kernel-lock lifecycle authority.

use std::{
    fs::{self, DirBuilder, File, OpenOptions, TryLockError},
    io,
    os::unix::fs::{DirBuilderExt, MetadataExt, OpenOptionsExt, PermissionsExt},
    path::{Path, PathBuf},
};

use crate::{
    ChannelName, EndpointRole, ShmChannelConfig, ShmError, ShmPayload,
    layout::{ChannelKind, LayoutDescriptor},
    mapping::MappedRegion,
};

/// Owner-only runtime directory containing fixed shared-memory channel files.
#[derive(Clone, Debug)]
pub struct ShmNamespace {
    root: PathBuf,
}

pub(crate) struct AttachedRegion {
    pub(crate) region: MappedRegion,
    pub(crate) endpoint_lock: File,
    pub(crate) peer_lock_path: PathBuf,
}

impl ShmNamespace {
    /// Creates or validates an absolute mode-0700 namespace directory.
    pub fn new(root: impl AsRef<Path>) -> Result<Self, ShmError> {
        let root = root.as_ref();
        if !root.is_absolute() {
            return Err(ShmError::NamespaceNotAbsolute {
                path: root.to_path_buf(),
            });
        }
        match fs::symlink_metadata(root) {
            Ok(_) => {}
            Err(error) if error.kind() == io::ErrorKind::NotFound => {
                let mut builder = DirBuilder::new();
                builder.mode(0o700);
                builder.create(root).map_err(|source| ShmError::Io {
                    operation: "create namespace",
                    source,
                })?;
                fs::set_permissions(root, fs::Permissions::from_mode(0o700)).map_err(|source| {
                    ShmError::Io {
                        operation: "secure namespace permissions",
                        source,
                    }
                })?;
            }
            Err(source) => {
                return Err(ShmError::Io {
                    operation: "inspect namespace",
                    source,
                });
            }
        }
        let metadata = fs::symlink_metadata(root).map_err(|source| ShmError::Io {
            operation: "inspect namespace",
            source,
        })?;
        if !metadata.file_type().is_dir() || metadata.file_type().is_symlink() {
            return Err(ShmError::NamespaceNotDirectory {
                path: root.to_path_buf(),
            });
        }
        let mode = metadata.mode() & 0o777;
        if mode != 0o700 {
            return Err(ShmError::NamespacePermissions { mode });
        }
        let process_uid = fs::metadata("/proc/self")
            .map_err(|source| ShmError::Io {
                operation: "inspect process identity",
                source,
            })?
            .uid();
        if metadata.uid() != process_uid {
            return Err(ShmError::NamespaceOwner {
                expected: process_uid,
                observed: metadata.uid(),
            });
        }
        Ok(Self {
            root: root.to_path_buf(),
        })
    }

    /// Creates a latest-value layout without claiming either endpoint.
    pub fn create_latest<R: ShmPayload>(
        &self,
        name: ChannelName,
        config: ShmChannelConfig,
    ) -> Result<(), ShmError> {
        self.create::<R>(ChannelKind::Latest, name, config)
    }

    /// Creates an ordered-event layout without claiming either endpoint.
    pub fn create_ordered<R: ShmPayload>(
        &self,
        name: ChannelName,
        config: ShmChannelConfig,
    ) -> Result<(), ShmError> {
        self.create::<R>(ChannelKind::Ordered, name, config)
    }

    /// Removes only a channel proven unused by region and endpoint locks.
    pub fn remove_if_unused(&self, name: ChannelName) -> Result<(), ShmError> {
        let paths = ChannelPaths::new(&self.root, name);
        if !paths.region.exists() {
            return Err(ShmError::NotFound {
                name: name.to_string(),
            });
        }
        let region_lock = open_existing_regular(&paths.region_lock, "open region lock")?;
        try_lock_region_exclusive(&region_lock, name)?;
        let publisher_lock = open_existing_regular(&paths.publisher_lock, "open publisher lock")?;
        try_lock_endpoint(&publisher_lock, name, EndpointRole::Publisher)?;
        let subscriber_lock =
            open_existing_regular(&paths.subscriber_lock, "open subscriber lock")?;
        try_lock_endpoint(&subscriber_lock, name, EndpointRole::Subscriber)?;
        fs::remove_file(&paths.region).map_err(|source| ShmError::Io {
            operation: "remove unused region",
            source,
        })?;
        Ok(())
    }

    fn create<R: ShmPayload>(
        &self,
        kind: ChannelKind,
        name: ChannelName,
        config: ShmChannelConfig,
    ) -> Result<(), ShmError> {
        let layout = LayoutDescriptor::for_payload::<R>(kind, config)?;
        let paths = ChannelPaths::new(&self.root, name);
        let region_lock = open_lock(&paths.region_lock, "open region lock")?;
        try_lock_region_exclusive(&region_lock, name)?;
        let publisher_lock = open_lock(&paths.publisher_lock, "open publisher lock")?;
        try_lock_endpoint(&publisher_lock, name, EndpointRole::Publisher)?;
        let subscriber_lock = open_lock(&paths.subscriber_lock, "open subscriber lock")?;
        try_lock_endpoint(&subscriber_lock, name, EndpointRole::Subscriber)?;

        let region_file = match OpenOptions::new()
            .read(true)
            .write(true)
            .create_new(true)
            .mode(0o600)
            .open(&paths.region)
        {
            Ok(file) => file,
            Err(error) if error.kind() == io::ErrorKind::AlreadyExists => {
                return Err(ShmError::AlreadyExists {
                    name: name.to_string(),
                });
            }
            Err(source) => {
                return Err(ShmError::Io {
                    operation: "create region",
                    source,
                });
            }
        };
        fs::set_permissions(&paths.region, fs::Permissions::from_mode(0o600)).map_err(
            |source| ShmError::Io {
                operation: "secure region permissions",
                source,
            },
        )?;
        region_file
            .set_len(layout.snapshot().total_length)
            .map_err(|source| ShmError::Io {
                operation: "size new region",
                source,
            })?;
        let region =
            MappedRegion::create(region_file, region_lock, layout, config.memory_policy())?;
        drop(region);
        Ok(())
    }

    pub(crate) fn attach<R: ShmPayload>(
        &self,
        kind: ChannelKind,
        name: ChannelName,
        config: ShmChannelConfig,
        role: EndpointRole,
    ) -> Result<AttachedRegion, ShmError> {
        let layout = LayoutDescriptor::for_payload::<R>(kind, config)?;
        let paths = ChannelPaths::new(&self.root, name);
        if !paths.region.exists() {
            return Err(ShmError::NotFound {
                name: name.to_string(),
            });
        }
        let region_lock = open_existing_regular(&paths.region_lock, "open region lock")?;
        try_lock_region_shared(&region_lock, name)?;
        let region_file = open_existing_regular(&paths.region, "open region")?;
        let observed = region_file
            .metadata()
            .map_err(|source| ShmError::Io {
                operation: "inspect region",
                source,
            })?
            .len();
        let expected = layout.snapshot().total_length;
        if observed != expected {
            return Err(ShmError::FileLength { expected, observed });
        }
        let region =
            MappedRegion::attach(region_file, region_lock, layout, config.memory_policy())?;
        let (endpoint_path, peer_lock_path) = match role {
            EndpointRole::Publisher => (&paths.publisher_lock, paths.subscriber_lock),
            EndpointRole::Subscriber => (&paths.subscriber_lock, paths.publisher_lock),
        };
        let endpoint_lock = open_existing_regular(endpoint_path, "open endpoint lock")?;
        if let Err(error) = try_lock_endpoint(&endpoint_lock, name, role) {
            region.record_attach_rejected();
            return Err(error);
        }
        match role {
            EndpointRole::Publisher => region.claim_publisher(),
            EndpointRole::Subscriber => region.claim_subscriber(),
        }
        Ok(AttachedRegion {
            region,
            endpoint_lock,
            peer_lock_path,
        })
    }
}

pub(crate) fn probe_endpoint(path: &Path) -> Result<Option<File>, ShmError> {
    let file = open_existing_regular(path, "open peer endpoint lock")?;
    match file.try_lock() {
        Ok(()) => Ok(Some(file)),
        Err(TryLockError::WouldBlock) => Ok(None),
        Err(TryLockError::Error(source)) => Err(ShmError::Io {
            operation: "probe peer endpoint lock",
            source,
        }),
    }
}

struct ChannelPaths {
    region: PathBuf,
    region_lock: PathBuf,
    publisher_lock: PathBuf,
    subscriber_lock: PathBuf,
}

impl ChannelPaths {
    fn new(root: &Path, name: ChannelName) -> Self {
        let name = name.as_str();
        Self {
            region: root.join(format!("{name}.region")),
            region_lock: root.join(format!("{name}.region.lock")),
            publisher_lock: root.join(format!("{name}.publisher.lock")),
            subscriber_lock: root.join(format!("{name}.subscriber.lock")),
        }
    }
}

fn open_lock(path: &Path, operation: &'static str) -> Result<File, ShmError> {
    let file = OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(false)
        .mode(0o600)
        .open(path)
        .map_err(|source| ShmError::Io { operation, source })?;
    fs::set_permissions(path, fs::Permissions::from_mode(0o600)).map_err(|source| {
        ShmError::Io {
            operation: "secure lock permissions",
            source,
        }
    })?;
    require_regular(&file, path)?;
    Ok(file)
}

fn open_existing_regular(path: &Path, operation: &'static str) -> Result<File, ShmError> {
    let file = OpenOptions::new()
        .read(true)
        .write(true)
        .open(path)
        .map_err(|source| ShmError::Io { operation, source })?;
    require_regular(&file, path)?;
    Ok(file)
}

fn require_regular(file: &File, path: &Path) -> Result<(), ShmError> {
    let metadata = file.metadata().map_err(|source| ShmError::Io {
        operation: "inspect shared-memory object",
        source,
    })?;
    if !metadata.file_type().is_file() {
        return Err(ShmError::NotRegularFile {
            path: path.to_path_buf(),
        });
    }
    Ok(())
}

fn try_lock_region_exclusive(file: &File, name: ChannelName) -> Result<(), ShmError> {
    match file.try_lock() {
        Ok(()) => Ok(()),
        Err(TryLockError::WouldBlock) => Err(ShmError::RegionBusy {
            name: name.to_string(),
        }),
        Err(TryLockError::Error(source)) => Err(ShmError::Io {
            operation: "lock region exclusively",
            source,
        }),
    }
}

fn try_lock_region_shared(file: &File, name: ChannelName) -> Result<(), ShmError> {
    match file.try_lock_shared() {
        Ok(()) => Ok(()),
        Err(TryLockError::WouldBlock) => Err(ShmError::RegionBusy {
            name: name.to_string(),
        }),
        Err(TryLockError::Error(source)) => Err(ShmError::Io {
            operation: "lock region for mapping lifetime",
            source,
        }),
    }
}

fn try_lock_endpoint(file: &File, name: ChannelName, role: EndpointRole) -> Result<(), ShmError> {
    match file.try_lock() {
        Ok(()) => Ok(()),
        Err(TryLockError::WouldBlock) => Err(ShmError::EndpointOccupied {
            name: name.to_string(),
            role,
        }),
        Err(TryLockError::Error(source)) => Err(ShmError::Io {
            operation: "lock endpoint",
            source,
        }),
    }
}
