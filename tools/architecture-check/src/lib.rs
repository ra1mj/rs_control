//! Deterministic compile-time layer-boundary validation over Cargo metadata.

use cargo_metadata::{DependencyKind as CargoDependencyKind, MetadataCommand};
use serde::Deserialize;
use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    path::Path,
};
use thiserror::Error;

/// Current architecture policy schema.
pub const POLICY_VERSION: u32 = 1;

/// Machine-readable workspace architecture policy.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
pub struct Policy {
    /// Policy schema version.
    pub version: u32,
    /// Executables allowed to compose more than one production layer.
    #[serde(default)]
    pub composition_roots: BTreeSet<String>,
    /// Rules indexed by class name.
    pub classes: BTreeMap<String, ClassPolicy>,
    /// Workspace package classification indexed by Cargo package name.
    pub members: BTreeMap<String, String>,
}

/// Rules for one package class.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
pub struct ClassPolicy {
    /// Classes this class may import.
    #[serde(default)]
    pub allowed_dependencies: BTreeSet<String>,
    /// Repository-relative directory prefixes valid for this class.
    pub path_prefixes: Vec<String>,
    /// Whether packages in this class may be composition roots.
    #[serde(default)]
    pub composition_root: bool,
}

/// Workspace graph independent of Cargo's serialized representation.
#[derive(Clone, Debug, Default, Deserialize, Eq, PartialEq)]
pub struct WorkspaceGraph {
    /// Workspace packages.
    #[serde(default)]
    pub packages: Vec<GraphPackage>,
    /// Internal dependency declarations.
    #[serde(default)]
    pub dependencies: Vec<GraphDependency>,
}

/// One classified workspace package.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
pub struct GraphPackage {
    /// Cargo package name.
    pub name: String,
    /// Repository-relative package directory.
    pub path: String,
}

/// One internal Cargo dependency declaration.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
pub struct GraphDependency {
    /// Importing package.
    pub source: String,
    /// Imported package.
    pub target: String,
    /// Cargo dependency kind.
    pub kind: EdgeKind,
}

/// Cargo dependency kind relevant to architecture rules.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum EdgeKind {
    /// Normal production dependency.
    Normal,
    /// Build-script dependency.
    Build,
    /// Test/example/benchmark-only dependency.
    Development,
}

/// Loads and parses a policy file.
pub fn load_policy(path: &Path) -> Result<Policy, ArchitectureError> {
    let text = fs::read_to_string(path).map_err(|source| ArchitectureError::ReadPolicy {
        path: path.to_owned(),
        source,
    })?;
    toml::from_str(&text).map_err(ArchitectureError::ParsePolicy)
}

/// Extracts the internal workspace graph from `cargo metadata`.
pub fn graph_from_cargo(workspace_root: &Path) -> Result<WorkspaceGraph, ArchitectureError> {
    let metadata = MetadataCommand::new()
        .manifest_path(workspace_root.join("Cargo.toml"))
        .no_deps()
        .exec()?;

    let workspace_packages: Vec<_> = metadata
        .packages
        .iter()
        .filter(|package| {
            metadata
                .workspace_members
                .iter()
                .any(|id| id == &package.id)
        })
        .collect();
    let package_names: BTreeSet<&str> = workspace_packages
        .iter()
        .map(|package| package.name.as_str())
        .collect();
    let metadata_root = metadata.workspace_root.as_std_path();

    let mut graph = WorkspaceGraph::default();
    for package in workspace_packages {
        let package_dir = package.manifest_path.parent().ok_or_else(|| {
            ArchitectureError::InvalidManifestPath(package.manifest_path.to_string())
        })?;
        let relative = package_dir.strip_prefix(metadata_root).map_err(|_| {
            ArchitectureError::InvalidManifestPath(package.manifest_path.to_string())
        })?;
        graph.packages.push(GraphPackage {
            name: package.name.to_string(),
            path: relative.as_str().replace('\\', "/"),
        });

        for dependency in &package.dependencies {
            if package_names.contains(dependency.name.as_str()) {
                graph.dependencies.push(GraphDependency {
                    source: package.name.to_string(),
                    target: dependency.name.to_string(),
                    kind: match dependency.kind {
                        CargoDependencyKind::Normal => EdgeKind::Normal,
                        CargoDependencyKind::Build => EdgeKind::Build,
                        CargoDependencyKind::Development => EdgeKind::Development,
                        _ => EdgeKind::Normal,
                    },
                });
            }
        }
    }
    graph
        .packages
        .sort_by(|left, right| left.name.cmp(&right.name));
    graph.dependencies.sort_by(|left, right| {
        (&left.source, &left.target, edge_rank(left.kind)).cmp(&(
            &right.source,
            &right.target,
            edge_rank(right.kind),
        ))
    });
    Ok(graph)
}

const fn edge_rank(kind: EdgeKind) -> u8 {
    match kind {
        EdgeKind::Normal => 0,
        EdgeKind::Build => 1,
        EdgeKind::Development => 2,
    }
}

/// Validates a workspace graph and returns every deterministic rule violation.
#[must_use]
pub fn validate(policy: &Policy, graph: &WorkspaceGraph) -> Vec<RuleViolation> {
    let mut violations = Vec::new();
    if policy.version != POLICY_VERSION {
        violations.push(RuleViolation::UnsupportedPolicyVersion {
            actual: policy.version,
            supported: POLICY_VERSION,
        });
    }

    for (class_name, class) in &policy.classes {
        for target in &class.allowed_dependencies {
            if !policy.classes.contains_key(target) {
                violations.push(RuleViolation::UnknownAllowedClass {
                    class: class_name.clone(),
                    target: target.clone(),
                });
            }
        }
    }

    let packages: BTreeMap<&str, &GraphPackage> = graph
        .packages
        .iter()
        .map(|package| (package.name.as_str(), package))
        .collect();

    for package in &graph.packages {
        let Some(class_name) = policy.members.get(&package.name) else {
            violations.push(RuleViolation::UnclassifiedPackage {
                package: package.name.clone(),
            });
            continue;
        };
        let Some(class) = policy.classes.get(class_name) else {
            violations.push(RuleViolation::UnknownPackageClass {
                package: package.name.clone(),
                class: class_name.clone(),
            });
            continue;
        };
        let normalized_path = format!("{}/", package.path.trim_end_matches('/'));
        if !class
            .path_prefixes
            .iter()
            .any(|prefix| normalized_path.starts_with(prefix))
        {
            violations.push(RuleViolation::PathClassMismatch {
                package: package.name.clone(),
                class: class_name.clone(),
                path: package.path.clone(),
            });
        }
        if class.composition_root && !policy.composition_roots.contains(&package.name) {
            violations.push(RuleViolation::CompositionRootNotAllowlisted {
                package: package.name.clone(),
            });
        }
    }

    for package_name in policy.members.keys() {
        if !packages.contains_key(package_name.as_str()) {
            violations.push(RuleViolation::StaleClassification {
                package: package_name.clone(),
            });
        }
    }

    for root in &policy.composition_roots {
        match policy
            .members
            .get(root)
            .and_then(|class_name| policy.classes.get(class_name))
        {
            Some(class) if class.composition_root => {}
            _ => violations.push(RuleViolation::InvalidCompositionRoot {
                package: root.clone(),
            }),
        }
    }

    for dependency in &graph.dependencies {
        let Some(source_package) = packages.get(dependency.source.as_str()) else {
            violations.push(RuleViolation::UnknownEdgeEndpoint {
                package: dependency.source.clone(),
            });
            continue;
        };
        let Some(target_package) = packages.get(dependency.target.as_str()) else {
            violations.push(RuleViolation::UnknownEdgeEndpoint {
                package: dependency.target.clone(),
            });
            continue;
        };
        let Some(source_class_name) = policy.members.get(&source_package.name) else {
            continue;
        };
        let Some(target_class_name) = policy.members.get(&target_package.name) else {
            continue;
        };
        let Some(source_class) = policy.classes.get(source_class_name) else {
            continue;
        };

        if target_class_name == "test-support" {
            if dependency.kind != EdgeKind::Development {
                violations.push(RuleViolation::ProductionTestSupportDependency {
                    source_package: dependency.source.clone(),
                    target_package: dependency.target.clone(),
                    kind: dependency.kind,
                });
            }
            continue;
        }

        if !source_class
            .allowed_dependencies
            .contains(target_class_name)
        {
            violations.push(RuleViolation::ForbiddenDependency {
                source_package: dependency.source.clone(),
                source_class: source_class_name.clone(),
                target_package: dependency.target.clone(),
                target_class: target_class_name.clone(),
                kind: dependency.kind,
            });
        }
    }

    violations.sort_by_key(ToString::to_string);
    violations.dedup();
    violations
}

/// Architecture policy violation.
#[derive(Clone, Debug, Eq, Error, PartialEq)]
pub enum RuleViolation {
    /// Policy schema cannot be interpreted by this checker.
    #[error("policy version {actual} is unsupported; expected {supported}")]
    UnsupportedPolicyVersion {
        /// Observed version.
        actual: u32,
        /// Supported version.
        supported: u32,
    },
    /// A class rule names a nonexistent target class.
    #[error("class {class} allows unknown class {target}")]
    UnknownAllowedClass {
        /// Source class.
        class: String,
        /// Unknown target class.
        target: String,
    },
    /// Workspace package has no explicit classification.
    #[error("workspace package {package} is not classified")]
    UnclassifiedPackage {
        /// Package name.
        package: String,
    },
    /// Package classification references a nonexistent class.
    #[error("workspace package {package} uses unknown class {class}")]
    UnknownPackageClass {
        /// Package name.
        package: String,
        /// Unknown class.
        class: String,
    },
    /// Package path disagrees with its declared class.
    #[error("workspace package {package} at {path} does not match class {class} path policy")]
    PathClassMismatch {
        /// Package name.
        package: String,
        /// Declared class.
        class: String,
        /// Observed path.
        path: String,
    },
    /// Policy contains a member no longer present in Cargo metadata.
    #[error("policy classifies missing workspace package {package}")]
    StaleClassification {
        /// Package name.
        package: String,
    },
    /// Composition-root class package is not explicitly allowlisted.
    #[error("composition-root package {package} is not allowlisted")]
    CompositionRootNotAllowlisted {
        /// Package name.
        package: String,
    },
    /// Composition-root allowlist points at a missing/non-root package.
    #[error("composition-root allowlist entry {package} is invalid")]
    InvalidCompositionRoot {
        /// Package name.
        package: String,
    },
    /// Dependency references a package absent from the supplied graph fixture.
    #[error("dependency edge references unknown package {package}")]
    UnknownEdgeEndpoint {
        /// Package name.
        package: String,
    },
    /// A normal/build dependency attempts to import test-only support.
    #[error("{source_package} has {kind:?} dependency on dev-only {target_package}")]
    ProductionTestSupportDependency {
        /// Importing package.
        source_package: String,
        /// Test-support package.
        target_package: String,
        /// Rejected dependency kind.
        kind: EdgeKind,
    },
    /// Dependency direction violates the class graph.
    #[error(
        "forbidden {kind:?} dependency {source_package} ({source_class}) -> {target_package} ({target_class})"
    )]
    ForbiddenDependency {
        /// Importing package.
        source_package: String,
        /// Importing package class.
        source_class: String,
        /// Imported package.
        target_package: String,
        /// Imported package class.
        target_class: String,
        /// Cargo dependency kind.
        kind: EdgeKind,
    },
}

/// Policy I/O, parsing or Cargo metadata failure.
#[derive(Debug, Error)]
pub enum ArchitectureError {
    /// Policy file could not be read.
    #[error("failed to read architecture policy {path}: {source}")]
    ReadPolicy {
        /// Policy path.
        path: std::path::PathBuf,
        /// Filesystem error.
        source: std::io::Error,
    },
    /// Policy TOML was invalid.
    #[error("invalid architecture policy TOML: {0}")]
    ParsePolicy(#[source] toml::de::Error),
    /// Cargo metadata failed.
    #[error("cargo metadata failed: {0}")]
    Metadata(#[from] cargo_metadata::Error),
    /// A workspace manifest path could not be made repository-relative.
    #[error("invalid workspace manifest path {0}")]
    InvalidManifestPath(String),
}

#[cfg(test)]
mod tests {
    use super::{EdgeKind, Policy, RuleViolation, WorkspaceGraph, validate};

    fn policy() -> Policy {
        toml::from_str(include_str!("../../../architecture.toml")).unwrap()
    }

    fn fixture(input: &str) -> WorkspaceGraph {
        toml::from_str(input).unwrap()
    }

    fn contains_forbidden(
        violations: &[RuleViolation],
        source_class: &str,
        target_class: &str,
    ) -> bool {
        violations.iter().any(|violation| {
            matches!(
                violation,
                RuleViolation::ForbiddenDependency {
                    source_class: source,
                    target_class: target,
                    ..
                } if source == source_class && target == target_class
            )
        })
    }

    #[test]
    fn rejects_application_module_to_hardware_module_fixture() {
        let graph = fixture(include_str!(
            "../tests/fixtures/application_module_to_hardware_module.toml"
        ));
        assert!(contains_forbidden(
            &validate(&policy(), &graph),
            "application_module",
            "hardware_module"
        ));
    }

    #[test]
    fn rejects_hardware_module_to_protocol_module_fixture() {
        let graph = fixture(include_str!(
            "../tests/fixtures/hardware_module_to_protocol_module.toml"
        ));
        assert!(contains_forbidden(
            &validate(&policy(), &graph),
            "hardware_module",
            "protocol_module"
        ));
    }

    #[test]
    fn rejects_protocol_module_to_application_module_fixture() {
        let graph = fixture(include_str!(
            "../tests/fixtures/protocol_module_to_application_module.toml"
        ));
        assert!(contains_forbidden(
            &validate(&policy(), &graph),
            "protocol_module",
            "application_module"
        ));
    }

    #[test]
    fn rejects_production_test_support_fixture() {
        let graph = fixture(include_str!(
            "../tests/fixtures/production_test_support.toml"
        ));
        assert!(validate(&policy(), &graph).iter().any(|violation| {
            matches!(
                violation,
                RuleViolation::ProductionTestSupportDependency {
                    kind: EdgeKind::Normal,
                    ..
                }
            )
        }));
    }
}
