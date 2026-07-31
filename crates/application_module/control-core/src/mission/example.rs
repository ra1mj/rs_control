//! Representative validated automatic project.

use message_model::{ActionName, ResourceId};

use super::{
    CancelScopeId, GuardId, MAX_MISSION_CHILDREN, MissionBuildError, MissionBuilder,
    MissionChildren, MissionIr, MissionNode, MissionNodeId, MissionNodeKind, MissionNodeName,
    MissionRegistry, MissionResources, TypedAction,
};

struct ExampleGoal;
struct ExampleResult;

/// Builds the representative G4 automatic project.
///
/// The graph contains sequence, guard branch, parallel execution, timeout,
/// retry, bounded loop, cancellation scope and a named subflow call.
pub fn representative_automatic_project() -> Result<MissionIr, MissionBuildError> {
    let mut registry = MissionRegistry::new();
    for action in ["navigate", "safe_wait", "inspect", "align", "dock"] {
        registry.register_action(ActionName::parse(action).expect("static action identity"))?;
    }
    registry.register_guard(GuardId::parse("route_clear").expect("static guard identity"))?;

    let mut builder = MissionBuilder::new(MissionNodeId::new(0));
    for node in [
        MissionNode::new(
            MissionNodeId::new(0),
            node_name("automatic_project"),
            MissionNodeKind::Sequence(children(&[1, 2, 3])?),
        ),
        MissionNode::new(
            MissionNodeId::new(1),
            node_name("route_branch"),
            MissionNodeKind::Guard {
                guard_id: GuardId::parse("route_clear").expect("static guard identity"),
                if_true: MissionNodeId::new(4),
                if_false: MissionNodeId::new(5),
            },
        ),
        MissionNode::new(
            MissionNodeId::new(2),
            node_name("parallel_observe_align"),
            MissionNodeKind::ParallelAll(children(&[7, 8])?),
        ),
        MissionNode::new(
            MissionNodeId::new(3),
            node_name("docking_cancel_scope"),
            MissionNodeKind::CancelScope {
                scope_id: CancelScopeId::parse("docking").expect("static scope identity"),
                child: MissionNodeId::new(10),
            },
        ),
        MissionNode::new(
            MissionNodeId::new(4),
            node_name("retry_navigation"),
            MissionNodeKind::Retry {
                child: MissionNodeId::new(6),
                max_attempts: Some(3),
            },
        ),
        action_node(5, "safe_wait", "chassis", 500_000_000)?,
        action_node(6, "navigate", "chassis", 5_000_000_000)?,
        action_node(7, "inspect", "camera", 2_000_000_000)?,
        MissionNode::new(
            MissionNodeId::new(8),
            node_name("align_timeout"),
            MissionNodeKind::Timeout {
                child: MissionNodeId::new(9),
                timeout_ns: 3_000_000_000,
            },
        ),
        action_node(9, "align", "arm", 2_000_000_000)?,
        MissionNode::new(
            MissionNodeId::new(10),
            node_name("bounded_docking_loop"),
            MissionNodeKind::Loop {
                child: MissionNodeId::new(11),
                max_iterations: Some(2),
            },
        ),
        MissionNode::new(
            MissionNodeId::new(11),
            node_name("dock_subflow"),
            MissionNodeKind::Subflow {
                child: MissionNodeId::new(12),
            },
        ),
        action_node(12, "dock", "chassis", 4_000_000_000)?,
    ] {
        builder.add_node(node)?;
    }
    builder.validate(&registry)
}

fn children(ids: &[u16]) -> Result<MissionChildren, MissionBuildError> {
    if ids.len() > MAX_MISSION_CHILDREN {
        return Err(MissionBuildError::ChildCapacityExceeded {
            maximum: MAX_MISSION_CHILDREN,
        });
    }
    let mut nodes = [MissionNodeId::new(0); MAX_MISSION_CHILDREN];
    for (index, id) in ids.iter().enumerate() {
        nodes[index] = MissionNodeId::new(*id);
    }
    MissionChildren::try_from_slice(&nodes[..ids.len()])
}

fn action_node(
    id: u16,
    action: &str,
    resource: &str,
    timeout_ns: u64,
) -> Result<MissionNode, MissionBuildError> {
    let resources = MissionResources::try_from_slice(&[
        ResourceId::parse(resource).expect("static resource identity")
    ])?;
    Ok(TypedAction::<ExampleGoal, ExampleResult>::new(
        ActionName::parse(action).expect("static action identity"),
        resources,
        timeout_ns,
    )
    .into_node(MissionNodeId::new(id), node_name(action)))
}

fn node_name(value: &str) -> MissionNodeName {
    MissionNodeName::parse(value).expect("static node identity")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn representative_project_contains_every_required_composition() {
        let ir = representative_automatic_project().unwrap();
        assert_eq!(ir.nodes().len(), 13);
        for required in [
            "automatic_project",
            "route_branch",
            "parallel_observe_align",
            "align_timeout",
            "retry_navigation",
            "bounded_docking_loop",
            "docking_cancel_scope",
            "dock_subflow",
        ] {
            assert!(
                ir.nodes()
                    .iter()
                    .any(|node| node.name().as_str() == required)
            );
        }
    }
}
