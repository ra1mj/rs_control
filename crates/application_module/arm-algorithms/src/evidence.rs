//! Collision-evidence typestate shared by plans, grasps and admission.

/// Marker for an artifact not checked by a qualified collision provider.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CollisionUnchecked {
    _private: (),
}

impl CollisionUnchecked {
    pub(crate) const fn new() -> Self {
        Self { _private: () }
    }
}

/// Marker accepted by a future physical executor.
///
/// This delivery intentionally exposes no constructor.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CollisionChecked {
    _private: (),
}
