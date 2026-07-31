# Implementation plan

## 1. Shared model

- [x] Add `chassis-model` crate and architecture classification.
- [x] Reuse the canonical body `Twist2` and add validated SI geometry/state.
- [x] Add typed feasibility/saturation results and uniform scaling.
- [x] Add validated fixed-size planar least-squares support.
- [x] Add common morphology fixture vectors and model tests.

## 2. Morphology kinematics

- [x] Add mecanum-4 analytic forward/inverse maps.
- [x] Add generic omni maps and standard omni-3/omni-4 geometry.
- [x] Add swerve-4 maps, reversal, rate limiting and zero-angle hold.
- [x] Keep all forward/inverse paths bounded and allocation-free.

## 3. Verification

- [x] Test common unsaturated round trips for every morphology.
- [x] Test saturation, reversal, zero speed and rate limiting.
- [x] Test invalid, singular and near-singular construction.
- [x] Run formatting, warnings-denied Clippy, architecture and full CI.

## 4. Knowledge and delivery

- [x] Add chassis kinematics specification and backend index entry.
- [x] Mark parent H1/H2 host-side items complete; leave hardware gates open.
- [x] Commit implementation, archive the task and push `main`.
