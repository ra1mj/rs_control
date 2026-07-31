# Implementation plan

## 1. Shared contracts

- [x] Extend `chassis-model` with immutable body pose and local timed sample.
- [x] Add validated body command/rate limits and saturation evidence.
- [x] Add shared shortest-angle and body/world transform helpers.

## 2. Estimation

- [x] Add and classify `state-estimation-2d`.
- [x] Implement age, ordering, skew and quality gates.
- [x] Implement wheel+IMU integration and wheel-only/stop policy.
- [x] Implement optional/required external pose correction and evidence.

## 3. Control and trajectory

- [x] Add and classify `trajectory-se2`.
- [x] Implement bounded body velocity and heading controllers.
- [x] Implement fixed-capacity validated trajectory and interpolation.
- [x] Implement body-frame SE(2) trajectory tracker.

## 4. Verification and delivery

- [x] Cover all nominal/degraded/unavailable and invalid-time paths.
- [x] Cover saturation, yaw wrap, frame transform and trajectory completion.
- [x] Prove estimator/controller hot paths allocate zero memory.
- [x] Add executable H3 specification and parent evidence.
- [x] Run full CI, commit, archive and push without closing hardware gates.
