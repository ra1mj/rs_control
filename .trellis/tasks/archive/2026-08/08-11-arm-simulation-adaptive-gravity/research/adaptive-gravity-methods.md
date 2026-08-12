# Adaptive Gravity Compensation: Literature and Industrial Practice

## Research question

Which adaptive gravity-compensation approach gives strong practical results
while matching methods that are already common in industrial robot products?

## Conclusion

The best fit for this project is **model-based gravity compensation plus
payload identification of mass and first moments**, implemented as:

1. a nominal rigid-body gravity model from the validated `ArmModel`;
2. a four-parameter tool-payload gravity regressor,
   `theta = [m, m*c_x, m*c_y, m*c_z]`;
3. bounded batch least squares over several eligible poses for initialization;
4. optional slow, gated, projected recursive least squares (RLS) for online
   refinement;
5. a separate residual path for friction or other unmodeled disturbances.

This combines the scope used by industrial load-identification workflows with
the online estimator family that the literature identifies as simple enough
for a real-time adaptive loop. Full-link inertial identification, recursive
total least squares with an IMU, nonlinear Kalman filtering, neural networks,
and a controller-wide adaptive law are useful alternatives in other contexts,
but are not the best MVP for gravity compensation.

## Why the four payload-gravity parameters are sufficient

For a rigid payload under gravity, joint torque is linear in payload mass and
first moments. Rotational inertia does not contribute to static gravity torque.
Therefore, identifying the payload inertia tensor cannot improve gravity-only
feed-forward evaluation, although it can improve high-acceleration inverse
dynamics.

The restricted parameter set also has better identifiability. Atkeson, An and
Hollerbach reported good load mass and center-of-mass estimates using a linear
Newton-Euler regressor and least squares, while load moments of inertia were
more difficult to estimate. This is direct evidence for choosing mass and
center of mass before expanding to full inertial parameters.

## Literature comparison

| Method | Evidence and observed strengths | Main limitations | Fit for this project |
| --- | --- | --- | --- |
| Batch OLS/WLS on a linear inverse-dynamics or gravity regressor | Atkeson et al. obtained good payload mass and center-of-mass estimates and accurate predicted load forces/torques. Leboutet et al. describe IDIM-OLS as the most common batch approach because it is efficient and easy to implement. | Sensitive to poor excitation, differentiation noise, outliers, and unconstrained nonphysical estimates. | **Best initialization method.** Gravity-only quasi-static samples avoid acceleration differentiation; use weighting, conditioning gates, and physical bounds. |
| Recursive OLS/WLS/RLS | The 2021 survey states that OLS/WLS can be formulated recursively and are suitable for real-time adaptive loops; they require neither detailed low-level controller knowledge nor sensitive covariance tuning. | Can drift under weak excitation, friction, saturation, changing data quality, or a forgetting factor that is too aggressive. | **Best online refinement method** when projection, covariance bounds, excitation tests, update gates, and rate limits are mandatory. |
| Instrumental variables (IDIM-IV) or DIDIM | The survey benchmark found these among the most appealing offline methods: fast convergence and better noise robustness without custom filtering. | Need precise knowledge of low-level controller structure/gains; difficult to place in a real-time adaptive loop. | Good future offline whole-arm calibration, not the gravity MVP. |
| EKF/UKF and other nonlinear Kalman filters | Can jointly estimate state and inertial parameters and can run online. | The survey reports strong sensitivity to initial covariance and tuning; sigma-point variants require many dynamics propagations per sample. | Too complex for four gravity parameters. |
| RTLS with IMU fusion | Farsoni et al. demonstrated real-time reconstruction of full payload inertial parameters using a multirate quaternion Kalman filter plus recursive total least squares. | Requires an inertial sensor and sensor fusion; the survey notes ordinary TLS is seldom used in robot identification and did not outperform OLS/WLS with adequate filtering. | Consider only if full payload inertia and an end-effector IMU become requirements. |
| Slotine-Li or online adaptive gravity control laws | Slotine and Li demonstrated rapid adaptation and improved tracking on a 2-DoF robot. Yang et al. later demonstrated constrained online gravity estimation with hardware experiments and asymptotic stability. | Changes the feedback controller and stability proof boundary, rather than only improving a bounded feed-forward model. Less aligned with vendor load-calibration workflows and this repository's existing admission boundary. | Research reference, not the first production surface. |
| Neural residual/gravity model | Can approximate complex unmodeled dynamics. | Harder to constrain, interpret, qualify, reset by model identity, and distinguish gravity from friction/contact. The survey notes the mapping between neural hyperparameters and physical dynamic parameters is not well understood. | Out of scope for the MVP. |

## Industrial-practice evidence

The exact numerical estimator used inside commercial controllers is generally
proprietary. Public manuals and SDKs nevertheless expose a consistent product
contract: run a bounded identification/calibration procedure, estimate tool
payload mass and center of gravity (sometimes full inertia), then use those
parameters in the robot's model-based compensation and safety checks.

### ABB RobotWare

ABB's current RobotWare 8 RAPID technical reference includes `LoadId` in
RobotWare Base and says the normal alternative is the `LoadIdentify` service
routine. It supports tool or payload identification, including an unknown-mass
mode. The documented result contains mass, center of gravity, moment axes, and
inertia. The procedure executes a defined set of measurement motions and must
restart from the beginning after a stop.

This is strong evidence for a deliberate, stateful calibration episode with
motion/operating-mode prerequisites rather than unrestricted continuous
learning.

Source: ABB, *Technical reference manual - RAPID Instructions, Functions and
Data types RW 8*, document `3HAC098387-001`, section 1.128, revision B:
<https://search.abb.com/library/Download.aspx?DocumentID=3HAC098387-001&LanguageCode=en&DocumentPartId=&Action=Launch>

### Universal Robots

Universal Robots exposes payload mass, payload center of gravity, and payload
inertia as controller state. Its RTDE guide states that `actual_TCP_force` is
compensated for forces and torques generated by the payload, while raw wrench
data is explicitly not payload-compensated. The official ROS 2 driver also
sets payload mass/CoG and verifies that the reported controller values match
the request.

Sources:

- UR RTDE guide: <https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/rtde-guide.html>
- Official UR ROS 2 driver payload handling:
  <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_controllers/src/gpio_controller.cpp>

### FAIRINO

FAIRINO's public SDK/ROS 2 repository contains a load-identification sequence
and exposes the result as `weight, x, y, z`, exactly matching payload mass plus
center of gravity.

Source:
<https://github.com/FAIR-INNOVATION/frcobot_ros2/blob/f8aac14d92764e3e8cce3927458911ee0b38c2db/fairino_hardware_v3_9_6/include/fairino_hardware/data_type_def.h#L261>

### AUBO public SDK example

The AUBO SDK example included in a public ROS 2 distribution generates a
payload-identification trajectory, records data without and with the payload,
runs `payloadIdentify`, and retrieves a `PayloadCalibResult`. This again shows
an excitation/calibration workflow followed by committed payload parameters.

Source:
<https://github.com/XieShaosong/aubo_robot_ros2/blob/jazzy-arcs/aubo_ros2_driver/lib/aubo_sdk-0.23.1-Linux_x86_64%2B71831e1/share/example/c%2B%2B/example_payload_identify.cpp>

## Recommended estimator contract

### Parameterization

- Keep all nominal link inertial parameters fixed to the validated model.
- Estimate only payload `theta = [m, h_x, h_y, h_z]`, where `h = m*c`.
- Convert first moments to CoG only when `m` is above a configured minimum.
- Project every accepted estimate into configured mass and CoG bounds.

### Observation equation

At an eligible low-motion sample:

`tau_measured - tau_nominal_gravity = Y_payload_gravity(q) * theta + noise`

Use the existing joint order and model-generation identity. Do not update from
samples with stale data, non-finite values, saturation, excessive velocity or
acceleration, contact/fault state, or insufficient regressor excitation.

### Initialization and online update

- Collect multiple sufficiently different gravity poses.
- Solve a weighted, regularized batch least-squares problem with QR or SVD.
- Reject the calibration if rank/condition and residual checks fail.
- Seed a square-root or covariance-form RLS estimator from that solution.
- Use a forgetting factor close to one, covariance eigenvalue bounds,
  innovation clipping, parameter projection, and per-update rate limits.
- Freeze online learning when excitation is weak; evaluation can continue from
  the last accepted estimate.
- Treat a candidate estimate as a proposal until confidence and residual
  improvement gates allow it to become the active compensation parameters.

### Separation from other disturbances

Do not let the payload estimator absorb Coulomb friction, stiction, contact,
cable forces, or broad whole-arm model error. These effects are not described
by the payload gravity regressor and will bias mass/CoG. A later disturbance
observer or separately identified friction model can address them without
corrupting the physically interpretable payload state.

## MVP recommendation

Adopt payload mass and CoG mismatch as the simulation's adaptive mismatch
class. Implement batch LS calibration plus bounded/gated RLS refinement. Do
not identify payload rotational inertia, link inertial parameters, or friction
in the first adaptive-gravity release.

This scope follows the public industrial contract, minimizes real-time cost,
avoids acceleration differentiation, preserves the existing model/admission
boundaries, and directly targets the torque component that gravity
compensation can correct.

## Primary references

1. C. G. Atkeson, C. H. An, and J. M. Hollerbach, “Estimation of Inertial
   Parameters of Manipulator Loads and Links,” *The International Journal of
   Robotics Research*, 1986. <https://doi.org/10.1177/027836498600500306>
2. J. Swevers, C. Ganseman, D. B. Tukel, J. de Schutter, and H. Van Brussel,
   “Optimal Robot Excitation and Identification,” *IEEE Transactions on
   Robotics and Automation*, 1997. <https://doi.org/10.1109/70.631234>
3. J.-J. E. Slotine and W. Li, “Adaptive Manipulator Control: A Case Study,”
   *IEEE Transactions on Automatic Control*, 1988.
   <https://doi.org/10.1109/9.14411>
4. S. Farsoni, C. Talignani Landi, F. Ferraguti, C. Secchi, and M. Bonfè,
   “Real-Time Identification of Robot Payload Using a Multirate
   Quaternion-Based Kalman Filter and Recursive Total Least-Squares,” *ICRA*,
   2018. <https://doi.org/10.1109/ICRA.2018.8461167>
5. Q. Leboutet et al., “Inertial Parameter Identification in Robotics: A
   Survey,” *Applied Sciences*, 2021. <https://doi.org/10.3390/app11094303>
6. T. Yang, N. Sun, Y. Fang, X. Xin, and H. Chen, “New Adaptive Control Methods
   for n-Link Robot Manipulators With Online Gravity Compensation: Design and
   Experiments,” *IEEE Transactions on Industrial Electronics*, 2021.
   <https://doi.org/10.1109/TIE.2021.3050371>
