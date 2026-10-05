# SO-ARM101 Project Domain Glossary

This document establishes the ubiquitous domain language for the SO-ARM101 robotic arm workspace, aligning architecture, algorithms, hardware adapters, and control interfaces.

---

## 1. Core Mechanics & Kinematic Entities

- **Joint**:
  A single rotational degree of freedom (DOF) of the SO-ARM101 manipulator. The arm features 6 revolute joints identified by canonical URDF names:
  1. `shoulder_pan`: Base yaw rotation around the Z-axis.
  2. `shoulder_lift`: Shoulder pitch elevation.
  3. `elbow_flex`: Elbow pitch flexion.
  4. `wrist_flex`: Wrist pitch flexion.
  5. `wrist_roll`: Wrist axial rotation (roll).
  6. `gripper`: End-effector jaw pinching joint.

- **Joint Space (\(q \in \mathbb{R}^6\))**:
  The coordinate space defined by the vector of all 6 joint angles, measured in radians (\(\text{rad}\)) with respects to URDF zero reference frames.

- **Task Space (\(SE(3)\))**:
  The 3D Cartesian position (\(x, y, z\)) and spatial orientation (SO(3) rotation matrix, RPY, or quaternion) of the manipulator's tool center frame (`gripper_frame_link`).

- **End-Effector (EE)**:
  The terminal gripping mechanism consisting of the stationary palm and moving jaw link, whose kinematic frame is designated as `gripper_frame_link`.

---

## 2. Model-Based Algorithms & Math

- **Forward Kinematics (FK)**:
  Mapping joint configuration \(q\) to the tool center frame pose \(T \in SE(3)\) relative to the `base_link` using rigid body tree dynamics (Pinocchio).

- **Inverse Kinematics (IK)**:
  Calculating the joint angles \(q\) required to achieve a target Cartesian position and/or orientation in Task Space.

- **Damped Least Squares (DLS IK)**:
  A Levenberg-Marquardt style numerical IK solver that incorporates a damping factor \(\lambda\) into the pseudoinverse formulation:
  \[
  \Delta q = J^T (J J^T + \lambda^2 I)^{-1} \Delta x
  \]
  Ensuring numerical stability and preventing velocity explosion near kinematic singularities.

- **Yoshikawa Manipulability Index (\(w\))**:
  A scalar metric (\(w = \sqrt{\det(J J^T)}\)) indicating distance from singular configurations and directional dexterity.

- **Quintic Polynomial**:
  A 5th-order polynomial trajectory interpolator \(s(t) = 10s^3 - 15s^4 + 6s^5\) ensuring zero velocity and zero acceleration boundary conditions at trajectory start and end.

---

## 3. Hardware & Communication Layer

- **STS3215 Serial Bus Servo**:
  Smart digital bus servos manufactured by Feetech/Waveshare communicating over half-duplex UART at 1,000,000 baud (1 Mbps).

- **Raw Position Pulse**:
  Integer angle feedback from the STS3215 internal 12-bit magnetic encoder, spanning \([0, 4095]\) pulses per full rotation (\(360^\circ\)).

- **Joint Calibration**:
  The adapter mapping converting raw servo pulses to canonical URDF joint angles in radians:
  \[
  q_{\text{rad}} = \text{direction\_sign} \cdot \frac{2\pi}{4096} \cdot (\text{raw\_pulse} - \text{zero\_offset})
  \]

- **Hardware Shadow**:
  A continuous synchronization state machine running at 50 Hz that samples real-time physical joint states from the serial bus and broadcasts them to `/joint_states`, enabling real-time RViz and MoveIt 2 visualization.
