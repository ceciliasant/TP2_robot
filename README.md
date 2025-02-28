# Simulation of the process of manipulation of the robot Yaskawa SDA10F

This MATLAB project simulates the Yaskawa SDA10F dual-arm robot assembling blocks from conveyor belts using trajectory planning, forward, inverse, and differential kinematics.

## 1. Robot Model

The Yaskawa SDA10F has two 7-DOF arms. The simulation models block picking, assembly, and transfer while respecting reach constraints.

## 2. Kinematics
- Forward Kinematics: Computes end-effector position using DH parameters.

- Inverse Kinematics: Uses an iterative Jacobian-based method (invKinematics).

- Differential Kinematics: Relates joint velocities to end-effector movement (jacobianGeom).

## 3. Key functions 
- invKinematics: Solves inverse kinematics.

- jacobianGeom: Computes the Jacobian.

- animateRobot: Animates robot motion.

## 4. Program Flow

1. Move above the block

2. Lower to pick up

3. Lift and position

4. Assemble blocks

5. Transfer to output conveyor

6. Return to home position

**Results** can be seen here: https://www.youtube.com/watch?v=hOHCbvwYhKE
