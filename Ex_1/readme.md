# RRP 3-DOF SCARA Robot (MATLAB) — Exercise 1

This exercise introduces the construction of a simple **3-DOF RRP SCARA-type robot** using MATLAB’s `rigidBodyTree`.  
The primary learning goal is to understand **how fixed transforms define robot geometry**, and how **joint motions are applied on top of those transforms** to produce forward kinematics.

---

## Robot Structure

The robot consists of:
- **Joint 1:** Revolute (rotation about Z)
- **Joint 2:** Revolute (rotation about Z)
- **Joint 3:** Prismatic (translation along Z)

Overall structure:


> `body3` acts as a rigid spacer to position the prismatic joint correctly.  
> It does **not** add a degree of freedom.

---

## File Description

### `RRP3dofSCARA.m`

This script:
- defines link lengths and base offsets
- creates a `rigidBodyTree`
- adds rigid bodies and joints
- assigns **fixed transforms** to place joints in space
- visualizes the robot using `show()`

---

## Requirements

- MATLAB (R2020a or later recommended)
- Robotics System Toolbox

---

## How to Run

1. Open MATLAB
2. Set your working directory to the folder containing the file
3. Run:

```matlab
RRP3dofSCARA


showdetails(robot)


config = homeConfiguration(robot);
T = getTransform(robot, config, 'endEffector', 'base');
