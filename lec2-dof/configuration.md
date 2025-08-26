---
author:
  - Wanxin Jin
date: Aug. 22, 2023
title: "Lecture 2: Robot Arm and Configuration"
---

# Robot Configuration

```{figure} ./configuration/robotarm.png
---
width: 40%
name: robotarm
---
Franka research robot arm
```

A robot manipulator is mechanically constructed by

- a set of rigid bodies, each body called a **link**,

- **joints**, connecting two consecutive links and providing motion
  constraints (degrees of freedom)

  ```{figure} ./configuration/joints.png
  ---
  width: 60%
  name: joints-types
  ---
  Different type of joints
  ```

- **actuators**, such as electric/hydraulic/pneumatic actuators,
  deliver forces or torques that cause the robot's links to move,

  ```{figure} ./configuration/electric_actuator.png
  ---
  width: 40%
  name: electric-actuators
  ---
  Electric actuators
  ```

  ```{figure} ./configuration/hydralic_actuator.png
  ---
  width: 40%
  name: hydralic_actuator
  ---
  Hydralic actuator
  ```

- **an end effector**, such as a gripper or hand, is attached to a
  specific link for specific tasks.

  ```{figure} ./configuration/endeffector2.png
  ---
  width: 40%
  name: endeffector2
  ---
  The Shadow Hand
  ```

The typical structure of a robot arm is an open
kinematic chain. From a topological viewpoint, a kinematic chain is
open when there is only one sequence of links connecting the two
ends of the chain. Alternatively,
kinematic chain is closed when a sequence of links forms a loop. This course mainly focuses on open chain arms.

```{admonition} MuJoCo Simulator Introduction
:class: tip

MuJoCo (Multi-Joint dynamics with Contact) is a physics engine designed for the simulation and control of robots and other articulated mechanisms. It is widely used in robotics research for its speed, accuracy, and support for advanced features such as soft contacts and complex constraints.

**Key Features:**

- Fast and accurate simulation of rigid body dynamics
- Support for soft and hard contacts
- Flexible model specification using XML
- Widely used in reinforcement learning and robotics research

**Getting Started:**

To use MuJoCo, you need to:

1. Install MuJoCo (UI) from [https://github.com/google-deepmind/mujoco/releases](https://github.com/google-deepmind/mujoco/releases)
2. Download Example Model:** [franka_fr3.zip](../lec2-dof/code/franka_fr3.zip)
3. Unzip and drag it into UI to simulate the Franka Research 3 Robot arm.




```

# Configuration

```{important}
The **configuration** of a robot is a complete specification of the
position of every point of the robot. Since a robot's links are rigid
and of a known shape, only a few coordinates  are needed to represent its configuration.
```

For example, in the following figure, the configuration of a door can be
represented by the hinge angle $\theta$. The
configuration of a point on a plane can be represented by two coordinates,
$(x, y)$. The configuration of a coin lying heads up on a flat table can
be represented by three coordinates: $(x, y)$ that specifies
the location of a particular point on the coin, and
$(\theta)$ that specifies the coin's orientation.

```{figure} ./configuration/examples.png
---
width: 90%
name: configuration
---
(a) The configuration of a door is represented by the angle $\theta$.
(b) The configuration of a point in a plane is represented by coordinates
$(x, y)$. (c) The configuration of a coin (heads up) on a table is
represented by $(x, y, \theta)$.
```

# Degrees of Freedom

:::{important}
The minimum number $n$ of coordinates needed to represent
the configuration of a robot is the **degree of freedom (dof)**. The $n$-dimensional space containing all possible configurations/coordinates
of a robot is called the **configuration space (C-space)**. A
configuration of a robot is represented by a point in its C-space.
:::

## General Rule

We have the following general rule to determine the number of dof of a robot:

::: {important}
degrees of freedom = number of variables - number of independent constraints
:::

```{figure} ./configuration/coin.png
---
width: 40%
name: corn
---
A coin in 3D
space
```

Example: consider a coin in 3D space in
{numref}`corn`, the
coordinates for the three points $A, B$, and $C$ are given by
$\left(x_{A}, y_{A}, z_{A}\right),\left(x_{B}, y_{B}, z_{B}\right)$, and
$\left(x_{C}, y_{C}, z_{C}\right)$, respectively. Each point has 3 dof, and therefore total dof are 9. However, consider the rigid body assumption, meaning that distance between any two points are fixed. This introduces 3 constraints. Therefore, a rigid body moving in 3D space has 9-3=6 dof.

- A rigid body moving in 3-dimensional space has $m=6$ dof.

- A rigid body moving in a 2-dimensional plane has $m=3$ dof.

## DOF of Different Joint Types

In a robot, a joint can be viewed as providing freedoms
to allow one rigid body to move relative to another. It can also be
viewed as providing constraints on the possible motions of the two rigid
bodies it connects. The following table summarizes the freedoms and
constraints provided by the various joint types.

|   Joint type    | dof (f) | # of constraints between two bodies in 3D space (c) |
| :-------------: | :-----: | :-------------------------------------------------: |
|  Revolute (R)   |    1    |                          5                          |
|  Prismatic (P)  |    1    |                          5                          |
|   Helical (H)   |    1    |                          5                          |
| Cylindrical (C) |    2    |                          4                          |
|  Universal (U)  |    2    |                          4                          |
|  Spherical (S)  |    3    |                          3                          |

Let $f$ be the number of freedoms
provided by joint $i$, and $c$ be the number of constraints provided
by joint $i$, where $f+c=m$. $m=3$ for planar mechanisms and
$m=6$ for 3D mechanisms.

## Grübler's Formula

<!-- Since a robot consists of multiple rigid bodies,  degrees of freedom of a
robot can be expressed as follows:

:::{important}
degrees of freedom =  number of variables - number of independent equations (constraints)
::: -->

:::{important}
Consider a mechanism consisting of $L$ links, where the ground is also
regarded as a link. Let $J$ be the number of joints, $m$ be dof of a rigid body ($m=3$ for planar mechanisms and
$m=6$ for 3D mechanisms), $f_{i}$ be the number of freedoms
provided by joint $i$, and $c_{i}$ be the number of constraints provided
by joint $i$, where $f_{i}+c_{i}=m$ for all $i$. Then the Grübler's
formula for dof of the mechanism is

$$
\begin{aligned}
\operatorname{dof} & =\underbrace{m(L-1)}_{\text {total number of freedoms for all rigid bodies }}-\underbrace{\sum_{i=1}^{J} c_{i}}_{\text {joint constraints }} \\
& =m(L-1)-\sum_{i=1}^{J}\left(m-f_{i}\right) \\
& =m(L-1-J)+\sum_{i=1}^{J} f_{i} .
\end{aligned}
$$

:::

This formula holds only if all joint constraints are independent. If
they are not independent then the formula provides a lower bound on the
number of degrees of freedom.

Below are some examples of using Grubler's
Formula to find the DoFs of different machenisms.

```{figure} ./configuration/gf_example1.png
---
width: 60%
name: gf_example1
---
(a) Four-bar linkage and (b) Slider-crank mechanism.
```

(Four-bar linkage): $L=4, J=4$, and $f_{i}=1, i=1, \ldots, 4$. Thus,
$\operatorname{dof}$=1.\
Slider-crank: $L=4$, $J=4,$ and $f_{i}=1, i=1, \ldots, 4$. Thus,
$\operatorname{dof}$=1.

```{figure} ./configuration/gf_example2.png
---
width: 60%
name: gf_example2
---
(a) $k$-link planar serial chain. (b) Five-bar planar linkage. (c)
Stephenson six-bar linkage. (d) Watt six-bar
linkage.
```

The $k$-link planar robot: $L=k+1$, $J=k$, $f_{i}=1$ for all $i$. Thus,
$\operatorname{dof}=3*((k+1)-1-k)+k=k$.\
The five-bar linkage: $L=5$, $J=5$, each $f_{i}=1$. Therefore,
$\operatorname{dof}=3*(5-1-5)+5=2$.\
Stephenson six-bar linkage: $L=6, J=7$, and $f_{i}=1$ for all $i$.
Therefore, $\operatorname{dof}=3*(6-1-7)+7=1$.\
Watt six-bar linkage: $L=6, J=7$, and $f_{i}=1$ for all $i$. Therefore,
$\operatorname{dof}=3*(6-1-7)+7=1$.

```{figure} ./configuration/gf_example3.png
---
width: 60%
name: gf_example3
---
A planar mechanism with two overlapping
joints.
```

The planar mechanism in {numref}`gf_example3` has three links that meet at a single
point on the right of the large link. Recalling that a joint by
definition connects exactly two links, the joint at this point should not be regarded as a single revolute joint. Rather,
it should be counted as two revolute joints overlapping each
other. The mechanism consists of eight links $(L=8)$, 8 revolute
joints, and 1 prismatic joint. The Grübler's formula:

$$\operatorname{dof}=3*(8-1-9)+9*(1)=3$$

```{figure} ./configuration/gf_example4.png
---
width: 60%
name: gf_example4
---
Stewart-Gough
platform
```

The Stewart-Gough platform consists of two platforms - the lower one
is regarded as ground, the upper one mobile - connected by
six universal-prismatic-spherical (UPS) legs. The total number of links
is $14(L=14)$. There are six universal joints (for each, $f_{i}=2$ ), six prismatic joints (for each, $f_{i}=1$ ), and six spherical joints (for each, $f_{i}=3$ ). The total number of joints is 18 . Using Grübler's formula, we have

$$\operatorname{dof}=6*(14-1-18)+6*(1)+6*(2)+6*(3)=6 .$$

<!--
# Configuration Space

Consider a point moving on the surface of a sphere. The C-space is
2-dimensional, as the configuration can be represented by two coordinates:
latitude and longitude. If a point moving on a plane, it also has
a two-dimensional C-space, with coordinates $(x, y)$. While
both  plane and   sphere surface are 2-dimensional, they do not
have the same shape. A larger sphere has the same
shape as a small sphere, in that it wraps around in the same way.
Only its size is larger. For that matter, an American
football also wraps around similarly to a sphere. The idea that the
2-dimensional surfaces of a small sphere, a large sphere, and a football
all have the same kind of shape is formally called "topologically equivalent". That is, if one can be continuously deformed into the
other without cutting or gluing.  A sphere can be deformed into a
football simply by stretching, without cutting or gluing, so those two
spaces are topologically equivalent. You cannot turn a sphere into a
plane without cutting it, however, so a sphere and a plane are not
topologically equivalent.


```{figure} ./configuration/topology_summary.jpeg
---
width: 60%
name: topology_summary
---
Three topologically different two-dimensional C-spaces and coordinate
representations. In the latitude-longitude representation of the sphere,
the latitudes $-90^{\circ}$ and $90^{\circ}$ each correspond to a single
point (the South Pole and the North Pole, respectively), and the
longitude parameter wraps around at $180^{\circ}$ and $-180^{\circ}$;
the edges with the arrows are glued together. Similarly, the coordinate
representations of the torus wrap around at the edges marked with
corresponding
arrows.
```

There are two coordinate parameterizations/representations for a
topology space.

-   **explicit parametrization:** A choice of $n$ coordinates, or
    parameters, to represent an $n$-dimensional space is called an
    explicit parametrization of the space. Such an explicit
    parametrization is valid for a particular range of the parameters
    (e.g., $\left[-90^{\circ}, 90^{\circ}\right]$ for latitude and
    $\left[-180^{\circ}, 180^{\circ}\right.$ ) for longitude for a
    sphere, where, on Earth, negative values correspond to \"south\" and
    \"west,\" respectively).

-   **implicit parametrization:** An implicit representation views the
    $n$-dimensional space as embedded in a Euclidean space of more than
    $n$ dimensions, just as a two-dimensional unit sphere can be viewed
    as a surface embedded in a three-dimensional Euclidean space. An
    implicit representation uses the coordinates of the
    higher-dimensional space (e.g., $(x, y, z)$ in the three-dimensional
    space), but subjects these coordinates to constraints that reduce
    the number of degrees of freedom (e.g., $x^{2}+y^{2}+z^{2}=1$ for
    the unit sphere). -->

# Workspace of Robot Arms

The workspace is the space of the environment a robot arm's
end-effector can access. Its shape and volume depend on the robot arm's
structure as well as mechanical joint limits. The workspace is independent of the task. In the following, below is the workspace of different commonly used robot arms.

Cartesian robot arm includes three prismatic joints whose axes
typically are mutually orthogonal.

```{figure} ./configuration/cartesian_robot.jpg
---
width: 50%
name: cartesian_robot
---
Cartesian robot arm and its
workspace
```

Cylindrical robot arm differs from Cartesian in that the first
prismatic joint is replaced with a revolute joint.

```{figure} ./configuration/cylindrical_robot.jpg
---
width: 50%
name: cylindrical_robot
---
Cylindrical robot arm and its
workspace
```

Spherical robot arm differs from cylindrical in that the 2nd prismatic
joint is replaced with a revolute joint.

```{figure} ./configuration/spherical_robot.jpg
---
width: 40%
name: spherical_robot
---
Spherical robot arm and its
workspace
```

A special robot arm is SCARA robot arm, which has the feature that
all the axes of motion are parallel.

```{figure} ./configuration/SCARA_robot.jpg
---
width: 40%
name: SCARA_robot
---
SCARA robot arm  and its
workspace
```

Anthropomorphic robot arm has three revolute joints; the
revolute axis of the first joint is orthogonal to the axes of the other
two which are parallel.

```{figure} ./configuration/anthropomophic_robot.jpg
---
width: 40%
name: anthropomophic_robot
---
Anthropomorphic robot arm  and its
workspace
```
