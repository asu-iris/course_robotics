---
author:
- Wanxin Jin
date: Sep. 05, 2023 Sep. 7, 2023 Sep. 12, 2023
title: "Lecture 4: Forward Kinematics"
---
# Forward Kinematics

A robot arm is a kinematic chain of rigid bodies (links) connected by
actuated joints. One end of the kinematic chain is mounted to a base and
the other end to an end-effector (gripper, tool). Joints have two tyipcal types (though we learned many):
revolute and prismatic, as in
{numref}`end-effector_frame`. The pose of a robot arm is
described by all joints (all DoFs). Each DOF is
typically associated with a joint variable. Forward
kinematics (FK) is to derive the relationship between joint variables and the
pose of the end-effector.

```{figure} ./fk/end-effector_frame.jpg
---
width: 70%
name: end-effector_frame
---
Description of the pose (position and orientation) of the end-effector
frame
```

Consider a robot arm of $n$ joints. Define $\boldsymbol{q}=[q_1, q_2, ..., q_n]$ as the $(n \times 1)$ vector of joint variables.
With respect to a base frame $O_{b}-x_{b} y_{b} z_{b}$, the FK
is expressed by the homogeneous transformation

$$
\boldsymbol{T}_{e}^{w}(\boldsymbol{q})=
\left[\begin{array}{cccc}
R^w_e & \boldsymbol{p}_{e}^{w}(\boldsymbol{q}) \\
\boldsymbol{0} &  1
\end{array}\right]
=
\left[\begin{array}{cccc}
\boldsymbol{n}_{e}^{w}(\boldsymbol{q}) & \boldsymbol{s}_{e}^{w}(\boldsymbol{q}) & \boldsymbol{a}_{e}^{w}(\boldsymbol{q}) & \boldsymbol{p}_{e}^{w}(\boldsymbol{q}) \\
0 & 0 & 0 & 1
\end{array}\right]
$$

where $R_e=[\boldsymbol{n}_{e}, \boldsymbol{s}_{e}, \boldsymbol{a}_{e}]$ and
$\{\boldsymbol{n}_{e}, \boldsymbol{s}_{e}, \boldsymbol{a}_{e}\}$ are the
unit vectors of  the end-effector frame, and
$\boldsymbol{p}_{e}$ is the position of the origin of the end-effector
frame. If the end-effector is a gripper, the origin of the end-effector
frame is located at the center of the gripper, $\boldsymbol{a}_{e}$ is
chosen in the approach direction to the object, $\boldsymbol{s}_{e}$ is
chosen in the sliding plane of the jaws,
and $\boldsymbol{n}_{e}$ is chosen to complete the right-handed rule.

```{admonition} MuJoCo Simulation of Franka Robot Arm
:class: tip

To run the Franka Emika Panda forward kinematics simulation locally:

1. **Download the FK code files:**
  - [Download franka_emika_panda.zip](./fk_code/franka_emika_panda.zip)
  - Unzip the file to any folder of your choice on your local system.

2. **Install MuJoCo and its Python bindings:**
  - Make sure you have Python installed.
  - Install MuJoCo and the Python package:
    ```sh
    pip install mujoco
    ```


3. **Run the simulation:**
   - Open the `demo.py` script in your code editor.
   - Set the `MODEL_PATH` variable in `demo.py` to the correct path of the `scene.xml` file in your local system. For example:
     ```python
     MODEL_PATH = "/path/to/your/franka_emika_panda/scene.xml"  # Use the provided scene.xml file
     model = mujoco.MjModel.from_xml_path(MODEL_PATH)
     ```
   - Save the changes to `demo.py`.
   - In your terminal, navigate to the folder containing `demo.py` and run:
     ```sh
     python demo.py
     ```

You should see a window displaying the Franka Emika Panda robot simulation. Press ESC in the viewer window to exit.
```


# Notation Convention


A robot arm composes of $n+1$ links connected by $n$ joints,
where Link 0 refers to the base. Each joint
variable provides 1 DOF. Define a frame attached to each link,
from Link 0 to Link $n$. Then,  the transformation of Frame $n$ with
respect to Frame 0 is (postmultiplication rule)

$$
\boldsymbol{T}_{n}^{0}(\boldsymbol{q})=\boldsymbol{T}_{1}^{0}\left(q_{1}\right) \boldsymbol{T}_{2}^{1}\left(q_{2}\right) \ldots \boldsymbol{T}_{n}^{n-1}\left(q_{n}\right) .
$$

Each homogeneous
transformation $\boldsymbol{T}_{i}^{i-1}\left(q_{i}\right)$, $i=1, \ldots n$, is a function of a
corresponding joint variable $q_i$.

Typically, the word frame can be different from the frame of Link 0 (base), the end-effector frame is different from the frame of Link n.The FK describing the pose of the
end-effector frame with respect to the base frame is

$$
\boldsymbol{T}_{e}^{w}(\boldsymbol{q})=\boldsymbol{T}_{0}^{w} \boldsymbol{T}_{n}^{0}(\boldsymbol{q}) \boldsymbol{T}_{e}^{n}
$$

where $\boldsymbol{T}_{0}^{w}$ and $\boldsymbol{T}_{e}^{n}$ are two
constant homogeneous transformations describing the position
and orientation of Frame 0 with respect to the world frame, and of the
end-effector frame with respect to Frame $n$, respectively.

Question: how to choose frame for each link to faciliate the computation of
$\boldsymbol{T}_{i}^{i-1}\left(q_{i}\right)$? 




# Denavit-Hartenberg (DH) Convention

DH Convention is to systematically and recursively
define a frame to each link and obtain the transformation between two
consecutive links.

```{figure} ./fk/DH_convention.jpg
---
width: 70%
name: DH_convention
---
Denavit-Hartenberg convention and parameters for link Frame
$i$
```

```{admonition}  Coordinate Frame Definition in DH Convention
With reference to {numref}`DH_convention`, joint $i$ connects Link $i-1$ to Link $i$; DH
convention sets the following rules to define link Frame
$O_{i}-x_{i} y_{i} z_{i}$, given previous link Frame
$O_{i-1}-x_{i-1} y_{i-1} z_{i-1}$:

-   Choose axis $z_{i}$ along the joint $i+1$ (righthand rule), and axis $z_{i-1}$ along the joint $i$.


-   Choose the origin $O_{i}$ at the intersection of $z_{i}$ with
    the common normal to $z_{i}$ and $z_{i-1}$ . 

-   Choose  $x_{i}$ along the common normal to      $z_{i}$ and $z_{i-1}$  with direction  from Joint $i$ to Joint $i+1$.

-   Choose  $y_{i}$ to complete a right-handed frame.
```

Once the link frames are established, the pose of
link Frame $O_{i}-x_{i} y_{i} z_{i}$ in the link Frame
$O_{i-1}-x_{i-1} y_{i-1} z_{i-1}$ can be determined by
four basic parameters, with the help of an intermediate frame $O_{i^{\prime}}-x_{i^{\prime}} y_{i^{\prime}} z_{i^{\prime}}$ shown in {numref}`DH_convention`:

- $a_{i}$ distance between $O_{i}$ and $O_{i^{\prime}}$ --- constant
  and depend only on the geometry of links
- $\alpha_{i}$ angle between axes $z_{i-1}$ and $z_{i}$ about axis
  $x_{i}$ --- constant and depend only on the geometry of links
- $d_{i}$ distance of between $O_{i-1}$ and $O_{i^{\prime}}$ along
  $z_{i-1}$ --- variable if Joint $i$ is prismatic, otherwise constant
- $\vartheta_{i}$ angle between axes $x_{i-1}$ and $x_{i}$ about axis
  $z_{i-1}$ --- variable if Joint $i$ is revolute, otherwise constant

To compute $\boldsymbol{T}_{i}^{i-1}\left(q_{i}\right)$, we perform four basic transformations. First, translate the Frame
$O_{i-1}-x_{i-1} y_{i-1} z_{i-1}$ by $d_{i}$ along axis $z_{i-1}$. Second,
Rotate the resulting frame by $\vartheta_{i}$ about $z_{i-1}$. These two steps lead to the
pose of the intermediate Frame $O_{i^{\prime}}-x_{i^{\prime}} y_{i^{\prime}} z_{i^{\prime}}$ in Frame $O_{i-1}-x_{i-1} y_{i-1} z_{i-1}$:

$$
\boldsymbol{T}_{i^{\prime}}^{i-1}=\left[\begin{array}{cccc}
c_{\vartheta_{i}} & -s_{\vartheta_{i}} & 0 & 0 \\
s_{\vartheta_{i}} & c_{\vartheta_{i}} & 0 & 0 \\
0 & 0 & 1 & d_{i} \\
0 & 0 & 0 & 1
\end{array}\right]
$$

Third, translate the intermediate Frame $i^{\prime}$ by $a_{i}$ along
axis $x_{i^{\prime}}$, and fourth, rotate it by $\alpha_{i}$ about axis
$x_{i^{\prime}}$. The last two steps lead  to the pose of
$O_{i}-x_{i} y_{i} z_{i}$ in  Frame $O_{i^{\prime}}-x_{i^{\prime}} y_{i^{\prime}} z_{i^{\prime}}$:

$$
\boldsymbol{T}_{i}^{i^{\prime}}=\left[\begin{array}{cccc}
1 & 0 & 0 & a_{i} \\
0 & c_{\alpha_{i}} & -s_{\alpha_{i}} & 0 \\
0 & s_{\alpha_{i}} & c_{\alpha_{i}} & 0 \\
0 & 0 & 0 & 1
\end{array}\right]
$$

In sum of the above four steps, the total transformation of link $i$'s Frame
$O_{i}-x_{i} y_{i} z_{i}$ in the link $i-1$'s Frame $O_{i-1}-x_{i-1} y_{i-1} z_{i-1}$ is

$$
\boldsymbol{T}_{i}^{i-1}\left(q_{i}\right)=\boldsymbol{T}_{i^{\prime}}^{i-1} \boldsymbol{T}_{i}^{i^{\prime}}=\left[\begin{array}{cccc}
c_{\vartheta_{i}} & -s_{\vartheta_{i}} c_{\alpha_{i}} & s_{\vartheta_{i}} s_{\alpha_{i}} & a_{i} c_{\vartheta_{i}} \\
s_{\vartheta_{i}} & c_{\vartheta_{i}} c_{\alpha_{i}} & -c_{\vartheta_{i}} s_{\alpha_{i}} & a_{i} s_{\vartheta_{i}} \\
0 & s_{\alpha_{i}} & c_{\alpha_{i}} & d_{i} \\
0 & 0 & 0 & 1
\end{array}\right]
$$

The DH convention gives a nonunique definition of the link frame in the
following cases.

- For Frame 0, only the direction of axis $z_{0}$ is
  specified; then $O_{0}$ and $x_{0}$ can be arbitrarily chosen.
- For
  Frame $n$, since there is no Joint $n+1, z_{n}$ is not uniquely defined
  while $x_{n}$ has to be normal to axis $z_{n-1}$. Typically, Joint $n$
  is revolute, and thus $z_{n}$ is to be aligned with the direction of
  $z_{n-1}$.
- When two consecutive joint axes intersect, the direction (+/-) of
  $x_{i}$ is arbitrary.
- When two consecutive joint axes parallel,
  $x_{i}$ is chosen such that $d_i=0$.
- When Joint $i$ is prismatic, the direction (+/-)
  of $z_{i-1}$ is arbitrary.

In all such cases, the principle of choosing
frames is to simplify the procedure; for instance, the axes of
consecutive frames can be made parallel.

```{admonition}  DH Convention


1.  Identify  each joint and set the directions of axes
    $z_{0}, \ldots, z_{n-1}$

2.  Choose Frame $O_{0}-x_{0} y_{0} z_{0}$. If possible, choose
    $\{\boldsymbol{x}_{0}, \boldsymbol{y}_{0}, \boldsymbol{z}_{0}\}$ to
    coincide with the base frame.

Repeat step 3 from link $i=1$ to link $n-1$ :

3.  Choose link i's Frame $O_{i}-x_{i} y_{i} z_{i}$ as follows.

    First, choose $O_{i}$ at the intersection of $z_{i}$ with the common
    normal to $z_{i-1}$ and $z_{i}$. If $z_{i-1}$ and $z_{i}$ are
    parallel and Joint $i$ is revolute, choose $O_{i}$ so that
    $d_{i}=0$; if Joint $i$ is prismatic, choose $O_{i}$ at as a
    mechanical limit. Second, choose  $x_{i}$ along the common
    normal to  $z_{i-1}$ and $z_{i}$ with direction from Joint $i$
    to Joint $i+1$, Third, choose axis $y_{i}$ using
    right-handed rule.

Calculate transformation:

4.  Choose final link's Frame $O_{n}-x_{n} y_{n} z_{n}$; if Joint $n$ is revolute,
    then align $z_{n}$ with $z_{n-1}$, otherwise, if Joint $n$ is
    prismatic, then choose $z_{n}$ arbitrarily. choose axis $x_{n}$
    along the common normal to axes $z_{n-1}$ and $z_{n}$.

5.  For $i=1, \ldots, n$, establish the four DH parameter
    $a_{i}, d_{i}, \alpha_{i}, \vartheta_{i}$, compute the consecutive
    transformation
    $\boldsymbol{T}_{i}^{i-1}\left(q_{i}\right)$, and compute the
     transformation
    $\boldsymbol{T}_{n}^{0}(\boldsymbol{q})=\boldsymbol{T}_{1}^{0} \ldots \boldsymbol{T}_{n}^{n-1}$.


Complete the assembly:

6.  Given $\boldsymbol{T}_{0}^{w}$ and $\boldsymbol{T}_{e}^{n}$, compute
    the kinematics function as $\boldsymbol{T}_{e}^{w}(\boldsymbol{q})=$
    $\boldsymbol{T}_{0}^{w} \boldsymbol{T}_{n}^{0} \boldsymbol{T}_{e}^{n}$.

  
```

# Examples

````{card}
```{figure} ./fk/3link_arm.jpg
---
width: 60%
name: 3link_arm
---
Three link robot arm
| Link  | $a_{i}$   |$\alpha_{i}$ |  $d_{i}$  | $\vartheta_{i}$|
|------ |--------- |--------------| --------- |-----------------|
|  1   |  $a_{1}$   |     0        |    0    |  $\vartheta_{1}$|
|  2  |   $a_{2}$   |     0       |     0    |  $\vartheta_{2}$|
|  3   |  $a_{3}$   |     0       |     0    |  $\vartheta_{3}$|

$$\boldsymbol{T}_{3}^{0}(\boldsymbol{q})=\boldsymbol{T}_{1}^{0} \boldsymbol{T}_{2}^{1} \boldsymbol{T}_{3}^{2}=\left[\begin{array}{cccc}
c_{123} & -s_{123} & 0 & a_{1} c_{1}+a_{2} c_{12}+a_{3} c_{123} \\
s_{123} & c_{123} & 0 & a_{1} s_{1}+a_{2} s_{12}+a_{3} s_{123} \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{array}\right]$$
```
````

````{card}
```{figure} ./fk/spherical_arm.jpg
---
width: 70%
name: spherical_arm
---
Three link robot arm
| Link  | $a_{i}$   |$\alpha_{i}$ |  $d_{i}$  | $\vartheta_{i}$|
 | ------ |---------| -------------- |--------- |-----------------|
  |  1   |     0    |   $-\pi / 2$   |    0     | $\vartheta_{1}$|
  |  2   |     0    |   $\pi / 2$   |  $d_{2}$  | $\vartheta_{2}$|
  |  3   |     0    |       0      |   $d_{3}$    | 0    |

$$\boldsymbol{T}_{3}^{0}(\boldsymbol{q})=\boldsymbol{T}_{1}^{0} \boldsymbol{T}_{2}^{1} \boldsymbol{T}_{3}^{2}=\left[\begin{array}{cccc}
c_{1} c_{2} & -s_{1} & c_{1} s_{2} & c_{1} s_{2} d_{3}-s_{1} d_{2} \\
s_{1} c_{2} & c_{1} & s_{1} s_{2} & s_{1} s_{2} d_{3}+c_{1} d_{2} \\
-s_{2} & 0 & c_{2} & c_{2} d_{3} \\
0 & 0 & 0 & 1
\end{array}\right]$$
```
````

````{card}
```{figure} ./fk/anthropomorphic_arm.jpg
---
width: 70%
name: anthropomorphic_arm
---
Three link robot arm
| Link  | $a_{i}$   |$\alpha_{i}$ |  $d_{i}$  | $\vartheta_{i}$|
 | ------ |---------| -------------- |--------- |-----------------|
   | 1    |    0      | $\pi / 2$  |      0   |   $\vartheta_{1}$|
  |  2  |   $a_{2}$   |     0      |      0   |   $\vartheta_{2}$|
  |  3 |   $a_{3}$    |    0      |      0    |  $\vartheta_{3}$|

$$\boldsymbol{T}_{3}^{0}(\boldsymbol{q})=\boldsymbol{T}_{1}^{0} \boldsymbol{T}_{2}^{1} \boldsymbol{T}_{3}^{2}=\left[\begin{array}{cccc}
c_{1} c_{23} & -c_{1} s_{23} & s_{1} & c_{1}\left(a_{2} c_{2}+a_{3} c_{23}\right) \\
s_{1} c_{23} & -s_{1} s_{23} & -c_{1} & s_{1}\left(a_{2} c_{2}+a_{3} c_{23}\right) \\
s_{23} & c_{23} & 0 & a_{2} s_{2}+a_{3} s_{23} \\
0 & 0 & 0 & 1
\end{array}\right]$$
```
````

````{card}
```{figure} ./fk/spherical_wrist.jpg
---
width: 70%
name: spherical_wrist
---
Three link robot arm
| Link  | $a_{i}$   |$\alpha_{i}$ |  $d_{i}$  | $\vartheta_{i}$|
 | ------ |---------| -------------- |--------- |-----------------|
   | 4    |    0    |    $-\pi / 2$  |      0   |      $\vartheta$| 
  |  5  |      0    |   $\pi / 2$   |     0   |   $\vartheta_{5}$|
  |  6 |   0        |   0        | $d_{6}$  | $\vartheta_{6}$|
  
$$\boldsymbol{T}_{6}^{3}(\boldsymbol{q})=\boldsymbol{T}_{4}^{3} \boldsymbol{T}_{5}^{4} \boldsymbol{T}_{6}^{5}=\left[\begin{array}{cccc}
c_{4} c_{5} c_{6}-s_{4} s_{6} & -c_{4} c_{5} s_{6}-s_{4} c_{6} & c_{4} s_{5} & c_{4} s_{5} d_{6} \\
s_{4} c_{5} c_{6}+c_{4} s_{6} & -s_{4} c_{5} s_{6}+c_{4} c_{6} & s_{4} s_{5} & s_{4} s_{5} d_{6} \\
-s_{5} c_{6} & s_{5} s_{6} & c_{5} & c_{5} d_{6} \\
0 & 0 & 0 & 1
\end{array}\right]$$
```
````

````{card}
```{figure} ./fk/Stanford_manipulator.jpg
---
width: 70%
name: Stanford_manipulator
---
 Stanford manipulator
```
$$\boldsymbol{T}_{6}^{0}=\boldsymbol{T}_{3}^{0} \boldsymbol{T}_{6}^{3}=\left[\begin{array}{cccc}
\boldsymbol{n}^{0} & \boldsymbol{s}^{0} & \boldsymbol{a}^{0} & \boldsymbol{p}^{0} \\
0 & 0 & 0 & 1
\end{array}\right]$$
with

$$\boldsymbol{p}_{6}^{0}=\left[\begin{array}{c}
c_{1} s_{2} d_{3}-s_{1} d_{2}+\left(c_{1}\left(c_{2} c_{4} s_{5}+s_{2} c_{5}\right)-s_{1} s_{4} s_{5}\right) d_{6} \\
s_{1} s_{2} d_{3}+c_{1} d_{2}+\left(s_{1}\left(c_{2} c_{4} s_{5}+s_{2} c_{5}\right)+c_{1} s_{4} s_{5}\right) d_{6} \\
c_{2} d_{3}+\left(-s_{2} c_{4} s_{5}+c_{2} c_{5}\right) d_{6}
\end{array}\right]$$
$$\begin{aligned}
& \boldsymbol{n}_{6}^{0}= {\left[\begin{array}{c}
c_{1}\left(c_{2}\left(c_{4} c_{5} c_{6}-s_{4} s_{6}\right)-s_{2} s_{5} c_{6}\right)-s_{1}\left(s_{4} c_{5} c_{6}+c_{4} s_{6}\right) \\
s_{1}\left(c_{2}\left(c_{4} c_{5} c_{6}-s_{4} s_{6}\right)-s_{2} s_{5} c_{6}\right)+c_{1}\left(s_{4} c_{5} c_{6}+c_{4} s_{6}\right) \\
-s_{2}\left(c_{4} c_{5} c_{6}-s_{4} s_{6}\right)-c_{2} s_{5} c_{6}
\end{array}\right] } \\
& \boldsymbol{s}_{6}^{0}=\left[\begin{array}{c}
c_{1}\left(-c_{2}\left(c_{4} c_{5} s_{6}+s_{4} c_{6}\right)+s_{2} s_{5} s_{6}\right)-s_{1}\left(-s_{4} c_{5} s_{6}+c_{4} c_{6}\right) \\
s_{1}\left(-c_{2}\left(c_{4} c_{5} s_{6}+s_{4} c_{6}\right)+s_{2} s_{5} s_{6}\right)+c_{1}\left(-s_{4} c_{5} s_{6}+c_{4} c_{6}\right) \\
s_{2}\left(c_{4} c_{5} s_{6}+s_{4} c_{6}\right)+c_{2} s_{5} s_{6}
\end{array}\right] \\
& \boldsymbol{a}_{6}^{0}=\left[\begin{array}{c}
c_{1}\left(c_{2} c_{4} s_{5}+s_{2} c_{5}\right)-s_{1} s_{4} s_{5} \\
s_{1}\left(c_{2} c_{4} s_{5}+s_{2} c_{5}\right)+c_{1} s_{4} s_{5} \\
-s_{2} c_{4} s_{5}+c_{2} c_{5}
\end{array}\right]
\end{aligned}$$
````

````{card}
```{figure} ./fk/anthropomorphic_manipulator.jpg
---
width: 80%
name: anthropomorphic_manipulator
---
 Anthropomorphic manipulator
 
|Link |  $a_{i}$  | $\alpha_{i}$  | $d_{i}$  | $\vartheta_{i}$|
| ------ |--------- |------------ |--------- |-----------------|
|  1    |    0    |   $\pi / 2$  |      0     | $\vartheta_{1}$|
|  2    | $a_{2}$ |       0      |      0     | $\vartheta_{2}$|
|  3    |    0    |   $\pi / 2$  |      0     | $\vartheta_{3}$|
|  4    |    0    |   $-\pi / 2$ |   $d_{4}$  | $\vartheta_{4}$|
|  5    |    0   |    $\pi / 2$  |      0     | $\vartheta_{5}$|
|  6    |    0  |         0      |   $d_{6}$  | $\vartheta_{6}$|

```
$$\boldsymbol{T}_{6}^{0}=\left[\begin{array}{cccc}
\boldsymbol{n}^{0} & \boldsymbol{s}^{0} & \boldsymbol{a}^{0} & \boldsymbol{p}^{0} \\
0 & 0 & 0 & 1
\end{array}\right]$$

with



$$\begin{aligned}
\boldsymbol{p}_{6}^{0}=&\left[\begin{array}{c}
a_{2} c_{1} c_{2}+d_{4} c_{1} s_{23}+d_{6}\left(c_{1}\left(c_{23} c_{4} s_{5}+s_{23} c_{5}\right)+s_{1} s_{4} s_{5}\right) \\
a_{2} s_{1} c_{2}+d_{4} s_{1} s_{23}+d_{6}\left(s_{1}\left(c_{23} c_{4} s_{5}+s_{23} c_{5}\right)-c_{1} s_{4} s_{5}\right) \\
a_{2} s_{2}-d_{4} c_{23}+d_{6}\left(s_{23} c_{4} s_{5}-c_{23} c_{5}\right)
\end{array}\right]\\
\boldsymbol{n}_{6}^{0}= & {\left[\begin{array}{c}
c_{1}\left(c_{23}\left(c_{4} c_{5} c_{6}-s_{4} s_{6}\right)-s_{23} s_{5} c_{6}\right)+s_{1}\left(s_{4} c_{5} c_{6}+c_{4} s_{6}\right) \\
s_{1}\left(c_{23}\left(c_{4} c_{5} c_{6}-s_{4} s_{6}\right)-s_{23} s_{5} c_{6}\right)-c_{1}\left(s_{4} c_{5} c_{6}+c_{4} s_{6}\right) \\
s_{23}\left(c_{4} c_{5} c_{6}-s_{4} s_{6}\right)+c_{23} s_{5} c_{6}
\end{array}\right] } \\
\boldsymbol{s}_{6}^{0}= & {\left[\begin{array}{c}
c_{1}\left(-c_{23}\left(c_{4} c_{5} s_{6}+s_{4} c_{6}\right)+s_{23} s_{5} s_{6}\right)+s_{1}\left(-s_{4} c_{5} s_{6}+c_{4} c_{6}\right) \\
s_{1}\left(-c_{23}\left(c_{4} c_{5} s_{6}+s_{4} c_{6}\right)+s_{23} s_{5} s_{6}\right)-c_{1}\left(-s_{4} c_{5} s_{6}+c_{4} c_{6}\right) \\
-s_{23}\left(c_{4} c_{5} s_{6}+s_{4} c_{6}\right)-c_{23} s_{5} s_{6}
\end{array}\right] } \\
\boldsymbol{a}_{6}^{0}= & {\left[\begin{array}{c}
c_{1}\left(c_{23} c_{4} s_{5}+s_{23} c_{5}\right)+s_{1} s_{4} s_{5} \\
s_{1}\left(c_{23} c_{4} s_{5}+s_{23} c_{5}\right)-c_{1} s_{4} s_{5} \\
s_{23} c_{4} s_{5}-c_{23} c_{5}
\end{array}\right] }
\end{aligned}$$

````

# Workspace

The robot arm workspace is the space reached by the origin of the
end-effector frame when the robot arm's joints take all allowable
values. The workspace includes reachable workspace and dexterous
workspace. The latter is the space that the origin of the end-effector
frame can reach with different orientations, while the former is the
space that the origin of the end-effector frame can reach with at least
one orientation.

For an $n$-DOF robot arm, the reachable workspace is formally defined

$$
\{
\boldsymbol{p}_{e}(\boldsymbol{q})\,|\, \quad q_{i m} \leq q_{i} \leq q_{i M} \quad i=1, \ldots, n,
\}
$$

where $q_{i m}\left(q_{i M}\right)$ denotes the minimum (maximum) values
at Joint $i$. The workspace is typically
reported in the data sheet of a robot manufacturer in terms of a
top view and a side view.

In a robot arm, if actual mechanical parameters differ from the
nominal value in data sheet, an error arises between
actual position reached and  theorical position computed via direct
kinematics. Such error is called accuracy. Nowdays, accuracy level of a decent-sized robot arm is typically below 1mm.

Another parameter  of a robot arm is repeatability
which gives a measure of the robot's ability to return to a
previously reached position. Repeatability depends on external disturbance and also on internal controller; it is typically smaller than accuracy. For
instance, for a robot arm with a maximum reach of $1.5 \mathrm{~m}$,
accuracy varies from 0.2 to $1 \mathrm{~mm}$, while
repeatability varies from 0.02 to $0.2 \mathrm{~mm}$.

# Kinematic Redundancy

A robot arm is kinematically redundant when its
DOF $n$ is greater than dimension $m$ of the task space (also named operational space), typically refering to the  motion space of end-effector (frame). Redundancy is a concept relative to the task space; a robot arm can be redundant with respect to a
task and nonredundant with respect to another.

Consider the three-DOF planar arm. If one only cares about the end effector's
position (i.e., the operation space is only the positional space of end effector), the robot arm is a
functional redundancy $(n=3, n=2)$. When one care about both position and angle of the end-effector, then the robot is nonredundant, i.e., $n=3, n=3$. On the
other hand, a four-DOF planar arm is intrinsically redundant
$(n=4, m=3)$.

At this point, a question may arise: Why to
intentionally utilize a redundant robot arm? This is because  redundancy can provide a robot arm with dexterity and
versatility in its motion. The typical example is the
human arm that has seven DOFs: three in the shoulder, one in the elbow
and three in the wrist, without considering the DOFs in the fingers.
Human arm is intrinsically redundant; in fact, if the base and
the hand position and orientation are both fixed,
the elbow can be moved, thanks to the additional available DOF. This can help us, for
instance, avoid obstacles in the workspace. Further,
if a joint of a redundant robot arm reaches its mechanical limit,
there might be other joints that allow execution of the
end-effector motion.

# Modified DH parameters (Optional)

```{figure} ./fk/DHParameter.png
---
width: 50%
name: DHParameter
---
Modified DH parameters (or called Craig's convention). Image from
Wiki
```

Some books use modified (proximal) DH parameters \[John J. Craig,
Introduction to Robotics: Mechanics and Control (3rd Edition)\]. The
difference between the classic  DH parameters and the modified
DH parameters are the locations of the coordinate to
each links and the order of the performed transformations.

Compared with the classic DH parameters, the coordinates of frame,
$O_{i-1}$, is put on Joint $i-1$, not the Joint $i$ in classic DH
convention;the coordinates of frame, $O_{i}$, is put on Joint $i$, not
the Joint $i+1$ in classic DH convention. Another difference is that
according to the modified convention, the transform matrix is given by
the following order of operations:

$$
T^{i-1}_i=\text{Rot}_{x_{i-1}}(\alpha_{i-1})\text{Trans}_{x_{i-1}}(a_{i-1})\text{Rot}_{z_{i}}(\theta_{i})\text{Trans}_{z_{i}}(d_{i})
$$

One example of using the above modified DH convention is Franka-Emika
Panda robot arm, an increasingly popular robot for research and
teaching. It has 7 joints which make it a redundant robot, that is, it
has more joints than it needs to achieve an arbitrary position and
orientation in the Cartesian workspace.

The DH parameter defined for Panda Robot arm is as follows:
[https://frankaemika.github.io/docs/control_parameters.html](https://frankaemika.github.io/docs/control_parameters.html). Please
find a great tutorial (using Python) about Panda Arm's forward
kinematics at

[https://github.com/jhavl/dkt](https://github.com/jhavl/dkt)

```{figure} ./fk/dh-diagram.png
---
width: 50%
name: dh-diagram
---
Panda's kinematic chain
```

```{figure} ./fk/cover.png
---
width: 50%
name: cover
---
Panda's kinematic chain (from Peter Corke's
Tutorial)
```


