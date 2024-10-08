---
author:
- Wanxin Jin
date: Aug. 24, 2023 Aug. 29, 2023 Aug. 31, 2023
title: "Lecture 3-5: Basic Kinematics"
---

# Basic Kinematics


The pose of a rigid body in 3D space is represented by its position and
orientation with respect to a reference frame. As shown in {numref}`pose_rigid_body`, let
$O-xyz$ be the reference frame and
$\{\boldsymbol{x}, \boldsymbol{y}, \boldsymbol{z}\}$ be the unit vectors
of the frame axes. 
To represent the pose of the rigid body, we
pick a fixed point $O^\prime$ on the rigid body, and attach an
 body frame $O^{\prime}-x^{\prime} y^{\prime} z^{\prime}$
to the body, with origin in $O^{\prime}$ and
$\{\boldsymbol{x}^{\prime}, \boldsymbol{y}^{\prime}, \boldsymbol{z}^{\prime}\}$
being the unit vectors.



```{figure} ./kinematics/pose_rigid_body.jpg
---
width: 80%
name: pose_rigid_body
---
Pose of a rigid body in a reference frame
```


To describe the position of the rigid body,  define a position vector
$\boldsymbol{o}^{\prime}$ pointing from the reference frame origin $O$
to the body frame origin $O'$. We express $\boldsymbol{o}^{\prime}$ in
terms of $\{\boldsymbol{x}, \boldsymbol{y}, \boldsymbol{z}\}$ of
reference frame:

$$\boldsymbol{o}^{\prime}=o_{x}^{\prime} \boldsymbol{x}+o_{y}^{\prime} \boldsymbol{y}+o_{z}^{\prime} \boldsymbol{z}
:=\left[\begin{array}{c}
o_{x}^{\prime} \\
o_{y}^{\prime} \\
o_{z}^{\prime}
\end{array}\right]
=\left[\begin{array}{c}
{\boldsymbol{o}^{\prime}}^T \boldsymbol{x} \\
{\boldsymbol{o}^{\prime}}^T \boldsymbol{y} \\
{\boldsymbol{o}^{\prime}}^T \boldsymbol{z}
\end{array}\right]$$

where $o_{x}^{\prime}, o_{y}^{\prime}, o_{z}^{\prime}$, called the coordinates of the vector $\boldsymbol{o}^{\prime}$, are projections
of $\boldsymbol{o}^{\prime}$ in 
$\{\boldsymbol{x}, \boldsymbol{y}, \boldsymbol{z}\}$.

To describe the orientation of the rigid body, we find the coordinates
of each unit axis 
$\{\boldsymbol{x}^{\prime}, \boldsymbol{y}^{\prime}, \boldsymbol{z}^{\prime}\}$
of the body frame, in the reference frame:

$$\begin{aligned}
& \boldsymbol{x}^{\prime}=x_{x}^{\prime} \boldsymbol{x}+x_{y}^{\prime} \boldsymbol{y}+x_{z}^{\prime} \boldsymbol{z} \\
& \boldsymbol{y}^{\prime}=y_{x}^{\prime} \boldsymbol{x}+y_{y}^{\prime} \boldsymbol{y}+y_{z}^{\prime} \boldsymbol{z} \\
& \boldsymbol{z}^{\prime}=z_{x}^{\prime} \boldsymbol{x}+z_{y}^{\prime} \boldsymbol{y}+z_{z}^{\prime} \boldsymbol{z} 
\end{aligned}$$

We write the above into the following compact notation:

$$\boldsymbol{R}=\left[\begin{array}{lll}
\boldsymbol{x}^{\prime} & \boldsymbol{y}^{\prime} & \boldsymbol{z}^{\prime} 
\end{array}\right]=\left[\begin{array}{ccc}
x_{x}^{\prime} & y_{x}^{\prime} & z_{x}^{\prime} \\
x_{y}^{\prime} & y_{y}^{\prime} & z_{y}^{\prime} \\
x_{z}^{\prime} & y_{z}^{\prime} & z_{z}^{\prime}
\end{array}\right]=\left[\begin{array}{lll}
\boldsymbol{x}^{\prime T} \boldsymbol{x} & \boldsymbol{y}^{\prime T} \boldsymbol{x} & \boldsymbol{z}^{\prime T} \boldsymbol{x} \\
\boldsymbol{x}^{\prime T} \boldsymbol{y} & \boldsymbol{y}^{\prime T} \boldsymbol{y} & \boldsymbol{z}^{\prime T} \boldsymbol{y} \\
\boldsymbol{x}^{\prime T} \boldsymbol{z} & \boldsymbol{y}^{\prime T} \boldsymbol{z} & \boldsymbol{z}^{\prime T} \boldsymbol{z}
\end{array}\right]$$(equ.rotation_matrix)

with each column being the coordinates of each unit vector 
$\{\boldsymbol{x}^{\prime}, \boldsymbol{y}^{\prime}, \boldsymbol{z}^{\prime}\}$.


:::{important}
The above $\boldsymbol{R}$ is called rotation matrix. Some properties of
Rotation Matrix:

-   Mutual orthogonality:
    $\boldsymbol{x}^{\prime T} \boldsymbol{y}^{\prime}=0$,
    $\boldsymbol{y}^{\prime T} \boldsymbol{z}^{\prime}=0$, and
    $\boldsymbol{z}^{\prime T} \boldsymbol{x}^{\prime}=0$

-   Unity: $\boldsymbol{x}^{\prime T} \boldsymbol{x}^{\prime}=1$,
    $\boldsymbol{y}^{\prime T} \boldsymbol{y}^{\prime}=1$, and
    $\boldsymbol{z}^{\prime T} \boldsymbol{z}^{\prime}=1$

-   Orthogonal matrix:
    $\boldsymbol{R}^{T} \boldsymbol{R}=\boldsymbol{I}_{3} \quad \rightarrow \quad \boldsymbol{R}^{T}=\boldsymbol{R}^{-1}$

-   The determinant of $\boldsymbol{R}$: $|\det \boldsymbol{R}|=1$
:::













# Elementary Rotations


Consider the origin of the body frame coincides with the origin of the
reference frame. The orientation of a body frame can be obtained by the
rotating the reference frame about its axes. Below, we
consider right-handed rotation convention: rotations are positive if
counter-clockwise about a axis.


```{figure} ./kinematics/elementary_rotations.jpg
---
width: 60%
name: elementary_rotations
---
Rotation of frame $O- x y z$ by an angle $\alpha$ about axis
$\boldsymbol{z}$
```



As shown in  {numref}`elementary_rotations`, suppose the body frame
$O- x^{\prime} y^{\prime} z^{\prime}$ is a result of rotating a
reference frame $O- x y z$ by an angle $\alpha$ about the axis
$\boldsymbol{z}$. Then, the rotation matrix of
$O- x^{\prime} y^{\prime} z^{\prime}$ is

$$\boldsymbol{R}_{z}(\alpha)=\left[\begin{array}{ccc}
\cos \alpha & -\sin \alpha & 0 \\
\sin \alpha & \cos \alpha & 0 \\
0 & 0 & 1
\end{array}\right]$$

Similarly, the rotation matrix for an angle $\beta$ about axis
$\boldsymbol{y}$ is

$$\boldsymbol{R}_{y}(\beta)  =\left[\begin{array}{ccc}
\cos \beta & 0 & \sin \beta \\
0 & 1 & 0 \\
-\sin \beta & 0 & \cos \beta
\end{array}\right]$$

and the rotation matrix for an angle $\gamma$ about axis
$\boldsymbol{x}$ is

$$\boldsymbol{R}_{x}(\gamma)  =\left[\begin{array}{ccc}
1 & 0 & 0 \\
0 & \cos \gamma & -\sin \gamma \\
0 & \sin \gamma & \cos \gamma
\end{array}\right] .$$

<!-- ```{note}
Takeaway: the rotation matrix can be interpreted geometrically as a
rotation about an axis in space needed to align a reference frame to a
body frame.
``` -->

# Transformation via Rotation Matrix
## Passive Rotation



```{figure} ./kinematics/coordinate_transformation.jpg
---
width: 60%
name: coordinate_transformation
---
Representation of a point $P$ in two different coordinate
frames
```


As in {numref}`coordinate_transformation`,
consider that the origin of the body frame coincides with the origin of
the reference frame, and that a point $P$ expressed in
$O- x y z$ is




$$\boldsymbol{p}=p_{x} \boldsymbol{x} +
p_{y} \boldsymbol{y} +
p_{z}\boldsymbol{z} 
=\left[\begin{array}{c}
p_{x} \\
p_{y} \\
p_{z}
\end{array}\right]$$ 

The same point can also be expressed in
$O- x^{\prime} y^{\prime} z^{\prime}$ as

$$\boldsymbol{p}=p_{x}^{\prime} \boldsymbol{x}^{\prime} +
p_{y}^{\prime} \boldsymbol{y}^{\prime} +
p_{z}^{\prime}\boldsymbol{z}^{\prime} =\boldsymbol{R}\left[\begin{array}{c}
p_{x}^{\prime} \\
p_{y}^{\prime} \\
p_{z}^{\prime}
\end{array}\right]$$
where we have applied the definition of the rotation matrix in {eq}`equ.rotation_matrix`. Therefore, one has

$$\left[\begin{array}{c}
p_{x} \\
p_{y} \\
p_{z}
\end{array}\right]=\boldsymbol{R}\left[\begin{array}{c}
p_{x}^{\prime} \\
p_{y}^{\prime} \\
p_{z}^{\prime}
\end{array}\right]$$

and we simply write the above as


<!-- To establish the relationship between two coordinates, we need to
replace the unite vectors
$\{\boldsymbol{x}^{\prime}, \boldsymbol{y}^{\prime}, \boldsymbol{x}^{\prime}\}$
of the body frame with their coordinates in the reference frame using
the rotation matrix $\boldsymbol{R}$. This leads to-->

$$
    \boldsymbol{p}=\boldsymbol{R}\boldsymbol{p}^{\prime}
$$(equ.transform) 

```{note}
$\boldsymbol{R}$ is a transformation (mapping) for the
coordinates of the same vector, from frame $O- x^{\prime} y^{\prime} z^{\prime}$ to the
to frame $O- x y z$.
```

## Active Rotation




In another perspective to look at {eq}`equ.transform`, we can interpret $\boldsymbol{p}^{\prime}$
be a vector also expressed in the reference frame $O- x y z$, that is,

$$
\boldsymbol{p}=p_{x}^{\prime} \boldsymbol{x} +
p_{y}^{\prime} \boldsymbol{y} +
p_{z}^{\prime}\boldsymbol{z}
$$


$\boldsymbol{R} \boldsymbol{p}^{\prime}$ turns the vector $\boldsymbol{p}^{\prime}$ to a new vector $\boldsymbol{p}$ in the same reference frame $O- x y z$, according to the
matrix $\boldsymbol{R}$.


```{note}
$\boldsymbol{R}$ can be also interpreted as the rotation
operator to rotate  a vector to a new vector in the same
coordinate system, where both vectors are expressed.
```


# Composition of Rotations

## Rotation around Current Frame

Let $O-{x_{0} y_{0} z_{0}}$,
$O- x_{1} y_{1} z_{1}, O- x_{2} y_{2} z_{2}$ be three frames with common
origin $O$. The same vector $\boldsymbol{p}$ can be expressed in each of
the above frames, denoted as
$\boldsymbol{p}^{0}, \boldsymbol{p}^{1}, \boldsymbol{p}^{2}$,
respectively. Let $\boldsymbol{R}_{i}^{j}$ denote the rotation matrix of
frame $i$ w.r.t. frame $j$. One has

$$\boldsymbol{p}^{1}=\boldsymbol{R}_{2}^{1} \boldsymbol{p}^{2}, \quad \boldsymbol{p}^{0}=\boldsymbol{R}_{1}^{0} \boldsymbol{p}^{1}, \quad \boldsymbol{p}^{0}=\boldsymbol{R}_{2}^{0} \boldsymbol{p}^{2}$$

From the above equations, one can derive

$$\boldsymbol{R}_{2}^{0}=\boldsymbol{R}_{1}^{0} \boldsymbol{R}_{2}^{1}$$

which is the composition of two rotations. $\boldsymbol{R}_{2}^{0}$ can
be thought of as first rotating $O- x_{0} y_{0} z_{0}$ to
$O- x_{1} y_{1} z_{1}$, according to $\boldsymbol{R}_{1}^{0}$, and then
rotating $O- x_{1} y_{1} z_{1}$ to $O - x_{2} y_{2} z_{2}$, according to
$\boldsymbol{R}_{2}^{1}$. Here, the first rotation matrix
$\boldsymbol{R}_{1}^{0}$ is expressed in $O- x_{0} y_{0} z_{0}$, and the
second rotation matrix $\boldsymbol{R}_{2}^{1}$ is expressed in
$O- x_{1} y_{1} z_{1}$ (we will call it current frame).



```{important}
Hence, we can conclude the following *postmultiplication rule*:

-   The frame with respect to which a rotation occurs (and the rotation
    matrix is 'expressed') is called the current frame.

-   The composition of each rotation around the current frame is
    obtained by *postmultiplication* of the rotation matrices in order.
```




## Rotation around Fixed Frame

Let's consider the following case. Suppose that we first rotate
$O- x_{0} y_{0} z_{0}$ to frame $O- x_{1} y_{1} z_{1}$, according to
$\boldsymbol{R}_{1}^{0}$ (which is expressed in the initial frame
$O- x_{0} y_{0} z_{0}$). Next, we rotate the current frame
$O- x_{1} y_{1} z_{1}$ to the frame $O- x_{2} y_{2} z_{2}$, according to
a new rotation matrix

$$\boldsymbol{R}_{1,2}^0$$

which is 'expressed' *still* in the initial frame $O- x_{0} y_{0} z_{0}$
(instead of the current frame $O- x_{1} y_{1} z_{1}$ itself). We call
this type of rotation \"the rotation around the fixed frame\", i.e., the rotation happens in the 'very original' frame.

To  apply the *postmultipcation rule*, we need to find out a 
rotation $\boldsymbol{R}_{2}^1$, which is equivalent to $\boldsymbol{R}_{1,2}^0$, but 'expressed' in the current
frame $O- x_{1} y_{1} z_{1}$.



```{note}
To do so, let's consider any vector $\boldsymbol{p}^1$ expressed in frame $O- x_{1} y_{1} z_{1}$. We follow the following procedure.

-   Step 1: passive rotation. Let's first transform $\boldsymbol{p}^1$ to the corrdinates in frame  0: $\boldsymbol{R}^0_1\boldsymbol{p}^1$

-   Step 2: active rotation. In frame 0, use the rotation matrix $\boldsymbol{R}_{1,2}^0$ to turn   $\boldsymbol{R}^0_1\boldsymbol{p}^1$ into a new vector but still in frame 0: 

$$\boldsymbol{R}_{1,2}^0\boldsymbol{R}^0_1\boldsymbol{p}^1$$

-   Step 3: passive rotation. Since  the above $\boldsymbol{R}_{1,2}^0\boldsymbol{R}^0_1\boldsymbol{p}^1$ is in frame 0, we want to transform it back to frame 1, by


$$(\boldsymbol{R}^0_1)^T\boldsymbol{R}_{1,2}^0\boldsymbol{R}^0_1\boldsymbol{p}^1$$

-   Step 4: active rotation. Now,  $(\boldsymbol{R}^0_1)^T\boldsymbol{R}_{1,2}^0\boldsymbol{R}^0_1\boldsymbol{p}^1$ is in frame 1, which is definitely different from our original $\boldsymbol{p}^1$.  So, we can consider this difference is due to we have applied an active rotation $\boldsymbol{R}^1_2$ to turn $\boldsymbol{p}^1$ into $(\boldsymbol{R}^0_1)^T\boldsymbol{R}_{1,2}^0\boldsymbol{R}^0_1\boldsymbol{p}^1$. That is to say,


$$\boldsymbol{R}^1_2\boldsymbol{p}^1=(\boldsymbol{R}^0_1)^T\boldsymbol{R}_{1,2}^0\boldsymbol{R}^0_1\boldsymbol{p}^1$$

```






<!-- This will lead to the concept of
similarity transformation derived using the perspective of \"rotation
matrix as a active transformation\" (how?).

:::{important}
Given a rotation matrix $\boldsymbol{R}^0$ 'expressed' in the frame
$O- x_{0} y_{0} z_{0}$, the equivalent rotation matrix
$\boldsymbol{R}^1$ 'expressed' in another frame $O- x_{1} y_{1} z_{1}$,
where $O- x_{1} y_{1} z_{1}$ is rotated by $\boldsymbol{R}_1^0$ from
$O- x_{0} y_{0} z_{0}$ is

$$\boldsymbol{R}^1=(\boldsymbol{R}_1^0)^T\boldsymbol{R}^0\boldsymbol{R}_1^0$$
::: -->

<!-- Back to the matrix $\boldsymbol{R}_{1,2}^0$, the equivalent rotation
matrix 'expressed' in the current frame $O- x_{1} y_{1} z_{1}$ is -->

Then, we have

$$\boldsymbol{R}_{2}^1=(\boldsymbol{R}_1^0)^T\boldsymbol{R}_{1,2}^0\boldsymbol{R}_1^0$$

We call the above equation the "similarity transformation": it is used to transform a rotation $\boldsymbol{R}_{1,2}^0$ from frame 0 to frame 1.


Following the *postmultipcation rule*, we can follow the to obtain
the total rotation

$$\boldsymbol{R}_{2}^{0}=\boldsymbol{R}_{1}^{0} \boldsymbol{R}_{2}^1=\boldsymbol{R}_{1}^{0} (\boldsymbol{R}_1^0)^T\boldsymbol{R}_{1,2}^0\boldsymbol{R}_1^0=\boldsymbol{R}_{1,2}^0\boldsymbol{R}_1^0$$


```{important}
Hence, we can conclude the following *premultiplication rule*:

-   The same frame with respect to which a rotation occurs (and the
    rotation matrix is 'expressed') is called the fixed frame.

-   The composition of each rotation around the fixed frame is obtained
    by *premultiplication* of the rotation matrices in order.
```





Note: composition of rotations not commutative!

# Rotation Parameterization (Representation)

A rotation matrix has 9 elements, its mutual orthogonality and unity
properties bring 6 constraints. Thus, each robotion matrix has 3DOFs! We only need to use three independent parameters
to represent a rotation matrix.

## Euler Angles

A rotation in space can be understood as a sequence of three elementary
rotations. Such rotation representation is called Euler-angles represnetation, and the elementary rotation angles are called Euler angles. To fully describe all possible orientations, two successive
rotations should not be made around parallel axes. In the following, two sets of Euler angles
are used; namely, the ZYZ Euler angles and the Roll-Pitch-Yaw (RPY) (or ZYX Euler angles).



```{figure} ./kinematics/zyz_euler_angles.jpg
---
width: 80%
name: zyz_euler_angles
---
ZYZ Euler angles
```



**ZYZ Euler Angles**: the rotation described by ZYZ Euler angles is done
by first, rotating the current frame by $\varphi$ about axis
$\boldsymbol{z}$, second, rotating the current frame by angle
$\vartheta$ about axis $\boldsymbol{y}^{\prime}$, and then, rotating the
current frame by the angle $\psi$ about axis
$\boldsymbol{z}^{\prime\prime}$. Thus, the rotation matrix from ZYZ
Euler angles
$\boldsymbol{\phi}=\left[\begin{array}{lll}\varphi & \vartheta & \psi\end{array}\right]^{T}$
is

$$\begin{aligned}
\boldsymbol{R}(\boldsymbol{\phi}) & =\boldsymbol{R}_{z}(\varphi) \boldsymbol{R}_{y^{\prime}}(\vartheta) \boldsymbol{R}_{z^{\prime \prime}}(\psi) =\left[\begin{array}{ccc}
c_{\varphi} c_{\vartheta} c_{\psi}-s_{\varphi} s_{\psi} & -c_{\varphi} c_{\vartheta} s_{\psi}-s_{\varphi} c_{\psi} & c_{\varphi} s_{\vartheta} \\
s_{\varphi} c_{\vartheta} c_{\psi}+c_{\varphi} s_{\psi} & -s_{\varphi} c_{\vartheta} s_{\psi}+c_{\varphi} c_{\psi} & s_{\varphi} s_{\vartheta} \\
-s_{\vartheta} c_{\psi} & s_{\vartheta} s_{\psi} & c_{\vartheta}
\end{array}\right] .
\end{aligned}$$

Inversely, given a rotation matrix

$$\boldsymbol{R}=\left[\begin{array}{lll}
r_{11} & r_{12} & r_{13} \\
r_{21} & r_{22} & r_{23} \\
r_{31} & r_{32} & r_{33}
\end{array}\right] .$$

the corresponding ZYZ Euler angles
$\boldsymbol{\phi}=\left[\begin{array}{lll}\varphi & \vartheta & \psi\end{array}\right]^{T}$
is

$$\begin{aligned}
& \varphi=\operatorname{Atan} 2\left(r_{23}, r_{13}\right) \\
& \vartheta=\operatorname{Atan} 2\left(\sqrt{r_{13}^{2}+r_{23}^{2}}, r_{33}\right) \\
& \psi=\operatorname{Atan} 2\left(r_{32},-r_{31}\right) .
\end{aligned}$$

if $s_{\vartheta}=0$, i.e., $(r_{23}, r_{13})\not=(0,0)$; otherwise,
only the sum or difference of $\varphi$ and $\psi$ is determined (why?)

**RPY (ZYX Euler angles)**: I think about RPY rotations is a process of "a fighter jet from parking --> taxiing (yaw) --> take-off (pitch) --> fighting (roll)". Formally, RPY is obtained by
first, rotating the reference frame by the angle $\varphi$ about the
current axis $\boldsymbol{z}$ (yaw), then, rotating the current frame
by $\vartheta$ about the current axis $\boldsymbol{y}$ (pitch), and then,
rotating the current frame by $\psi$ about current axis $\boldsymbol{x}$
(roll). Thus, the resulting rotation matrix is obtained by
postmultiplication rule.

$$\begin{aligned}
\boldsymbol{R}(\boldsymbol{\phi}) & =\boldsymbol{R}_{z}(\varphi) \boldsymbol{R}_{y}(\vartheta) \boldsymbol{R}_{x}(\psi)  =\left[\begin{array}{ccc}
c_{\varphi} c_{\vartheta} & c_{\varphi} s_{\vartheta} s_{\psi}-s_{\varphi} c_{\psi} & c_{\varphi} s_{\vartheta} c_{\psi}+s_{\varphi} s_{\psi} \\
s_{\varphi} c_{\vartheta} & s_{\varphi} s_{\vartheta} s_{\psi}+c_{\varphi} c_{\psi} & s_{\varphi} s_{\vartheta} c_{\psi}-c_{\varphi} s_{\psi} \\
-s_{\vartheta} & c_{\vartheta} s_{\psi} & c_{\vartheta} c_{\psi}
\end{array}\right] .
\end{aligned}$$

Inversely, given rotation matrix

$$\boldsymbol{R}=\left[\begin{array}{lll}
r_{11} & r_{12} & r_{13} \\
r_{21} & r_{22} & r_{23} \\
r_{31} & r_{32} & r_{33}
\end{array}\right],$$

the corresponding ZYX Euler Angles is

$$\begin{aligned}
& \varphi=\operatorname{Atan} 2\left(r_{21}, r_{11}\right) \\
& \vartheta=\operatorname{Atan} 2\left(-r_{31}, \sqrt{r_{32}^{2}+r_{33}^{2}}\right) \\
& \psi=\operatorname{Atan} 2\left(r_{32}, r_{33}\right) .
\end{aligned}$$

if $c_{\vartheta}\not=0$; otherwise, only the sum or difference of
$\varphi$ and $\psi$ can be determined.

## Angle Axis

Given a rotation 
defined by an angle $\vartheta$ around a unit axis vector
$\boldsymbol{r}=\left[r_{x}, r_{y},  r_{z}\right]^{T}$  in
the reference frame $O- x y z$, the  angle-axis  representation is defined as $(\vartheta, \boldsymbol{r})$.



```{figure} ./kinematics/angle_axis.jpg
---
width: 60%
name: zyz_euler_angles2
---
Rotation of an angle about an axis.
```




To derive the rotation matrix
$\boldsymbol{R}(\vartheta, \boldsymbol{r})$ from
$(\vartheta, \boldsymbol{r})$, we first create a body frame
$O_1- x_1 y_1 z_1$ with its axis $\boldsymbol{z_1}$ aligned with
$\boldsymbol{r}$. Then, the rotation of $\vartheta$ about $\boldsymbol{r}$ can be  'expressed' in the current frame:
$\boldsymbol{R}_{z}(\vartheta)$. Since our goal is to find the rotation
matrix $\boldsymbol{R}(\vartheta, \boldsymbol{r})$ 'expressed' in the
reference frame $O- x y z$, we similarly apply the previous similarity
transformation:

$$\boldsymbol{R}(\vartheta, \boldsymbol{r})= \boldsymbol{R}_{1} \boldsymbol{R}_{z}(\vartheta) (\boldsymbol{R}_{1})^T$$

with $\boldsymbol{R}_{1}$ denoting the rotation of the body frame in reference framework. From  {numref}`zyz_euler_angles2`, we have

$$\boldsymbol{R}_{1}=\boldsymbol{R}_{z}(\alpha) \boldsymbol{R}_{y}(\beta)$$

In sum, the rotation matrix is

$$\boldsymbol{R}(\vartheta, \boldsymbol{r})=\left[\begin{array}{ccc}
r_{x}^{2}\left(1-c_{\vartheta}\right)+c_{\vartheta} & r_{x} r_{y}\left(1-c_{\vartheta}\right)-r_{z} s_{\vartheta} & r_{x} r_{z}\left(1-c_{\vartheta}\right)+r_{y} s_{\vartheta} \\
r_{x} r_{y}\left(1-c_{\vartheta}\right)+r_{z} s_{\vartheta} & r_{y}^{2}\left(1-c_{\vartheta}\right)+c_{\vartheta} & r_{y} r_{z}\left(1-c_{\vartheta}\right)-r_{x} s_{\vartheta} \\
r_{x} r_{z}\left(1-c_{\vartheta}\right)-r_{y} s_{\vartheta} & r_{y} r_{z}\left(1-c_{\vartheta}\right)+r_{x} s_{\vartheta} & r_{z}^{2}\left(1-c_{\vartheta}\right)+c_{\vartheta}
\end{array}\right] .$$

with

$$\begin{gathered}
\sin \alpha=\frac{r_{y}}{\sqrt{r_{x}^{2}+r_{y}^{2}}} \quad \cos \alpha=\frac{r_{x}}{\sqrt{r_{x}^{2}+r_{y}^{2}}} \quad
\sin \beta=\sqrt{r_{x}^{2}+r_{y}^{2}} \quad \cos \beta=r_{z} .
\end{gathered}$$

Also, the following property holds:

$$\boldsymbol{R}(-\vartheta,-\boldsymbol{r})=\boldsymbol{R}(\vartheta, \boldsymbol{r})$$

Inversely, given rotation matrix


$$\boldsymbol{R}=\left[\begin{array}{lll}
r_{11} & r_{12} & r_{13} \\
r_{21} & r_{22} & r_{23} \\
r_{31} & r_{32} & r_{33}
\end{array}\right],$$

then,

$$\begin{aligned}
 \vartheta=\cos ^{-1}\left(\frac{r_{11}+r_{22}+r_{33}-1}{2}\right) \quad \boldsymbol{r}=\frac{1}{2 \sin \vartheta}\left[\begin{array}{l}
r_{32}-r_{23} \\
r_{13}-r_{31} \\
r_{21}-r_{12}
\end{array}\right],
\end{aligned}$$ 


if $\sin \vartheta \neq 0$; otherwise,
$(\vartheta, \boldsymbol{r})$ is undefined.

## Quaternion
Given a rotation 
defined by an angle $\vartheta$ around a unit axis vector
$\boldsymbol{r}=\left[r_{x}, r_{y},  r_{z}\right]^{T}$  in
the reference frame $O- x y z$,  its quaternion is defined as

$$\mathcal{Q}=\{\eta, \boldsymbol{\epsilon}\}, \quad\text{with}
\quad
\eta=\cos \frac{\vartheta}{2}, \quad \boldsymbol{\epsilon}=\sin \frac{\vartheta}{2} \boldsymbol{r}$$

$\eta$ is called the scalar part while
$\boldsymbol{\epsilon}=\left[\epsilon_{x} , \epsilon_{y} , \epsilon_{z}, \right]^{T}$
is called vector part of the quaternion, and
$\eta^{2}+\epsilon_{x}^{2}+\epsilon_{y}^{2}+\epsilon_{z}^{2}=1$. It is
worth remarking that, unlike the angle/axis representation, a rotation
by $(-\vartheta, -\boldsymbol{r})$ gives the same quaternion as that by
$(\vartheta, \boldsymbol{r})$.

The rotation matrix corresponding to a given quaternion is

$$\boldsymbol{R}(\eta, \boldsymbol{\epsilon})=\left[\begin{array}{ccc}
2\left(\eta^{2}+\epsilon_{x}^{2}\right)-1 & 2\left(\epsilon_{x} \epsilon_{y}-\eta \epsilon_{z}\right) & 2\left(\epsilon_{x} \epsilon_{z}+\eta \epsilon_{y}\right) \\
2\left(\epsilon_{x} \epsilon_{y}+\eta \epsilon_{z}\right) & 2\left(\eta^{2}+\epsilon_{y}^{2}\right)-1 & 2\left(\epsilon_{y} \epsilon_{z}-\eta \epsilon_{x}\right) \\
2\left(\epsilon_{x} \epsilon_{z}-\eta \epsilon_{y}\right) & 2\left(\epsilon_{y} \epsilon_{z}+\eta \epsilon_{x}\right) & 2\left(\eta^{2}+\epsilon_{z}^{2}\right)-1
\end{array}\right]$$

Inversely, given rotation matrix

$$\boldsymbol{R}=\left[\begin{array}{lll}
r_{11} & r_{12} & r_{13} \\
r_{21} & r_{22} & r_{23} \\
r_{31} & r_{32} & r_{33}
\end{array}\right],$$ 

the quaternion is 

$$\begin{aligned}
\eta  =\frac{1}{2} \sqrt{r_{11}+r_{22}+r_{33}+1} \qquad
\boldsymbol{\epsilon}  =\frac{1}{2}\left[\begin{array}{l}
\operatorname{sgn}\left(r_{32}-r_{23}\right) \sqrt{r_{11}-r_{22}-r_{33}+1} \\
\operatorname{sgn}\left(r_{13}-r_{31}\right) \sqrt{r_{22}-r_{33}-r_{11}+1} \\
\operatorname{sgn}\left(r_{21}-r_{12}\right) \sqrt{r_{33}-r_{11}-r_{22}+1}
\end{array}\right],
\end{aligned}$$ 

where $\operatorname{sgn}(x)=1$ for $x \geq 0$ and
$\operatorname{sgn}(x)=-1$ for $x<0$.

Similar to the inverse of a rotation matrix $\boldsymbol{R}$, a quaternion also has its own inverse,  denoted as $\mathcal{Q}^{-1}$, corresponding to $\boldsymbol{R}^{-1}=\boldsymbol{R}^{T}$. Quanternion inverse can be easily computed as

$$\mathcal{Q}^{-1}=\{\eta,-\boldsymbol{\epsilon}\}$$

Let $\mathcal{Q}_{1}=\left\{\eta_{1}, \boldsymbol{\epsilon}_{1}\right\}$
and $\mathcal{Q}_{2}=\left\{\eta_{2}, \boldsymbol{\epsilon}_{2}\right\}$
denote the quaternions corresponding to the rotation matrices
$\boldsymbol{R}_{1}$ and $\boldsymbol{R}_{2}$, respectively. The
quaternion corresponding to the product
$\boldsymbol{R}_{1} \boldsymbol{R}_{2}$ is given by

$$\mathcal{Q}_{1} * \mathcal{Q}_{2}=\left\{\eta_{1} \eta_{2}-\boldsymbol{\epsilon}_{1}^{T} \boldsymbol{\epsilon}_{2}, \eta_{1} \boldsymbol{\epsilon}_{2}+\eta_{2} \boldsymbol{\epsilon}_{1}+\boldsymbol{\epsilon}_{1} \times \boldsymbol{\epsilon}_{2}\right\}$$
where the quaternion product operator \"\*\" has been formally
introduced.

# Homogeneous Transformations


```{figure} ./kinematics/homogenenous_transformation.jpg
---
width: 80%
name: homogenenous_transformation
---
Representation of a point $P$ in different coordinate
frames
```




Consider a reference frame $O_{0}- x_{0} y_{0} z_{0}$ and body frame 
$O_{1}- x_{1} y_{1} z_{1}$. Let $\boldsymbol{o}_{1}^{0}$ be the
coordinate of the origin of body frame in reference frame, and
$\boldsymbol{R}_{1}^{0}$ be the rotation matrix of body frame in reference frame. Let $\boldsymbol{p}^{0}$ be the coordinate of any point $P$
in reference frame, and $\boldsymbol{p}^{1}$ be the coordinate of the
same point $P$ in body frame. Then, we have the following relationship

$$\boldsymbol{p}^{0}=\boldsymbol{o}_{1}^{0}+\boldsymbol{R}_{1}^{0} \boldsymbol{p}^{1}$$

To achieve a compact representation, we first introduce the concept of "homogeneous coordinate" of a
3D vector $\boldsymbol{p}$, defined as

$$\widetilde{\boldsymbol{p}}=\left[\begin{array}{l}
\boldsymbol{p} \\
1
\end{array}\right] .$$

Then, the above relationship can be compactly written as

$$\widetilde{\boldsymbol{p}}^{0}=\boldsymbol{T}_{1}^{0} \widetilde{\boldsymbol{p}}^{1}$$

with

$$\boldsymbol{T}_{1}^{0}=\left[\begin{array}{cc}
\boldsymbol{R}_{1}^{0} & \boldsymbol{o}_{1}^{0} \\
\mathbf{0}^{T} & 1
\end{array}\right]$$

which is called *homogeneous transformation matrix*. 

<!-- All homogeneous -->
<!-- transformation matrices belong to the *Special Euclidean Group*, denoted -->
<!-- as $\boldsymbol{A}_{1}^{0}\in SE(3)$. -->

The inverse  of the homogeneous transformation $\boldsymbol{T}_{1}^{0}$,

$$
\widetilde{\boldsymbol{p}}^{1}=\boldsymbol{T}_{0}^{1} \widetilde{\boldsymbol{p}}^{0}=\left(\boldsymbol{T}_{1}^{0}\right)^{-1} \widetilde{\boldsymbol{p}}^{0}
$$


with 

$$\boldsymbol{T}_{0}^{1}=\left[\begin{array}{cc}
\boldsymbol{R}_{1}^{0 T} & -\boldsymbol{R}_{1}^{0 T} \boldsymbol{o}_{1}^{0} \\
\mathbf{0}^{T} & 1
\end{array}\right]=\left[\begin{array}{cc}
\boldsymbol{R}_{0}^{1} & -\boldsymbol{R}_{0}^{1} \boldsymbol{o}_{1}^{0} \\
\mathbf{0}^{T} & 1
\end{array}\right].$$

Notice that for the homogeneous transformation matrix the orthogonality
property does not hold:

$$\boldsymbol{T}^{-1} \neq \boldsymbol{T}^{T}$$

Following the derivation of sequential rotation transformation, it is easy to
verify that a sequence of homogeneous transformations can be composed by
the *postmultiplication rule*

$$\widetilde{\boldsymbol{p}}^{0}=\boldsymbol{T}_{1}^{0} \boldsymbol{T}_{2}^{1} \ldots \boldsymbol{T}_{n}^{n-1} \widetilde{\boldsymbol{p}}^{n}$$

where $\boldsymbol{T}_{i}^{i-1}$ denotes the homogeneous transformation
of frame $i$ with respect to the *current* frame $i-1$.
