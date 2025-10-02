---
author:
- Wanxin Jin
date: Sep. 19, 2023
title: "Lecture 10: Derivative of Transformation"
---

# Velocity Kinematics
Notation convention warning of this lecture: if a vector has no superscript, it means this vector is expressed in the world frame by default; otherwise, the frame the coordinates of this vector is with respect to will be specified on its superscript.


## Derivative of Rotation Matrix
Consider a time-varying rotation matrix
$\boldsymbol{R}=\boldsymbol{R}(t)$. Based on the property of rotation matrix, we always have 

$$\boldsymbol{R}(t) \boldsymbol{R}^{T}(t)=\boldsymbol{I}$$

Differentiating the above equation with respect to time gives

$$\dot{\boldsymbol{R}}(t) \boldsymbol{R}^{T}(t)+\boldsymbol{R}(t) \dot{\boldsymbol{R}}^{T}(t)=\boldsymbol{0}$$

Let's define a new matrix

$$\boldsymbol{S}=\dot{\boldsymbol{R}} \boldsymbol{R}^{T}$$(equ.skew) 

Then, 

$$
\boldsymbol{S}+\boldsymbol{S}^{T}=\boldsymbol{0}
$$

Any matrix that satisfies the above equation is called skew-symmetric matrix (a square matrix whose transpose equals its negative). 


From {eq}`equ.skew`, we have


$$\dot{\boldsymbol{R}}=\boldsymbol{S} \boldsymbol{R}$$(equ.skew2)

Now, let's find out what is $\boldsymbol{S}$ and its physics interpretation.

Suppose ${\boldsymbol{R}}$ represents a rotation of a moving body frame
$O'-x'y'z'$ with respect to a fixed reference frame $O-xyz$. Let's consider a point $P$, rigidly fixed to the body frame (thus moving as body is moving), with its  body frame coordinate $\boldsymbol{p}^{\prime}$. Then, its coordinate in the reference frame $\boldsymbol{p}$ is 

$$\boldsymbol{p}(t)=\boldsymbol{R}(t) \boldsymbol{p}^{\prime}$$(equ.coordinate) 

which is time varying. 


Taking
the derivative to  both sides of {eq}`equ.coordinate` yields

$$\dot{\boldsymbol{p}}=\dot{\boldsymbol{R}} \boldsymbol{p}^{\prime}=\boldsymbol{S} \boldsymbol{R} \boldsymbol{p}^{\prime}$$(equ.skew3)

where we have used {eq}`equ.skew`.

Recall in your mechanics courses, given the angular velocity $\boldsymbol{\omega}$ of a moving body with respect to a reference frame, any point $P$ fixed on this body has a velocity in the reference frame as 

$$
\dot{\boldsymbol{p}}=\boldsymbol{\omega} \times  \boldsymbol{p}=\boldsymbol{\omega} \times \boldsymbol{R} \boldsymbol{p}^{\prime}
$$(equ.bke)

where $\boldsymbol{\omega}=\left[\begin{array}{lll}\omega_{x} & \omega_{y} & \omega_{z}\end{array}\right]^{T}$ is the angular velocity of the moving body, expressed in the  reference frame.



If we compare {eq}`equ.skew3` and {eq}`equ.bke`, we can find

$$\boldsymbol{S}=[\boldsymbol{\omega} \times]=\boldsymbol{S}(\boldsymbol{\omega})=\left[\begin{array}{ccc}
0 & -\omega_{z} & \omega_{y} \\
\omega_{z} & 0 & -\omega_{x} \\
-\omega_{y} & \omega_{x} & 0
\end{array}\right]$$

Thus, this skew-symmetric matrix $\boldsymbol{S}$ can be obtained directly from the body's angular velocity $\boldsymbol{\omega}$. We typically write it as $\boldsymbol{S}(\boldsymbol{\omega})$.

In sum, we can conclude the derivative of a rotation matrix is

$$\dot{\boldsymbol{R}}=\boldsymbol{S}(\boldsymbol{\omega}) \boldsymbol{R}$$


*Note that the above angular velocity $\boldsymbol{\omega}$ is expressed in the reference frame!*

One property of $\boldsymbol{S}(\boldsymbol{\omega})$: For any rotation matrix
$\boldsymbol{R}$, one has

$$\boldsymbol{R} \boldsymbol{S}(\boldsymbol{\omega}) \boldsymbol{R}^{T}=\boldsymbol{S}(\boldsymbol{R} \boldsymbol{\omega})$$(equ.skewproperty)

## Derivative of Rotation + Translation

Consider a transformation mapping the  coordinate mapping of a point $P$ (NOT necessarily fixed on a moving body) from the body frame
$O_1-x_1y_1z_1$ to reference frame $O_0-x_0y_0z_0$

$$\boldsymbol{p}^{0}=\boldsymbol{o}_{1}^{0}(t)+\boldsymbol{R}_{1}^{0}(t) \boldsymbol{p}^{1}$$

Differentiating the above equation with respect to time yields

$$
\begin{aligned}
\dot{\boldsymbol{p}}^{0}=\dot{\boldsymbol{o}}_{1}^{0}+\boldsymbol{R}_{1}^{0} \dot{\boldsymbol{p}}^{1}+\dot{\boldsymbol{R}}_{1}^{0} \boldsymbol{p}^{1}&=\dot{\boldsymbol{o}}_{1}^{0}+\boldsymbol{R}_{1}^{0} \dot{{\boldsymbol{p}}}^{1}+\boldsymbol{S}\left(\boldsymbol{\omega}_{1}^{0}\right) \boldsymbol{R}_{1}^{0} \boldsymbol{p}^{1}\\
&=\dot{\boldsymbol{o}}_{1}^{0}+\boldsymbol{R}_{1}^{0} \dot{{\boldsymbol{p}}}^{1}+ \boldsymbol{\omega}_{1}^{0} \times \boldsymbol{p}^{0}
\end{aligned}
$$(equ.vkin1)

In  {eq}`equ.vkin1`, when  $P$ is moving on the body,  it will have a local velocity $\dot{{\boldsymbol{p}}}^{1}$ in the *in the body frame*.  Note that the second line is derived due to $\boldsymbol{R}_{1}^{0} \boldsymbol{p}^{1}=\boldsymbol{p}^{0}$ and $\boldsymbol{S}\left(\boldsymbol{\omega}_{1}^{0}\right)=[\omega_{1}^{0} \times] $.




<!-- Since
$\boldsymbol{R}_{1}^{0} \boldsymbol{p}^{1}=\boldsymbol{p}^{0}$ and $\boldsymbol{S}\left(\boldsymbol{\omega}_{1}^{0}\right)=[\omega_{1}^{0} \times] $, the above equation {eq}`equ.vkin1` becomes

$$\dot{\boldsymbol{p}}^{0}=\dot{\boldsymbol{o}}_{1}^{0}+\boldsymbol{R}_{1}^{0} \dot{\boldsymbol{p}}^{1}+\omega_{1}^{0} \times \boldsymbol{p}^{0}$$(equ.vkin2) -->



<!-- Here are some explaination of each term on the right side of {eq}`equ.vkin2`:
* $\dot{\boldsymbol{o}}_{1}^{0}$ is the contribution to point $P$'s total velocity (in reference frame) by the translation of body frame.

* $\boldsymbol{R}_{1}^{0} \dot{\boldsymbol{p}}^{1}$ is the contribution to point $P$'s total velocity (in reference frame)  by the local movement of $P$ on the body frame. Here, $\dot{\boldsymbol{p}}^{1}$ is the local velocity in the body frame, and $\boldsymbol{R}_{1}^{0} \dot{\boldsymbol{p}}^{1}$ is the transformed local velocity to the reference frame.

* $\boldsymbol{S}\left(\boldsymbol{\omega}_{1}^{0}\right) \boldsymbol{R}_{1}^{0} \boldsymbol{p}^{1}=\omega_{1}^{0} \times \boldsymbol{p}^{0}$ is the contribution to point $P$'s total velocity by the rotation of the body frame. -->



Notice that, if $\boldsymbol{p}^{1}$ is fixed in the moving body frame , {eq}`equ.vkin1` becomes

$$\dot{\boldsymbol{p}}^{0}=\dot{\boldsymbol{o}}_{1}^{0}+\boldsymbol{\omega}_{1}^{0} \times \boldsymbol{p}^{0}$$

# Apply to links of a robot arm

Consider the Link $i$ of a robot arm in {numref}`manipulator_linki` (the frames follow the DH convention).  Let
$\boldsymbol{r}_{i-1, i}^{i-1}$ denote the position of the origin of
Frame $i$ from the origin of Frame $i-1$, expressed in Frame $i-1$.

Before we proceed, a quick question: how to obtain $\boldsymbol{p}_{i-1}$ and $\boldsymbol{R}_{i-1}$? and how to obtain $\boldsymbol{r}_{i-1, i}^{i-1}$?


```{figure} ./diff_kinematics/manipulator_linki.jpg
---
width: 80%
name: manipulator_linki
---
Link $i$ of a
robot arm
```



## Linear (Translational) Velocity

The position of Frame $i$ is

$$\boldsymbol{p}_{i}=\boldsymbol{p}_{i-1}+\boldsymbol{R}_{i-1} \boldsymbol{r}_{i-1, i}^{i-1}$$(equ.linki_pos)





Similar to {eq}`equ.vkin1`, taking the derivative of both sides in the above equation with respect to time $t$, we have

$$
\begin{aligned}
\dot{\boldsymbol{p}}_{i}&=\dot{\boldsymbol{p}}_{i-1}+\boldsymbol{R}_{i-1} \dot{\boldsymbol{r}}_{i-1, i}^{i-1}+\boldsymbol{\omega}_{i-1} \times \boldsymbol{R}_{i-1} \boldsymbol{r}_{i-1, i}^{i-1}\\&=\dot{\boldsymbol{p}}_{i-1}+\boldsymbol{v}_{i-1, i}+\boldsymbol{\omega}_{i-1} \times \boldsymbol{r}_{i-1, i}
\end{aligned}
$$(equ.linki_pos_vel)


Here, we define $\boldsymbol{v}_{i-1, i}=\boldsymbol{R}_{i-1}\dot{\boldsymbol{r}}_{i-1, i}^{i-1}$, which is local translational velocity of Frame $i$ in Frame $i-1$, but expressed in the reference frame.



If joint $i$ is revolute joint, 

$$\boldsymbol{v}_{i-1, i}=\boldsymbol{\omega}_{i-1, i}\times\boldsymbol{r}_{i-1, i}$$(equ.rel_v)

where $\boldsymbol{\omega}_{i-1, i}$ is relative angular velocity of Frame $i$ in Frame $i-1$. More specifically, 

$$\boldsymbol{\omega}_{i-1, i}=\dot{\vartheta}\boldsymbol{z}_{i-1}$$(equ.relative_omega)

What is $\boldsymbol{z}_{i-1}$? This is question for you!

Combine {eq}`equ.linki_pos_vel`, {eq}`equ.rel_v` and {eq}`equ.relative_omega`, we have:
**If Joint $i$ is revolute**

$$\begin{aligned}
\dot{\boldsymbol{p}}_{i} =\dot{\boldsymbol{p}}_{i-1}+(\dot{\vartheta}_{i} \boldsymbol{z}_{i-1}+\boldsymbol{\omega}_{i-1}) \times \boldsymbol{r}_{i-1, i}
\end{aligned}$$







If joint $i$ is prismatic joint, 

$$\boldsymbol{v}_{i-1, i}=\dot{d_i}\boldsymbol{z}_{i-1}$$(equ.relative_d)


Combine {eq}`equ.linki_pos_vel`, {eq}`equ.rel_v` and {eq}`equ.relative_d`, we have:
**If Joint $i$ is prismatic**

$$\begin{aligned}
\dot{\boldsymbol{p}}_{i} =\dot{\boldsymbol{p}}_{i-1}+\dot{d}_{i} \boldsymbol{z}_{i-1} +\boldsymbol{\omega}_{i-1}\times \boldsymbol{r}_{i-1, i}
\end{aligned}$$






## Angular Velocity

Next, let's consider the rotation of Frame $i$ 


$$\boldsymbol{R}_{i}=\boldsymbol{R}_{i-1} \boldsymbol{R}_{i}^{i-1}$$

Its time derivative is

$$\boldsymbol{S}\left(\boldsymbol{\omega}_{i}\right) \boldsymbol{R}_{i}=\boldsymbol{S}\left(\boldsymbol{\omega}_{i-1}\right) \boldsymbol{R}_{i}+\boldsymbol{R}_{i-1} \boldsymbol{S}\left(\boldsymbol{\omega}_{i-1, i}^{i-1}\right) \boldsymbol{R}_{i}^{i-1}$$(equ.linki_rot_vel)

where $\boldsymbol{\omega}_{i-1, i}^{i-1}$ denotes the angular velocity
of Frame $i$ with respect to Frame $i-1$ expressed in Frame $i-1$. The
second term on the right-hand side of {eq}`equ.linki_rot_vel` can be rewritten as

$$\boldsymbol{R}_{i-1} \boldsymbol{S}\left(\boldsymbol{\omega}_{i-1, i}^{i-1}\right) \boldsymbol{R}_{i}^{i-1}=\boldsymbol{R}_{i-1} \boldsymbol{S}\left(\boldsymbol{\omega}_{i-1, i}^{i-1}\right) \boldsymbol{R}_{i-1}^{T} \boldsymbol{R}_{i-1} \boldsymbol{R}_{i}^{i-1}=\boldsymbol{S}\left(\boldsymbol{R}_{i-1} \boldsymbol{\omega}_{i-1, i}^{i-1}\right) \boldsymbol{R}_{i}$$

by recalling the property of the skew-symmetric matrix in {eq}`equ.skewproperty`. Then,

$$\boldsymbol{S}\left(\boldsymbol{\omega}_{i}\right) \boldsymbol{R}_{i}=\boldsymbol{S}\left(\boldsymbol{\omega}_{i-1}\right) \boldsymbol{R}_{i}+\boldsymbol{S}\left(\boldsymbol{R}_{i-1} \boldsymbol{\omega}_{i-1, i}^{i-1}\right) \boldsymbol{R}_{i}$$(equ.linki_rot_vel2)

If we cancel out all $ \boldsymbol{R}_{i}$ on both sides of {eq}`equ.linki_rot_vel2`, we can get

$$\boldsymbol{\omega}_{i}=\boldsymbol{\omega}_{i-1}+\boldsymbol{R}_{i-1} \boldsymbol{\omega}_{i-1, i}^{i-1}=\boldsymbol{\omega}_{i-1}+\boldsymbol{\omega}_{i-1, i}$$(equ.linki_rot_vel3)

Thus,  the angular velocity $\boldsymbol{\omega}_{i}$ of Link $i$ depends only on the angular velocity $\boldsymbol{\omega}_{i-1}$ of Link $i-1$ and their relative angular velocity $\boldsymbol{\omega}_{i-1, i}$.


If joint $i$ is revolute joint, 

$$
\boldsymbol{\omega}_{i}  =\boldsymbol{\omega}_{i-1}+\dot{\vartheta}_{i} \boldsymbol{z}_{i-1}
$$



If joint $i$ is prismatic. 

$$
\boldsymbol{\omega}_{i}  =\boldsymbol{\omega}_{i-1}
$$









## Summary

Considering different joint types for Joint $i$ at a robot arm, according to {eq}`equ.linki_pos_vel` and {eq}`equ.linki_rot_vel3`, we can obtain


:::{important}
**If Joint $i$ is prismatic**

$$\begin{aligned}
\boldsymbol{\omega}_{i} & =\boldsymbol{\omega}_{i-1} \\
\dot{\boldsymbol{p}}_{i} & =\dot{\boldsymbol{p}}_{i-1}+\dot{d}_{i} \boldsymbol{z}_{i-1}+\boldsymbol{\omega}_{i-1} \times \boldsymbol{r}_{i-1, i}
\end{aligned}$$

**If Joint $i$ is revolute**

$$\begin{aligned}
\boldsymbol{\omega}_{i} & =\boldsymbol{\omega}_{i-1}+\dot{\vartheta}_{i} \boldsymbol{z}_{i-1} \\
\dot{\boldsymbol{p}}_{i} & =\dot{\boldsymbol{p}}_{i-1}+\boldsymbol{\omega}_{i} \times \boldsymbol{r}_{i-1, i}
\end{aligned}$$
:::