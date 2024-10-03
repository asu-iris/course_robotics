---
author:
- Wanxin Jin
date: Sep. 19, 2023
title: "Lecture 10: Derivative of Transformation"
---

# Velocity Kinematics

## Derivative of Rotation
Consider a time-varying rotation matrix
$\boldsymbol{R}=\boldsymbol{R}(t)$. Based on the property of rotation matrix, we always have 

$$\boldsymbol{R}(t) \boldsymbol{R}^{T}(t)=\boldsymbol{I}$$

Differentiating the above equation with respect to time gives

$$\dot{\boldsymbol{R}}(t) \boldsymbol{R}^{T}(t)+\boldsymbol{R}(t) \dot{\boldsymbol{R}}^{T}(t)=\boldsymbol{0}$$

Let's define a new matrix

$$\boldsymbol{S}=\dot{\boldsymbol{R}} \boldsymbol{R}^{T}$$(equ.skew) 

Then, the above equation becomes

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



If we look at and compare {eq}`equ.skew3` and {eq}`equ.bke`, we can find

$$\boldsymbol{S}=[\boldsymbol{\omega} \times]=\boldsymbol{S}(\boldsymbol{\omega})=\left[\begin{array}{ccc}
0 & -\omega_{z} & \omega_{y} \\
\omega_{z} & 0 & -\omega_{x} \\
-\omega_{y} & \omega_{x} & 0
\end{array}\right]$$

Thus, this skew-symmetric matrix $\boldsymbol{S}$ can be obtained directly from the body's angular velocity $\boldsymbol{\omega}$. We typically write it as $\boldsymbol{S}(\boldsymbol{\omega})$.

In sum, we can conclude the derivative of a rotation matrix is

$$\dot{\boldsymbol{R}}=\boldsymbol{S}(\boldsymbol{\omega}) \boldsymbol{R}$$


*Note that the above angular velocity $\boldsymbol{\omega}$ is expressed in the reference frame! (not in the body frame)*

One property of $\boldsymbol{S}(\boldsymbol{\omega})$: For any rotation matrix
$\boldsymbol{R}$, one has

$$\boldsymbol{R} \boldsymbol{S}(\boldsymbol{\omega}) \boldsymbol{R}^{T}=\boldsymbol{S}(\boldsymbol{R} \boldsymbol{\omega})$$(equ.skewproperty)

## Derivative of Rotation + Translation

Consider a transformation mapping the  coordinate mapping of a point $P$ (NOT necessarily fixed on a moving body) from the body frame
$O_1-x_1y_1z_1$ to reference frame $O_0-x_0y_0z_0$

$$\boldsymbol{p}^{0}=\boldsymbol{o}_{1}^{0}(t)+\boldsymbol{R}_{1}^{0}(t) \boldsymbol{p}^{1}$$

Differentiating the above equation with respect to time yields

$$\dot{\boldsymbol{p}}^{0}=\dot{\boldsymbol{o}}_{1}^{0}+\boldsymbol{R}_{1}^{0} \dot{\boldsymbol{p}}^{1}+\dot{\boldsymbol{R}}_{1}^{0} \boldsymbol{p}^{1}=\dot{\boldsymbol{o}}_{1}^{0}+\boldsymbol{R}_{1}^{0} \dot{{\boldsymbol{p}}}^{1}+\boldsymbol{S}\left(\boldsymbol{\omega}_{1}^{0}\right) \boldsymbol{R}_{1}^{0} \boldsymbol{p}^{1}$$(equ.vkin1)

In the above  {eq}`equ.vkin1`, because point $P$ is not necessarily fixed on the body, and it may have its *local movement in the body frame* with the local velocity $\dot{{\boldsymbol{p}}}^{1}$. Special care should be paied to the notation of $\dot{{\boldsymbol{p}}}^{1}$: $\dot{{\boldsymbol{p}}}^{1}$ is purely taking the derivative to xyz body coordinate instead of  body xyz axises.




Since
$\boldsymbol{R}_{1}^{0} \boldsymbol{p}^{1}=\boldsymbol{p}^{0}$ and $\boldsymbol{S}\left(\boldsymbol{\omega}_{1}^{0}\right)=[\omega_{1}^{0} \times] $, the above equation {eq}`equ.vkin1` becomes

$$\dot{\boldsymbol{p}}^{0}=\dot{\boldsymbol{o}}_{1}^{0}+\boldsymbol{R}_{1}^{0} \dot{\boldsymbol{p}}^{1}+\omega_{1}^{0} \times \boldsymbol{p}^{0}$$(equ.vkin2)



Here are some explaination of each term on the right side of {eq}`equ.vkin2`:
* $\dot{\boldsymbol{o}}_{1}^{0}$ is the contribution to point $P$'s total velocity (in reference frame) by the translation of body frame.

* $\boldsymbol{R}_{1}^{0} \dot{\boldsymbol{p}}^{1}$ is the contribution to point $P$'s total velocity (in reference frame)  by the local movement of $P$ on the body frame. Here, $\dot{\boldsymbol{p}}^{1}$ is the local velocity in the body frame, and $\boldsymbol{R}_{1}^{0} \dot{\boldsymbol{p}}^{1}$ is the transformed local velocity to the reference frame.

* $\boldsymbol{S}\left(\boldsymbol{\omega}_{1}^{0}\right) \boldsymbol{R}_{1}^{0} \boldsymbol{p}^{1}=\omega_{1}^{0} \times \boldsymbol{p}^{0}$ is the contribution to point $P$'s total velocity by the rotation of the body frame.



Notice that, if $\boldsymbol{p}^{1}$ is fixed in the moving body frame , then {eq}`equ.vkin2` can be further reduced to

$$\dot{\boldsymbol{p}}^{0}=\dot{\boldsymbol{o}}_{1}^{0}+\boldsymbol{\omega}_{1}^{0} \times \boldsymbol{p}^{0}$$

# Apply to links of a robot arm

Consider the Link $i$ of a robot arm in {numref}`manipulator_linki`. We can apply DH convention to find the transformation between Frame $i-1$ (attached to link $i-1$) and
Frame $i$ (attached to this link $i$). Next, we find out what the relationship between the velocity of those two links.


```{figure} ./diff_kinematics/manipulator_linki.jpg
---
width: 80%
name: manipulator_linki
---
Link $i$ of a
robot arm
```



## Linear (Translational) Velocity

Let $\boldsymbol{p}_{i-1}$ and $\boldsymbol{p}_{i}$ be the position of
the origins of Frame $i-1$ and Frame $i$, respectively. Also, let
$\boldsymbol{r}_{i-1, i}^{i-1}$ denote the position of the origin of
Frame $i$ from the origin of Frame $i-1$, expressed in Frame $i-1$.


Let's consider $\boldsymbol{p}_{i-1}$ be position of Frame $i-1$ in the reference frame (therefore, i omit the superscript here), and $\boldsymbol{R}_{i-1}$ be the rotation of Frame $i-1$ in the reference frame (i omit the superscript again). Then, the position of Frame $i$ in the reference frame is

$$\boldsymbol{p}_{i}=\boldsymbol{p}_{i-1}+\boldsymbol{R}_{i-1} \boldsymbol{r}_{i-1, i}^{i-1}$$(equ.linki_pos)


Before we proceed, a question: how to obtain $\boldsymbol{p}_{i-1}$ and $\boldsymbol{R}_{i-1}$? and how to obtain $\boldsymbol{r}_{i-1, i}^{i-1}$?



Similar to {eq}`equ.vkin1`, taking the derivative of both sides in the above equation with respect to time $t$, we have

$$
\begin{aligned}
\dot{\boldsymbol{p}}_{i}&=\dot{\boldsymbol{p}}_{i-1}+\boldsymbol{R}_{i-1} \dot{\boldsymbol{r}}_{i-1, i}^{i-1}+\boldsymbol{\omega}_{i-1} \times \boldsymbol{R}_{i-1} \boldsymbol{r}_{i-1, i}^{i-1}\\&=\dot{\boldsymbol{p}}_{i-1}+\boldsymbol{v}_{i-1, i}+\boldsymbol{\omega}_{i-1} \times \boldsymbol{r}_{i-1, i}
\end{aligned}
$$(equ.linki_pos_vel)


Here, we define $\boldsymbol{v}_{i-1, i}=\boldsymbol{R}_{i-1}\dot{\boldsymbol{r}}_{i-1, i}^{i-1}$, which is local translational velocity of Frame $i$ in Frame $i-1$, but expressed in the reference frame.
Please carefully look at how first row becomes second row, and ask yourself what each term stands for on the second row, similar to how we have analyzed to{eq}`equ.vkin2`. 


```{note}

Now let's think about what happens to {eq}`equ.linki_pos_vel` if joint $i$ is revolute joint. In that case, 
* how to compute  $\boldsymbol{r}_{i-1, i}$? Hint: think about forward kinematics.
* what is  $\boldsymbol{v}_{i-1, i}$? Hint: think about what is the relative motion of Frame $i$ in Frame $i-1$. You may answer:

$$\boldsymbol{v}_{i-1, i}=\boldsymbol{\omega}_{i-1, i}\times\boldsymbol{r}_{i-1, i}$$(equ.rel_v)

But what is $\boldsymbol{\omega}_{i-1, i}$? Actually, it is relative angular velocity of Frame $i$ in Frame $i-1$, expressed in the reference frame. 


So, next quesetion, hwo to compute $\boldsymbol{\omega}_{i-1, i}$? Actually

$$\boldsymbol{\omega}_{i-1, i}=\dot{\vartheta}\boldsymbol{z}_{i-1}$$(equ.relative_omega)

What is $\boldsymbol{z}_{i-1}$? This is question for you!

Combine {eq}`equ.linki_pos_vel`, {eq}`equ.rel_v` and {eq}`equ.relative_omega`, we have:
**If Joint $i$ is revolute**

$$\begin{aligned}
\dot{\boldsymbol{p}}_{i} =\dot{\boldsymbol{p}}_{i-1}+(\dot{\vartheta}_{i} \boldsymbol{z}_{i-1}+\boldsymbol{\omega}_{i-1}) \times \boldsymbol{r}_{i-1, i}
\end{aligned}$$
```





```{note}

Now let's think about what happens to {eq}`equ.linki_pos_vel` if joint $i$ is prismatic joint. In that case, 
* what is  $\boldsymbol{v}_{i-1, i}$? Hint: think about what is the relative motion of Frame $i$ in Frame $i-1$.  Actually

$$\boldsymbol{v}_{i-1, i}=\dot{d_i}\boldsymbol{z}_{i-1}$$(equ.relative_d)


Combine {eq}`equ.linki_pos_vel`, {eq}`equ.rel_v` and {eq}`equ.relative_d`, we have:
**If Joint $i$ is prismatic**

$$\begin{aligned}
\dot{\boldsymbol{p}}_{i} =\dot{\boldsymbol{p}}_{i-1}+\dot{d}_{i} \boldsymbol{z}_{i-1} +\boldsymbol{\omega}_{i-1}\times \boldsymbol{r}_{i-1, i}
\end{aligned}$$

```




## Angular Velocity

Next, let's consider the rotation of Frame $i$ in the reference frame (omit the superscipt 0 again)


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


```{note}

Now let's think about what happens to {eq}`equ.linki_rot_vel3` if joint $i$ is revolute joint. In that case,  what is $\boldsymbol{\omega}_{i-1, i}$ and how to compute it? 


Actually, it is  {eq}`equ.relative_omega`! Combine {eq}`equ.linki_rot_vel3` and {eq}`equ.relative_omega`, we have:
**If Joint $i$ is revolute**

$$
\boldsymbol{\omega}_{i}  =\boldsymbol{\omega}_{i-1}+\dot{\vartheta}_{i} \boldsymbol{z}_{i-1}
$$
```


```{note}

Now let's think about what happens to {eq}`equ.linki_rot_vel3` if joint $i$ is prismatic. In that case,  what is $\boldsymbol{\omega}_{i-1, i}$ and how to compute it? 


Actually, it is  $0$! Thus, we have:
**If Joint $i$ is prismatic**

$$
\boldsymbol{\omega}_{i}  =\boldsymbol{\omega}_{i-1}
$$
```








## Summary

Considering different joint types for Joint $i$ at a robot arm, according to {eq}`equ.linki_pos_vel` and {eq}`equ.linki_rot_vel3`, we can obtain


:::{important}
**If Joint $i$ is prismatic**

$$\begin{aligned}
\boldsymbol{\omega}_{i} & =\boldsymbol{\omega}_{i-1} \\
\dot{\boldsymbol{p}}_{i} & =\dot{\boldsymbol{p}}_{i-1}+\dot{d}_{i} \boldsymbol{z}_{i-1}+\boldsymbol{\omega}_{i} \times \boldsymbol{r}_{i-1, i}
\end{aligned}$$

**If Joint $i$ is revolute**

$$\begin{aligned}
\boldsymbol{\omega}_{i} & =\boldsymbol{\omega}_{i-1}+\dot{\vartheta}_{i} \boldsymbol{z}_{i-1} \\
\dot{\boldsymbol{p}}_{i} & =\dot{\boldsymbol{p}}_{i-1}+\boldsymbol{\omega}_{i} \times \boldsymbol{r}_{i-1, i}
\end{aligned}$$
:::