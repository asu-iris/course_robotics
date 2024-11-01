---
author:
- Wanxin Jin
date: Oct. 5, 2023
title: "Lecture 14: Statics and Duality"
---

# Statics

The concept of statics in robotics is to find the relationship between  force
applied to the end-effector and the generalized torques applied to the
joints when the robot arm is at equilibrium.  We will apply the [principle of
virtual work](https://en.wikipedia.org/wiki/Virtual_work#:~:text=The%20principle%20of%20virtual%20work%20states%20that%20in%20equilibrium%20the,the%20reaction%2C%20or%20constraint%20forces.) to determine this relationship.

```{important}
The principle of virtual work states that in equilibrium the virtual work of the forces applied to a system is zero
```


Let $\boldsymbol{\tau}$
be the $(n \times 1)$ joint torques and $\boldsymbol{\gamma}_{e}=\begin{bmatrix}
\boldsymbol{f}_{e}\\
\boldsymbol{\mu}_{e}
\end{bmatrix}$ the
$(r \times 1)$ end-effector forces, including linear force $\boldsymbol{f}_{e}$ and moment
$\boldsymbol{\mu}_{e}$.


The visual work by the joint torques:

$$d W_{\tau}=\boldsymbol{\tau}^{T} d \boldsymbol{q}$$

where $d \boldsymbol{q}$ is  the  joint virtual displacement.

The visual work by the end-effector forces:

$$d W_{\gamma}=\boldsymbol{f}_{e}^{T} d \boldsymbol{p}_{e}+\boldsymbol{\mu}_{e}^{T} \boldsymbol{\omega}_{e} d t =\boldsymbol{f}_{e}^{T} \boldsymbol{J}_{P}(\boldsymbol{q}) d \boldsymbol{q}+\boldsymbol{\mu}_{e}^{T} \boldsymbol{J}_{O}(\boldsymbol{q}) d \boldsymbol{q}  =\boldsymbol{\gamma}_{e}^{T} \boldsymbol{J}(\boldsymbol{q}) d \boldsymbol{q}$$

where $d \boldsymbol{p}_{e}$ is the linear virtual displacement and
$\boldsymbol{\omega}_{e} d t$ is the angular virtual displacement of the end-effector.

According to the principle of virtual work, the manipulator is at static
equilibrium if and only if

$$\delta W_{\tau}=\delta W_{\gamma}, \quad \forall \delta \boldsymbol{q}$$

This leads to the statics equation:

$$\boldsymbol{\tau}=\boldsymbol{J}^{T}(\boldsymbol{q}) \boldsymbol{\gamma}_{e}$$

stating a relationship between the end-effector forces and joint
torques when the robot is at its equilibrium.


</br>

# Kineto-Statics Duality

The kineto-statics duality states that

$$\begin{aligned}
    \boldsymbol{v}_{e}=\boldsymbol{J}(\boldsymbol{q}) \dot{\boldsymbol{q}} \quad\quad\quad 
    \boldsymbol{\tau}=\boldsymbol{J}^{T}(\boldsymbol{q}) \boldsymbol{\gamma}_{e}
\end{aligned}$$

-   The range space $\mathcal{R}\left(\boldsymbol{J}^{T}\right)$ of $\boldsymbol{J}^{T}$ is the subspace
     in $\mathbb{R}^{n}$, where the
    the joint torques that can balance the end-effector forces at robot pose $\boldsymbol{q}$.

-   The null space $\mathcal{N}\left(\boldsymbol{J}^{T}\right)$ of $\boldsymbol{J}^{T}$ is the subspace
     in $\mathbb{R}^{r}$ of
    the end-effector forces that do not require any balancing joint
    torques at robot pose $\boldsymbol{q}$.



```{figure} ../lec11-12/diff_kinematics/operation_to_joint.jpg
---
width: 70%
name: coordinate_mapping
---
Mapping between the end-effector force space and the joint torque
space
```


It is worth remarking that the end-effector forces
$\gamma_{e} \in \mathcal{N}\left(\boldsymbol{J}^{T}\right)$ are entirely
absorbed by the mechanical structure of the robot arm. 


From fundamental relationship in linear algebra, the relations between the two subspaces are established by

$$\mathcal{N}(\boldsymbol{J}) \equiv \mathcal{R}^{\perp}\left(\boldsymbol{J}^{T}\right) \quad \mathcal{R}(\boldsymbol{J}) \equiv \mathcal{N}^{\perp}\left(\boldsymbol{J}^{T}\right)$$

and then, once the manipulator Jacobian is known, it is possible to
characterize completely differential kinematics and statics in terms of
the range and null spaces of the Jacobian and its transpose.

# Velocity and Force Transformation (Optional)


```{figure} ../lec11-12/diff_kinematics/coordinate_mapping.jpg
---
width: 70%
name: coordinate_mapping
---
Representation of linear and angular velocities in different
coordinate frames on the same rigid
body
```


The kineto-statics duality  can be useful to
find the transformation of velocities and forces between two
coordinate frames. Consider a reference coordinate frame
$O_{0}-x_{0} y_{0} z_{0}$ and a rigid body moving with respect to such a
frame. Then let $O_{1}-x_{1} y_{1} z_{1}$ and $O_{2}-x_{2} y_{2} z_{2}$
be two coordinate frames attached to the body. The relationships between
translational and rotational velocities of the two frames with respect
to the reference frame are given by

$$\begin{aligned}
\boldsymbol{\omega}_{2} & =\boldsymbol{\omega}_{1} \\
\dot{\boldsymbol{p}}_{2} & =\dot{\boldsymbol{p}}_{1}+\boldsymbol{\omega}_{1} \times \boldsymbol{r}_{12} .
\end{aligned}$$

The above relations can be compactly written as

$$\left[\begin{array}{c}
\dot{\boldsymbol{p}}_{2} \\
\boldsymbol{\omega}_{2}
\end{array}\right]=\left[\begin{array}{cc}
\boldsymbol{I} & -\boldsymbol{S}\left(\boldsymbol{r}_{12}\right) \\
\boldsymbol{O} & \boldsymbol{I}
\end{array}\right]\left[\begin{array}{c}
\dot{\boldsymbol{p}}_{1} \\
\boldsymbol{\omega}_{1}
\end{array}\right]$$

On the other hand, if vectors are referred to their own frames, it is

$$\left[\begin{array}{c}
\dot{\boldsymbol{p}}_{2}^{2} \\
\boldsymbol{\omega}_{2}^{2}
\end{array}\right]=\left[\begin{array}{cc}
\boldsymbol{R}_{1}^{2} & -\boldsymbol{R}_{1}^{2} \boldsymbol{S}\left(\boldsymbol{r}_{12}^{1}\right) \\
\boldsymbol{O} & \boldsymbol{R}_{1}^{2}
\end{array}\right]\left[\begin{array}{c}
\dot{\boldsymbol{p}}_{1}^{1} \\
\boldsymbol{\omega}_{1}^{1}
\end{array}\right]$$

giving the relationship of velocity transformation between two frames.
The above can be compactly written as

$$\boldsymbol{v}_{2}^{2}=\boldsymbol{J}_{1}^{2} \boldsymbol{v}_{1}^{1}$$

By virtue of the kineto-statics duality, the force transformation
between two frames is

$$\boldsymbol{\gamma}_{1}^{1}=\boldsymbol{J}_{1}^{2 T} \boldsymbol{\gamma}_{2}^{2}$$

Finally, notice that the above analysis is instantaneous in that, if a
coordinate frame varies with respect to the other, it is necessary to
recompute the Jacobian of the transformation through the computation of
the related rotation matrix of one frame with respect to the other.
