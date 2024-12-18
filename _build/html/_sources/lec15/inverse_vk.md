---
author:
- Wanxin Jin
date: Oct. 12, 2023
title: "Lecture 15: Inverse Velocity Kinematics"
---

# Inverse Velocity Kinematics
Suppose that a robot arm has a configuration
$\boldsymbol{q}\in\mathbb{R}^n$, and the motion of the end-effector is
  $\boldsymbol{v}_{e}\in\mathbb{R}^r$. We want to find the
corresponding joint velocity $\dot{\boldsymbol{q}}$ using the Jacobian

$$\boldsymbol{v}_{e}=\boldsymbol{J}(\boldsymbol{q})\dot{\boldsymbol{q}}$$

This problem is called as *inverse velocity kinematics*.

# Non-redundant robot arm

When a robot arm  is non-redundant, that is, $r=n$, and $\boldsymbol{q}$ is not
a singular configuration, one can directly inverse the Jacobian

$$\dot{\boldsymbol{q}}=\text{inv}(\boldsymbol{J}(\boldsymbol{q})) \boldsymbol{v}_{e}$$

# Redundant robot arm

When the robot arm is redundant $(r<n)$,  there are infinite solutions
$\dot{\boldsymbol{q}}$. We need to pick the
best (in a sense discussed below) solution $\dot{\boldsymbol{q}}$ using optimization.

## Minimal Norm

Once the end-effector velocity $\boldsymbol{v}_{e}$ and Jacobian
$\boldsymbol{J}$ are given, we establish the following optimization to
pick $\dot{\boldsymbol{q}}$

$$\begin{aligned}
        \min_{\dot{\boldsymbol{q}}} \quad &\frac{1}{2} \dot{\boldsymbol{q}}^{T} \boldsymbol{W} \dot{\boldsymbol{q}}\\
        \text{s.t.}\quad & \boldsymbol{v}_{e}  =\boldsymbol{J}(\boldsymbol{q}) \dot{\boldsymbol{q}}
    \end{aligned}$$

Here, $\boldsymbol{W}$ is a weight matrix (usually diagonal).
This means that we want to pick the best $\dot{\boldsymbol{q}}$ in the
sense of having the minimal norm of joint velocities. If $\boldsymbol{q}$ is not a singular configuration, the
optimal solution to the above optimization is

$$\dot{\boldsymbol{q}}=\boldsymbol{W}^{-1} \boldsymbol{J}^{T}\left(\boldsymbol{J} \boldsymbol{W}^{-1} \boldsymbol{J}^{T}\right)^{-1} \boldsymbol{v}_{e} .$$

In particular, if $\boldsymbol{W}=\boldsymbol{I}$, the solution
simplifies into

$$\dot{\boldsymbol{q}}=\boldsymbol{J}^{\dagger} \boldsymbol{v}_{e}$$

with 

$$
\boldsymbol{J}^{\dagger}=\boldsymbol{J}^{T}\left(\boldsymbol{J} \boldsymbol{J}^{T}\right)^{-1}
$$

is the right pseudo-inverse of $\boldsymbol{J}$.

## Close-to-Reference

When we are given a reference $\dot{\boldsymbol{q}}_{\text{ref}}$, we want to pick the
best $\dot{\boldsymbol{q}}$ in the sense of having the minimal distance
to $\dot{\boldsymbol{q}}_{\text{ref}}$. Thus, we establish the following
optimization

$$\begin{aligned}
        \min_{\dot{\boldsymbol{q}}} \quad &
        \frac{1}{2}\left(\dot{\boldsymbol{q}}-\dot{\boldsymbol{q}}_{\text{ref}}\right)^{T}\left(\dot{\boldsymbol{q}}-\dot{\boldsymbol{q}}_{\text{ref}}\right)\\
        \text{s.t.}\quad & \boldsymbol{v}_{e}  =\boldsymbol{J}(\boldsymbol{q}) \dot{\boldsymbol{q}}
    \end{aligned}$$

If $\boldsymbol{q}$ is not a singular
configuration, the solution to the above optimization is

$$\dot{\boldsymbol{q}}=\boldsymbol{J}^{\dagger} \boldsymbol{v}_{e}+\left(\boldsymbol{I}_{n}-\boldsymbol{J}^{\dagger} \boldsymbol{J}\right) \dot{\boldsymbol{q}}_{\text{ref}}$$

The obtained solution has two terms. The first term is the same as the
solution of minimizing the joint velocity norm. In the second term,
$\left(\boldsymbol{I}-\boldsymbol{J}^{\dagger} \boldsymbol{J}\right)$ is
null space projection matrix $\boldsymbol{P}$ (recall we have mentioned
in our previous lecture), which allows the projection of any
$\dot{\boldsymbol{q}}_{0}$ in the null space of $\boldsymbol{J}$.

The final question is how to set 
$\dot{\boldsymbol{q}}_{\text{ref}}$?


A typical choice of $\dot{\boldsymbol{q}}_{\text{ref}}$
is

$$\dot{\boldsymbol{q}}_{\text{ref}}=k_{0}\left(\frac{\partial w(\boldsymbol{q}_{\text{ref}})}{\partial \boldsymbol{q}_{\text{ref}}}\right)^{T}$$

where $k_{0}>0$ and $w(\boldsymbol{q}_{\text{ref}})$ is another objective function--- we want ${\boldsymbol{q}}_{\text{ref}}$ moves in a direction of maximizing this objective function. The choice of this objective $w(\boldsymbol{q}_{\text{ref}})$ can be 

\(1\) Manipulability objective function:

$$w(\boldsymbol{q}_\text{ref})=\sqrt{\operatorname{det}\left(\boldsymbol{J}(\boldsymbol{q}_\text{ref}) \boldsymbol{J}^{T}(\boldsymbol{q}_\text{ref})\right)}$$

By maximizing this objective function, we want the refernece $\boldsymbol{q}_\text{ref}$ to move away from
singularities.

\(2\) The distance from joint limits:

$$w(\boldsymbol{q}_\text{ref})=-\frac{1}{2 n} \sum_{i=1}^{n}\left(\frac{q_{i,\text{ref}}-\bar{q}_{i}}{q_{i,M}-q_{i,m}}\right)^{2}$$

where $q_{i,M}\left(q_{i,m}\right)$ is the maximum (minimum) joint limit
and $\bar{q}_{i}$ the middle point of the joint range; thus, by
maximizing this distance, we aim to keep the joint
variables close to the center of their ranges.

\(3\) The distance from an obstacle:

$$w(\boldsymbol{q}_\text{ref})=\min _{\boldsymbol{p}_\text{ref}}\|\boldsymbol{p}(\boldsymbol{q}_\text{ref})-\boldsymbol{o}\|$$

where $\boldsymbol{o}$ is the position  of the
obstacle.

# Singularity

The inverse velocity kinematics for both the redundant and non-redundant
manipulators require the robot current configuration $\boldsymbol{q}$ is
nonsingular, i.e., the Jacboobian $\boldsymbol{J}(\boldsymbol{q})$ has
full rank. At a singular configuration,
it is possible to find a solution
$\dot{\boldsymbol{q}}$ only if
$\boldsymbol{v}_{e} \in \mathcal{R}(\boldsymbol{J})$. This situation
means that  $\boldsymbol{v}_{e}$ is physically achievable by
finding a joint velocity $\dot{\boldsymbol{q}}$, even though it is at a
singular configuration. If instead
$\boldsymbol{v}_{e} \notin \mathcal{R}(\boldsymbol{J})$, the system of
equations has no solution; this means that $\boldsymbol{v}_{e}$ cannot
be achievable by the robot at the given posture.

Inversion of the Jacobian can represent a serious problem not only
at a singularity but also in the neighborhood of a singularity. In the
neighborhood of a singularity, a relatively small $\boldsymbol{v}_{e}$
(in terms of its norm) which can cause large joint velocities. To overcome  "singularity neighbourhood" issue, one can modify the pseudo-inverse of $\boldsymbol{J}$ to

$$\boldsymbol{J}^{\star}=\boldsymbol{J}^{T}\left(\boldsymbol{J} \boldsymbol{J}^{T}+k^{2} \boldsymbol{I}\right)^{-1}$$

 It can be shown that the above modification
is equivalent to minimizing the following objective function (why? a homework problem)

$$\min_{\dot{\boldsymbol{q}}}\quad \frac{1}{2}\left(\boldsymbol{v}_{e}-\boldsymbol{J} \dot{\boldsymbol{q}}\right)^{T}\left(\boldsymbol{v}_{e}-\boldsymbol{J} \dot{\boldsymbol{q}}\right)+\frac{1}{2} k^{2} \dot{\boldsymbol{q}}^{T} \dot{\boldsymbol{q}}$$
