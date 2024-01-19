---
author:
- Wanxin Jin
date: Oct. 12, 2023
title: "Lecture 15: Inverse Velocity Kinematics"
---

# Inverse Velocity Kinematics
Suppose that a robot current configuration is
$\boldsymbol{q}\in\mathbb{R}^n$, and the motion of the end-effector is
assigned to $\boldsymbol{v}_{e}\in\mathbb{R}^r$. We want to find the
corresponding joint velocity $\dot{\boldsymbol{q}}$ using the Jacobian

$$\boldsymbol{v}_{e}=\boldsymbol{J}(\boldsymbol{q})\dot{\boldsymbol{q}}$$

This problem is corned as *inverse velocity kinematics*.

# Non-redundant Manipulator

When the manipulator is non-redundant $r=n$ and $\boldsymbol{q}$ is not
a singular configuration, one can directly inverse the Jacobian

$$\dot{\boldsymbol{q}}=\text{inv}(\boldsymbol{J}(\boldsymbol{q})) \boldsymbol{v}_{e}$$

# Redundant Manipulator

When the manipulator is redundant $(r<n)$, the Jacobian matrix has more
columns than rows, and thus there are infinite solutions
$\dot{\boldsymbol{q}}$. We need to resort to optimization to pick the
best solution $\dot{\boldsymbol{q}}$ in some sense!

## Minimal Norm Selction

Once the end-effector velocity $\boldsymbol{v}_{e}$ and Jacobian
$\boldsymbol{J}$ are given, we establish the following optimization to
pick $\dot{\boldsymbol{q}}$

$$\begin{aligned}
        \min_{\dot{\boldsymbol{q}}} \quad &\frac{1}{2} \dot{\boldsymbol{q}}^{T} \boldsymbol{W} \dot{\boldsymbol{q}}\\
        \text{s.t.}\quad & \boldsymbol{v}_{e}  =\boldsymbol{J}(\boldsymbol{q}) \dot{\boldsymbol{q}}
    \end{aligned}$$

This means that we want to pick the best $\dot{\boldsymbol{q}}$ in the
sense of having the minimal norm of joint velocities. By some
derivation, if $\boldsymbol{q}$ is not a singular configuration, the
optimal solution to the above optimization is

$$\dot{\boldsymbol{q}}=\boldsymbol{W}^{-1} \boldsymbol{J}^{T}\left(\boldsymbol{J} \boldsymbol{W}^{-1} \boldsymbol{J}^{T}\right)^{-1} \boldsymbol{v}_{e} .$$

In particular, if $\boldsymbol{W}=\boldsymbol{I}$, the solution
simplifies into

$$\dot{\boldsymbol{q}}=\boldsymbol{J}^{\dagger} \boldsymbol{v}_{e} \quad \text{with} \quad \boldsymbol{J}^{\dagger}=\boldsymbol{J}^{T}\left(\boldsymbol{J} \boldsymbol{J}^{T}\right)^{-1}$$

is the right pseudo-inverse of $\boldsymbol{J}$.

## Close-to-Reference Selction

We have other criteria to pick the best $\dot{\boldsymbol{q}}$. When we
are given a reference $\dot{\boldsymbol{q}}_{0}$, we want to pick the
best $\dot{\boldsymbol{q}}$ in the sense of having the minimal distance
to $\dot{\boldsymbol{q}}_{0}$. Thus, we establish the following
optimization

$$\begin{aligned}
        \min_{\dot{\boldsymbol{q}}} \quad &
        \frac{1}{2}\left(\dot{\boldsymbol{q}}-\dot{\boldsymbol{q}}_{0}\right)^{T}\left(\dot{\boldsymbol{q}}-\dot{\boldsymbol{q}}_{0}\right)\\
        \text{s.t.}\quad & \boldsymbol{v}_{e}  =\boldsymbol{J}(\boldsymbol{q}) \dot{\boldsymbol{q}}
    \end{aligned}$$

After doing some derivation, if $\boldsymbol{q}$ is not a singular
configuration, the solution to the above optimization is

$$\dot{\boldsymbol{q}}=\boldsymbol{J}^{\dagger} \boldsymbol{v}_{e}+\left(\boldsymbol{I}_{n}-\boldsymbol{J}^{\dagger} \boldsymbol{J}\right) \dot{\boldsymbol{q}}_{0} .$$

The obtained solution has two terms. The first term is the same as the
solution of minimizing the joint velocity norm. In the second term,
$\left(\boldsymbol{I}-\boldsymbol{J}^{\dagger} \boldsymbol{J}\right)$ is
null space projection matrix $\boldsymbol{P}$ (recall we have mentioned
in our previous lecture), which allows the projection of any
$\dot{\boldsymbol{q}}_{0}$ in the null space of $\boldsymbol{J}$.

The final question is how to specify the reference
$\dot{\boldsymbol{q}}_{0}$ for a redundant manipulator. A typical choice
is

$$\dot{\boldsymbol{q}}_{0}=k_{0}\left(\frac{\partial w(\boldsymbol{q})}{\partial \boldsymbol{q}}\right)^{T}$$

where $k_{0}>0$ and $w(\boldsymbol{q})$ is a secondary objective
function of the joint variables. Since the solution moves along the
direction of the gradient of the objective function, it attempts to
maximize it locally the secondary objective. Such a secondary objective
can be the following:

\(1\) The manipulability measure:
$$w(\boldsymbol{q})=\sqrt{\operatorname{det}\left(\boldsymbol{J}(\boldsymbol{q}) \boldsymbol{J}^{T}(\boldsymbol{q})\right)}$$

By maximizing this measure, redundancy is exploited to move away from
singularities.

\(2\) The distance from mechanical joint limits:

$$w(\boldsymbol{q})=-\frac{1}{2 n} \sum_{i=1}^{n}\left(\frac{q_{i}-\bar{q}_{i}}{q_{i M}-q_{i m}}\right)^{2}$$

where $q_{i M}\left(q_{i m}\right)$ is the maximum (minimum) joint limit
and $\bar{q}_{i}$ the middle point of the joint range; thus, by
maximizing this distance, redundancy is exploited to keep the joint
variables close to the center of their ranges.

\(3\) The distance from an obstacle:

$$w(\boldsymbol{q})=\min _{\boldsymbol{p}, \boldsymbol{o}}\|\boldsymbol{p}(\boldsymbol{q})-\boldsymbol{o}\|$$

where $\boldsymbol{o}$ is the position vector of a suitable point on the
obstacle.

# Singularity

The inverse velocity kinematics for both the redundant and non-redundant
manipulators require the current robot configuration $\boldsymbol{q}$ is
nonsingular, i.e., the Jacobian $\boldsymbol{J}(\boldsymbol{q})$ has
full rank. At a singular configuration,
$\boldsymbol{v}_{e}=\boldsymbol{J} \dot{\boldsymbol{q}}$ contains
linearly dependent equations, and it is possible to find a solution
$\dot{\boldsymbol{q}}$ only if
$\boldsymbol{v}_{e} \in \mathcal{R}(\boldsymbol{J})$. This situation
means that the assigned $\boldsymbol{v}_{e}$ is physically achievable by
finding a joint velocity $\dot{\boldsymbol{q}}$, even though it is at a
singular configuration. If instead
$\boldsymbol{v}_{e} \notin \mathcal{R}(\boldsymbol{J})$, the system of
equations has no solution; this means that $\boldsymbol{v}_{e}$ cannot
be achievable by the manipulator at the given posture.

Inversion of the Jacobian can represent a serious inconvenience not only
at a singularity but also in the neighborhood of a singularity. In the
neighborhood of a singularity, a relatively small $\boldsymbol{v}_{e}$
(in terms of its norm) which can cause large joint velocities. Consider
the example of the shoulder singularity for the anthropomorphic arm. If
a path is assigned to the end-effector which passes nearby the base
rotation axis, the base joint is forced to make a rotation of about
$\pi$ in a relatively short time to allow the end-effector to keep
tracking the imposed trajectory. An solution overcoming the problem of
inverting differentia kinematics in the neighbourhood of a singularity
is provided by the so-called damped least-squares (DLS) inverse

$$\boldsymbol{J}^{\star}=\boldsymbol{J}^{T}\left(\boldsymbol{J} \boldsymbol{J}^{T}+k^{2} \boldsymbol{I}\right)^{-1}$$

where $k$ is a damping factor that renders the inversion better
conditioned from a numerical viewpoint. It can be shown that such a
solution can be obtained by minimizing the following objective

$$\min_{\dot{\boldsymbol{q}}}\quad \frac{1}{2}\left(\boldsymbol{v}_{e}-\boldsymbol{J} \dot{\boldsymbol{q}}\right)^{T}\left(\boldsymbol{v}_{e}-\boldsymbol{J} \dot{\boldsymbol{q}}\right)+\frac{1}{2} k^{2} \dot{\boldsymbol{q}}^{T} \dot{\boldsymbol{q}}$$
