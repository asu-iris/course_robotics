---
author:
- Wanxin Jin
date: Sep. 14, 2023
title: "Lecture 9: Analytic Inverse Kinematics (IK)"
---

# Inverse Kinematics

The inverse kinematics (IK) problem is to determine the joint variables
given end-effector position and orientation. The solution to IK is of
fundamental importance in order to transform the motion assignment in
the end-effector in the operational space into the joint space motions
for control purpose. The IK problem is complex since:

-   Forward kinematics in general are nonlinear, and thus IK is not
    always possible to find a closed-form solution.

-   Multiple or infinite solutions may exist for a IK problem, e.g., in
    the case of a redundant manipulator.

-   There might be no admissible solutions, in view of the manipulator
    kinematic limits.

In this lecture, we only talk about the IK problem that permits a
closed-form solution. This is only valid for some special-structured
kinematic structures. Those IK solutions are all derived based on
geometric intuition. For more generalized case of IK, we will leave it
after we have learned differential kinematics.

# IK of Three-link Planar Arm


```{figure} ./kinematics/3link_arm.jpg
---
width: 50%
name: 3link_arm
---
A three link robot arm
```

Suppose the pose of the end-effector is specified with its planar
position $(p_{x}, p_{y})$ and the angle $\phi$ with the axis $x_{0}$. We
want to use IK to find the corresponding joint variables
$\boldsymbol{q}=[\vartheta_{1}, \vartheta_{2}, \vartheta_{3}]$.

To achieve so, we first identify the following relation holds

$$\label{c1.l2.3link.sum}
    \phi=\vartheta_{1}+\vartheta_{2}+\vartheta_{3}$$

Also, the following equations can be obtained:

$$\begin{aligned}
& p_{W x}=p_{x}-a_{3} c_{\phi}=a_{1} c_{1}+a_{2} c_{12} \\
& p_{W y}=p_{y}-a_{3} s_{\phi}=a_{1} s_{1}+a_{2} s_{12}
\end{aligned}$$

which describe the position of point $W$, i.e., the origin of Frame 2;
depending only on $\vartheta_{1}$ and $\vartheta_{2}$.

The cosine theorem to the triangle formed by links $a_{1}, a_{2}$ and
the segment connecting $W$ and $O$ gives

$$p_{W x}^{2}+p_{W y}^{2}=a_{1}^{2}+a_{2}^{2}-2 a_{1} a_{2} \cos \left(\pi-\vartheta_{2}\right) ;$$

Solving $c_2$ leads to

$$c_{2}=\frac{p_{W x}^{2}+p_{W y}^{2}-a_{1}^{2}-a_{2}^{2}}{2 a_{1} a_{2}}$$

For the existence of the triangle, it must be
$\sqrt{p_{W x}^{2}+p_{W y}^{2}} \leq a_{1}+a_{2}$. This condition is not
satisfied when the given point is outside the arm reachable workspace.
Then, under the assumption of admissible solutions, it is

$$\vartheta_{2}= \pm \cos ^{-1}\left(c_{2}\right)$$

Thus, two admissible $\vartheta_{2}$ are obtained: the elbow-up posture
is obtained for $\vartheta_{2} \in(-\pi, 0)$ while the elbow-down
posture is obtained for $\vartheta_{2} \in(0, \pi)$, as shown in the
figure blow.


```{figure} ./kinematics/postures_2link_arm.jpg
---
width: 50%
name: postures_2link_arm
---
Admissible postures for a two-link planar
arm
```

To find $\vartheta_{1}$ consider the angles $\alpha$ and $\beta$ in the
above figure. Notice that the determination of $\alpha$ depends on the
sign of $p_{W x}$ and $p_{W y}$; then, it is necessary to compute
$\alpha$ as

$$\alpha=\operatorname{Atan} 2\left(p_{W y}, p_{W x}\right) .$$

To compute $\beta$, applying again the cosine theorem yields

$$c_{\beta} \sqrt{p_{W x}^{2}+p_{W y}^{2}}=a_{1}+a_{2} c_{2}$$

and resorting to the expression of $c_{2}$ given above leads to

$$\beta=\cos ^{-1}\left(\frac{p_{W x}^{2}+p_{W y}^{2}+a_{1}^{2}-a_{2}^{2}}{2 a_{1} \sqrt{p_{W x}^{2}+p_{W y}^{2}}}\right)$$

with $\beta \in(0, \pi)$ so as to preserve the existence of triangles.
Then, it is

$$\vartheta_{1}=\alpha \pm \beta$$

where the positive sign holds for $\vartheta_{2}<0$ and the negative
sign for $\vartheta_{2}>0$. Finally, $\vartheta_{3}$ is computed from
([\[c1.l2.3link.sum\]](#c1.l2.3link.sum){reference-type="ref"
reference="c1.l2.3link.sum"}).

# IK of Manipulators of Spherical Wrist

Most of manipulators are kinematically simple, since they are typically
formed by an arm and a spherical wrist. This choice is partly motivated
by the difficulty to find IK in the general case. In particular, a
six-DOF kinematic structure has closed-form IK solutions if:

-   three consecutive revolute joint axes intersect at a common point,
    like for the spherical wrist;

-   three consecutive revolute joint axes are parallel, like the
    three-link robot arm

Inspired by IK to a three-link planar arm, an itermediate point on the
manipulator can be found, such that the IK problem can be decoupled into
two lower-dimentional sub-problems. Specifically, for a manipulator with
spherical wrist, the natural choice is to locate such point $W$ at the
wrist position, i.e., at the intersection of the three revolute axes of
the wrist. If the end-effector pose is $\boldsymbol{p}_{e}$ and
$\boldsymbol{R}_{e}=\left[\begin{array}{lll}\boldsymbol{n}_{e} & \boldsymbol{s}_{e} & \boldsymbol{a}_{e}\end{array}\right]$,
the wrist position will be

$$\boldsymbol{p}_{W}=\boldsymbol{p}_{e}-d_{6} \boldsymbol{a}_{e}$$

```{figure} ./kinematics/analytic_ik_decouple.jpg
---
width: 50%
name: analytic_ik_decouple
---
Manipulator with spherical
wrist
```

which is a function of the sole joint variables that determine the arm
position. Hence, in the case of a (nonredundant) three-DOF arm, the
inverse kinematics can be solved according to the following steps:

-   Compute the wrist position
    $\boldsymbol{p}_{W}\left(q_{1}, q_{2}, q_{3}\right)$.

-   Solve inverse kinematics for $\left(q_{1}, q_{2}, q_{3}\right)$.

-   Compute $\boldsymbol{R}_{3}^{0}\left(q_{1}, q_{2}, q_{3}\right)$.

-   Compute
    $\boldsymbol{R}_{6}^{3}\left(\vartheta_{4}, \vartheta_{5}, \vartheta_{6}\right)=\boldsymbol{R}_{3}^{0 T} \boldsymbol{R}$

-   Solve inverse kinematics for orientation
    $\left(\vartheta_{4}, \vartheta_{5}, \vartheta_{6}\right)$

Therefore, on the basis of this kinematic decoupling, IK for the arm
separately from the IK for the spherical wrist. Below are presented the
solutions for two typical arms (spherical and anthropomorphic) as well
as the solution for the spherical wrist.

## IK of Spherical Arm

Consider the spherical arm:


```{figure} ./kinematics/spherical_arm.jpg
---
width: 50%
name: spherical_arm
---
Spherical arm
```


Its kinematics is
$$\boldsymbol{T}_{3}^{0}(\boldsymbol{q})=\boldsymbol{A}_{1}^{0} \boldsymbol{A}_{2}^{1} \boldsymbol{A}_{3}^{2}=\left[\begin{array}{cccc}
c_{1} c_{2} & -s_{1} & c_{1} s_{2} & c_{1} s_{2} d_{3}-s_{1} d_{2} \\
s_{1} c_{2} & c_{1} & s_{1} s_{2} & s_{1} s_{2} d_{3}+c_{1} d_{2} \\
-s_{2} & 0 & c_{2} & c_{2} d_{3} \\
0 & 0 & 0 & 1
\end{array}\right]$$

To find the joint variables $\vartheta_{1}, \vartheta_{2}, d_{3}$
corresponding to $\boldsymbol{p}_{W}=[p_{Wx}, p_{Wy}, p_{Wz}]^T$, we
equate the first three elements of the fourth columns of the matrices
and thus obtain the following after some equation manipulation

$$\left[\begin{array}{c}
p_{W x} c_{1}+p_{W y} s_{1} \\
-p_{W z} \\
-p_{W x} s_{1}+p_{W y} c_{1}
\end{array}\right]=\left[\begin{array}{c}
d_{3} s_{2} \\
-d_{3} c_{2} \\
d_{2}
\end{array}\right]$$

Then, solving the above equation for
$\vartheta_{1}, \vartheta_{2}, d_{3}$ yields

$$\vartheta_{1}=2 \operatorname{Atan} 2\left(-p_{W x} \pm \sqrt{p_{W x}^{2}+p_{W y}^{2}-d_{2}^{2}}, d_{2}+p_{W y}\right)$$

$$\vartheta_{2}=\operatorname{Atan} 2\left(p_{W x} c_{1}+p_{W y} s_{1}, p_{W z}\right)$$

$$d_{3}=\sqrt{\left(p_{W x} c_{1}+p_{W y} s_{1}\right)^{2}+p_{W z}^{2}}$$

## IK of Anthropomorphic Arm

Consider the anthropomorphic arm:


```{figure} ./kinematics/anthropomorphic_arm.jpg
---
width: 50%
name: anthropomorphic_arm
---
Anthropomorphic arm
```


$$\boldsymbol{T}_{3}^{0}(\boldsymbol{q})=\boldsymbol{A}_{1}^{0} \boldsymbol{A}_{2}^{1} \boldsymbol{A}_{3}^{2}=\left[\begin{array}{cccc}
c_{1} c_{23} & -c_{1} s_{23} & s_{1} & c_{1}\left(a_{2} c_{2}+a_{3} c_{23}\right) \\
s_{1} c_{23} & -s_{1} s_{23} & -c_{1} & s_{1}\left(a_{2} c_{2}+a_{3} c_{23}\right) \\
s_{23} & c_{23} & 0 & a_{2} s_{2}+a_{3} s_{23} \\
0 & 0 & 0 & 1
\end{array}\right]$$

To find the joint variables $\vartheta_{1}, \vartheta_{2}, d_{3}$
corresponding to $\boldsymbol{p}_{W}=[p_{Wx}, p_{Wy}, p_{Wz}]^T$, we
equate the first three elements of the fourth columns of the matrices
and thus obtain the following

$$\begin{aligned}
& p_{W x}=c_{1}\left(a_{2} c_{2}+a_{3} c_{23}\right) \\
& p_{W y}=s_{1}\left(a_{2} c_{2}+a_{3} c_{23}\right) \\
& p_{W z}=a_{2} s_{2}+a_{3} s_{23} .
\end{aligned}$$

There exist four solutions:

$$\left(\vartheta_{1, \mathrm{I}}, \vartheta_{2, \mathrm{I}}, \vartheta_{3, \mathrm{I}}\right) \quad\left(\vartheta_{1, \mathrm{I}}, \vartheta_{2, \mathrm{III}}, \vartheta_{3, \mathrm{II}}\right) \quad\left(\vartheta_{1, \mathrm{II}}, \vartheta_{2, \mathrm{II}}, \vartheta_{3, \mathrm{I}}\right) \quad\left(\vartheta_{1, \mathrm{II}}, \vartheta_{2, \mathrm{IV}}, \vartheta_{3, \mathrm{II}}\right)$$

with

$$c_{3}=\frac{p_{W x}^{2}+p_{W y}^{2}+p_{W z}^{2}-a_{2}^{2}-a_{3}^{2}}{2 a_{2} a_{3}}$$

$$s_{3}^+= \sqrt{1-c_{3}^{2}}\quad\quad s_{3}^-= -\sqrt{1-c_{3}^{2}}$$

$$\begin{aligned}
\vartheta_{3, \mathbf{I}}&=\operatorname{Atan} 2 \left( s_3^{+}, c_{3}\right)\\
\vartheta_{3, \mathrm{II}} & =\operatorname{Atan} 2 \left( s_3^{-}, c_{3}\right)
\end{aligned}$$

$$\begin{aligned}
\vartheta_{1, \mathrm{I}} & =\operatorname{Atan} 2\left(p_{W y}, p_{W x}\right) \\
\vartheta_{1, \mathrm{II}} & =\operatorname{Atan} 2\left(-p_{W y},-p_{W x}\right) .
\end{aligned}$$

$$\begin{aligned}
\vartheta_{2, \mathrm{I}}=\operatorname{Atan} 2 & \left(\left(a_{2}+a_{3} c_{3}\right) p_{W z}-a_{3} s_{3}^{+} \sqrt{p_{W x}^{2}+p_{W y}^{2}},\right. 
\left.\left(a_{2}+a_{3} c_{3}\right) \sqrt{p_{W x}^{2}+p_{W y}^{2}}+a_{3} s_{3}^{+} p_{W z}\right) \\
\vartheta_{2, \mathrm{II}}=\operatorname{Atan} 2 & \left(a_{2}+a_{3} c_{3}\right) p_{W z}+a_{3} s_{3}^{+} \sqrt{p_{W x}^{2}+p_{W y}^{2}},  \left.-\left(a_{2}+a_{3} c_{3}\right) \sqrt{p_{W x}^{2}+p_{W y}^{2}}+a_{3} s_{3}^{+} p_{W z}\right)
\end{aligned}$$

$$\begin{aligned}
\vartheta_{2, \mathrm{III}}=\operatorname{Atan2} & \left(\left(a_{2}+a_{3} c_{3}\right) p_{W z}-a_{3} s_{3}^{-} \sqrt{p_{W x}^{2}+p_{W y}^{2}},\right.  \left.\left(a_{2}+a_{3} c_{3}\right) \sqrt{p_{W x}^{2}+p_{W y}^{2}}+a_{3} s_{3}^{-} p_{W z}\right) \\
\vartheta_{2, \mathrm{IV}}=\operatorname{Atan} 2 & \left(\left(a_{2}+a_{3} c_{3}\right) p_{W z}+a_{3} s_{3}^{-} \sqrt{p_{W x}^{2}+p_{W y}^{2}},\right.  \left.-\left(a_{2}+a_{3} c_{3}\right) \sqrt{p_{W x}^{2}+p_{W y}^{2}}+a_{3} s_{3}^{-} p_{W z}\right)
\end{aligned}$$

which are illustrated below: shoulder-right/elbow-up,
shoulder-left/elbowup, shoulder-right/elbow-down,
shoulder-left/elbow-down; obviously, the forearm orientation is
different for the two pairs of solutions.


```{figure} ./kinematics/4IK_solutions_anthropomorphic_arm.jpg
---
width: 50%
name: 4IK_solutions_anthropomorphic_arm
---
The four configurations of an anthropomorphic arm compatible with a
given wrist
position
```

## IK of Spherical Wrist

Consider the spherical wrist below.


```{figure} ./kinematics/spherical_wrist.jpg
---
width: 50%
name: spherical_wrist
---
Spherical wrist
```


To find the joint variables
$\vartheta_{4}, \vartheta_{5}, \vartheta_{6}$ corresponding to a given
end-effector orientation $\boldsymbol{R}_{6}^{3}$. As previously pointed
out, these angles constitute a set of Euler angles ZYZ with respect to
Frame 3.

$$\boldsymbol{R}_{6}^{3}=\left[\begin{array}{rrr}
n_{x}^{3} & s_{x}^{3} & a_{x}^{3} \\
n_{y}^{3} & s_{y}^{3} & a_{y}^{3} \\
n_{z}^{3} & s_{z}^{3} & a_{z}^{3}
\end{array}\right]$$

from its expression in terms of the joint variables, it is possible to
compute

$$\begin{aligned}
& \vartheta_{4}=\operatorname{Atan} 2\left(a_{y}^{3}, a_{x}^{3}\right) \\
& \vartheta_{5}=\operatorname{Atan} 2\left(\sqrt{\left(a_{x}^{3}\right)^{2}+\left(a_{y}^{3}\right)^{2}}, a_{z}^{3}\right) \\
& \vartheta_{6}=\operatorname{Atan} 2\left(s_{z}^{3},-n_{z}^{3}\right)
\end{aligned}$$

for $\vartheta_{5} \in(0, \pi)$, and

$$\begin{aligned}
& \vartheta_{4}=\operatorname{Atan} 2\left(-a_{y}^{3},-a_{x}^{3}\right) \\
& \vartheta_{5}=\operatorname{Atan} 2\left(-\sqrt{\left(a_{x}^{3}\right)^{2}+\left(a_{y}^{3}\right)^{2}}, a_{z}^{3}\right) \\
& \vartheta_{6}=\operatorname{Atan} 2\left(-s_{z}^{3}, n_{z}^{3}\right)
\end{aligned}$$

for $\vartheta_{5} \in(-\pi, 0)$
