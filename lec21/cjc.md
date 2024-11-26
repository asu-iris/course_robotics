---
author:
- Wanxin Jin
title: "Lecture 21: Centralized Joint Control"
---

# Centralized Joint Control

In the previous sections, we have discussed the design of independent (decentralized)
joint controllers for each joint. Each joint is viewed as a single input and single-output
system, and the coupling effects between the joints 
considered as disturbances. However, this has the limitation: when the coupling effects between joints is not ignorable, the decentralized performance may
generate large tracking errors. In this case, it would be desirable to design
control algorithms that explicitly take into considertaion  the details of coupling effect to
compensate for it. This leads to centralized control algorithms.

In the following control design for a $n$-joint robot arm, we consider
a robot arm without external end-effector forces and  joint
friction. The equation of motion of the robot arm thus is

$$
    \boldsymbol{B}(\boldsymbol{q}) \ddot{\boldsymbol{q}}+\boldsymbol{C}(\boldsymbol{q}, \dot{\boldsymbol{q}}) \dot{\boldsymbol{q}}+\boldsymbol{g}(\boldsymbol{q})=\boldsymbol{\tau}.$$(equ.cjc_robotarm)

The controller we want to design is

$$
\boldsymbol{\tau}=\boldsymbol{u}=\textbf{Controller}(\boldsymbol{q}, \boldsymbol{\dot{q}}, \boldsymbol{q}_d, \boldsymbol{\dot{q}}_d)$$(equ.cjc_controller)

i.e., the controller takes as input the robot's current joint
position $\boldsymbol{q}$, joint velocity
$\boldsymbol{\dot{q}}$, desired joint position $\boldsymbol{q}_d$,
and desired joint velocity $\boldsymbol{\dot{q}}_d$, and outputs the
joint torque $\boldsymbol{u}$.

Thus, motion of equation for the robot arm with the controller is

$$
    \boldsymbol{B}(\boldsymbol{q}) \ddot{\boldsymbol{q}}+\boldsymbol{C}(\boldsymbol{q}, \dot{\boldsymbol{q}}) \dot{\boldsymbol{q}}+\boldsymbol{g}(\boldsymbol{q})=\textbf{Controller}(\boldsymbol{q}, \boldsymbol{\dot{q}}, \boldsymbol{q}_d, \boldsymbol{\dot{q}}_d)$$(equ.cjc_system)

## PD Control with Gravity Compensation

Given a *constant* desired joint position $\boldsymbol{q}_{d}$, we want
to design a controller {eq}`equ.cjc_controller`, such that from any initial robot configuration,
say $\boldsymbol{q}_0$, the controlled robot arm {eq}`equ.cjc_system` will eventually reach
$\boldsymbol{q}_{d}$. If we define the joint error as 

$${\boldsymbol{e}}=\boldsymbol{q}_{d}-\boldsymbol{q}$$

we want

$$\boldsymbol{e}(t)\rightarrow \boldsymbol{0}\quad \text{as}\quad
t\rightarrow \infty.$$

Below, we will design the controller
{eq}`equ.cjc_controller` based on the Lyapunov method (see
background of Lyapunov method in `Numerical Inverse Kinematics`). 


First, we take the
vector $\begin{bmatrix}\boldsymbol{e}\\
\boldsymbol{\dot{q}}\end{bmatrix}$ as the system state vector. Choose
the following positive quadratic form as Lyapunov function:

$$V(\dot{\boldsymbol{q}}, {\boldsymbol{e}})=\frac{1}{2} \dot{\boldsymbol{q}}^{T} \boldsymbol{B}(\boldsymbol{q}) \dot{\boldsymbol{q}}+\frac{1}{2} {\boldsymbol{e}}^{T} \boldsymbol{K}_{P} {\boldsymbol{e}} \quad >0 \quad \forall \dot{\boldsymbol{q}}, {\boldsymbol{e}} \neq \mathbf{0}$$

where $\boldsymbol{K}_{P}$ is an $(n \times n)$ symmetric positive
definite matrix. Differentiating
$V(\dot{\boldsymbol{q}}, {\boldsymbol{e}})$ with respect to time, by
recalling that $\boldsymbol{q}_{d}$ is constant, yields

$$\dot{V}=\dot{\boldsymbol{q}}^{T} \boldsymbol{B}(\boldsymbol{q}) \ddot{\boldsymbol{q}}+\frac{1}{2} \dot{\boldsymbol{q}}^{T} \dot{\boldsymbol{B}}(\boldsymbol{q}) \dot{\boldsymbol{q}}-\dot{\boldsymbol{q}}^{T} \boldsymbol{K}_{P} {\boldsymbol{e}}$$(equ.cjc_vdot)

From {eq}`equ.cjc_robotarm`, we can solve for
$\boldsymbol{B} \ddot{\boldsymbol{q}}$ and substituting it to {eq}`equ.cjc_vdot` gives

$$\label{equ.lypuanovdot}
    \dot{V}=\frac{1}{2} \underbrace{\dot{\boldsymbol{q}}^{T}(\dot{\boldsymbol{B}}(\boldsymbol{q})-2 \boldsymbol{C}(\boldsymbol{q}, \dot{\boldsymbol{q}})) \dot{\boldsymbol{q}}}_{0}+\dot{\boldsymbol{q}}^{T}\left(\boldsymbol{u}-\boldsymbol{g}(\boldsymbol{q})-\boldsymbol{K}_{P} {\boldsymbol{e}}\right)$$ (equ.cjc_vdot2)

The first term on the right-hand side is zero since the matrix
$\dot{\boldsymbol{B}}-2 \boldsymbol{C}$ is a
skew-symmetric matrix (see in the property of dynamics in `Dynamics`).
For the second term to be negative definite, we can set the controller {eq}`equ.cjc_controller` as

:::{important}
$$\label{equ.controller2}
\boldsymbol{u}=\underbrace{\boldsymbol{g}(\boldsymbol{q})}_{\text{gravitational compensation}}+\underbrace{\boldsymbol{K}_{P} \boldsymbol{e}}_{\text{proportional control}}\quad\underbrace{-\boldsymbol{K}_{D} \dot{\boldsymbol{q}}}_{\text{derivative control}}$$(equ.cjc_pd)

with $\boldsymbol{K}_{D}$ positive definite matrix. The above controller {eq}`equ.cjc_pd` includes a
gravitational compensation term and linear
proportional-derivative (PD) control term.
:::




```{figure} ../lec19/control/PD_control_with_gravity_compensation.jpeg
---
width: 90%
name: pv_control_rl_1
---
Control diagram of PD control with gravity
compensation
```



With the above controller {eq}`equ.cjc_pd`, the time derivative of the Lyapunov
function in {eq}`equ.cjc_vdot2` becomes

$$\dot{V}=-\boldsymbol{\dot{q}}^T\boldsymbol{K}_D\boldsymbol{\dot{q}} \quad <0 \quad \forall \dot{\boldsymbol{q}}  \neq \mathbf{0}$$

and thus, $V$ decreases long as $\dot{\boldsymbol{q}} \neq \mathbf{0}$
for all system trajectories. Thus, the robot arm will reach
an equilibrium configuration, with condition $\dot{V}= 0$ and
$\dot{\boldsymbol{q}}=\mathbf{0}$. 


To find the equilibrium
configuration, we look at the robot dynamics under control (by plug the controller {eq}`equ.cjc_pd` back into {eq}`equ.cjc_system`)

$$\boldsymbol{B}(\boldsymbol{q}) \ddot{\boldsymbol{q}}+\boldsymbol{C}(\boldsymbol{q}, \dot{\boldsymbol{q}}) \dot{\boldsymbol{q}}+\boldsymbol{g}(\boldsymbol{q})=\boldsymbol{g}(\boldsymbol{q})+\boldsymbol{K}_{P} {\boldsymbol{e}}-\boldsymbol{K}_{D} \dot{\boldsymbol{q}}$$(equ.cjc_system)

With the condition $\dot{\boldsymbol{q}}=\mathbf{0}$, it follows that
$\ddot{\boldsymbol{q}} =\mathbf{0}$, and then from the above dynamics {eq}`equ.cjc_system`,
we have

$$\boldsymbol{K}_{P} {\boldsymbol{e}}=\mathbf{0} \quad\rightarrow\quad {\boldsymbol{e}}=\boldsymbol{q}_{d}-\boldsymbol{q} = \mathbf{0}$$

Therefore, the robot arm will eventually reach equilibrium, which is
exactly the desired configuration $\boldsymbol{q}_{d}$.

<!-- The above derivation rigorously shows that any manipulator equilibrium
posture is globally asymptotically stable under a controller with a PD
linear action and a nonlinear gravity compensating action. Stability is
ensured for any choice of $\boldsymbol{K}_{P}$ and $\boldsymbol{K}_{D}$,
as long as these are positive definite matrices. The control law
requires the on-line computation of the term
$\boldsymbol{g}(\boldsymbol{q})$. -->

</br> </br> </br>
## Inverse Dynamics Control

One limitation of the above controller is that the above controller can
only follow a stationary desired $\boldsymbol{q}_d$, but is not good at
tracking a changing desired joint signal: i.e.,

$$\label{equ.joint_signal}
\boldsymbol{q}_d(t),\quad\text{with} \quad \boldsymbol{\dot{q}}_d(t)\neq\boldsymbol{0} \quad \text{and} \quad \boldsymbol{\ddot{q}}_d(t)\neq\boldsymbol{0}$$

The approach of inverse dynamics control is trying to address the above
limitation. The idea of inverse dynamics control is to find a controller {eq}`equ.cjc_controller`
 that can directly lead to a linear 
system dynamics. To do so, we set the controller
([\[equ.controller\]](#equ.controller){reference-type="ref"
reference="equ.controller"}) in the following particular form:

:::{important} 
$$\label{equ:u}
    \boldsymbol{u}=\boldsymbol{B}(\boldsymbol{q}) \boldsymbol{y}+\boldsymbol{C}(\boldsymbol{q}, \dot{\boldsymbol{q}}) \dot{\boldsymbol{q}}+\boldsymbol{g}(\boldsymbol{q})$$
:::

where $\boldsymbol{y}$ is a new input vector whose expression is to be
determined yet. The above controller directly leads to the controlled
manipulator
([\[equ.manipulator_control\]](#equ.manipulator_control){reference-type="ref"
reference="equ.manipulator_control"}) as

$$\boldsymbol{B}(\boldsymbol{q}) \ddot{\boldsymbol{q}}+\boldsymbol{C}(\boldsymbol{q}, \dot{\boldsymbol{q}}) \dot{\boldsymbol{q}}+\boldsymbol{g}(\boldsymbol{q})=\boldsymbol{B}(\boldsymbol{q}) \boldsymbol{y}+\boldsymbol{C}(\boldsymbol{q}, \dot{\boldsymbol{q}}) \dot{\boldsymbol{q}}+\boldsymbol{g}(\boldsymbol{q})$$

leading to 

$$\label{equ.dyn_new}
\boldsymbol{\ddot{q}}=\boldsymbol{y}$$ 

Again, we will discuss how to set
this new control input vector $\boldsymbol{y}$, but the particular form
of controller ([\[equ:u\]](#equ:u){reference-type="ref"
reference="equ:u"}) leads to a linear relationship between the input
signal $\boldsymbol{y}$ and the manipulator's joint acceleration
$\ddot{\boldsymbol{q}}$.

The control diagram corresponding to the controller
([\[equ:u\]](#equ:u){reference-type="ref" reference="equ:u"}) is shown
in Fig. [2](#fig:2){reference-type="ref" reference="fig:2"}. The
controller ([\[equ:u\]](#equ:u){reference-type="ref" reference="equ:u"})
is termed inverse dynamics control as it is based on the computation of
inverse dynamics. From
([\[equ.dyn_new\]](#equ.dyn_new){reference-type="ref"
reference="equ.dyn_new"}, the controlled manipulator system is linear
and decoupled with respect to new input $\boldsymbol{y}$.


```{figure} ../lec19/control/inverse_dynamics_control.jpeg
---
width: 70%
name: inverse_dynamics_control
---
Exact linearization performed by inverse dynamics
control
```



Thus, the manipulator control problem is reduced to that of finding a
control input signal $\boldsymbol{y}$. To this end,

:::{important}
$$\label{equ.y}
    \boldsymbol{y}=\ddot{\boldsymbol{{q}}}_d+\boldsymbol{K}_{P} ({\boldsymbol{q}}_{d}-\boldsymbol{q})+\boldsymbol{K}_{D} (\dot{\boldsymbol{q}}_{d}-\dot{\boldsymbol{q}})$$
:::

Here, $\boldsymbol{K}_{P}$ and $\boldsymbol{K}_{D}$ are positive
definite matrices. Plugging the above $\boldsymbol{y}$ signal into
([\[equ.dyn_new\]](#equ.dyn_new){reference-type="ref"
reference="equ.dyn_new"}) leads to 

$$\label{equ.error_dyn2}
    (\ddot{\boldsymbol{{q}}}_d-\ddot{\boldsymbol{q}})+\boldsymbol{K}_{P} ({\boldsymbol{q}}_{d}-\boldsymbol{q})+\boldsymbol{K}_{D} (\dot{\boldsymbol{q}}_{d}-\dot{\boldsymbol{q}})=\boldsymbol{0}$$

Again, if we define the error
$\boldsymbol{e}={\boldsymbol{q}}_{d}-{\boldsymbol{q}}$, the above
([\[equ.error_dyn\]](#equ.error_dyn){reference-type="ref"
reference="equ.error_dyn"}) becomes the following error dynamics

$$\label{equ.error_dyn}
    \ddot{\boldsymbol{e}}+\boldsymbol{K}_{P} \dot{\boldsymbol{e}}+\boldsymbol{K}_{D} {\boldsymbol{e}}=\boldsymbol{0}$$

expressing the dynamics of position error while tracking the given
trajectory. This is a standard second-order error system, where its
convergence depends on the value of $\boldsymbol{K}_P$ and
$\boldsymbol{K}_D$. Particularly, we usually choose 

$$\label{equ.kpkd}
    \boldsymbol{K}_{P}=\operatorname{diag}\left\{\omega_{n 1}^{2}, \ldots, \omega_{n n}^{2}\right\} \quad \boldsymbol{K}_{D}=\operatorname{diag}\left\{2 \zeta_{1} \omega_{n 1}, \ldots, 2 \zeta_{n} \omega_{n n}\right\}$$


Then, the controlled manipulator dynamics is replaced with $n$ linear
and decoupled second-order subsystems. For each joint $i$, please see my
control background notes for how the values of the damping ratio
$\zeta_i$ and natural frequency $\omega_{ni}$ can affect the trajectory
of $\boldsymbol{e}(t)$.

The resulting block scheme is illustrated in Fig.
[3](#fig:3){reference-type="ref" reference="fig:3"}. Notice that the
inverse dynamics controller requires computation of the inertia matrix
$\boldsymbol{B}(\boldsymbol{q})$,
$\boldsymbol{C}(\boldsymbol{q}, \dot{\boldsymbol{q}})$, and
$\boldsymbol{g}(\boldsymbol{q})$. Nonetheless, this technique is based
on the assumption of perfect cancellation of dynamic terms, and then it
is quite natural to raise questions about sensitivity and robustness
problems due to unavoidably imperfect compensation.



```{figure} ../lec19/control/inverse_dynamics_control2.jpeg
---
width: 70%
name: inverse_dynamics_control2
---
Control diagram of joint space inverse dynamics
control
```

