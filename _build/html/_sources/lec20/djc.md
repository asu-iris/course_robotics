---
author:
- Wanxin Jin
title: "Lecture 20: Decentralized Joint Control"
---

# Decentralized Joint Control

The equation of motion of a manipulator without end-effector contact
force and any joint friction is

$$
    \boldsymbol{B}(\boldsymbol{q}) \ddot{\boldsymbol{q}}+\boldsymbol{C}(\boldsymbol{q}, \dot{\boldsymbol{q}}) \dot{\boldsymbol{q}}+\boldsymbol{g}(\boldsymbol{q})=\boldsymbol{\tau}$$(equ.control_arm_dyn)

From the previous chapter, the joint-motor
transmission gives

$$
    \boldsymbol{q}_{m}=\boldsymbol{K}_{r} \boldsymbol{q}$$(equ.motor_angle)

<!-- $\boldsymbol{K}_{r}=\text{diag}(k_{r_1}, k_{r_2}, ..., k_{r_n})$ -->

where $\boldsymbol{K}_{r}$
is a diagonal matrix, and each diagonal element $k_{r_i}$ is the gear
ratio of joint $i$. Let
$\boldsymbol{\tau}_{m}$ denote
the vector of all motor torques, one can write

$$
\boldsymbol{\tau}= \boldsymbol{K}_{r}\boldsymbol{\tau}_{m}$$(equ.motor_torque)

Substituting {eq}`equ.motor_torque` and {eq}`equ.motor_angle`
into the robot arm dynamics {eq}`equ.control_arm_dyn`
leads to

 $$
    \boldsymbol{K}_{r}^{-1} \boldsymbol{B}(\boldsymbol{q}) \boldsymbol{K}_{r}^{-1} \ddot{\boldsymbol{q}}_{m}+\boldsymbol{K}_{r}^{-1} \boldsymbol{C}(\boldsymbol{q}, \dot{\boldsymbol{q}}) \boldsymbol{K}_{r}^{-1} \dot{\boldsymbol{q}}_{m}+\boldsymbol{K}_{r}^{-1} \boldsymbol{g}(\boldsymbol{q})=\boldsymbol{\tau}_{m}$$(equ.control_dyn2)

## The idea of how we do decentralized control



We have previously (in dynamics lecture) analyzed the inertia matrix
$\boldsymbol{B}(\boldsymbol{q})$, one can write it as 

$$
    \boldsymbol{B}(\boldsymbol{q})=\overline{\boldsymbol{B}}+\Delta \boldsymbol{B}(\boldsymbol{q})
$$(equ.bmat2)


where $\overline{\boldsymbol{B}}$ is the diagonal matrix whose constant
elements represent the average inertia at each joint, and elements in
$\Delta \boldsymbol{B}(\boldsymbol{q})$ represent the matrix of
non-diagonal part. 


Substituting {eq}`equ.bmat2`
into {eq}`equ.control_dyn2` leads to 

$$
    \underbrace{\boldsymbol{K}_{r}^{-1} \overline{\boldsymbol{B}} \boldsymbol{K}_{r}^{-1}}_{\boldsymbol{I}_m} \ddot{\boldsymbol{q}}_{m}+\underbrace{\boldsymbol{K}_{r}^{-1} \Delta \boldsymbol{B}(\boldsymbol{q}) \boldsymbol{K}_{r}^{-1} \ddot{\boldsymbol{q}}_{m}+\boldsymbol{K}_{r}^{-1} \boldsymbol{C}(\boldsymbol{q}, \dot{\boldsymbol{q}}) \boldsymbol{K}_{r}^{-1} \dot{\boldsymbol{q}}_{m}+\boldsymbol{K}_{r}^{-1} \boldsymbol{g}(\boldsymbol{q})}_{\boldsymbol{D}}=\boldsymbol{\tau}_{m}$$(equ.decouple_model)


Here, $\boldsymbol{I}_m$ is also a diagonal
matrix, whose each diagonal entry encodes the inertia  at that joint.  $\boldsymbol{D}$ in {eq}`equ.decouple_model`
 is a coupling term between different joints.




The above equation {eq}`equ.decouple_model` corresponds to the following diagram


```{figure} ../lec19/control/decentralized_control3.jpeg
---
width: 99%
name: decentralized_control3
---
Dynamics diagram of a motor-driven
robot arm.
```

As shown in both {eq}`equ.decouple_model` and {numref}`decentralized_control3`, if we ignore the coupling term $\boldsymbol{D}$, 

$$\boldsymbol{I}_m\boldsymbol{\ddot{q}}_m=\boldsymbol{\tau}_m,$$


This means that if we ignore  complex term
$\boldsymbol{D}$, each joint is a single-in-single-output system (because $\boldsymbol{I}_m$ is a diagonal matrix).  However, the coupling term $\boldsymbol{D}$ in {eq}`equ.decouple_model` prevents us
from considering so. How do we overcome this in our control design? 

<!-- In the reminder of this chapter, i will use ${\theta}_m$ and $q_m$ interchangeably to denote the variable of each joint, without writing the subscript joint index. For a single joint, if no coupling term $\boldsymbol{D}$, the dynamics equation at the joint is -->



<!-- $$\label{equ.single_joint_dyn}
    {I}_m{\ddot{\theta}_m}=\tau_m$$  -->

```{important}
In our control design, we  consider the
 each joint as a single-in-single-out system, but view the complex coupling effect term
$\boldsymbol{D}$ as a disturbance input to each joint.
```

</br></br>
# Single-Joint Control Diagram

In the reminder of this chapter, i will use ${\theta}_m$ and $q_m$ interchangeably as the variable of each joint, droping the subscript of  joint index. For a single joint,  the dynamics equation at that joint is a row of {eq}`equ.decouple_model`, writing as

$$
I_m\ddot{\theta}_m+D=\tau_m
$$(equ.single_joint_dyn)

Let's recall the DC motor model in our previous chapter:

$$
\tau_m=k_ti_a=k_t\frac{(v_a-k_v\dot{\theta}_m)}{R_a}=k_t\frac{(G_vv_c-k_v\dot{\theta}_m)}{R_a}
$$(equ.single_joint_motor)

where ${k}_{t}$ is the
torque constants; ${i}_{a}$ is the armature current; ${v}_{a}$ is the
vector of armature voltage; ${R}_{a}$ is the armature resistance;
${k}_{v}$ is the back EMF constant; $\boldsymbol{v}_{c}$ is the voltage
of the servomotor, and $G_v$ is input voltage amplifier. Those  parameters have
been shown in previous chapter. We will write $v_a=G_vv_c$ in {eq}`equ.single_joint_motor` in the following derivation. 


Equaling {eq}`equ.single_joint_dyn` and {eq}`equ.single_joint_motor`, one has



$$
I_m\ddot{\theta}_m+D=\tau_m=I_m\ddot{\theta}_m+\frac{k_t}{R_a}(\frac{DR_a}{k_t})=k_t\frac{(v_a-k_v\dot{\theta}_m)}{R_a}
$$(equ.motor)



The above {eq}`equ.motor` corresponds to the following diagram

```{figure} ../lec19/control/motor_model3.jpeg
---
width: 80%
name: motor_model3
---
Diagram of a single motor-joint model, with coupled joint term $D$
treated as disturbance.
```



In the above diagram, the closed loop is due to the DC motor model
itself. For the above diagram, the input ($v_a$) to output ($\theta_m$) transfer function $G(s)$ is

$$
    G(s)=\frac{{\Theta}(s)}{V_a(s)}=\frac{k_m}{s(1+T_ms)}$$(equ.motortf)

with 

$$
k_m=\frac{1}{k_v} \qquad\qquad
T_m=\frac{R_aI_m}{k_tk_v}
% \frac{\frac{k_t}{R_aI_ms}}{1+\frac{k_t}{R_aI_ms}k_v}=
$$

The  input ($v_a$) to output velocity ($\dot{\theta}_m$) transfer function is 


$$\frac{\dot{\Theta}(s)}{V_a(s)}=\frac{k_m}{1+T_ms}$$



```{admonition} Minimal material on control basics 


Please read my hand-writing note on control basics, in order to help you understand the following content of this chapter.

[Minimal notes on control basics](./control_basics.pdf)


```



```{admonition}  Minimal Introduction to Closed-Loop (Feedback) Control Systems

A typical closed-loop control diagram is shown below.
Here, $\theta_r$ is the reference input; $\theta_m$ is the output; $D$
is the disturbance; $G(s)$ is the transfer function of the plant to be
controlled; $C(s)$ is the controller; $H(s)$ is the backward pass
transfer function (usually a constant to model the sensor). The goal of
control design is to find $C(s)$ such that $\theta_m$ is able to track
$\theta_r$, while $D$ has as little effect on $\theta_m$ as possible.

:::{figure} ../lec19/control/fd_control2.jpeg
---
width: 90%
name: fd_control2
---
A typical control system diagram
:::

For the above control system, the forward path transfer function is


$$C(s)G(s)$$ 

The backward path transfer function is

$$H(s)$$ 

The
open loop transfer function is 

$$C(s)G(s)H(s)$$

 The input-to-output
transfer function is

$$
\frac{\Theta_m(s)}{\Theta_r(s)}=\frac{C(s)G(s)}{1+C(s)G(s)H(s)}
$$

 The
disturbance-to-output transfer function is

$$
\frac{\Theta_m(s)}{D(s)}=\frac{G(s)}{1+C(s)G(s)H(s)}
$$


The input ($\theta_r$) to output ($\theta_m$) response of the
closed-loop system will be determined by the roots of the
characteristics equation 

$$1+C(s)G(s)H(s)=0$$
```










</br></br>
# Position Feedback Control Design

The single joint system in {numref}`motor_model3` is the plant $G(s)$ we want to design the controller $C(s)$ for.

The plant
transfer function $G(s)$ is {eq}`equ.motortf`, rewritten below


$$G(s)=\frac{k_m}{s(1+T_ms)}$$

We want to design a proportional-integral controller 

$$\begin{gathered}
C_{P}(s)=K_{P} \frac{1+s T_{P}}{s} 
\end{gathered}$$ 

with $K_P$ and $T_P$ are controller parameters.

The control diagram for the plant  {numref}`motor_model3` thus is shown below (note that the backward pass
constant $k_{TP}$ is fixed, this can be thought of as the sensor gain given).

:::{figure} ../lec19/control/p_control2.jpg
---
width: 99%
name: p_control2
---
Control diagram of singple-joint position 
control
:::



On the diagram {numref}`p_control2`, the transfer function of the forward path is

$$G(s)C_P(s)=\frac{k_{m} K_{P}\left(1+s T_{P}\right)}{s^{2}\left(1+s T_{m}\right)}$$

The transfer function of the backward pass is

$$H(s)=k_{T P}$$


The open loop transfer function is

$$
G(s)C_P(s)H(s)=\frac{k_{T P}k_{m} K_{P}\left(1+s T_{P}\right)}{s^{2}\left(1+s T_{m}\right)}
$$




The closed-loop input/output transfer function is

$$\frac{\Theta_{m}(s)}{\Theta_{r}(s)}=\frac{C_P(s)G(s)}{1+C_P(s)G(s)H(s)}=\frac{{k_{m}K_P(1+T_Ps)}}{k_{TP}k_{m}K_P(1+T_Ps)+s^2(1+sT_m)},$$(equ.closed_loop)


</br>

### Input-to-output stability analysis


To analyze the input-to-output stability of the closed-loop control system {eq}`equ.closed_loop`, we look at the roots (also called poles) of its characteristic equation, which can be factorized into the following
form

$${k_{TP}k_{m}K_P(1+T_Ps)+s^2(1+sT_m)}
=
{\left({\omega_{n}^{2}}+{2 \zeta }{\omega_{n}}s+{s^{2}}\right)(1+s \tau)}=0$$(equ.cha_equ)

The roots (also called poles) to the characteristics equation {eq}`equ.cha_equ` is 


$$
\begin{aligned}
s_1&=-\zeta \omega_{n}, + j \sqrt{1-\zeta^{2}} \omega_{n}\\
s_2&=-\zeta \omega_{n}, - j \sqrt{1-\zeta^{2}} \omega_{n}\\
s_3&=-1 / \tau
\end{aligned}
$$

Here,  $\omega_{n}$ and $\zeta$ are the natural frequency and damping
ratio for the complex roots $s_1$ and $s_2$, and $s_3$ is a real pole.
Please see my [Minimal notes on control basics](./control_basics.pdf) for explaination of how the values of $\omega_{n}$, $\zeta$ and $s_3$ effect the time-domain performance of the closed-loop control system.



The location of the above poles $(s_1, s_2, s_3)$ on s-plane depends on the of the open-loop gain 

$$\frac{k_{m} K_{P} k_{T P} T_{P}}{T_{m}}$$(equ.ol_gain)



The root locus (i.e., the trajectory of the above poles on $s-$plane)  can be drawn by taking different open loop gain value {eq}`equ.ol_gain`. Here, we derive into two cases:

-   If $T_{P}<T_{m}$, the root locus is shown. Because there is always a pole (root) living on the right-half s-plane regardless of the choice of $\frac{k_{m} K_{P} k_{T P} T_{P}}{T_{m}}$, the closed-loop control system {eq}`equ.closed_loop` thus is inherently unstable.

    :::{figure} ../lec19/control/p_control_rl_1.jpg
    ---
    width: 90%
    name: p_control_rl_1
    ---
    Root locus when
    $T_{P}<T_{m}$
    :::



-   If $T_{P}>T_{m}$, the root locus is shown as below. First, all poles can be located on the left half of s-plane, thus closed-loop control system {eq}`equ.closed_loop` is stable. Also,  as $T_{P}$
    increases, the absolute value of the real part of the two complex poles, ($s_1$ and $s_2$) tending towards the asymptotes increases too, and the     system has faster time response (please see my [Minimal notes on control basics](./control_basics.pdf) for explaination)


    :::{figure} ../lec19/control/p_control_rl_2.jpg
    ---
    width: 90%
    name: p_control_rl_2
    ---
    Root locus when
    $T_{P}>T_{m}$
    :::



</br>

### Disturbance-to-output performance



```{admonition} Final value theorem


If a continuous signal $f(t)$ has its Laplace transformation $F(s)$,
then the final value theorem states

$$\lim_{t\rightarrow\infty}f(t)=\lim_{s\rightarrow 0}sF(s)$$
```


The closed-loop disturbance-to-output transfer function is

$$\frac{\Theta_{m}(s)}{D(s)}=-\frac{\frac{R_a}{K_t}G(s)}{1+C(s)G(s)H(s)}=-\frac{\frac{R_a}{K_t}k_ms}{k_{TP}k_{m}K_P(1+T_Ps)+s^2(1+sT_m)}$$

If $\theta_r(t)=0$ (i.e., no input signal), based on the final value thoerem (see the above), we have


$$\lim_{t\rightarrow \infty} {\theta_{m}(t)}= \lim_{s\rightarrow 0} s{\Theta_{m}(s)}=\lim_{s\rightarrow 0} s\frac{\Theta_{m}(s)}{D(s)}{D(s)}$$
thus,

$$\lim_{t\rightarrow \infty} {\theta_{m}(t)}= 
    \begin{cases}
        0\qquad &\text{if}\quad D(t)\,\,\text{is constant signal}\\
        \frac{R_a}{K_t}\frac{1}{K_{P} k_{T P}} \qquad &\text{if}\quad D(t)\,\,\text{is ramp signal, such as $D(t)=vt$ }\\
    \end{cases}$$

Hence, the controller can mitigate the effect of disturbance, and
the mitigation is controlled by

$$K_{P} k_{T P}$$

which can be interpreted as the disturbance rejection factor for velocity (or
higher-order) disturbance. Increasing $K_{P}$ can help with reducing the
effect of $D$, but too high $K_P$ can lead to unacceptable oscillations
of the output, as implied from the root locus.




<!-- 
</br></br>
# Single-Joint Position and Velocity Feedback Control

To control the motor system in Fig.
[2](#fig:motor_model2){reference-type="ref"
reference="fig:motor_model2"}, we use both position and velocity
controller $$\begin{gathered}
C_{P}(s)=K_{P} \quad \text{and} \quad C_{V}(s)=K_{V} \frac{1+s T_{V}}{s} 
\end{gathered}$$ where $K_p$, $K_v$, and $T_v$ are the design variables.

The control diagram is shown below (Note that $k_{TV}$ and $k_{TP}$ in
the backward pass are fixed).


```{figure} ../lec19/control/pv_control.jpeg
---
width: 70%
name: pv_control
---
Control diagram of position and velocity
control
```

which can be further reduced into the following diagram:


```{figure} ../lec19/control/pv_control2.jpeg
---
width: 70%
name: pv_control2
---
Corresponding to Fig. [7](#fig.pv_control){reference-type="ref"
reference="fig.pv_control"}, the equivalent control
diagram
```

The transfer function of the forward path is

$$P(s)=C(s)G(s)=\frac{k_{m} K_{P} K_{V}\left(1+s T_{V}\right)}{s^{2}\left(1+s T_{m}\right)}$$

The transfer function of the backward path is

$$H(s)=k_{T P}\left(1+s \frac{k_{T V}}{K_{P} k_{T P}}\right) .$$

For simplicity, one can design 

$$T_m=T_v,$$ 

then that the poles of the
closed-loop system move on the root locus as a function of the loop gain
$k_{m} K_{V} k_{T V}$ is shown in the following figure:


```{figure} ../lec19/control/pv_control_rl_1.jpg
---
width: 50%
name: pv_control_rl_1
---
Root locus
```


By increasing the position gain $K_{P}$, it is possible to confine the
closed-loop poles into a region of the complex plane with large absolute
values of the real part.

The closed-loop input/output transfer function is

$$\frac{\Theta_{m}(s)}{\Theta_{r}(s)}=\frac{C(s)G(s)}{1+C(s)G(s)H(s)}=\frac{\frac{1}{k_{T P}}}{1+\frac{s k_{T V}}{K_{P} k_{T P}}+\frac{s^{2}}{k_{m} K_{P} k_{T P} K_{V}}}=\frac{\frac{1}{k_{T P}}}{1+\frac{2 \zeta s}{\omega_{n}}+\frac{s^{2}}{\omega_{n}^{2}}}$$

It can be recognized that, with a suitable choice of the gains, it is
possible to obtain any value of natural frequency $\omega_{n}$ and
damping ratio $\zeta$. Hence, if $\omega_{n}$ and $\zeta$ are given as
design requirements, the following relations is

$$K_{V} k_{T V}=\frac{2 \zeta \omega_{n}}{k_{m}}$$

$$K_{P} k_{T P} K_{V}=\frac{\omega_{n}^{2}}{k_{m}}$$

For given transducer constants $k_{T P}$ and $k_{T V}$, $K_{V}$ and
$K_{P}$ can be chosen. The closed-loop disturbance/output transfer
function is

$$\frac{\Theta_{m}(s)}{D(s)}=-\frac{\frac{R_a}{k_t}G(s)}{1+P(s)H(s)}=-\frac{\frac{R_a}{k_t}k_ms}{(1+sT_m)(s^2+k_mK_Vk_{TV}s+k_mK_PK_Vk_{TP})}$$

which shows that the disturbance rejection factor is

If $\Theta_r(t)=0$, based on the final value theory, we have

$$\lim_{t\rightarrow \infty} {\theta_{m}(t)}= \lim_{s\rightarrow 0} s{\Theta_{m}(s)}=\lim_{s\rightarrow 0} s\frac{\Theta_{m}(s)}{D(s)}{D(s)}$$

thus,

$$\lim_{t\rightarrow \infty} {\theta_{m}(t)}= 
    \begin{cases}
        0\qquad &\text{if}\quad D(t)\,\,\text{is constant signal}\\
        \frac{R_a}{k_t}\frac{1}{K_{P} k_{T P} K_{V}} \qquad &\text{if}\quad D(t)\,\,\text{is ramp signal, such as $D(t)=vt$ }\\
    \end{cases}$$

Hence, the controller can cancel the effect of constant disturbance, and
the quantity

$$K_{P} k_{T P} K_{V}$$



can be interpreted as the disturbance rejection factor for velocity (or
higher-order) disturbance. Increasing $K_{P}$ and $K_V$ can help with
reducing the effect of $D$. -->
