---
author:
- Wanxin Jin
date: Nov. 17, 2023
title: "Lecture 19: Overview of Robot Arm Control"
---

# Overview of Robot Arm Control

The control of a robot arm is to determine the time sequence of
control inputs (i.e., joint torques) to achieve a specified task. In many robotics tasks, the task is specified usually in the
operational space (such as end-effector motion and forces), whereas
control inputs (joint torque) is usually made in the joint space. There are two
control schemes: joint space control and operational space control. In
both schemes, the control diagram is closed-loops





```{figure} ./control/joint_control.jpg
---
width: 99%
name: joint_control222
---
Joint space control
```



The joint space control is shown in {numref}`joint_control222`. Here, inverse kinematics is
use to convert the specified end-effector motion
$\boldsymbol{x}_{d}$ to the desired joint motion
$\boldsymbol{q}_{d}$. Then, a joint space controller
 is designed to allow the actual joint value $\boldsymbol{q}$ to
track $\boldsymbol{q}_{d}$. As you can see, the controller   *directly*
regulates the joint tracking error $\boldsymbol{q}_{d}-\boldsymbol{q}_{}$, instead of operational space error $\boldsymbol{x}_{d}-\boldsymbol{x}_{e}$. Thus, the end-effector pose
$\boldsymbol{x}_{e}$ is controlled in an open-loop fashion (instead, $\boldsymbol{q}_{}$ is controlled in closed-loop way). This can lead to operational space control error if IK has some error.



```{figure} ./control/operational_space_control.jpg
---
width: 99%
name: operational_space_control222
---
Operational space control
```



The operational space control is shown in {numref}`operational_space_control222`. 
Here, the operational space motion $\boldsymbol{x}_{e}$ is directly fed back
to the controller. Thus, it addresses the drawbacks of joint space
control. However, the control algorithm  typically is more complex in design and it requires  measuring the operational space motion
$\boldsymbol{x}_{e}$, which is typically challenging.



</br> </br>


# Modeling Joint Motors

According to the previous lecture, the dynamics equation of a
robot arm only with joint input torque $\boldsymbol{\tau}$ is:

$$\boldsymbol{B}(\boldsymbol{q}) \ddot{\boldsymbol{q}}+\boldsymbol{C}(\boldsymbol{q}, \dot{\boldsymbol{q}}) \dot{\boldsymbol{q}}+\boldsymbol{F}_{v} \dot{\boldsymbol{q}}+\boldsymbol{g}(\boldsymbol{q})=\boldsymbol{\tau}$$


```{figure} ../lec18/dynamics/motori_kinematics.jpg
---
width: 70%
name: motori_kinematics
---
Transmission between  motor and
joint
```


## Transmissions between Motor and Joint

As shown in {numref}`motori_kinematics`, let $\boldsymbol{q}_{m}$ denote the vector of motor joint 
displacements (motor's rotor angle). The transmissions - assumed to be
rigid and with no backlash - between the motor and joint motion is

$$\label{equ.transmission_model}
    \boldsymbol{K}_{r} \boldsymbol{q}=\boldsymbol{q}_{m}$$

where $\boldsymbol{K}_{r}$ is a $(n \times n)$ diagonal matrix,
typically much greater than identity. Let $\boldsymbol{\tau}_{m}$
denotes the vector of the motor driving torques, based on principle
of virtual work, one has the following transmission between motor torque
and joint input torque:

$$\label{equ.transmission_model2}
\boldsymbol{\tau}_{m}=\boldsymbol{K}_{r}^{-1} \boldsymbol{\tau}$$

## Electric Motor Model

For a direct current (DC) motor, we have the following model:

$$\label{equ.dc_model}
    \begin{aligned}
 \boldsymbol{\tau}_m& =\boldsymbol{K}_{t} \boldsymbol{i}_{a} \\
\boldsymbol{v}_{a} & =\boldsymbol{R}_{a} \boldsymbol{i}_{a}+\boldsymbol{K}_{v} \dot{\boldsymbol{q}}_{m} \\
\boldsymbol{v}_{a} & =\boldsymbol{G}_{v} \boldsymbol{v}_{c}
\end{aligned}$$

Here, $\boldsymbol{K}_{t}$ is the torque constants; $\boldsymbol{i}_{a}$
is the armature currents; $\boldsymbol{v}_{a}$ is the vector of armature
voltages; $\boldsymbol{R}_{a}$ is the armature resistances;
$\boldsymbol{K}_{v}$ is the back EMF constants; $\boldsymbol{G}_{v}$ is
the amplifiers; and $\boldsymbol{v}_{c}$ is the voltages of the
servomotors.

Based on
([\[equ.transmission_model\]](#equ.transmission_model){reference-type="ref"
reference="equ.transmission_model"}-[\[equ.dc_model\]](#equ.dc_model){reference-type="ref"
reference="equ.dc_model"}), we have

$$\label{equ.voltage_control}
    \boldsymbol{\tau}=\boldsymbol{K}_{r} \boldsymbol{K}_{t} \boldsymbol{R}_{a}^{-1}\left(\boldsymbol{G}_{v} \boldsymbol{v}_{c}-\boldsymbol{K}_{v} \boldsymbol{K}_{r} \dot{\boldsymbol{q}}\right)$$

The above equation unveils a relationship between the applied servomotor
voltage $\boldsymbol{v}_c$, the generated joint torque
$\boldsymbol{\tau}$, and the joint velocity $\boldsymbol{\dot{q}}$. The
overall diagram modeling DC motor and manipulator is shown in Fig.
[4](#fig.voltage_control){reference-type="ref"
reference="fig.voltage_control"}.



```{figure} ./control/voltage_control.jpg
---
width: 70%
name: voltage_control
---
DC motor with
manipulator
```



# "Manipulator as an Integrator"

If the following assumptions hold for
([\[equ.voltage_control\]](#equ.voltage_control){reference-type="ref"
reference="equ.voltage_control"}):

-   the elements of matrix $\boldsymbol{K}_{r}$, characterizing the
    transmissions, are much greater than unity;

-   the elements of matrix $\boldsymbol{R}_{a}$ are very small, which is
    typical in the case of high-efficiency servomotors;

-   the values of the joint torques $\boldsymbol{\tau}$ required for the
    execution of the desired motions are not too large;

then

$$\boldsymbol{G}_{v} \boldsymbol{v}_{c} \approx \boldsymbol{K}_{v} \boldsymbol{K}_{r} \dot{\boldsymbol{q}}$$
and further

$$\boldsymbol{v}_{c} = \boldsymbol{G}_{v}^{-1} \boldsymbol{K}_{v} \boldsymbol{K}_{r} \dot{\boldsymbol{q}}$$

The above equation says that, under the above-stated assumptions, the
whole system, i.e., DC motor plus manipulator, can be considered as a
voltage-to-velocity system: the joint velocity of a manipulator of is
proportional to the voltage input to the servomotor. In other words, the
manipulator plus the drive system can be considered as an integrator!

The benefit of this system is that it is a decentralized control system
(each joint can be controlled independently of the others): the velocity
of the $i$-th joint depends only on the $i$-th control voltage, since
the matrix
$\boldsymbol{G}_{v}^{-1} \boldsymbol{K}_{v} \boldsymbol{K}_{r}$ is
diagonal.

# Torque-Controlled System (more general)

If the assumption in the above section does not hold, the whole system,
for example, when $\left(\boldsymbol{K}_{r}=\boldsymbol{I}\right)$, we
do not have such a nice property of \"manipulator as an integrator\". In
those cases, we still have the following relation between the computed
joint torque and armature current

$$\label{equ.current_torque}
    \boldsymbol{K}_r^{-1}\boldsymbol{\tau}= \boldsymbol{\tau}_m =\boldsymbol{K}_{t} \boldsymbol{i}_{a}$$

Thus, typically, we have a local feedback control system inside the
motor to regulate the armature current $\boldsymbol{i}_a$. This local
control system will establish the following relationship between the
voltage of the servomotor and the current:

$$\label{equ.voltage_current}
    \boldsymbol{i}_{a}=\boldsymbol{G}_{i} \boldsymbol{v}_{c}$$

Therefore, based on
([\[equ.current_torque\]](#equ.current_torque){reference-type="ref"
reference="equ.current_torque"}) and
([\[equ.voltage_control\]](#equ.voltage_control){reference-type="ref"
reference="equ.voltage_control"}), we have

$$
    \boldsymbol{\tau}=\boldsymbol{K}_r\boldsymbol{K}_{t}\boldsymbol{G}_{i} \boldsymbol{v}_{c}$$

The above equation says that, in the general case, the whole system,
i.e., DC motor plus manipulator, can be considered as a
voltage-to-torque system: you can directly regulate the joint torque of
a manipulator by regulating the voltage to the servomotor.
