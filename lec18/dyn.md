---
author:
- Wanxin Jin
date: Oct. 24, 2023
title: "Lecture 18: Dynamics (Lagrange Formulation)"
---


(chapter-dyn)=
# Dynamics (Lagrangian formulation) 
Compared to the previous kinematics concerning how a
robot moves, robot dynamics concerns about why
a robot moves. Dynamics equation establishes a relationship between the
forces/torques and their acceleration/velocities/positions.


Lagrange method provides a systematic way to derive dynamics
equation of a mechanical system. Suppose that the configuration space of a $n$-DOF  mechanical
system is  described by a set of
*independent* variables $q_{i}, i=1, \ldots, n$, named generalized
coordinates, the Lagrangian of the system is defined as 

$$\mathcal{L}=\mathcal{T}-\mathcal{U}$$

where $\mathcal{T}$ and $\mathcal{U}$ are the kinetic energy and
potential energy of the system, respectively. The dynamics
from the Lagrangian is

$$\frac{d}{d t} \frac{\partial \mathcal{L}}{\partial \dot{q}_{i}}-\frac{\partial \mathcal{L}}{\partial q_{i}}=\xi_{i} \quad i=1, \ldots, n$$(equ.lf2)

where $\xi_{i}$ is the generalized force associated with $q_{i}$ (the
generalized force $\xi_i$ is dual to $q_i$ in the sense that
$\xi_i*q_i$ generate power). In compact form, the above equation can be
written as

$$\frac{d}{d t}\left(\frac{\partial \mathcal{L}}{\partial \dot{\boldsymbol{q}}}\right)^{T}-\left(\frac{\partial \mathcal{L}}{\partial \boldsymbol{q}}\right)^{T}=\boldsymbol{\xi}$$(equ.lf)

For an open-chain robot arm, the generalized
coordinates are joint variables $\boldsymbol{q}$. The generalized forces
are the net torque/force at each joint.


#  Kinetic Energy of a Robot Arm


Consider a  $n$-DoF robot arm. The kinetic energy is due
to the motion of each link.  The total kinetic energy is given by 

$$\mathcal{T}=\sum_{i=1}^{n}\mathcal{T}_{\ell_{i}}$$

where $\mathcal{T}_{\ell_{i}}$ is the kinetic energy of Link $i$.

```{figure} ./dynamics/linki_kinematics.jpg
---
width: 50%
name: linki_kinematics1
---
Motion of Link $i$ 
```



As shown in  {numref}`linki_kinematics1`, the kinetic energy contribution of Link $i$ is given by

$$
            \mathcal{T}_{\ell_{i}}=\frac{1}{2} \int_{V_{\ell_{i}}} \dot{\boldsymbol{p}}_{i}^{* T} \dot{\boldsymbol{p}}_{i}^{*} \rho d V$$ (equ.ke_total)

where  $\rho$ is the density of the elementary particle of volume
$d V$; $\dot{\boldsymbol{p}}_{i}^{*}$ denotes the linear velocity vector of the elementary particle; and  $V_{\ell_{i}}$ is the volume of Link $i$. Here, the position
 $\boldsymbol{p}_{i}^{*}$ of the elementary particle is expressed in the base frame. 

The
position  $\boldsymbol{p}_{l_{i}}$ of the link center of mass (COM) defined as  

$$\boldsymbol{p}_{\ell_{i}}=\frac{1}{m_{\ell_{i}}} \int_{V_{\ell_{i}}} \boldsymbol{p}_{i}^{*} \rho d V$$
where $m_{\ell_{i}}$ is the  mass of link $i$. We also define

$$\boldsymbol{r}_{i}=\left[\begin{array}{lll}
r_{i x} & r_{i y} & r_{i z}
\end{array}\right]^{T}=\boldsymbol{p}_{i}^{*}-\boldsymbol{p}_{\ell_{i}}$$




 As a result, the velocity of an elementary particle of link $i$ 
 can be expressed as

$$\label{equ.2}
    \begin{aligned}
\dot{\boldsymbol{p}}_{i}^{*} & =\dot{\boldsymbol{p}}_{\ell_{i}}+\boldsymbol{\omega}_{i} \times \boldsymbol{r}_{i}  =\dot{\boldsymbol{p}}_{\ell_{i}}+\boldsymbol{S}\left(\boldsymbol{\omega}_{i}\right) \boldsymbol{r}_{i}
\end{aligned}$$ (equ.ke_vel)

where $\dot{\boldsymbol{p}}_{\ell_{i}}$ is the linear velocity of COM and $\boldsymbol{\omega}_{i}$ is the angular velocity of
the link. 



By substituting {eq}`equ.ke_vel` into
{eq}`equ.ke_total`, it leads to that the total kinetic energy of link $i$, $ \mathcal{T}_{\ell_{i}}$, is the sumation of the following three terms

**Translational term**

$$\frac{1}{2} \int_{V_{\ell_{i}}} \dot{\boldsymbol{p}}_{\ell_{i}}^{T} \dot{\boldsymbol{p}}_{\ell_{i}} \rho d V=\frac{1}{2} m_{\ell_{i}} \dot{\boldsymbol{p}}_{\ell_{i}}^{T} \dot{\boldsymbol{p}}_{\ell_{i}}$$(eq.term_trans)

**Cross term**

$$2\left(\frac{1}{2} \int_{V_{\ell_{i}}} \dot{\boldsymbol{p}}_{\ell_{i}}^{T} \boldsymbol{S}\left(\boldsymbol{\omega}_{i}\right) \boldsymbol{r}_{i} \rho d V\right)=2\left(\frac{1}{2} \dot{\boldsymbol{p}}_{\ell_{i}}^{T} \boldsymbol{S}\left(\boldsymbol{\omega}_{i}\right) \int_{V_{\ell_{i}}}\left(\boldsymbol{p}_{i}^{*}-\boldsymbol{p}_{\ell_{i}}\right) \rho d V\right)=0$$(eq.term_cross)


**Rotational term**



$$\frac{1}{2} \int_{V_{\ell_{i}}} \boldsymbol{r}_{i}^{T} \boldsymbol{S}^{T}\left(\boldsymbol{\omega}_{i}\right) \boldsymbol{S}\left(\boldsymbol{\omega}_{i}\right) \boldsymbol{r}_{i} \rho d V=\frac{1}{2} \boldsymbol{\omega}_{i}^{T}\left(\int_{V_{\ell_{i}}} \boldsymbol{S}^{T}\left(\boldsymbol{r}_{i}\right) \boldsymbol{S}\left(\boldsymbol{r}_{i}\right) \rho d V\right) \boldsymbol{\omega}_{i}=\frac{1}{2} \boldsymbol{\omega}_{i}^{T} \boldsymbol{I}_{\ell_{i}} \boldsymbol{\omega}_{i}$$(eq.term_rot)

Recall that 

$$\boldsymbol{S}\left(\boldsymbol{r}_{i}\right)=\left[\begin{array}{ccc}
0 & -r_{i z} & r_{i y} \\
r_{i z} & 0 & -r_{i x} \\
-r_{i y} & r_{i x} & 0
\end{array}\right]$$

Then,

$$\begin{aligned}
\boldsymbol{I}_{\ell_{i}} & =\left[\begin{array}{ccc}
\int\left(r_{i y}^{2}+r_{i z}^{2}\right) \rho d V & -\int r_{i x} r_{i y} \rho d V & -\int r_{i x} r_{i z} \rho d V \\
* & \int\left(r_{i x}^{2}+r_{i z}^{2}\right) \rho d V & -\int r_{i y} r_{i z} \rho d V \\
* & * & \int\left(r_{i x}^{2}+r_{i y}^{2}\right) \rho d V
\end{array}\right] 
\end{aligned}$$

is the inertia tensor relative to the COM of Link $i$. Since $\boldsymbol{r}_i$ is expressed in the base frame, this inertia tensor is thus also
expressed in the base frame, thus is configuration-dependent.

Let's consider $\boldsymbol{r}_i^i$ be expressed in the frame of link $i$

$$\boldsymbol{r}_{i}=\boldsymbol{R}_{i} \boldsymbol{r}_{i}^{i}$$

where $\boldsymbol{R}_{i}$ is the rotation matrix from Link $i$ frame to
the base frame. Then, from {eq}`eq.term_rot`,  we have 

$$
\begin{aligned}
&\frac{1}{2} \boldsymbol{\omega}_{i}^{T}\left(\int_{V_{\ell_{i}}} \boldsymbol{S}^{T}\left(\boldsymbol{R}_{i} \boldsymbol{r}_{i}^{i}\right) \boldsymbol{S}\left(\boldsymbol{R}_{i} \boldsymbol{r}_{i}^{i}\right) \rho d V\right) \boldsymbol{\omega}_{i}\\
=&
\frac{1}{2} \boldsymbol{\omega}_{i}^{T}\left(\int_{V_{\ell_{i}}} 
\boldsymbol{R}_{i} \boldsymbol{S}^{T}\left(\boldsymbol{r}_{i}^{i}\right) \boldsymbol{R}_{i} ^T \boldsymbol{R}_{i} \boldsymbol{S}\left( \boldsymbol{r}_{i}^{i}\right) \boldsymbol{R}_{i}^T
\rho d V\right) \boldsymbol{\omega}_{i}\\
=&
\frac{1}{2} \boldsymbol{\omega}_{i}^{T} \boldsymbol{R}_{i} \left(\int_{V_{\ell_{i}}} 
\boldsymbol{S}^{T}\left(\boldsymbol{r}_{i}^{i}\right)   \boldsymbol{S}\left( \boldsymbol{r}_{i}^{i}\right) 
\rho d V\right) \boldsymbol{R}_{i}^T\boldsymbol{\omega}_{i}\\
=&
\frac{1}{2} \boldsymbol{\omega}_{i}^{T} \boldsymbol{R}_{i} \boldsymbol{I}_{\ell_{i}}^{i} \boldsymbol{R}_{i}^{T}\boldsymbol{\omega}_{i}
\end{aligned}
$$

Here, 

$$ 
\boldsymbol{I}_{\ell_{i}}^{i} =\left(\int_{V_{\ell_{i}}} 
\boldsymbol{S}^{T}\left(\boldsymbol{r}_{i}^{i}\right)   \boldsymbol{S}\left( \boldsymbol{r}_{i}^{i}\right) 
\rho d V\right)
$$

 is expressed in the body frame (link $i$ frame), and thus is configuration-independent.


By summing the translational and rotational terms, the total kinetic energy in {eq}`equ.ke_total` of link $i$ is

$$\mathcal{T}_{\ell_{i}}=\frac{1}{2} m_{\ell_{i}} \dot{\boldsymbol{p}}_{\ell_{i}}^{T} \dot{\boldsymbol{p}}_{\ell_{i}}+\frac{1}{2} \boldsymbol{\omega}_{i}^{T} \boldsymbol{R}_{i} \boldsymbol{I}_{\ell_{i}}^{i} \boldsymbol{R}_{i}^{T} \boldsymbol{\omega}_{i}$$(equ.ke_total2)

Now, let's find out the linear and angular velocities of Link $i$ using Jacobian! Using our previous method to find Jacobian up to link $i$, we have


$$\begin{aligned}
\dot{\boldsymbol{p}}_{\ell_{i}} & =\boldsymbol{\jmath}_{P 1}^{\left(\ell_{i}\right)} \dot{q}_{1}+\ldots+\boldsymbol{J}_{P i}^{\left(\ell_{i}\right)} \dot{q}_{i}=\boldsymbol{J}_{P}^{\left(\ell_{i}\right)} \dot{\boldsymbol{q}} \\
\boldsymbol{\omega}_{i} & =\boldsymbol{\jmath}_{O 1}^{\left(\ell_{i}\right)} \dot{q}_{1}+\ldots+\boldsymbol{J}_{O i}^{\left(\ell_{i}\right)} \dot{q}_{i}=\boldsymbol{J}_{O}^{\left(\ell_{i}\right)} \dot{\boldsymbol{q}},
\end{aligned}$$

where (think about why the columns after $i$ is zeros?)

$$\begin{aligned}
\boldsymbol{J}_{P}^{\left(\ell_{i}\right)}  =\left[\begin{array}{llllll}
\boldsymbol{J}_{P 1}^{\left(\ell_{i}\right)} & \ldots & \boldsymbol{J}_{P i}^{\left(\ell_{i}\right)} & \mathbf{0} & \ldots & \mathbf{0}
\end{array}\right]  \qquad
\boldsymbol{J}_{O}^{\left(\ell_{i}\right)}  =\left[\begin{array}{llllll}
\boldsymbol{J}_{O 1}^{\left(\ell_{i}\right)} & \ldots & \boldsymbol{J}_{O i}^{\left(\ell_{i}\right)} & \mathbf{0} & \ldots & \mathbf{0}
\end{array}\right] ;
\end{aligned}$$(equ.com_jac)

where

$$\begin{gathered}
\boldsymbol{J}_{P j}^{\left(\ell_{i}\right)}= \begin{cases}\boldsymbol{z}_{j-1} & \text { for a prismatic joint } \\
\boldsymbol{z}_{j-1} \times\left(\boldsymbol{p}_{\ell_{i}}-\boldsymbol{p}_{j-1}\right) &
\text { for a revolute joint }\end{cases} \qquad
\boldsymbol{\jmath}_{O j}^{\left(\ell_{i}\right)}= \begin{cases}\mathbf{0} & \text { for a prismatic joint } \\
\boldsymbol{z}_{j-1} & \text { for a revolute joint. }\end{cases}
\end{gathered}$$

where $\boldsymbol{p}_{j-1}$ is the position of the origin of Frame
$j-1$ and $\boldsymbol{z}_{j-1}$ is the unit vector of axis $z$ of Frame
$j-1$.

It follows that the kinetic energy of Link $i$ in {eq}`equ.ke_total2` can be written as

$$\mathcal{T}_{\ell_{i}}=\frac{1}{2} m_{\ell_{i}} \dot{\boldsymbol{q}}^{T} \boldsymbol{J}_{P}^{\left(\ell_{i}\right) T} \boldsymbol{J}_{P}^{\left(\ell_{i}\right)} \dot{\boldsymbol{q}}+\frac{1}{2} \dot{\boldsymbol{q}}^{T} \boldsymbol{J}_{O}^{\left(\ell_{i}\right) T} \boldsymbol{R}_{i} \boldsymbol{I}_{\ell_{i}}^{i} \boldsymbol{R}_{i}^{T} \boldsymbol{J}_{O}^{\left(\ell_{i}\right)} \dot{\boldsymbol{q}}$$
<!-- 
## Kinetic Energy of Motor $i$


```{figure} ./dynamics/motori_kinematics.jpg
---
width: 50%
name: motori_kinematics
---
Kinematic description of Motor
$i$
```

The motor of Joint $i$ is assumed to be located on Link $i-1$. The
kinetic energy contribution of the motor of Joint $i$ can be computed in
a formally analogous way to that of the link. Consider the typical case
of rotary electric motors (that can actuate both revolute and prismatic
joints by means of suitable transmissions). It can be assumed that the
contribution of the fixed part (stator) is included in that of the link
on which such motor is located, and thus the sole contribution of the
rotor is to be computed.

The kinetic energy of Rotor $i$ can be written as

$$\mathcal{T}_{m_{i}}=\frac{1}{2} m_{m_{i}} \dot{\boldsymbol{p}}_{m_{i}}^{T} \dot{\boldsymbol{p}}_{m_{i}}+\frac{1}{2} \boldsymbol{\omega}_{m_{i}}^{T} \boldsymbol{I}_{m_{i}} \boldsymbol{\omega}_{m_{i}}$$

where $m_{m_{i}}$ is the mass of the rotor,
$\dot{\boldsymbol{p}}_{m_{i}}$ denotes the linear velocity of the centre
of mass of the rotor, $\boldsymbol{I}_{m_{i}}$ is the inertia tensor of
the rotor relative to its centre of mass, and
$\boldsymbol{\omega}_{m_{i}}$ denotes the angular velocity of the rotor.

Let $\vartheta_{m_{i}}$ denote the angular position of the rotor. On the
assumption of a rigid transmission, one has

$$k_{r i} \dot{q}_{i}=\dot{\vartheta}_{m_{i}}$$

where $k_{r i}$ is gear reduction ratio. For a prismatic joint, gear
reduction ratio is a dimensional quantity.

The total angular velocity of the rotor is

$$\boldsymbol{\omega}_{m_{i}}=\boldsymbol{\omega}_{i-1}+k_{r i} \dot{q}_{i} \boldsymbol{z}_{m_{i}}$$

where $\boldsymbol{\omega}_{i-1}$ is the angular velocity of Link $i-1$
on which the motor is located, and $\boldsymbol{z}_{m_{i}}$ denotes the
unit vector along the rotor axis.

To express the rotor kinetic energy as a function of the joint
variables, it is worth expressing the linear velocity of the rotor
centre of mass as

$$\dot{\boldsymbol{p}}_{m_{i}}=\boldsymbol{J}_{P}^{\left(m_{i}\right)} \dot{\boldsymbol{q}}$$

The Jacobian to compute is then

$$\boldsymbol{J}_{P}^{\left(m_{i}\right)}=\left[\begin{array}{llllll}
\boldsymbol{J}_{P 1}^{\left(m_{i}\right)} & \ldots & \boldsymbol{J}_{P, i-1}^{\left(m_{i}\right)} & \mathbf{0} & \ldots & \mathbf{0}
\end{array}\right]$$

whose columns are given by

$$\boldsymbol{J}_{P j}^{\left(m_{i}\right)}= \begin{cases}\boldsymbol{z}_{j-1} & \text { for a prismatic joint } \\ \boldsymbol{z}_{j-1} \times\left(\boldsymbol{p}_{m_{i}}-\boldsymbol{p}_{j-1}\right) & \text { for a revolute joint }\end{cases}$$

where $\boldsymbol{p}_{j-1}$ is the position vector of the origin of
Frame $j-1$.

The angular velocity expressed as a function of the joint variables is
$$\boldsymbol{\omega}_{m_{i}}=\boldsymbol{J}_{O}^{\left(m_{i}\right)} \dot{\boldsymbol{q}}$$

The Jacobian to compute is then

$$\boldsymbol{J}_{O}^{\left(m_{i}\right)}=\left[\begin{array}{llllll}
\boldsymbol{J}_{O 1}^{\left(m_{i}\right)} & \ldots & \boldsymbol{J}_{O, i}^{\left(m_{i}\right)} & \mathbf{0} & \ldots & \mathbf{0}
\end{array}\right]$$

whose columns are

$$\boldsymbol{\jmath}_{O j}^{\left(m_{i}\right)}= \begin{cases}\boldsymbol{J}_{O j}^{\left(\ell_{i}\right)} & j=1, \ldots, i-1 \\ k_{r i} \boldsymbol{z}_{m_{i}} & j=i .\end{cases}$$

Hence, the kinetic energy of Rotor $i$ can be written as

$$\mathcal{T}_{m_{i}}=\frac{1}{2} m_{m_{i}} \dot{\boldsymbol{q}}^{T} \boldsymbol{J}_{P}^{\left(m_{i}\right) T} \boldsymbol{J}_{P}^{\left(m_{i}\right)} \dot{\boldsymbol{q}}+\frac{1}{2} \dot{\boldsymbol{q}}^{T} \boldsymbol{J}_{O}^{\left(m_{i}\right) T} \boldsymbol{R}_{m_{i}} \boldsymbol{I}_{m_{i}}^{m_{i}} \boldsymbol{R}_{m_{i}}^{T} \boldsymbol{J}_{O}^{\left(m_{i}\right)} \dot{\boldsymbol{q}}$$

## Total Kinetic Energy -->

```{important}
Finally, by summing the kinetic energies of all Links, the total
kinetic energy of a robot arm is 

$$\mathcal{T}=\frac{1}{2} \dot{\boldsymbol{q}}^{T} \boldsymbol{B}(\boldsymbol{q}) \dot{\boldsymbol{q}}=\frac{1}{2} \sum_{i=1}^{n} \sum_{j=1}^{n} b_{i j}(\boldsymbol{q}) \dot{q}_{i} \dot{q}_{j}$$(equ.ke_total_final)

where

$$\begin{aligned}
\boldsymbol{B}(\boldsymbol{q})=\sum_{i=1}^{n}\left(m_{\ell_{i}}\right. & \boldsymbol{J}_{P}^{\left(\ell_{i}\right) T} \boldsymbol{J}_{P}^{\left(\ell_{i}\right)}+\boldsymbol{J}_{O}^{\left(\ell_{i}\right) T} \boldsymbol{R}_{i} \boldsymbol{I}_{\ell_{i}}^{i} \boldsymbol{R}_{i}^{T} \boldsymbol{J}_{O}^{\left(\ell_{i}\right)}\left.\right)
\end{aligned}$$(equ.bmatrix222)

is the $(n \times n)$ inertia matrix which is symmetric, positive
definite, and configuration - dependent.
```

</br></br></br>

# Potential Energy of a Robot Arm

The potential energy of a robot arm is given by the sum of the contributions of each
link:

$$\mathcal{U}=\sum_{i=1}^{n}\mathcal{U}_{\ell_{i}}$$

The potential energy of Link $i$ is

$$\mathcal{U}_{\ell_{i}}=-\int_{V_{\ell_{i}}} \boldsymbol{g}^{T} \boldsymbol{p}_{i}^{*} \rho d V=-m_{\ell_{i}} \boldsymbol{g}^{T} \boldsymbol{p}_{\ell_{i}}$$

where $\boldsymbol{g}$ is the gravity acceleration vector in the
base frame (e.g., $\boldsymbol{g}=$
$\left[\begin{array}{lll}0 & 0 & -g\end{array}\right]^{T}$ if $z$ is the
vertical axis), and $\boldsymbol{p}_{\ell_{i}}$ the center of mass of
Link $i$. 


```{important}
The total potential energy of the robot arm is 

$$\mathcal{U}=-\sum_{i=1}^{n}m_{\ell_{i}} \boldsymbol{g}^{T} \boldsymbol{p}_{\ell_{i}}$$(equ.pe_total_final)

which shows that potential energy is only a function of  the joint
variable ${\boldsymbol{q}}$ because 
$\boldsymbol{p}_{\ell_{i}}$ is a function
of $\boldsymbol{q}$,
```

</br></br></br>

# Dynamics Equation (Different Forms)

### Vector Form
Having computed the total kinetic energy $\mathcal{T}(\boldsymbol{q}, \dot{\boldsymbol{q}})$ in {eq}`equ.ke_total_final` and potential energy $\mathcal{U}(\boldsymbol{q})$ in {eq}`equ.pe_total_final` of the robot arm,
the Lagrangian of the system is

$$\mathcal{L}(\boldsymbol{q}, \dot{\boldsymbol{q}})=\mathcal{T}(\boldsymbol{q}, \dot{\boldsymbol{q}})-\mathcal{U}(\boldsymbol{q})$$


```{important}
By applying the vector form of Lagrange formulation {eq}`equ.lf`,
we have the vector form of the dynamics equation:

$$\boldsymbol{B}(\boldsymbol{q}) \ddot{\boldsymbol{q}}+\boldsymbol{n}(\boldsymbol{q}, \dot{\boldsymbol{q}})=\boldsymbol{\xi}$$(equ.arm_dyn)

where

$$\boldsymbol{n}(\boldsymbol{q}, \dot{\boldsymbol{q}})=\dot{\boldsymbol{B}}(\boldsymbol{q}) \dot{\boldsymbol{q}}-\frac{1}{2}\left(\frac{\partial}{\partial \boldsymbol{q}}\left(\dot{\boldsymbol{q}}^{T} \boldsymbol{B}(\boldsymbol{q}) \dot{\boldsymbol{q}}\right)\right)^{T}+\left(\frac{\partial \mathcal{U}(\boldsymbol{q})}{\partial \boldsymbol{q}}\right)^{T}$$
```


### Detailed Form

Below, let's also apply the detailed form of Lagrange formulation {eq}`equ.lf2`. Notice that $\mathcal{U}$ does not depend on
$\dot{\boldsymbol{q}}$ and

$$\begin{aligned}
\frac{d}{d t}\left(\frac{\partial \mathcal{L}}{\partial \dot{q}_{i}}\right)=\frac{d}{d t}\left(\frac{\partial \mathcal{T}}{\partial \dot{q}_{i}}\right) & =\sum_{j=1}^{n} b_{i j}(\boldsymbol{q}) \ddot{q}_{j}+\sum_{j=1}^{n} \frac{d b_{i j}(\boldsymbol{q})}{d t} \dot{q}_{j} =\sum_{j=1}^{n} b_{i j}(\boldsymbol{q}) \ddot{q}_{j}+\sum_{j=1}^{n} \sum_{k=1}^{n} \frac{\partial b_{i j}(\boldsymbol{q})}{\partial q_{k}} \dot{q}_{k} \dot{q}_{j}
\end{aligned}$$

and

$$\frac{\partial \mathcal{T}}{\partial q_{i}}=\frac{1}{2} \sum_{j=1}^{n} \sum_{k=1}^{n} \frac{\partial b_{j k}(\boldsymbol{q})}{\partial q_{i}} \dot{q}_{k} \dot{q}_{j}$$

Further,

$$\begin{aligned}
\frac{\partial \mathcal{U}}{\partial q_{i}} & =-\sum_{j=1}^{n}m_{\ell_{j}} \boldsymbol{g}^{T} \frac{\partial \boldsymbol{p}_{\ell_{j}}}{\partial q_{i}} =-\sum_{j=1}^{n}m_{\ell_{j}} \boldsymbol{g}^{T} \boldsymbol{J}_{P i}^{\left(\ell_{j}\right)}(\boldsymbol{q})=g_{i}(\boldsymbol{q})
\end{aligned}$$

```{important}
By applying the detailed form of Lagrange formulation {eq}`equ.lf2`, the detailed form of the dynamics equation is 

$$\sum_{j=1}^{n} b_{i j}(\boldsymbol{q}) \ddot{q}_{j}+\sum_{j=1}^{n} \sum_{k=1}^{n} h_{i j k}(\boldsymbol{q}) \dot{q}_{k} \dot{q}_{j}+g_{i}(\boldsymbol{q})=\xi_{i} \quad i=1, \ldots, n .$$(equ.arm_dyn2)


where

$$h_{i j k}=\frac{\partial b_{i j}}{\partial q_{k}}-\frac{1}{2} \frac{\partial b_{j k}}{\partial q_{i}}$$(equ.hmatrix)

and 

$$
g_{i}(\boldsymbol{q})   =-\sum_{j=1}^{n}m_{\ell_{j}} \boldsymbol{g}^{T} \boldsymbol{J}_{P i}^{\left(\ell_{j}\right)}(\boldsymbol{q})
$$(equ.gmatrix)

```


A physical interpretation to each term in  {eq}`equ.arm_dyn2` 

-   Acceleration term $\sum_{j=1}^{n} b_{i j}(\boldsymbol{q}) \ddot{q}_{j}$: the diagonal coefficient $b_{i i}$ represents the
    moment of inertia at Joint $i$ axis, and off-diagonal coefficient
    $b_{i j}$ accounts for the coupling effect of acceleration of Joint $i$ on
    Joint $j$.

-   For the quadratic velocity term $\sum_{j=1}^{n} \sum_{k=1}^{n} h_{i j k}(\boldsymbol{q}) \dot{q}_{k} \dot{q}_{j}$: the term
    $h_{i j j} \dot{q}_{j}^{2}$ is the centrifugal effect to
    Joint $i$ by  Joint $j$. The term
    $h_{i j k} \dot{q}_{j} \dot{q}_{k}$ represents the Coriolis effect
     on Joint $i$ by Joints $j$ and $k$.

-   For the configuration-dependent term $g_{i}(\boldsymbol{q})$: the term $g_{i}$ represents
    the moment generated at Joint $i$ axis  by the gravity.


### Christoffel-Symbols Form


In may textbooks/papers, there is another commonly-used form of dynamics equation derived from {eq}`equ.arm_dyn2`. We introduce below.  The quadratic velocity term in  {eq}`equ.arm_dyn2` has


 $$\begin{aligned}
    \sum_{j=1}^{n} \sum_{k=1}^{n}h_{ijk}\dot{q}_{k} \dot{q}_{j}&=
\sum_{j=1}^{n} \sum_{k=1}^{n}\Big(\frac{\partial b_{i j}}{\partial q_{k}}-\frac{1}{2} \frac{\partial b_{j k}}{\partial q_{i}}\Big)\dot{q}_{k} \dot{q}_{j}\\
&=\sum_{j=1}^{n} \sum_{k=1}^{n}\Big(\frac{1}{2}\frac{\partial b_{i j}}{\partial q_{k}}+\frac{1}{2}\frac{\partial b_{i j}}{\partial q_{k}}-\frac{1}{2} \frac{\partial b_{j k}}{\partial q_{i}}\Big)\dot{q}_{k} \dot{q}_{j}\\
&=\sum_{j=1}^{n} \sum_{k=1}^{n}\underbrace{\Big(\frac{1}{2}\frac{\partial b_{i j}}{\partial q_{k}}+\frac{1}{2}\frac{\partial b_{i k}}{\partial q_{j}}-\frac{1}{2} \frac{\partial b_{j k}}{\partial q_{i}}\Big)}_{c_{ijk}}\dot{q}_{k} \dot{q}_{j}
\end{aligned}$$

Thus, we define

$$c_{ijk}=\frac{1}{2}\Big(\frac{\partial b_{i j}}{\partial q_{k}}+\frac{\partial b_{i k}}{\partial q_{j}}- \frac{\partial b_{j k}}{\partial q_{i}}\Big)$$

which is called _Christoffel-symbols_.
Then, the dynamics equation in {eq}`equ.arm_dyn2` becomes

$$
\begin{aligned}
\sum_{j=1}^{n} b_{i j}(\boldsymbol{q}) \ddot{q}_{j}+\sum_{j=1}^{n} \sum_{k=1}^{n} h_{i j k}(\boldsymbol{q}) \dot{q}_{k} \dot{q}_{j}+g_{i}(\boldsymbol{q})&=
\sum_{j=1}^{n} b_{i j}(\boldsymbol{q}) \ddot{q}_{j}+\sum_{j=1}^{n} \sum_{k=1}^{n} c_{i j k}(\boldsymbol{q}) \dot{q}_{k} \dot{q}_{j}+g_{i}(\boldsymbol{q})\\
&=\sum_{j=1}^{n} b_{i j}(\boldsymbol{q}) \ddot{q}_{j}+\sum_{j=1}^{n} \Big(\sum_{k=1}^{n}c_{ijk}\dot{q}_{k}\Big) \dot{q}_{j}+g_{i}(\boldsymbol{q})\\
&=\sum_{j=1}^{n} b_{i j}(\boldsymbol{q}) \ddot{q}_{j}+\sum_{j=1}^{n} C_{ij} \dot{q}_{j}+g_{i}(\boldsymbol{q})\\
&=\xi_{i}
\quad i=1, \ldots, n .
\end{aligned}
$$(equ.arm_dyn3)

with

$$C_{ij}=\sum_{k=1}^{n}c_{ijk}\dot{q}_{k}$$

```{important}
By writing  {eq}`equ.arm_dyn3` into a compact vector form, we have a new form of dynamics equation

$$\boldsymbol{B}(\boldsymbol{q}) \ddot{\boldsymbol{q}}+\boldsymbol{C}(\boldsymbol{q}, \dot{\boldsymbol{q}}) \dot{\boldsymbol{q}}+\boldsymbol{g}(\boldsymbol{q})={\boldsymbol{\xi}}$$(equ.arm_dyn4)

where $\boldsymbol{C}$ is a  $(n \times n)$ matrix such that its
elements $C_{i j}$ is calculated by

$$C_{ij}=\sum_{k=1}^{n}c_{ijk}\dot{q}_{k}$$ (equ.cmatrix)

with 

$$c_{ijk}=\frac{1}{2}\Big(\frac{\partial b_{i j}}{\partial q_{k}}+\frac{\partial b_{i k}}{\partial q_{j}}- \frac{\partial b_{j k}}{\partial q_{i}}\Big)$$(equ.cmatrix_item)


```


## Generalized forces/torques



The generalized force $\boldsymbol{\xi}$ applied on the right side of {eq}`equ.arm_dyn` or {eq}`equ.arm_dyn2` includes

$$\boldsymbol{\xi}=\underbrace{\boldsymbol{\tau}}_{\text{motor torque}}-\underbrace{\boldsymbol{F}_{v} \dot{\boldsymbol{q}}}_{\text{viscous friction torques}}-\underbrace{\boldsymbol{F}_{s} \operatorname{sgn}(\dot{\boldsymbol{q}})}_{\text{Coulomb friction torques}}-\underbrace{\boldsymbol{J}^{T}(\boldsymbol{q}) \boldsymbol{h}_{e}}_{\text{torques  by  contact forces.}}$$

where $\boldsymbol{F}_{v}$ denotes the $(n \times n)$ diagonal matrix of
viscous friction coefficients; $\boldsymbol{F}_{s}$ is an $(n \times n)$
diagonal matrix and $\operatorname{sgn}(\dot{\boldsymbol{q}})$ denotes
the $(n \times 1)$ vector whose components are given by the sign
functions of the single joint velocities; and $\boldsymbol{h}_{e}$
denotes the vector of force and moment exerted by the end-effector on
the environment.




</br></br>

## Skew-symmetry of $\dot{\boldsymbol{B}}-2\boldsymbol{C}$

The idea: the total time derivative of kinetic energy of a robot arm equals the power generated by all the forces/torques
including the gravity.

The time derivative of the kinetic energy: 

$$\label{equ.diff_kinenergy}
  \frac{d \mathcal{T}}{dt }=\frac{1}{2}\frac{d}{dt}\left(
    \dot{\boldsymbol{q}}^{T} \boldsymbol{B}(\boldsymbol{q})\dot{\boldsymbol{q}}
    \right)=\dot{\boldsymbol{q}}^{T}\boldsymbol{B}(\boldsymbol{q})\ddot{\boldsymbol{q}}+\frac{1}{2}\left(
    \dot{\boldsymbol{q}}^{T} \dot{\boldsymbol{B}}(\boldsymbol{q})\dot{\boldsymbol{q}}
    \right)$$(equ.dke_dt)

The power generated by all (generalized) external forces including
gravity: 

$$\label{equ.power_forces}
\dot{\boldsymbol{q}}^{T}(-\boldsymbol{F}_v{\dot{\boldsymbol{q}}}-\boldsymbol{F}_s\text{sgn}({\dot{\boldsymbol{q}}})-\boldsymbol{g}(\boldsymbol{q})+\boldsymbol{\tau}-\boldsymbol{J}^T(\boldsymbol{q})\boldsymbol{h}_{e})$$(equ.power)

Equalling {eq}`equ.power` to {eq}`equ.dke_dt`, we have

 $$
    \dot{\boldsymbol{q}}^{T}\boldsymbol{B}(\boldsymbol{q})\ddot{\boldsymbol{q}}+\frac{1}{2}\left(
    \dot{\boldsymbol{q}}^{T} \dot{\boldsymbol{B}}(\boldsymbol{q})\dot{\boldsymbol{q}}
    \right)=\dot{\boldsymbol{q}}^{T}(-\boldsymbol{F}_v{\dot{\boldsymbol{q}}}-\boldsymbol{F}_s\text{sgn}({\dot{\boldsymbol{q}}})-\boldsymbol{g}(\boldsymbol{q})+\boldsymbol{\tau}-\boldsymbol{J}^T(\boldsymbol{q})\boldsymbol{h}_{e})$$(equ.intermediate)

Recall the dynamics equation:

$$\boldsymbol{B}(\boldsymbol{q}) \ddot{\boldsymbol{q}}+\boldsymbol{C}(\boldsymbol{q}, \dot{\boldsymbol{q}}) \dot{\boldsymbol{q}}+\boldsymbol{g}(\boldsymbol{q})=\boldsymbol{\tau}-\boldsymbol{J}^{T}(\boldsymbol{q}) \boldsymbol{h}_{e}-\boldsymbol{F}_{v} \dot{\boldsymbol{q}}-\boldsymbol{F}_{s} \operatorname{sgn}(\dot{\boldsymbol{q}})
$$(equ.dyn_equ)

We multiply $\dot{\boldsymbol{q}}^T$ on both sides of {eq}`equ.dyn_equ` and subtract
{eq}`equ.dyn_equ` from {eq}`equ.intermediate` on both sides. This yields

$$\frac{1}{2}  \dot{\boldsymbol{q}}^{T} \boldsymbol{B}(\boldsymbol{q})\dot{\boldsymbol{q}}-\dot{\boldsymbol{q}}^T\boldsymbol{C}(\boldsymbol{q}, \dot{\boldsymbol{q}}) \dot{\boldsymbol{q}}=\frac{1}{2}  \dot{\boldsymbol{q}}^{T} \big( \boldsymbol{B}(\boldsymbol{q})\dot{\boldsymbol{q}}-2\boldsymbol{C}(\boldsymbol{q}, \dot{\boldsymbol{q}})\big) \dot{\boldsymbol{q}}=\boldsymbol{0}$$

which holds for any $\dot{\boldsymbol{q}}$. It means that
$\boldsymbol{B}(\boldsymbol{q})\dot{\boldsymbol{q}}-2\boldsymbol{C}(\boldsymbol{q}, \dot{\boldsymbol{q}})$
is a skew-symmetric matrix.


</br></br></br>

# Example: Two-link Planar Arm 

Consider the two-link planar arm below.



```{figure} ./dynamics/2link_arm.jpg
---
width: 50%
name: 2link_arm
---
wo-link planar arm
```


The vector of generalized coordinates is
$\boldsymbol{q}=\left[\begin{array}{ll}\vartheta_{1} & \vartheta_{2}\end{array}\right]^{T}$.
Let $\ell_{1}, \ell_{2}$ be the distances of the centres of mass of the
two links from the respective joint axes. Also let
$m_{\ell_{1}}, m_{\ell_{2}}$ be the masses of the two links, and let $I_{\ell_{1}}, I_{\ell_{2}}$ be
the moments of inertia relative to the Centers of mass of the two links,
respectively.

With the chosen coordinate frames, by applying {eq}`equ.com_jac`, the Jacobians for the COM of each
link are

$$\boldsymbol{J}_{P}^{\left(\ell_{1}\right)}=\left[\begin{array}{cc}
-\ell_{1} s_{1} & 0 \\
\ell_{1} c_{1} & 0 \\
0 & 0
\end{array}\right] \quad J_{P}^{\left(\ell_{2}\right)}=\left[\begin{array}{cc}
-a_{1} s_{1}-\ell_{2} s_{12} & -\ell_{2} s_{12} \\
a_{1} c_{1}+\ell_{2} c_{12} & \ell_{2} c_{12} \\
0 & 0
\end{array}\right]
\quad
\boldsymbol{J}_{O}^{\left(\ell_{1}\right)}=\left[\begin{array}{ll}
0 & 0 \\
0 & 0 \\
1 & 0
\end{array}\right] \quad J_{O}^{\left(\ell_{2}\right)}=\left[\begin{array}{ll}
0 & 0 \\
0 & 0 \\
1 & 1
\end{array}\right]$$


By applying {eq}`equ.bmatrix222`, the inertia matrix is

$$\boldsymbol{B}(\boldsymbol{q})=\left[\begin{array}{cc}
b_{11}\left(\vartheta_{2}\right) & b_{12}\left(\vartheta_{2}\right) \\
b_{21}\left(\vartheta_{2}\right) & b_{22}
\end{array}\right]$$

$$\begin{aligned}
b_{11}= & I_{\ell_{1}}+m_{\ell_{1}} \ell_{1}^{2}+I_{\ell_{2}}+m_{\ell_{2}}\left(a_{1}^{2}+\ell_{2}^{2}+2 a_{1} \ell_{2} c_{2}\right) \\
b_{12}= & b_{21}=I_{\ell_{2}}+m_{\ell_{2}}\left(\ell_{2}^{2}+a_{1} \ell_{2} c_{2}\right) \\
b_{22}= & I_{\ell_{2}}+m_{\ell_{2}} \ell_{2}^{2}
\end{aligned}$$

<!-- Compared to the previous example, the inertia matrix is now
configurationdependent. Notice that the term $k_{r 2} I_{m_{2}}$ in the
off-diagonal term of the inertia matrix derives from having considered
the rotational part of the motor kinetic energy as due to the total
angular velocity, i.e., its own angular velocity and that of the
preceding link in the kinematic chain. At first approximation,
especially in the case of high values of the gear reduction ratio, this
contribution could be neglected; in the resulting reduced model, motor
inertias would appear uniquely in the elements on the diagonal of the
inertia matrix with terms of the type $k_{r i}^{2} I_{m_{i}}$ -->

By applying {eq}`equ.cmatrix_item`,  the Christoffel symbols are

$$\begin{aligned}
& c_{111}=\frac{1}{2} \frac{\partial b_{11}}{\partial q_{1}}=0 \\
& c_{112}=c_{121}=\frac{1}{2} \frac{\partial b_{11}}{\partial q_{2}}=-m_{\ell_{2}} a_{1} \ell_{2} s_{2}=h \\
& c_{122}=\frac{\partial b_{12}}{\partial q_{2}}-\frac{1}{2} \frac{\partial b_{22}}{\partial q_{1}}=h \\
& c_{211}=\frac{\partial b_{21}}{\partial q_{1}}-\frac{1}{2} \frac{\partial b_{11}}{\partial q_{2}}=-h \\
& c_{212}=c_{221}=\frac{1}{2} \frac{\partial b_{22}}{\partial q_{1}}=0 \\
& c_{222}=\frac{1}{2} \frac{\partial b_{22}}{\partial q_{2}}=0
\end{aligned}$$

By applying {eq}`equ.cmatrix`, the matrix $\boldsymbol{C}(\boldsymbol{q},\boldsymbol{\dot{q}})$  is 

$$\boldsymbol{C}(\boldsymbol{q}, \dot{\boldsymbol{q}})=\left[\begin{array}{cc}
h \dot{\vartheta}_{2} & h\left(\dot{\vartheta}_{1}+\dot{\vartheta}_{2}\right) \\
-h \dot{\vartheta}_{1} & 0
\end{array}\right]$$

<!-- We can verify that

$$\begin{aligned}
\dot{\boldsymbol{B}}(\boldsymbol{q})-2 \boldsymbol{C}(\boldsymbol{q}, \dot{\boldsymbol{q}})   =\left[\begin{array}{cc}
0 & -2 h \dot{\vartheta}_{1}-h \dot{\vartheta}_{2} \\
2 h \dot{\vartheta}_{1}+h \dot{\vartheta}_{2} & 0
\end{array}\right]
\end{aligned}$$

is skew-symmetry.  -->


As for the
gravitational terms, here the gravity accelration vector is
$\boldsymbol{g}=\left[\begin{array}{lll}0 & -g & 0\end{array}\right]^{T}$. By applying {eq}`equ.gmatrix`

$$\begin{aligned}
& g_{1}=\left(m_{\ell_{1}} \ell_{1}+m_{\ell_{2}} a_{1}\right) g c_{1}+m_{\ell_{2}} \ell_{2} g c_{12} \\
& g_{2}=m_{\ell_{2}} \ell_{2} g c_{12} .
\end{aligned}$$

Given only motor toruqes $\tau_{1}$ and $\tau_{2}$  applied to the
each joint, the 
dynamics equation is 


$$\begin{aligned}
\left(I_{\ell_{1}}\right. & \left.+m_{\ell_{1}} \ell_{1}^{2}+I_{\ell_{2}}+m_{\ell_{2}}\left(a_{1}^{2}+\ell_{2}^{2}+2 a_{1} \ell_{2} c_{2}\right)\right) \ddot{\vartheta}_{1} \\
& +\left(I_{\ell_{2}}+m_{\ell_{2}}\left(\ell_{2}^{2}+a_{1} \ell_{2} c_{2}\right)\right) \ddot{\vartheta}_{2}  -2 m_{\ell_{2}} a_{1} \ell_{2} s_{2} \dot{\vartheta}_{1} \dot{\vartheta}_{2}-m_{\ell_{2}} a_{1} \ell_{2} s_{2} \dot{\vartheta}_{2}^{2} \\
& +\left(m_{\ell_{1}} \ell_{1}+m_{\ell_{2}} a_{1}\right) g c_{1}+m_{\ell_{2}} \ell_{2} g c_{12}=\tau_{1} \\
\left(I_{\ell_{2}}\right. & \left.+m_{\ell_{2}}\left(\ell_{2}^{2}+a_{1} \ell_{2} c_{2}\right)\right) \ddot{\vartheta}_{1}+\left(I_{\ell_{2}}+m_{\ell_{2}} \ell_{2}^{2}\right) \ddot{\vartheta}_{2} \\
& +m_{\ell_{2}} a_{1} \ell_{2} s_{2} \dot{\vartheta}_{1}^{2}+m_{\ell_{2}} \ell_{2} g c_{12}=\tau_{2}
\end{aligned}$$


