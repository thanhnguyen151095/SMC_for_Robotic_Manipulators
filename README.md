# SMC_for_Robotic_Manipulators
Sliding Mode Control for Robotic Manipulators
# Introduction
Robotic manipulators are extensively utilized across diverse fields, spanning from industrial plants and medical treatment to military operations, education, and space exploration. Their versatility not only enhances productivity and product quality but also mitigates risks in hazardous environments, reducing potential harm to human workers. Consequently, ensuring robust trajectory tracking performance and maintaining high stability in robotic arms become crucial. However, due to their complexity as highly interconnected systems with multiple inputs and outputs, robots often face challenges in maintaining performance amidst external uncertainties and disturbances. As a result, designing controllers capable of ensuring precise trajectory tracking remains a significant ongoing research focus.

Several control strategies have been developed to address the trajectory tracking challenge in robots, such as computed torque control, PID, SMC, and adaptive control. SMC is notable for its simplicity and robustness against uncertainties and disturbances, gaining considerable attention in recent years.

# Problem Formulation
## Robot System Description
The dynamic of an $n$-joint robot arm can be expressed as follows:

$$
\mathcal H\left( q  \right)\ddot q  + \mathcal Q(q ,\dot q )\dot q + \mathcal G(q ) + \mathcal F(\dot q ) = \tau  - \tau _D \quad\quad (1)
$$

where $q \in {\mathbb{R}^{n \times 1}}$, $\dot q \in {\mathbb{R}^{n \times 1}}$, and $\ddot q  \in {\mathbb{R}^{n \times 1}}$ represent the vectors for joint position, joint velocity, and joint acceleration, respectively. The matrix ${\mathcal H}(q) =  {\mathcal H _0}(q) + \Delta {\mathcal H}(q)\in {\mathbb{R}^{n \times n}}$ corresponds to the inertia matrix. 
$\mathcal Q(q ,\dot q ) = {\mathcal Q_0}(q ,\dot q ) + \Delta \mathcal Q(q ,\dot q ) \in {\mathbb{R}^{n \times n}}$ captures the effects of centrifugal and Coriolis forces. 
The gravity vector is denoted by $\mathcal G(q ) = {\mathcal G _0}(q ) +\Delta \mathcal G(q )\in {\mathbb{R}^{n \times 1}}$. 
${\mathcal H _0}\left( \theta  \right)$, ${\mathcal G _0}(\theta )$, and ${\mathcal Q _0}(\theta ,\dot \theta )$ are the nominal values of the dynamic model, while $\Delta {\mathcal H}\left( q  \right)$, $\Delta{\mathcal G}(q )$, and $\Delta{\mathcal Q}(q ,\dot q )$ represent the uncertainties in these parameters. 
The vector $\tau\in {\mathbb{R}^{n \times 1}}$ corresponds to the control input torques, while $\mathcal F(\dot q) \in {\mathbb{R}^{n \times 1}}$ and $\tau_D \in {\mathbb{R}^{n \times 1}}$ denote friction forces and external disturbances, respectively. 

Eq. (1) can be reformulated as:

$$
\ddot q =    {\mathcal H _0}^{-1}(q)(- {\mathcal Q _0}(q ,\dot q )\dot q - {\mathcal G _0}(q ) + \tau + \Sigma ) \quad\quad (2)
$$

where $\Sigma = -\Delta {\mathcal H}(q)q - \Delta \mathcal G(q )-\Delta \mathcal Q(q,\dot q )\dot q - \mathcal F(\dot q )  - \tau _D \in {\mathbb{R}^{n \times 1}}$ encapsulates all uncertainties and external disturbances, and is bounded by $\left\| \Sigma  \right\| \le {\bar \Sigma}$.

Property 1: The inertia matrix $\mathcal H _0(q)$ is positive and symmetric, and is bounded as:

$$
0 < \lambda_{min}(H_0(q))  \le |H_0(q)| \le \lambda_{max}(H_0(q)) \quad\quad (3)
$$

where $\lambda_{max}({H_0}(q))$ and  $\lambda_{min}({H_0}(q))$  denote the maximum and minimum eigenvalues of ${\mathcal H}_0(q)$. 

Property 2: The passivity property of the robotic manipulator can be described as:

$$
x^T (\dot{ {\mathcal H _0}}(q) - 2{\mathcal Q _0}(q ,\dot q ))x=0 \quad\quad (4)
$$

for any $x \in {\mathbb{R}^{n \times 1}}$. 

This research aims to design an SMC law, denoted as $\tau$, characterized by minimized chattering behavior. This control law is intended to regulate the joint position of the robot arm, ensuring both swift and precise tracking of the desired position.

# Design of SMC

The tracking errors for position and velocity are defined as follows:

$$
e = q - q_{d}; \quad \dot{e} = \dot{q} - \dot{q}_d \quad\quad (5)
$$

Here, $q_d$ and $\dot{q}_d$ represent the desired position and velocity, respectively.

Next, we can establish the sliding mode surface as:

$$
s = \dot{e} + c e \quad\quad (6)
$$

In this equation, $c$ is a positive constant.

Taking the time derivative of the sliding mode surface in Eq. (6) and using the system model in Eq. (2), We obtain:

$$
\begin{aligned}
\dot{s} &= \ddot{e} + c \dot{e} \quad\quad\quad\quad\quad (7)\\
&= \ddot{q} - \ddot{q}_d + c \dot{e} \\
&= {\mathcal H _0}^{-1}(q)(- {\mathcal Q _0}(q ,\dot q )\dot q - {\mathcal G _0}(q ) + \tau + \Sigma ) - \ddot{q}_d + c \dot{e} \\
\end{aligned}
$$

To design the control law, we first select a Lyapunov function as $V=0.5s^T s$, then its time derivative can be given as:

$$
\begin{aligned}
\dot{V} &= s^T \dot{s} \quad\quad\quad\quad\quad (7)\\
&= s^T \left ( \right) \\
&= {\mathcal H _0}^{-1}(q)(- {\mathcal Q _0}(q ,\dot q )\dot q - {\mathcal G _0}(q ) + \tau + \Sigma ) - \ddot{q}_d + c \dot{e} \\
\end{aligned}
$$





# Simulation Results and Discussions

# Conclusion

# References

