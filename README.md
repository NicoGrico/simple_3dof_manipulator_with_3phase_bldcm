# Modeling of a Three-Phase BLDC Motor

## 1. Requirements for Robotic Actuators
To realize joint movements, actuators are required to provide the necessary torque (or forces). These can be pneumatic, hydraulic, or electrical in nature, each offering its own specific advantages. Generally, however, these actuators should possess several critical performance characteristics. For instance, a **low moment of inertia** combined with a **high power-to-weight ratio** is essential. This enables the actuators to execute fast and precise movements without being significantly hindered by their own mass (Siciliano et al. 2009, p. 193f).

According to Siciliano et al. (2009, p. 194f), **electric servomotors** are the most frequently used actuators in robotic applications. Among these, the two most common types are:
* Permanent Magnet DC motors (PMDC)
* Brushless DC motors (BLDC)

## 2. BLDC Motor Characteristics
In this repository a **three-phase BLDC motor** was modeled. In contrast to conventional DC motors, the stator consists of multi-phase windings, while the rotor is equipped with permanent magnets. By eliminating brushes and commutators, electrical and mechanical losses are reduced, resulting in higher efficiency and longevity. Furthermore, moving the windings to the stator improves heat dissipation. Overall, this leads to a more compact design and allows for lower-inertia rotors compared to conventional permanent magnet DC motors (Siciliano et al. 2009, p. 195).

## 3. Simulation Environment and Components
For this purpose, a **DC voltage source** was connected with **six MOSFETs** from the Simscape library. Since the direct voltage signals from the voltage source are only compatible with Simscape components, a **Simscape Three-Phase Measurement Block** was inserted, and the voltages were tapped at the measurement output.

### Functional Principle
The functionality of a BLDC motor is based on the interaction of two magnetic fields: 
1. An electromagnetic field from a coil on the stator.
2. A magnetic field from a permanent magnet on the rotor.
The magnetic attraction between the north and south poles generates torque and rotates the rotor. The rotation of the rotor is maintained by changing the direction of the current in the stator windings, a process known as **commutation**.

## 4. Control Logic and Sensor Integration
To switch the stator windings at the correct time, it is essential to determine the position of the rotor. This requires at least **three Hall-effect sensors**, which are each offset by 120° (Yamashita et al. 2017, p. 3).
The Hall-effect sensors are simulated within a **Simulink Function block** using an `if` group. For this, the electrical rotor position $\theta_e$ is utilized. Depending on the Hall status and the direction of rotation, the MOSFET control signals are implemented in a subsequent `if` group.

---

![Alternativer Text](images/overview_three_phase_inverter.png)

---

# Implementation in Simulink

Based on the mathematical foundations, four **Simulink Function blocks** were developed to calculate the following aspects: current, electromotive force (EMF), electromagnetic torque, and rotor speed.
The equations listed previously are implemented directly within these Function blocks, with the corresponding variables interconnected. Rather than providing a detailed breakdown of every block, the following key implementation aspects are highlighted:

## 1. Electromotive Force (EMF) Simulation
The electromotive force is simulated using a **trapezoidal function**. To achieve this, the electrical rotor angle (defined as the mechanical rotor angle $\theta_m$ multiplied by the number of pole pairs $p$) is processed within an `if` statement. The EMF reaches its peak whenever the magnetic poles of the rotor pass directly by the stator coils.

## 2. Simulation Start-up Logic
Since the rotor speed $w_m$ is a fundamental variable in the motor equations, it must be ensured that the rotor successfully accelerates from a standstill at the start of the simulation. This is managed using two **Delay blocks** at the "Rotor Speed Block":
* A $Z^{-1}$ delay, which applies a non-zero motor torque for the first simulation step.
* A $Z^{-4}$ delay, which ensures the load torque $\tau_l$ is only engaged after four simulation steps.

## 3. Unit Consistency
Precise adherence to units is critical, particularly regarding the rotor variables:
* **Angular Velocity:** $w_m$ is initially output from the Motor Speed Block in $rad/s$, whereas the voltage constant may be provided in $V/kRPM$.
* **Rotor Position:** The position is required within a range of $0$ to $2\pi$ for the control logic, but the raw integration value reflects the total cumulative rotation in radians.

---
