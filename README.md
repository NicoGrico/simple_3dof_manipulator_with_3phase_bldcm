To realize joint movements, actuators are required to provide the necessary torque (or forces). These can be pneumatic, hydraulic, or electrical in nature, each offering its own specific advantages.
Generally, however, these actuators should possess several critical performance characteristics. For instance, a low moment of inertia combined with a high power-to-weight ratio is essential.
This enables the actuators to execute fast and precise movements without being significantly hindered by their own mass (Siciliano et al. 2009, p. 193f).
According to Siciliano et al. (2009, p. 194f), electric servomotors are the most frequently used actuators in robotic applications. Among these, the two most common types are permanent magnet DC motors (PMDC) and brushless DC motors (BLDC).
In this repository a three-phase BLDC motor was modeled. In contrast to conventional DC motors, the stator consists of multi-phase windings, while the rotor is equipped with permanent magnets.
By eliminating brushes and commutators, electrical and mechanical losses are reduced, resulting in higher efficiency and longevity. Furthermore, moving the windings to the stator improves heat dissipation.
Overall, this leads to a more compact design and allows for lower-inertia rotors compared to conventional permanent magnet DC motors (Siciliano et al. 2009, p. 195).
For this purpose, a DC voltage source was connected with six MOSFETs from the Simscape library. Since the direct voltage signals from the voltage source are only compatible with Simscape components,
a Simscape Three-Phase Measurement Block was inserted, and the voltages were tapped at the measurement output .
The functionality of a BLDC motor is based on the interaction of two magnetic fields: an electromagnetic field from a coil on the stator and a magnetic field from a permanent magnet on the rotor.
