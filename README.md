# A Complementary filter accuracy analysis 
In this repository I will explore the main issue (noises) linked to signal acquirement and prove on a graph the effect of the Complementary filter on these results. Starting from a sinusoidal wave to which an artificial Gaussian noise is applied, the project proves how the computationally inexpensive blending of sensor data (Complementary filter) can rebuild a steady signal from a disturbed one. My objective is to visualise how the algorithm behaves in differents simulated scenarios, each of them determined by set bias and Gaussian Noise. For simplicity, I have chosen a 2D case, where we can imagine a drone flying.

------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

The "noise" idea

No measurement in practical application is totally aligned with a "clean" model of data acquirement. The term "noise" is used to describe these casual variations.   

Note: in our simulation, a Gaussian noise will be injected into a sinusoidal wave. This kind of disturb is a mathematical model widely-used for simulation and tests:

- it closely mimics natural interferences, such as the noise produced by digital sensors or hardware fluctuations.

- since it is based on a normal distribution, it is highly manageable to study, analyze, and implement in software.

- its fluctuations are simply added to the lean signal, making it easier to implement algorithm able to eliminate it.

- the “Central Limit Theorem” states that the sum of a large number of independent random variables (as in this simulation) tends toward a Gaussian bell-curve distribution


The Complementary Filter

A complementary filter is a quick and effective method for blending measurements from an accelerometer and a gyroscope to generate an estimate for orientation.

The filter works assigning specific weights to the measurements provided by the two sensors, so as to:

-	Smooth spikes measured from the accelerometer
  
-	Reduce the offset caused by the gyroscope drift
Before moving on a brief description of the two sensors considered for this simulation, consider the following statement, necessary to understand the conclusion drawn from each experiment.
In the complementary filter, the value of Bias settles at a constant limit, called B_max, described by the following equation:

$$
B_{\max} = \frac{B[\tau]}{1 - e^{-k_{\text{drift}} \cdot t}}
$$,

where k is a constant which expresses how “fast” B(tau) is growing to B_max, t stands for time since the filter started.
Such specification is necessary because, in absence of this formula, the simulation would have lost the crucial detail of the offset: by simply adding B to the to the true angular velocity value, the growing offset would not be visible, since a graph plotting the measured velocity would only represent “true_speed” simply offset of a value B.

Accelerometer

A 3D accelerometer estimates the tilt angles (pitch and roll) by measuring the projections of the gravity vector. For instance, pitch is computed as:

$$
\theta = \arctan\left( \frac{a_x}{\sqrt{a_y^2 + a_z^2}} \right)
$$

where a represents the acceleration measured along each respective axis (x, y, z). 
However, accelerometers are highly sensitive to high-frequency noise: mechanical vibrations and external forces temporarily add up to the gravity vector, instantly distorting the true angular measurement.
For this analysis, I assume that the sensor is only affected from noises from the system’s vibrations, and any static bias has been previously calibrated to zero.

Gyroscope
A 3D gyroscope is a sensor which measures the angular rate by applying the theory of the Coriolis effect, which refers to the force of inertia that acts on objects in motion in relation to a rotating frame.
It is composed of a mass which, when experiences a force, causes a change in the value of capacitance which is then converted into an electrical signal proportional to the angular rate.
The sensor estimates the value of delta_theta in a 3D space computing

$$
\Delta\theta = \omega \cdot dt
$$

where omega must not be intended as the real delta_theta / delta_t, since offset (B) and micro-vibrations (mu) occur:

$$
\omega = \omega_{\text{real}} + B + \mu
$$

In my code, I decided to consider a constantly generated Gaussian noise to simulate disturbs such as micro-vibrations :
omega_measured = true_speed + B + omega_noise.

The weight

The weight ("alpha", in the equation in the code) assigned to raw data is proportional to the level of accuracy expected from each sensor, considering both noise and offset (see above): for short time interval "tau", the algorithm expects higher reliability to the measurement from the gyroscope, since the accelerometer may detect high spikes in the change of pitch over time, due to the noise.

Technically speaking, the filter “low passes” the accelerometer and “high passes” the gyroscope.
Thanks to these two sources and weights, the filter can counterbalance the tendence to drift of the gyroscope and smooth eventual noise spikes detected from the accelerometer.

The time constant τ
In a complementary filter, τ (tau) is the time constant that determines the cutoff frequency between the high-pass filter (for the gyroscope) and the low-pass filter (for the accelerometer). It represents the time scale (in seconds) at which the filter transitions from trusting the gyroscope to trusting the accelerometer.For events shorter than τ seconds, the filter relies mostly on the gyroscope to capture fast movements without vibration noise. For timescales longer than τ seconds, the filter relies on the accelerometer to correct for gyroscope drift.

- Increasing τ (e.g., 1.0 s to 5.0 s): Trusts the gyroscope longer and rejects linear acceleration or mechanical vibrations more aggressively.Larger τ are preferable in sustained-accelerations systems (like a fast-moving drone or car), and a low-drift gyroscope.

- Decreasing τ (e.g., 0.1 s to 0.5 s): Trusts the accelerometer more and corrects gyroscope drift faster.Smaller τ are preferable when the system operates in a relatively vibration-free environment, and gyroscope drifts quickly.


```math
\tau = \frac{\alpha \cdot dt}{1 - \alpha}
```

The physical quantity also determines the offset angle around which the measurement tends to settle:

```math
\Delta\theta_{\infty} = \tau \cdot B
```


```math
\alpha = \frac{\tau f}{\tau f_c + 1}
```

and 


```math
f_c = \frac{1}{2\pi\tau}
```

where f_c is the frequency at which the filter starts 


The choice of τ depends on the environment the sensor is supposed to be used:

| Phenomenon | Prefers | Reason |
| :--- | :--- | :--- |
| Gyro bias | Small $\tau$ | $b\tau = \Delta\theta_{\infty}$ |
| Linear acceleration, bumps | Large $\tau$ | Disturbance lasts for a time; the filter averages for $\tau$ |
| Vibrations | Large $\tau$ | Attenuation grows with $f \times \tau$ |


Due to the relationship between α and τ, a precise evaluation for τ is necessary. For example, using α = 0.98 at f = 50 Hz gives τ = 0.98 s; at f = 1 kHz it gives 0.049 s, resulting in very different behaviour. 



RMSE 
So as to get a visible report of different results obtained through different B_max and k-drift, I decided to make the program compute the RMSE (Root Mean Square Error) for each set of 40 values, where each value correspond to one measurement acquirement from the sensor. 
The mean value corresponds to true_angle.
This measure is an appropriate evaluation metric because it represents the standard deviation of the residuals.


The following section is about the esteem and the comprehension of the filter accuracy, obtained through a mathematical abstraction and analysis. 
My analysis is based both on the study of RMSE and graphs for different scenarios, each of them characterized by set weight (alpha), noise (mu/ omega_measured) and drift (B/ B_max).
So as to provide a simple and clear representation of the filter mechanism, I wrote a program which simulates it.

-	Just like the project objective, this simulation is built upon data acquired from a gyro and an accelerometer
  
-	Np.random.normal function draws random sample values normally distributed around a certain mean value. To get the most realistic value possible, according to the sensitivity toward vibrations (causes of random spikes in the  measurement), I set higher spread for the accelerometer (np.random.normal(3.0, 1)). The specific value and spread (3.0 and 1) means that the moving system actually does not experience any change in heading (3.0 degrees).
  
-	Consequently, the angular speed (degree/second) mean has been set to 0.0.
  
-	For clarity and simplicity, a “complementary angle” value is set to 3.00 degree: it corresponds to the first one obtained.
  
-	The program computes these values inside the filter equation, and outputs the resulting angle.

As a first take, make reference to the table below

(not to be intended as a test report)

| Tau interval | Accelerometer angle (degree) | Gyro omega (degree/s) | Filtered angle |
| :--- | :--- | :--- | :--- |
| 1 | 2.12863 | -0.65616 | 2.97614 |
| 2 | 2.15961 | -1.06365 | 2.94938 |
| 3 | 2.31481 | -0.47217 | 2.93207 |
| 4 | 2.61712 | -0.40057 | 2.92184 |
| 5 | 4.28818 (SPIKE!!) | 0.90776 | 2.95806 |

  
The following report shows the results obtained at different set values (I focused on alpha and drift). 
Note: alpha, representing the weight assigned to the two sensors, is usually a value that fluctuates between 0.96 and 0.99, causing the “complementary one” going from 0.04 and 0.01 (1-0.96 = 0.04; 1-0.97 = 0.03).

In the simulation I'm going to choose the most suitable τ value according to the differfent scenarios (each of them determined by set B_max and Gaussian Noise)

### B_max = 1.25 °/s, noise = (0.0, 0.50), τ = 0.25 s
[Comparison](percorso/della/tua/immagine.png)

[RMSE](percorso/della/tua/immagine.png)

### B_max = 1.25 °/s, noise = (0.0, 0.75), τ = 0.25 s
[Comparison](percorso/della/tua/immagine.png)

[RMSE](percorso/della/tua/immagine.png)

### B_max = 1.25 °/s, noise = (0.0, 0.75), τ = 0.50 s
[Comparison](percorso/della/tua/immagine.png)

[RMSE](percorso/della/tua/immagine.png)

### B_max = 1.00 °/s, noise = (0.0, 0.75), τ = 0.50 s
[Comparison](percorso/della/tua/immagine.png)

[RMSE](percorso/della/tua/immagine.png)

### B_max = 2.00 °/s , noise = (0.0, 0.75), τ = 0.15 s
[Comparison](percorso/della/tua/immagine.png)

[RMSE](percorso/della/tua/immagine.png)

### B_max = 2.00 °/s , noise = (0.0, 1.00), τ = 0.15 s
[Comparison](percorso/della/tua/immagine.png)

[RMSE](percorso/della/tua/immagine.png)








___________________________________________________________________________________________________________________________________________________________________________________

• Drones and UAVs: Estimates roll, pitch, and yaw from IMU sensors (accelerometers, gyroscopes, and magnetometers) on resource-constrained flight controllers.
• Robotics and High-Vibration Environments: Tracks orientation when low compute overhead or resilience to vibration/over-ranging is needed.
• Wearables, Gait, and Biomechanics: Measures human locomotion, pedestrian navigation, or animal movement by separating slow orientation changes, mid-frequency gait, and fast strikes or vibrations.
• VR/AR: Provides responsive head or device tilt tracking using low-cost MEMS sensors.
• Signal and Audio Processing: Splits signals into complementary frequency bands (such as power- or delay-complementary filter pairs) for crossover networks and sub-band processing.
 
Among everyday applications, a notable one is in inertial navigation systems. The random drift rate of the gyroscope will seriously affect the positioning accuracy of the navigation system. Therefore, the inertial navigation system has strict requirements for the random drift rate of the gyroscope, and generally should reach 0.01°/h or even smaller.
Through Kalman esteem, the vehicle can still navigate and track his path precisely, even in absence of GNSS (INS (Inertial Navigation System) or Dead Reckoning).

 
