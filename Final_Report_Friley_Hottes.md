# Title

In this project, we will be implementing three different orientation algorithms to estimate the orientation of a  phone over time. A naïve gyroscope integration (dead reckoning) algorithm is used in Question 1 and Question 4a. The Mahony filter is used in Questions 2 and 3, and serves as the baseline for comparison in Question 4. Finally, the TRIAD method is introduced in Question 4b and compared against the other methods in Questions 4c and 4d.

## Algorithm Explanations

### Naïve Gyroscope Integration (Dead Reckoning)

The naïve gyroscope integration algorithm estimates orientation by directly integrating the angular velocity over time. It assumes that the gyroscope data is accurate and doesn't include any corrections from other sensors. This method is simple and fast but suffers from drift over time, especially if there's any bias or noise in the gyroscope. It works well in the short term but quickly becomes inaccurate during longer or more complex motion.

### Mahony Filter

The Mahony filter is a nonlinear complementary filter that combines gyroscope integration with feedback correction using accelerometer and magnetometer data. It helps correct drift and stabilize orientation over time by aligning the estimated orientation with gravity (from the accelerometer) and magnetic north (from the magnetometer). It also estimates gyroscope bias dynamically. This makes the Mahony filter more reliable than dead reckoning, especially for longer durations or when precise orientation is important.

### TRIAD Method

The TRIAD (Tri-Axial Attitude Determination) method estimates orientation using two reference vectors: gravity and magnetic north. It constructs two orthonormal frames — one from known inertial vectors and one from measured body-frame vectors — and computes the rotation matrix that aligns them. Unlike the other two methods, TRIAD is algebraic and doesn't rely on time integration, which means it gives an immediate estimate of orientation at each time step, but doesn’t track changes over time or handle gyroscope data.

## Question 1

In question 1 we implement a naïve gyroscope integration in order to estimate orientation with a set of given inputs. These inputs simulate perfect gyroscope data with no bias or drift.

The given inputs can be represented by various rotation matrixes.

1. $R_{x,\frac{\pi}{4}}$ -  Rotate about the **x-axis** by $\frac{\pi}{4}$ radians  
2. $R_{z,\frac{\pi}{4}}$ - Rotate about the **z-axis** by $\frac{\pi}{4}$ radians  
3. $R_{x,-\frac{\pi}{4}}$ - Rotate about the **x-axis** by $-\frac{\pi}{4}$ radians  
4. $R_{z,-\frac{\pi}{4}}$ - Rotate about the **z-axis** by $-\frac{\pi}{4}$ radians  

If we multiply all these rotation matrices we can see our expected result:

$R_{x,\frac{\pi}{4}} R_{z,\frac{\pi}{4}} R_{x,-\frac{\pi}{4}} R_{z,-\frac{\pi}{4}} = \begin{bmatrix}
0.8536 & 0.1464 & -0.5000 \\
-0.2500 & 0.9571 & -0.1464 \\
0.4571 & 0.2500 & 0.8536
\end{bmatrix}$

Which corresponds with this quaternion:

$q_{final}=0.9571 <  0.1036, -0.2500, -0.1036 >$

### Results

$q(1)=0.9571 <  0.1036, -0.2500, -0.1036 >$

We simulated this rotation over time and obtained the exact same final orientation as expected from the matrix multiplication. This verifies that our implementation of quaternion integration is correct. You can see a visualization of this rotation in Fig. 1.1 in the fig file under `fig\Fig 1-1.mp4`.

### Does it hold that $q(0) = q(1)$?

No, it does not hold that $q(0) = q(1)$. We can see from our results that:

$q(0)=1 <  0, 0, 0 >$

$q(1)=0.9571 <  0.1036, -0.2500, -0.1036 >$

This also makes conceptual sense because rotations in 3D space are not commutative.

## Question 2: Mahony Filter Implementation

In question 2 we implement a Mahony filter to estimate a phone's orientation over time with a given csv file containing gyroscope, accelerometer, and magnetometer data over time. The motion consists of a phone that is still for a few seconds, and is then picked up and rotated around randomly, but slowly and smoothly, and then set back down.

We tuned out K values to be the following based on trial and error.

$k_p=1, k_I=.3, k_a=.8, k_m=.2$

We visualized this motion using MuJoCo that can be seen in `fig\Fig_2-2.mp4`

### Results

To calculate the rotation angle over time, we convert the estimated orientation quaternion at each time step into an axis-angle representation. We then extract the angle component and plot it with respect to time.

Figures 2-1a and 2-1b below show the rotation angle over time in radians and degrees, respectively. The phone remains still at the beginning, is then slowly rotated in multiple directions, and eventually returns to rest.

Over the course of the motion, the phone rotated a total of approximately 8,572 radians, or 491,165 degrees. This total includes all incremental rotations.

![Figure 2-1 Rads](fig/Fig_2-1a_(rad).png)
*Fig 2-1a: Estimated rotation angle over time (radians)*

![Figure 2-1 Degrees](fig/Fig_2-1b_(degree).png)
*Fig 2-1b: Estimated rotation angle over time (degrees)*

### Observations

Doing this project, we noticed some quirks of the Mahony filter.

flipping on certain edges or not
drifting to magnetic north
how gains have changed things
confinming answers with rotated estimate - vhata in the spatial
3D visualzation.
put a magnet by the phone, can be seen when we do 3D visualization
