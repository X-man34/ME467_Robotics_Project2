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

## Question 2: Mahony Filter Implementation

In order to estimate the phone's orientation over time, we implemented the Mahony Filter. The Mahony Filter uses gyroscope data to integrate our orientation and also applies correction to the gyroscope based on data from the accelerometer and magnetometer measurements, which localize the phone's orientation based on gravity and magnetic north.

flipping on certain edges or not
drifting to magnetic north
how gains have changed things
confinming answers with rotated estimate - vhata in the spatial
3D visualzation