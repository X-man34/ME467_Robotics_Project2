### To integrate our orientation, we use this formula

$q_{k+1} = e^{\frac{1}{2}  \Omega(u)\delta t}q_k$

where:

- $k = \frac{t}{\delta t}$ is the integration step (huh? wdym by integration step?)

- $e^{\Omega(u)\theta} = \cos(\|u\|\theta)\, I_4 + \sin(\|u\|\theta)\, \frac{\Omega(u)}{\|u\|}$, where we plug in $\theta = \frac{1}{2}\delta t$

- $\Omega(u) = \begin{bmatrix} 0 & -u^\top \\ u & -u_\times \end{bmatrix}$

- $u = \omega_y - \hat{b} + k_p \omega_{mes}$

    - $\omega_y = [\omega_{y,x},\omega_{y,y},\omega_{y,z}]$, inputs from data

    - $\hat{b}_{k+1} = \hat{b}_k - k_I \omega_{mes} \delta t$

    - $\omega_{mes} = k_a v_a \times \hat{v}_a + k_m v_m \times \hat{v}_m$

        - $v_a = \frac{[a_x, a_y, a_z]}{||a||}$ and $v_m = \frac{[m_x, m_y, m_z]}{||m||}$

        - $\hat{v}_{a, k+1} = e^{-u \times \delta t} \hat{v}_{a,k}$, and $\hat{v}_{m,k+1} = e^{-u \times \delta t} \hat{v}_{m,k}$

        - $e^{u \times \theta} = I_3 + sin(||u|| \theta) \frac{u_{\times}}{||u||} + (1 - cos(||u|| \theta )) \frac{u_{\times}^2}{||u||^2}$, where we plug in $\theta = \delta t$ and $u = -u$

### Gains:

$k_p$: Proportional gain used in the orientation update to scale the correction from the measurement error.

$k_I$: Integral gain used for updating the gyroscope bias estimate.

$k_a$: Gain promoting error correction using normalized acceleration vector $v_a$ and its estimate $\hat{v}_a$.

$k_m$: Gain promoting error correction using normalized magnetic field vector $v_m$ and its estimate $\hat{v}_m$.


### Initial Conditions:

- **Orientation:**

  The initial quaternion is set to the identity:

  $
  q(0) = [1, 0, 0, 0]
  $

- **Gyroscope Bias:**

  The initial bias is assumed to be zero:

  $
  \hat{b}(0) = [0, 0, 0]^T
  $

- **Accelerometer Reference:**

  The inertial gravity vector is defined as:

  $
  a_0 = [0, 0, 9.81]
  $

  so the normalized vector becomes:

  $
  v_a(0) = \frac{a_0}{\|a_0\|} = [0, 0, 1]
  $

- **Magnetometer Reference:**

  The normalized magnetic north vector is given by:

  $
  m^* = \begin{bmatrix} 0.087117 \\ 0.37923 \\ -0.92119 \end{bmatrix}
  $

  thus:

  $
  v_m(0) = \frac{m^*}{\|m^*\|} = \begin{bmatrix} 0.087117 \\ 0.37923 \\ -0.92119 \end{bmatrix}
  $

- **Estimated Vectors:**

  The initial estimated directions are set to the inertial references:

  $
  \hat{v}_a(0) = v_a(0), \quad \hat{v}_m(0) = v_m(0)
  $

- **Time Step:**

  The integration step is constant:

  $
  \delta t = 0.01 \, \text{s}
  $
