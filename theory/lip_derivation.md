# Linear Inverted Pendulum — Theoretical Derivation

## 1. Physical System

The Linear Inverted Pendulum (LIP) models a bipedal system as a point
mass m at constant height h, supported by a massless rigid leg.

```
        • m  (COM)
       /|
      / |  h = constant
     /  |
    / θ |
───●────────────────  ground
   p (ZMP)
   |←x-p→|
```

Variables:
- m   : total body mass (kg)
- x   : COM horizontal position (m)
- z   : COM height = h = constant (m)
- p   : ZMP position at ground level (m)
- θ   : leg angle from vertical (rad)
- L   : leg length (m)

Geometric relations:

    x - p = L * sin(θ)
    h     = L * cos(θ)
    tan(θ) = (x - p) / h

## 2. Assumptions

1. Mass concentrated at COM — massless leg
2. Rigid, inextensible leg
3. z = h = constant (key assumption — enforces linearity)
4. No friction
5. Point contact at ground in p

---

## 3. Lagrangian Mechanics

### Position vector of COM
    r = (x, h)

### Velocity
    r_dot = (x_dot, z_dot) = (x_dot, 0)   since z = h = constant

### Kinetic energy
    T = 0.5 * m * ||r_dot||^2 = 0.5 * m * x_dot^2

### Potential energy
    V = m * g * z = m * g * h = constant

### Lagrangian
    L = T - V = 0.5 * m * x_dot^2 - m*g*h

---

## 4. Euler-Lagrange Equation

    d/dt(∂L/∂x_dot) - ∂L/∂x = Q_x

Where Q_x is the generalized force along x.

    ∂L/∂x_dot = m * x_dot   →   d/dt(∂L/∂x_dot) = m * x_ddot
    ∂L/∂x     = 0

Therefore:
    m * x_ddot = Q_x

---

## 5. Reaction Force Computation

The leg exerts a reaction force R along its axis (massless leg — pure compression).

Components:
    R_x = R * sin(θ)   (horizontal)
    R_z = R * cos(θ)   (vertical)

Vertical equilibrium (z_ddot = 0 since z = constant):
    m * z_ddot = R_z - m*g = 0
    R * cos(θ) = m*g
    R = m*g / cos(θ)

Horizontal force:
    Q_x = R_x = R * sin(θ) = m*g * sin(θ)/cos(θ) = m*g * tan(θ)

Since tan(θ) = (x - p) / h:
    Q_x = m*g * (x - p) / h

---

## 6. Equation of Motion

Substituting into Euler-Lagrange:

    m * x_ddot = m*g * (x - p) / h

    x_ddot = (g/h) * (x - p)

    x_ddot = ω² * (x - p)

Where:
    ω = sqrt(g/h)   — natural frequency of the LIP

### Key remark
Linearity is an **exact** consequence of the constraint z = h = constant.
This is NOT a small-angle approximation — the LIP is inherently linear.

---

## 7. State Space Formulation

Define state vector:
    X = [x, x_dot]^T

Define control input:
    u = p   (ZMP position)

Rewrite equation of motion:
    x_ddot = ω² * x - ω² * u

Continuous state space:
    X_dot = Ac * X + Bc * u

    Ac = | 0    1  |       Bc = |  0   |
         | ω²   0  |            | -ω²  |

---

## 8. Discretization

Using first-order Euler method with timestep dt:

    X_{k+1} = (I + Ac*dt) * X_k + Bc*dt * u_k
    X_{k+1} = A * X_k + B * u_k

    A = | 1       dt  |       B = |  0      |
        | ω²*dt   1   |           | -ω²*dt  |

For h = 0.8m, g = 9.81 m/s², dt = 0.01s:
    ω = sqrt(9.81 / 0.8) = 3.502 rad/s
    ω² = 12.2625

---

## 9. ZMP Stability Constraint

The Zero Moment Point (ZMP) is the point on the ground where the
resultant of contact forces creates no moment.

Stability condition:
    p_min ≤ p ≤ p_max

Where the support polygon for a foot centered at x_foot:
    p_min = x_foot - foot_size/2
    p_max = x_foot + foot_size/2

If ZMP exits the support polygon → the system tips over.

Relation between ZMP and COM:
    p = x - (h/g) * x_ddot

This shows that if COM accelerates too fast, ZMP exits the polygon.
The MPC enforces this constraint at every predicted timestep.

---

## 10. Extension to 3D

In 3D, the LIP decouples into two independent planes:

Sagittal plane (forward walking, x-axis):
    x_ddot = ω² * (x - p_x)

Frontal plane (lateral balance, y-axis):
    y_ddot = ω² * (y - p_y)

Both equations are identical in form — two independent MPC problems,
one per plane. This is the elegance of the LIP model.

---

## References

1. Kajita, S. et al. (2001). The 3D Linear Inverted Pendulum Mode:
   A simple modeling for a biped walking pattern generation.
   IEEE/RSJ IROS 2001.

2. Kajita, S. et al. (2003). Biped Walking Pattern Generation by using
   Preview Control of Zero-Moment Point.
   IEEE ICRA 2003.

3. Wieber, P.B. (2006). Trajectory Free Linear Model Predictive Control
   for Stable Walking in the Presence of Strong Perturbations.
   IEEE-RAS International Conference on Humanoid Robots 2006.

4. Vukobratovic, M., Borovac, B. (2004). Zero-Moment Point — Thirty Five
   Years of its Life. International Journal of Humanoid Robotics.
