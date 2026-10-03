# Unscented Kalman Filter (UKF)

## Problem Statement

A robot needs to know where it is, but no single source of information is trustworthy on its own.

- **Odometry** (wheel speed `v` and turn rate `ω`) is smooth but drifts. Integrating it alone accumulates error without bound.
- **Landmark measurements** (range and bearing to known points) do not drift, but each one is noisy.

The task is to fuse the two into one best estimate of the robot's pose, along with an honest measure of how uncertain that estimate is.

The setup used here is a unicycle robot:

- State: `x = [px, py, yaw]`
- Control input: `u = [v, ω]`
- Motion model: `px' = px + v·cos(yaw)·dt`, `py' = py + v·sin(yaw)·dt`, `yaw' = yaw + ω·dt`
- Measurement: range `r` and bearing `b` to each known landmark, where `r = ‖landmark − p‖` and `b = atan2(Δy, Δx) − yaw`

Both models are **nonlinear**: the motion model through `cos(yaw)` and `sin(yaw)`, the measurement model through the square root and `atan2`. A standard Kalman filter only handles linear models, so it cannot be applied directly. The UKF is one way to handle this.

## Theory

### Kalman filter recap

A Kalman filter tracks a Gaussian belief `N(x, P)` and alternates two steps:

- **Predict:** push the belief forward through the motion model, adding process noise `Q`.
- **Update:** correct it with a measurement `z`, weighted by how much you trust the measurement (`R`) versus the prediction (`P`).

For linear models the equations are exact. For nonlinear models, the hard part is: *how does a Gaussian change when you pass it through a nonlinear function?*

### Two ways to answer that

- **EKF:** linearize the function around the current mean using a Jacobian, then use the linear equations. Simple, but it needs derivatives and is only accurate when the function is nearly linear over the spread of the belief.
- **UKF:** approximate the *distribution* instead of the *function*. Pick a small set of points that capture the mean and covariance of the Gaussian, push each one through the true nonlinear function, and recompute the mean and covariance from the results. No Jacobians are needed.

### Sigma points

For an `n`-dimensional state, the UKF uses `2n + 1` sigma points:

```
χ₀     = x
χᵢ     = x + (√((n+λ)P))ᵢ      for i = 1..n
χᵢ₊ₙ   = x − (√((n+λ)P))ᵢ      for i = 1..n
```

The matrix square root is a Cholesky factor `L` with `L Lᵀ = (n+λ)P`, and `(·)ᵢ` is its i-th column. The points sit at the center and along each principal direction of the uncertainty, in both directions.

### Scaling parameters and weights

The spread of the points is controlled by three parameters, `α`, `β`, `κ`:

```
λ = α²(n + κ) − n
```

Each point carries a weight for the mean and one for the covariance:

```
Wm₀ = λ / (n+λ)                     Wmᵢ = 1 / (2(n+λ))
Wc₀ = λ / (n+λ) + (1 − α² + β)      Wcᵢ = 1 / (2(n+λ))
```

- `α` sets how far the points spread from the mean (typically small, 1e-3 to 1).
- `β` encodes prior knowledge of the distribution. `β = 2` is optimal for Gaussians.
- `κ` is a secondary scaling term, usually 0.
- With `α < 1`, `Wm₀` can be **negative**. That is legitimate; the weights still sum to 1.

### Predict step

1. Draw sigma points from `(x, P)`.
2. Propagate each through the motion model: `χᵢ⁻ = f(χᵢ, u)`.
3. Recombine:

```
x⁻ = Σ Wmᵢ χᵢ⁻
P⁻ = Σ Wcᵢ (χᵢ⁻ − x⁻)(χᵢ⁻ − x⁻)ᵀ + Q
```

### Update step

1. Redraw sigma points from the predicted `(x⁻, P⁻)`.
2. Push each through the measurement model: `Zᵢ = h(χᵢ)`.
3. Compute the predicted measurement and its statistics:

```
ẑ   = Σ Wmᵢ Zᵢ
S   = Σ Wcᵢ (Zᵢ − ẑ)(Zᵢ − ẑ)ᵀ + R          innovation covariance
Pxz = Σ Wcᵢ (χᵢ − x⁻)(Zᵢ − ẑ)ᵀ              cross covariance
```

4. Compute the gain and correct:

```
K = Pxz S⁻¹
x = x⁻ + K (z − ẑ)
P = P⁻ − K S Kᵀ
```

`K` decides how much of the innovation `z − ẑ` is trusted. A large `R` (noisy sensor) makes `S` large and `K` small, so the filter leans on its prediction. A large `P⁻` does the opposite.

### Handling angles

Angles wrap at ±π, so two ordinary operations break:

- An arithmetic mean of `+3.1` and `−3.1` gives `0`, but the true mean direction is about `±π`. Angles need a **circular mean**: `atan2(Σ W sin θ, Σ W cos θ)`.
- A plain difference like `3.1 − (−3.1) = 6.2` is wrong. Angle residuals must be **wrapped** back into `[−π, π)`.

This applies to the yaw state and to every bearing measurement.

### Checking the filter is honest

Low error is not enough; the reported covariance `P` must match reality. Two standard checks, both averaged over a run:

- **NEES** = `(x − x_true)ᵀ P⁻¹ (x − x_true)`. Should average about `n` (here 3).
- **NIS** = `yᵀ S⁻¹ y`, where `y = z − ẑ`. Should average about `m` (here 6).

Values well below the target mean the filter is underconfident (`Q` or `R` too large). Values well above mean it is overconfident, which is the dangerous direction because it will start ignoring good measurements.

## References

- Wan and van der Merwe, "The Unscented Kalman Filter for Nonlinear Estimation" (2000)
- Thrun, Burgard, Fox, *Probabilistic Robotics*, ch. 3
- Roger Labbe, *Kalman and Bayesian Filters in Python*, UKF chapter