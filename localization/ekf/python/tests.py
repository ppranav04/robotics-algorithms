import math
import numpy as np
import pytest
 
import EKF
 
# EKF_estimate references Q and R as bare globals.
EKF.Q = np.diag([0.05, 0.05, np.deg2rad(1.0)]) ** 2
EKF.R = np.diag([0.3, np.deg2rad(3.0)]) ** 2
 
EPS = 1e-6          # finite-difference step size
TOL = 1e-4           # allowed gap between analytic and numerical Jacobian
 
 
# ---------------------------------------------------------------------------
# Finite-difference helpers
# ---------------------------------------------------------------------------
 
def finite_diff_jacobian(f, x, extra_args=()):
    """Central-difference Jacobian of f(x, *extra_args) w.r.t. x.
 
    x is a (n,1) column vector. f must return a (m,1) column vector.
    Returns an (m,n) matrix.
    """
    n = x.shape[0]
    f0 = f(x, *extra_args)
    m = f0.shape[0]
    J = np.zeros((m, n))
    for i in range(n):
        dx = np.zeros_like(x)
        dx[i, 0] = EPS
        f_plus = f(x + dx, *extra_args)
        f_minus = f(x - dx, *extra_args)
        J[:, i] = ((f_plus - f_minus) / (2 * EPS)).flatten()
    return J
 
 
# ---------------------------------------------------------------------------
# Jacobian correctness
# ---------------------------------------------------------------------------
 
def test_jacob_f_matches_finite_difference():
    state = np.array([[1.5], [-0.7], [0.9]])   # arbitrary non-trivial pose
    control = np.array([[1.2], [0.3]])          # arbitrary non-zero control
 
    analytic = EKF.jacob_f(state, control)
    numeric = finite_diff_jacobian(EKF.motion_model, state, extra_args=(control,))
 
    assert np.allclose(analytic, numeric, atol=TOL), (
        f"jacob_f mismatch:\nanalytic=\n{analytic}\nnumeric=\n{numeric}"
    )
 
 
def test_jacob_f_matches_finite_difference_zero_yaw():
    # theta = 0 is a common edge case (sin=0, cos=1) worth checking
    # separately from a generic pose above.
    state = np.array([[0.0], [0.0], [0.0]])
    control = np.array([[0.8], [0.4]])
 
    analytic = EKF.jacob_f(state, control)
    numeric = finite_diff_jacobian(EKF.motion_model, state, extra_args=(control,))
 
    assert np.allclose(analytic, numeric, atol=TOL)
 
 
def test_jacob_h_matches_finite_difference():
    state = np.array([[1.0], [1.0], [0.3]])
    landmark = np.array([[5.0], [4.0]])   # must not coincide with state x,y (r=0 -> singular)
 
    analytic = EKF.jacob_h(state, landmark)
    numeric = finite_diff_jacobian(EKF.observation_model, state, extra_args=(landmark,))
 
    assert np.allclose(analytic, numeric, atol=TOL), (
        f"jacob_h mismatch:\nanalytic=\n{analytic}\nnumeric=\n{numeric}"
    )
 
 
def test_jacob_h_matches_finite_difference_various_positions():
    # Sweep a handful of robot poses / landmark placements so a bug that
    # only shows up in one quadrant (e.g. a sign error tied to atan2's
    # branch) doesn't slip through a single lucky test case.
    cases = [
        (np.array([[0.0], [0.0], [0.0]]), np.array([[3.0], [0.0]])),
        (np.array([[0.0], [0.0], [0.0]]), np.array([[0.0], [3.0]])),
        (np.array([[2.0], [-3.0], [1.2]]), np.array([[-4.0], [5.0]])),
        (np.array([[-1.0], [-1.0], [-2.5]]), np.array([[2.0], [-6.0]])),
    ]
    for state, landmark in cases:
        analytic = EKF.jacob_h(state, landmark)
        numeric = finite_diff_jacobian(EKF.observation_model, state, extra_args=(landmark,))
        assert np.allclose(analytic, numeric, atol=TOL), (
            f"jacob_h mismatch at state={state.T}, landmark={landmark.T}:\n"
            f"analytic=\n{analytic}\nnumeric=\n{numeric}"
        )
 
 
# ---------------------------------------------------------------------------
# Sanity / known-case tests
# ---------------------------------------------------------------------------
 
def test_motion_model_zero_control_is_identity():
    state = np.array([[3.0], [-2.0], [0.7]])
    zero_control = np.array([[0.0], [0.0]])
    result = EKF.motion_model(state, zero_control)
    assert np.allclose(result, state)
 
 
def test_motion_model_moves_forward_along_heading():
    # Facing along +x (theta=0), driving forward should only change x.
    state = np.array([[0.0], [0.0], [0.0]])
    control = np.array([[1.0], [0.0]])   # v=1, omega=0
    result = EKF.motion_model(state, control)
    assert result[0, 0] == pytest.approx(EKF.dt, abs=1e-9)   # x advanced by v*dt
    assert result[1, 0] == pytest.approx(0.0, abs=1e-9)       # y unchanged
    assert result[2, 0] == pytest.approx(0.0, abs=1e-9)       # theta unchanged
 
 
def test_observation_model_known_geometry():
    # Robot at origin facing +x, landmark 3 units directly ahead.
    state = np.array([[0.0], [0.0], [0.0]])
    landmark = np.array([[3.0], [0.0]])
    z = EKF.observation_model(state, landmark)
    assert z[0, 0] == pytest.approx(3.0)     # range
    assert z[1, 0] == pytest.approx(0.0)      # bearing: straight ahead
 
 
def test_observation_model_landmark_to_the_side():
    # Robot at origin facing +x, landmark directly to the left (+y).
    state = np.array([[0.0], [0.0], [0.0]])
    landmark = np.array([[0.0], [3.0]])
    z = EKF.observation_model(state, landmark)
    assert z[0, 0] == pytest.approx(3.0)
    assert z[1, 0] == pytest.approx(math.pi / 2)
 
 
def test_ekf_estimate_perfect_measurement_matches_prediction():
    # If z exactly equals the predicted observation, the innovation
    # should be ~0 and the update should leave xEst == xPred.
    xEst = np.array([[0.0], [0.0], [0.0]])
    PEst = np.eye(3) * 0.5
    u = np.array([[1.0], [0.1]])
    landmark = np.array([[5.0], [2.0]])
 
    xPred = EKF.motion_model(xEst, u)
    z = EKF.observation_model(xPred, landmark)   # noiseless "perfect" measurement
 
    xNew, PNew = EKF.EKF_estimate(xEst, PEst, u, z, landmark)
 
    assert np.allclose(xNew, xPred, atol=1e-8)
    # Covariance should shrink (update always reduces uncertainty here
    # since R is finite, not infinite).
    assert np.all(np.diag(PNew) <= np.diag(PEst))
 
 
if __name__ == "__main__":
    import sys
    sys.exit(pytest.main([__file__, "-v"]))