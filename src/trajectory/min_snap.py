import numpy as np
import cvxpy as cp


# BASIS FUNCTIONS 
def poly_basis(tau, n=8):
    return np.array([tau**i for i in range(n)])


def poly_derivative_basis(tau, d, n=8):
    basis = np.zeros(n)
    for i in range(d, n):
        coef = 1
        for k in range(d):
            coef *= (i - k)
        basis[i] = coef * tau**(i - d)
    return basis



# MINIMUM SNAP 1D
def minimum_snap_1d(waypoints, times, corridors=None, dim='x'):

    waypoints = np.array(waypoints, dtype=float)
    n_seg = len(waypoints) - 1
    n_coeff = 8

    assert len(times) == n_seg + 1

    # decision variable
    p = cp.Variable(n_seg * n_coeff)

    # quadratic + linear cost
    H = np.zeros((n_seg * n_coeff, n_seg * n_coeff))
    f = np.zeros(n_seg * n_coeff)

    # COST (SNAP + TRACKING)
    for i in range(n_seg):

        idx = i * n_coeff
        T = max(times[i+1] - times[i], 1e-2)

        taus = np.linspace(0, 1, 20)
        dtau = 1.0 / len(taus)

        for tau in taus:

            #SNAP COST
            d4 = poly_derivative_basis(tau, 4) / (T**4)
            H[idx:idx+n_coeff, idx:idx+n_coeff] += dtau * np.outer(d4, d4)

            #SMOOTHING
            d1 = poly_derivative_basis(tau, 1) / T
            d2 = poly_derivative_basis(tau, 2) / (T**2)

            H[idx:idx+n_coeff, idx:idx+n_coeff] += dtau * (
                0.01 * np.outer(d1, d1) +
                0.1 * np.outer(d2, d2)
            )

            #POSITION TRACKING
            alpha = 10.0  # tuning parameter

            # linear interpolation reference
            ref = waypoints[i] + (waypoints[i+1] - waypoints[i]) * tau

            basis = poly_basis(tau)

            # quadratic term
            H[idx:idx+n_coeff, idx:idx+n_coeff] += dtau * alpha * np.outer(basis, basis)

            # linear term
            f[idx:idx+n_coeff] += -2 * dtau * alpha * ref * basis

    # numerical stability
    H += 1e-6 * np.eye(H.shape[0])

    # cost function
    cost = cp.quad_form(p, cp.psd_wrap(H)) + f @ p


    # SLACK VARIABLES
    if corridors is not None:
        slack = cp.Variable(n_seg, nonneg=True)
        cost += 1000 * cp.sum(slack)

    constraints = []


    # WAYPOINT CONSTRAINTS
    for i in range(n_seg):

        idx = i * n_coeff

        constraints.append(poly_basis(0) @ p[idx:idx+n_coeff] == waypoints[i])
        constraints.append(poly_basis(1) @ p[idx:idx+n_coeff] == waypoints[i+1])


    # CONTINUITY (C1, C2, C3)
    for i in range(n_seg - 1):

        idx1 = i * n_coeff
        idx2 = (i + 1) * n_coeff

        T1 = max(times[i+1] - times[i], 1e-2)
        T2 = max(times[i+2] - times[i+1], 1e-2)

        for d in range(1, 4):
            constraints.append(
                poly_derivative_basis(1, d) @ p[idx1:idx1+n_coeff] / (T1**d)
                ==
                poly_derivative_basis(0, d) @ p[idx2:idx2+n_coeff] / (T2**d)
            )


    # BOUNDARY CONDITIONS
    constraints += [
        poly_derivative_basis(0, 1) @ p[0:n_coeff] == 0,
        poly_derivative_basis(0, 2) @ p[0:n_coeff] == 0,
    ]

    idx_last = (n_seg - 1) * n_coeff
    T_last = max(times[-1] - times[-2], 1e-2)

    constraints += [
        poly_derivative_basis(1, 1) @ p[idx_last:idx_last+n_coeff] / T_last == 0,
        poly_derivative_basis(1, 2) @ p[idx_last:idx_last+n_coeff] / (T_last**2) == 0,
    ]


    # CORRIDOR CONSTRAINTS
    if corridors is not None:

        assert len(corridors) == n_seg

        for i in range(n_seg):

            idx = i * n_coeff

            xmin, xmax, ymin, ymax = corridors[i]

            margin = 0.1

            if dim == 'x':
                lower, upper = xmin - margin, xmax + margin
            else:
                lower, upper = ymin - margin, ymax + margin

            taus = np.linspace(0, 1, 15)

            for tau in taus:
                pos = poly_basis(tau) @ p[idx:idx+n_coeff]

                constraints += [
                    pos >= lower - slack[i],
                    pos <= upper + slack[i]
                ]


    # SOLVE
    prob = cp.Problem(cp.Minimize(cost), constraints)

    try:
        prob.solve(
            solver=cp.OSQP,
            warm_start=True,
            max_iter=200000,
            eps_abs=1e-5,
            eps_rel=1e-5,
            verbose=False
        )
    except:
        print("[WARN] OSQP failed → trying SCS")
        prob.solve(solver=cp.SCS, max_iters=20000, verbose=False)

    if prob.status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
        raise RuntimeError(f"Optimization failed! Status: {prob.status}")

    return p.value.reshape((n_seg, n_coeff))



# 2D WRAPPER
def minimum_snap_2d(path, times, corridors):

    path = np.array(path, dtype=float)

    assert len(times) == len(path)

    x_wp = path[:, 0]
    y_wp = path[:, 1]

    px = minimum_snap_1d(x_wp, times, corridors, dim='x')
    py = minimum_snap_1d(y_wp, times, corridors, dim='y')

    return px, py