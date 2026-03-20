import numpy as np


def trajectory_sample(px, py, times, t):
    n_seg = px.shape[0]


    # HANDLE OUT-OF-BOUNDS TIME
    if t <= times[0]:
        i = 0
        tau = 0.0

    elif t >= times[-1]:
        i = n_seg - 1
        tau = 1.0

    else:
        # find segment
        for i in range(n_seg):
            if times[i] <= t <= times[i+1]:
                break

        T = times[i+1] - times[i]
        tau = (t - times[i]) / (T + 1e-6)

    tau = np.clip(tau, 0.0, 1.0)


    # POLYNOMIAL EVALUATION
    basis = np.array([tau**k for k in range(8)])

    x = px[i] @ basis
    y = py[i] @ basis

    return x, y