"""Standalone IMU/GNSS fusion routine used for early experimentation.

Usage:
    python fusion_single.py --imu-file IMU.dat --gnss-file GNSS.csv [options]

The script implements attitude initialisation, a simple Kalman filter and an
optional RTS smoother in a single file. Command&ndash;line flags allow tuning of
process and measurement noise, bias dynamics and GNSS weighting.  Results and
diagnostic figures are saved to ``results/``.  This mirrors the MATLAB function
``GNSS_IMU_Fusion_single.m`` for cross-language parity.
"""

import argparse
import os
import logging

from dependency_bootstrap import ensure_dependencies

ensure_dependencies()

import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from kalman import GNSSIMUKalman, rts_smoother
from utils import compute_C_ECEF_to_NED, zero_base_time
from constants import GRAVITY, EARTH_RATE
from paths import (
    imu_path as _imu_path_helper,
    gnss_path as _gnss_path_helper,
    ensure_results_dir as _ensure_results,
    normalize_gnss_headers,
)

logging.basicConfig(level=logging.INFO, format="%(message)s")


def butter_lowpass_filter(data, cutoff=5.0, fs=400.0, order=4):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype="low")
    return filtfilt(b, a, data, axis=0)



def quat_mult(q, r):
    w0, x0, y0, z0 = q
    w1, x1, y1, z1 = r
    return np.array([
        w0*w1 - x0*x1 - y0*y1 - z0*z1,
        w0*x1 + x0*w1 + y0*z1 - z0*y1,
        w0*y1 - x0*z1 + y0*w1 + z0*x1,
        w0*z1 + x0*y1 - y0*x1 + z0*w1,
    ])


def quat_from_rate(omega, dt):
    theta = np.linalg.norm(omega) * dt
    if theta == 0:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = omega / np.linalg.norm(omega)
    half = theta / 2.0
    return np.array([np.cos(half), *(np.sin(half) * axis)])


def quat_to_rot(q):
    w, x, y, z = q
    return np.array([
        [1-2*(y**2+z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1-2*(x**2+z**2), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1-2*(x**2 + y**2)],
    ])


def main():
    _ensure_results()
    logging.info("Ensured 'results/' directory exists.")
    parser = argparse.ArgumentParser()
    parser.add_argument('--imu-file', default='IMU_X001.dat')
    parser.add_argument('--gnss-file', default='GNSS_X001.csv')
    parser.add_argument('--gnss_weight', type=float, default=1.0)
    parser.add_argument('--accel_bias_noise', type=float, default=1e-5)
    parser.add_argument('--gyro_bias_noise', type=float, default=1e-5)
    parser.add_argument('--accel_noise', type=float, default=0.1)
    parser.add_argument('--pos_noise', type=float, default=0.0,
                        help='Process noise for position states')
    parser.add_argument('--vel_noise', type=float, default=0.0,
                        help='Additional process noise for velocity states')
    parser.add_argument('--pos_meas_noise', type=float, default=1.0,
                        help='Measurement noise for GNSS position')
    parser.add_argument(
        '--vel_meas_noise',
        type=float,
        default=3.0,
        help='Measurement noise for GNSS velocity (std dev, m/s)',
    )
    parser.add_argument('--static_window', type=int, default=400)
    parser.add_argument('--smoother', action='store_true')
    args = parser.parse_args()

    gnss = pd.read_csv(str(_gnss_path_helper(args.gnss_file)))
    gnss = normalize_gnss_headers(gnss)
    imu = np.loadtxt(str(_imu_path_helper(args.imu_file)))

    gnss_time = zero_base_time(gnss['Posix_Time'].values)
    pos_ecef = gnss[['X_ECEF_m','Y_ECEF_m','Z_ECEF_m']].values
    vel_ecef = gnss[['VX_ECEF_mps','VY_ECEF_mps','VZ_ECEF_mps']].values

    # reference point
    lat0, lon0, _ = 0.0, 0.0, 0.0
    valid = (pos_ecef[:,0]!=0)|(pos_ecef[:,1]!=0)|(pos_ecef[:,2]!=0)
    if valid.any():
        idx0 = np.argmax(valid)
        x0,y0,z0 = pos_ecef[idx0]
        a = 6378137.0
        e_sq = 6.69437999014e-3
        p = np.sqrt(x0**2 + y0**2)
        theta = np.arctan2(z0*a, p*(1-e_sq))
        lon0 = np.arctan2(y0,x0)
        lat0 = np.arctan2(z0 + e_sq*a*np.sin(theta)**3/(1-e_sq), p - e_sq*a*np.cos(theta)**3)
    C = compute_C_ECEF_to_NED(lat0, lon0)
    ref = pos_ecef[idx0]
    pos_ned = np.array([C @ (r - ref) for r in pos_ecef])
    vel_ned = np.array([C @ v for v in vel_ecef])

    dt_imu = np.mean(np.diff(imu[:100,1]))
    imu_time = np.arange(len(imu)) * dt_imu
    gyro = imu[:,2:5]/dt_imu
    acc = imu[:,5:8]/dt_imu
    acc = butter_lowpass_filter(acc)
    gyro = butter_lowpass_filter(gyro)

    win = args.static_window
    mags = np.linalg.norm(acc, axis=1)
    stds = np.array([mags[i:i+win].std() for i in range(len(mags)-win)])
    idx = np.argmin(stds)
    start, end = idx, idx+win
    static_acc = acc[start:end].mean(axis=0)
    static_gyro = gyro[start:end].mean(axis=0)

    g_ned = np.array([0, 0, GRAVITY])
    omega_ned = EARTH_RATE * np.array([np.cos(lat0), 0, -np.sin(lat0)])

    v1_b = -static_acc/np.linalg.norm(static_acc)
    v2_b = static_gyro/np.linalg.norm(static_gyro) if np.linalg.norm(static_gyro)>0 else np.array([1,0,0])
    v1_n = g_ned/np.linalg.norm(g_ned)
    v2_n = omega_ned/np.linalg.norm(omega_ned)
    w1,w2 = 0.9999,0.0001
    B = w1*np.outer(v1_n,v1_b)+w2*np.outer(v2_n,v2_b)
    S = B+B.T
    sigma = np.trace(B)
    Z = np.array([B[1,2]-B[2,1],B[2,0]-B[0,2],B[0,1]-B[1,0]])
    K = np.zeros((4,4))
    K[0,0]=sigma
    K[0,1:]=Z
    K[1:,0]=Z
    K[1:,1:]=S-sigma*np.eye(3)
    eigval,eigvec=np.linalg.eigh(K)
    q = eigvec[:,np.argmax(eigval)]
    if q[0]<0:
        q=-q
    q=np.array([q[0],-q[1],-q[2],-q[3]])

    Cbn = quat_to_rot(q).T
    acc_bias = static_acc + Cbn@g_ned
    gyro_bias = static_gyro - Cbn@omega_ned

    kf = GNSSIMUKalman(
        args.gnss_weight,
        accel_noise=args.accel_noise,
        accel_bias_noise=args.accel_bias_noise,
        gyro_bias_noise=args.gyro_bias_noise,
        pos_proc_noise=args.pos_noise,
        vel_proc_noise=args.vel_noise,
        pos_meas_noise=args.pos_meas_noise,
        vel_meas_noise=args.vel_meas_noise,
    )
    kf.init_state(pos_ned[0], vel_ned[0], acc_bias, gyro_bias)

    N = len(imu)
    x_hist = np.zeros((N,12))
    P_hist = np.zeros((N,12,12))
    F_list=[]
    Q_list=[]
    q_cur=q.copy()

    # containers for analysis
    pos_res = []
    vel_res = []
    norm_pos_res = []
    norm_vel_res = []
    upd_times = []
    quats = []

    gnss_idx=0
    for i in range(N):
        if i>0:
            dt=imu_time[i]-imu_time[i-1]
        else:
            dt=dt_imu
        omega=gyro[i]-kf.kf.x[9:12]
        dq=quat_from_rate(omega,dt)
        q_cur=quat_mult(q_cur,dq)
        q_cur/=np.linalg.norm(q_cur)
        quats.append(q_cur.copy())
        R_bn=quat_to_rot(q_cur)
        kf.predict(dt,R_bn,acc[i],g_ned)
        if gnss_idx<len(gnss_time)-1 and abs(imu_time[i]- gnss_time[gnss_idx])<dt_imu/2:
            x_upd, resid, S = kf.update_gnss(pos_ned[gnss_idx],vel_ned[gnss_idx])
            upd_times.append(imu_time[i])
            pos_res.append(resid[0:3])
            vel_res.append(resid[3:6])
            std = np.sqrt(np.diag(S))
            norm_pos_res.append(resid[0:3]/std[0:3])
            norm_vel_res.append(resid[3:6]/std[3:6])
            gnss_idx+=1
        x_hist[i]=kf.kf.x
        P_hist[i]=kf.kf.P
        F_list.append(kf.kf.F)
        Q_list.append(kf.kf.Q)

    if args.smoother:
        xs, Ps = rts_smoother(x_hist,P_hist,F_list,Q_list)
        x_hist=xs

    north_jump=x_hist[0,0]
    print(f"Plotted Davenport position North: First = {north_jump:.4f}")

    # convert logs to arrays
    pos_res = np.asarray(pos_res)
    vel_res = np.asarray(vel_res)
    norm_pos_res = np.asarray(norm_pos_res)
    norm_vel_res = np.asarray(norm_vel_res)
    upd_times = np.asarray(upd_times)
    quats = np.asarray(quats)

    # attitude angles
    if len(quats) > 0:
        rot = R.from_quat(quats[:, [1,2,3,0]])
        euler = rot.as_euler('xyz', degrees=True)
        t = imu_time[:len(quats)]

        fig, ax = plt.subplots(3,1,sharex=True)
        lbl = ['roll','pitch','yaw']
        for i in range(3):
            ax[i].plot(t, euler[:,i], label=lbl[i])
            ax[i].set_ylabel('deg')
            ax[i].legend()
        ax[-1].set_xlabel('Time (s)')
        fig.suptitle('Attitude Angles Over Time')
        plt.tight_layout()
        from utils.matlab_fig_export import save_matlab_fig
        save_matlab_fig(fig, 'results/attitude_angles')
        plt.close()

    if len(pos_res) > 0:
        labels = ['North','East','Down']
        fig, ax = plt.subplots(2,1,sharex=True)
        for i in range(3):
            ax[0].plot(upd_times, pos_res[:,i], label=labels[i])
            ax[1].plot(upd_times, vel_res[:,i], label=labels[i])
        ax[0].set_ylabel('Pos residual (m)')
        ax[1].set_ylabel('Vel residual (m/s)')
        ax[1].set_xlabel('Time (s)')
        ax[0].legend()
        ax[1].legend()
        fig.suptitle('Position & Velocity Residuals')
        plt.tight_layout()
        save_matlab_fig(fig2, 'results/residual_timeseries')
        plt.close()

        fig, axs = plt.subplots(2,3, figsize=(12,6))
        for i in range(3):
            axs[0,i].hist(pos_res[:,i], bins=50)
            axs[0,i].set_title(f'Pos {labels[i]}')
            axs[1,i].hist(vel_res[:,i], bins=50)
            axs[1,i].set_title(f'Vel {labels[i]}')
        fig.suptitle('Residual Distributions')
        plt.tight_layout()
        save_matlab_fig(fig3, 'results/residual_hist')
        plt.close()

        for name, data in [('Pos N', pos_res[:,0]),('Pos E', pos_res[:,1]),('Pos D', pos_res[:,2]),
                           ('Vel N', vel_res[:,0]),('Vel E', vel_res[:,1]),('Vel D', vel_res[:,2])]:
            print(f"{name}: mean={data.mean():.3f}, std={data.std():.3f}")

if __name__=='__main__':
    main()
