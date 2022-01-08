#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ============================================================================ #
import serial
import datetime
import os
import math
import argparse
import numpy as np
import matplotlib.pyplot as plt


def serial_import(filename, serial_port, serial_baudrate):
    with serial.Serial(serial_port, serial_baudrate, timeout=10) as ser:
        ser.flush()
        print(f'serial port {serial_port} ({serial_baudrate}) listening...')
        firstline = ser.readline()
        if not firstline:
            print('serial import timeout :(')
            exit(1)
        ser.timeout = 0.1  # shorten timeout after first line
        lines = ser.readlines()
        lines.insert(0, firstline)
        # save to file
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        with open(filename, 'w') as f:
            for line in lines:
                f.write(line.decode())


def export_fig(name, fig=plt):
    # fit
    fig.tight_layout()
    # save
    for ext in ['.svg']:
        fig.savefig(filename + '_' + name + ext)


def plt_label(title, xlabel, ylabel):
    plt.grid()
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.tight_layout()


def iir_tra(enc, acc, u, T1, K1, dt, p_enc, p_acc, p_mod):
    if p_mod == None:
        p_mod = 1 - p_enc - p_acc
    if p_acc == None:
        p_acc = 1 - p_enc - p_mod
    n = len(enc)
    out = np.zeros(n)
    out[0] = enc[0]
    for i in range(1, n):
        v_acc = out[i-1] + acc[i-1] * dt
        A = math.exp(-dt/T1)
        v_mod = A * out[i-1] + K1 * (1-A) * u[i-1]
        out[i] = p_enc * enc[i] + p_acc * v_acc + p_mod * v_mod
    return out


def iir_rot(enc, gyr, acc, u, T1, K1, dt, p_enc, p_gyr, p_acc, p_mod):
    if p_mod == None:
        p_mod = 1 - p_enc - p_acc - p_gyr
    if p_acc == None:
        p_acc = 1 - p_enc - p_mod - p_gyr
    n = len(enc)
    out = np.zeros(n)
    out[0] = (enc[0] * p_enc + gyr[0] * p_gyr) / (p_enc + p_gyr)
    for i in range(1, n):
        v_acc = out[i-1] + acc[i-1] * dt
        A = math.exp(-dt/T1)
        v_mod = A * out[i-1] + K1 * (1-A) * u[i-1]
        out[i] = p_enc * enc[i] + p_gyr * \
            gyr[i-1] + p_acc * v_acc + p_mod * v_mod
    return out


def calc_x_y_th(v, w, dt):
    n = len(v)
    x, y = np.zeros(n), np.zeros(n)
    th = np.zeros(n)
    for i in range(1, n):
        th[i] = th[i-1] + w[i-1] * dt
        x[i] = x[i-1] + v[i-1] * math.cos(th[i]) * dt
        y[i] = y[i-1] + v[i-1] * math.sin(th[i]) * dt
    return x, y, th


def process(filename):
    # load
    raw = np.loadtxt(filename, delimiter='\t')
    raw = raw.T  # row based
    enc_raw = raw[0:2]
    gyro = raw[2]
    accel = raw[3:5]
    # voltage = raw[5]
    u_pwm = raw[5:7]
    # u_pwm = np.vstack((
    #     0.0 * np.ones(len(raw[0])),
    #     0.4 * np.ones(len(raw[0]))
    # ))

    # plot battery voltage
    # plt.figure()
    # plt.plot(voltage.T)
    # plt_label('Battery Voltage', 'sample', 'Voltage [V]')

    # constants
    dt = 1e-3  # [s]
    machine_rotation_radius = 18  # [mm]

    # calc diff; [L, R, (L+R)/2, (R-L)/2]
    enc_diff = np.diff(enc_raw, axis=1)
    enc_diff = np.vstack((enc_diff, (enc_diff[0] + enc_diff[1])/2))
    enc_diff = np.vstack((enc_diff, (enc_diff[1] - enc_diff[0])/2))
    v_enc = enc_diff[2] / dt
    w_enc = enc_diff[3] / dt / machine_rotation_radius
    n = len(enc_diff[0])

    # plot encoder difference
    plt.figure()
    plt.plot(enc_diff.T)
    plt_label('Diff of Encoder Value', 'sample', 'Angular Velocity [deg/s]')
    plt.legend(['Left', 'Right', '(+Left+Right)/2', '(-Left+Right)/2'])
    # export_fig('enc_diff')

    # rotational velocity
    T1 = 0.1499
    K1 = 66.72
    w_gyr = gyro
    acc = accel[1]
    u = u_pwm[1]
    w_cmp_a = iir_rot(w_enc, w_gyr, acc, u, T1, K1, dt,
                      p_enc=0.0, p_gyr=0.5, p_acc=None, p_mod=0)
    w_cmp_m = iir_rot(w_enc, w_gyr, acc, u, T1, K1, dt,
                      p_enc=0.0, p_gyr=0.5, p_acc=0, p_mod=None)
    w_cmp_eam = iir_rot(w_enc, w_gyr, acc, u, T1, K1, dt,
                        p_enc=0.0, p_gyr=0.2, p_acc=0.4, p_mod=0.4)
    # plot
    plt.figure()
    plt.plot(w_enc)
    plt.plot(w_gyr)
    plt.plot(w_cmp_a)
    plt.plot(w_cmp_m)
    plt.plot(w_cmp_eam)
    plt_label('Rotational Velocity', 'Time [ms]', 'Angular Velocity [deg/s]')
    plt.legend([
        'Encoder',
        'IMU Gyro',
        'Gyro + Accel',
        'Gyro + Model',
        'Gyro + Accel + Model',
    ])

    # translational velocity
    T1 = 0.3694
    K1 = 5833
    acc = accel[0]
    u = u_pwm[0]
    v_cmp_a = iir_tra(v_enc, acc, u, T1, K1, dt,
                      p_enc=0.05, p_acc=None, p_mod=0)
    v_cmp_m = iir_tra(v_enc, acc, u, T1, K1, dt,
                      p_enc=0.05, p_acc=0, p_mod=None)
    v_cmp_eam = iir_tra(v_enc, acc, u, T1, K1, dt,
                        p_enc=0.1, p_acc=0.45, p_mod=None)
    # plot
    plt.figure()
    plt.plot(v_enc)
    plt.plot(v_cmp_a)
    plt.plot(v_cmp_m)
    plt.plot(v_cmp_eam)
    plt_label('Translational Velocity', 'Time [ms]', 'Velocity [mm/s]')
    plt.legend([
        'with Encoder',
        'with Encoder and Accel (IMU)',
        'with Encoder and Model (System Identification)',
        'with Encoder, Accel and Model',
    ], loc='best')

    # plot velocity
    plt.figure()
    plt.grid()
    plt.title('Translational and Rotational Velocity')
    plt.xlabel('Time [ms]')
    p1 = plt.plot(v_cmp_eam, 'C0')
    plt.ylabel('Translational Velocity [mm/s]')
    plt.gca().yaxis.label.set_color('C0')
    p2 = plt.twinx().plot(gyro, 'C1')
    plt.ylabel('Rotational Velocity [rad/s]')
    plt.gca().yaxis.label.set_color('C1')
    plt.tight_layout()

    # x-y plot
    plt.figure()
    x, y, th = calc_x_y_th(v_enc, w_gyr, dt)
    plt.plot(x, y)
    x, y, th = calc_x_y_th(v_cmp_a, w_gyr, dt)
    plt.plot(x, y)
    x, y, th = calc_x_y_th(v_cmp_m, w_gyr, dt)
    plt.plot(x, y)
    # x, y, th = calc_x_y_th(v_cmp_eam, w_gyr, dt)
    # plt.plot(x, y)
    plt_label('Planar Shape', 'x [mm]', 'y [mm]')
    ax = plt.gca()
    ax.grid(which='major', linestyle='-')
    ax.grid(which='minor', linestyle=':')
    ax.set_xticks(np.arange(-360, 360, 15))
    ax.set_xticks(np.arange(-360, 360, 5), minor=True)
    ax.set_yticks(ax.get_xticks())
    ax.set_yticks(ax.get_xticks(minor=True), minor=True)
    plt.axis('equal')
    plt.legend([
        'with Encoder',
        'with Encoder and Accel (IMU)',
        'with Encoder and Model (System Identification)',
        # 'with Encoder, Accel and Model',
    ], loc='best')
    plt.tight_layout()

    # show
    plt.show()


# parse args
parser = argparse.ArgumentParser()
parser.add_argument('files', help="csv data file list", nargs='*')
parser.add_argument("--port", "-p", help="serial port", default='/dev/ttyUSB0')
parser.add_argument("--baud", "-b", help="serial baudrate", default=2_000_000)
args = parser.parse_args()

# get files from argument
files = args.files

# if no file is specified, import from serial
if not files:
    datetime_string = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    filename = 'data/' + datetime_string + '.csv'
    serial_import(filename, args.port, args.baud)
    files.append(filename)

# process all input data
for filename in files:
    process(filename)
