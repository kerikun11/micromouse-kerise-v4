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


def process(filename):
    # load
    raw = np.loadtxt(filename, delimiter='\t')
    raw = raw.T  # row based
    enc_raw = raw[0:2]
    gyro = raw[2]
    accel = raw[3:5]
    voltage = raw[5]
    u = raw[6:8]
    # u = np.vstack((
    #     0.0 * np.ones(len(raw[0])),
    #     0.4 * np.ones(len(raw[0]))
    # ))

    # plot
    plt.figure()
    plt.plot(voltage.T)
    plt_label('Battery Voltage', 'sample', 'Voltage [V]')

    # constants
    dt = 1e-3  # [s]
    machine_rotation_radius = 18  # [mm]

    # calc diff; [L, R, (L+R)/2, (R-L)/2]
    enc_diff = np.diff(enc_raw, axis=1)
    enc_diff = np.vstack((enc_diff, (enc_diff[0] + enc_diff[1])/2))
    enc_diff = np.vstack((enc_diff, (enc_diff[1] - enc_diff[0])/2))
    # plot
    plt.figure()
    plt.plot(enc_diff.T)
    plt_label('Diff of Encoder Value', 'sample', 'Angular Velocity [deg/s]')
    plt.legend(['Left', 'Right', '(+Left+Right)/2', '(-Left+Right)/2'])
    # export_fig('enc_diff')

    # rotational velocity
    w_enc = enc_diff[3]/dt/machine_rotation_radius
    w_gyr = gyro[0:-1]
    # IIR
    n = len(enc_diff[0])
    v_enc = enc_diff[2]/dt
    p_gyr = 0.0
    p_enc = 0.0
    p_acc = 0.0
    p_mod = 1 - p_enc - p_acc - p_gyr
    T1 = 0.1499
    K1 = 66.72
    w_eam = np.zeros(n)
    w_eam[0] = w_gyr[0]
    for i in range(1, n):
        w_acc = w_eam[i-1] + accel[1, i] * dt
        w_mod = math.exp(-dt/T1) * w_eam[i-1] + K1*(1-math.exp(-dt/T1))*u[1, i]
        w_eam[i] = p_gyr * w_gyr[i] + p_enc * \
            w_enc[i] + p_acc * w_acc + p_mod * w_mod
    # plot
    plt.figure()
    plt.plot(np.vstack((w_enc, w_gyr, w_eam)).T)
    plt_label('Rotational Velocity', 'Time [ms]', 'Angular Velocity [deg/s]')
    plt.legend(['Differential of Encoders', 'IMU Gyro', 'IIR Filtered'])

    # translational velocity
    # IIR
    n = len(enc_diff[0])
    v_enc = enc_diff[2]/dt
    p_enc = 0.0
    p_acc = 0.0
    p_mod = 1 - p_enc - p_acc
    T1 = 0.3694
    K1 = 5833
    v_eam = np.zeros(n)
    v_eam[0] = v_enc[0]
    for i in range(1, n):
        v_acc = v_eam[i-1] + accel[0, i] * dt
        v_mod = math.exp(-dt/T1) * v_eam[i-1] + K1*(1-math.exp(-dt/T1))*u[0, i]
        v_eam[i] = p_enc * v_enc[i] + p_acc * v_acc + p_mod * v_mod
    # plot
    v_tra = np.vstack((v_enc, v_eam))
    plt.figure()
    plt.plot(v_tra.T)
    plt_label('Translational Velocity', 'Time [ms]', 'Velocity [mm/s]')
    plt.legend(['Differential of Encoders', 'IIR Filtered'])

    # plot velocity
    plt.figure()
    plt.grid()
    plt.title('Translational and Rotational Velocity')
    plt.xlabel('Time [ms]')
    p1 = plt.plot(v_eam, 'C0')
    plt.ylabel('Translational Velocity [mm/s]')
    plt.gca().yaxis.label.set_color('C0')
    p2 = plt.twinx().plot(gyro, 'C1')
    plt.ylabel('Rotational Velocity [rad/s]')
    plt.gca().yaxis.label.set_color('C1')

    # x-y plot
    x, y = np.zeros(n), np.zeros(n)
    th = np.zeros(n)
    for i in range(1, n):
        v, w = v_eam[i], gyro[i]
        th[i] = th[i-1] + w * dt
        x[i] = x[i-1] + v * math.cos(th[i]) * dt
        y[i] = y[i-1] + v * math.sin(th[i]) * dt
    # print(f'({x[-1]}, {y[-1]}, {th[-1]}')
    # plot
    plt.figure()
    plt.plot(x, y)
    plt_label('Planar Shape', 'x [mm]', 'y [mm]')
    ax = plt.gca()
    ax.grid(which='major', linestyle='-')
    ax.grid(which='minor', linestyle=':')
    ax.set_xticks(np.arange(-360, 360, 15))
    ax.set_xticks(np.arange(-360, 360, 5), minor=True)
    ax.set_yticks(ax.get_xticks())
    ax.set_yticks(ax.get_xticks(minor=True), minor=True)
    plt.axis('equal')
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
