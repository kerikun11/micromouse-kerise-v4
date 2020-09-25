#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ============================================================================ #
import serial
import datetime
import os
import numpy as np
import matplotlib.pyplot as plt
import argparse
from matplotlib.ticker import ScalarFormatter


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


def process(filename):
    # load
    raw = np.loadtxt(filename, delimiter='\t')
    dt = 1e-3
    t = dt * np.arange(raw.shape[0])
    v_tra = raw[:, 0:2]
    a_tra = raw[:, 2:4]
    u_tra = raw[:, 4:8]
    v_rot = raw[:, 8:10]
    a_rot = raw[:, 10:12]
    u_rot = raw[:, 12:16]
    x = raw[:, 16:18]
    y = raw[:, 18:20]
    th = raw[:, 20:22]

    # calculate input sum
    u_tra = np.hstack(
        (u_tra, np.sum(u_tra, axis=1).reshape(u_tra.shape[0], 1)))
    u_rot = np.hstack(
        (u_rot, np.sum(u_rot, axis=1).reshape(u_rot.shape[0], 1)))

    # plot
    fig, axs = plt.subplots(4, 1, figsize=(8, 10))
    ylabels = ['vel. [m/s]', 'pwm input',
               'vel. [rad/s]', 'pwm input']
    titles = [
        'Translational Velocity',
        'Translational PWM Input',
        'Rotational Velocity',
        'Rotational PWM Input',
    ]
    data = [v_tra, u_tra, v_rot, u_rot]
    for i in range(axs.size):
        ax = axs[i]
        ax.plot(t, data[i], lw=2)
        ax.set_ylabel(ylabels[i])
        ax.set_title(titles[i])
        ax.grid()
        ax.yaxis.set_major_formatter(ScalarFormatter(useMathText=True))
        ax.ticklabel_format(style="sci", axis="y", scilimits=(0, 0))
        ax.set_xlabel('Time [ms]')

    legends = ['FF', 'FB p', 'FB i', 'FB d', 'FF+FB']
    axs[int(axs.size/2)-1].legend(legends, ncol=5)
    axs[-1].legend(legends, ncol=5)
    legends = ['Reference', 'Estimated']
    axs[0].legend(legends)
    axs[int(axs.size/2)].legend(legends)

    # fit
    fig.tight_layout()

    # plot xy
    fig_xt = plt.figure(figsize=(8, 6))
    plt.plot(x, y)
    ax = plt.gca()
    ax.grid(which='major', linestyle='-')
    ax.grid(which='minor', linestyle=':')
    ax.set_xticks(np.arange(-360, 360, 15))
    ax.set_xticks(np.arange(-360, 360, 5), minor=True)
    ax.set_yticks(ax.get_xticks())
    ax.set_yticks(ax.get_xticks(minor=True), minor=True)
    plt.axis('equal')
    plt.xlabel('x [mm]')
    plt.ylabel('y [mm]')

    # save
    for ext in [
        # '.png',
        '.svg',
    ]:
        fig.savefig(filename + ext)

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
