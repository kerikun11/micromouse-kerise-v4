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
    with serial.Serial(serial_port, serial_baudrate, timeout=30) as ser:
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
    # load csv
    raw = np.loadtxt(filename, delimiter='\t')
    dt = 1e-3
    t = dt * np.arange(raw.shape[0])
    v_tra = raw[:, 0:2]
    v_rot = raw[:, 2:4]
    x = raw[:, 4:6]
    y = raw[:, 6:8]
    th = raw[:, 8:10]
    ref = raw[:, 10:14]
    wd = raw[:, 14:18]
    tof = raw[:, 18:19]

    # plot velocity
    fig_v, axs = plt.subplots(2, 1, figsize=(8, 6))
    ylabels = ['trans. vel. [m/s]', 'rot. vel. [rad/s]']
    titles = ['Translational Velocity', 'Rotational Velocity']
    legends = ['Reference', 'Estimated']
    data = [v_tra, v_rot]
    for i in range(axs.size):
        ax = axs[i]
        ax.plot(t, data[i], lw=2)
        ax.set_ylabel(ylabels[i])
        ax.set_title(titles[i])
        ax.grid()
        ax.yaxis.set_major_formatter(ScalarFormatter(useMathText=True))
        ax.ticklabel_format(style="sci", axis="y", scilimits=(0, 0))
        ax.set_xlabel('Time [ms]')
        ax.legend(legends)
    plt.tight_layout()

    # plot xy
    fig_xy = plt.figure(figsize=(8, 6))
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
    plt.legend(['Reference', 'Estimated'])
    plt.tight_layout()

    # plot x_vs_wd
    fig_ref, axs = plt.subplots(2, 1, figsize=(8, 6))
    titles = ['Reflector Raw Value', 'Wall Distance']
    ylabels = ['reflector value', 'wall distance [mm]']
    legends = ['Left Side', 'Left Front', 'Right Front', 'Right Side']
    data = [ref, wd]
    for i in range(axs.size):
        ax = axs[i]
        ax.plot(x[:, 0], data[i], lw=2)
        ax.set_ylabel(ylabels[i])
        ax.set_title(titles[i])
        ax.grid()
        ax.set_xlabel('Translational Position (x) [mm]')
        ax.legend(legends)
    plt.tight_layout()

    # plot tof
    plt.figure(figsize=(8, 6))
    plt.plot(x[:, 0], np.hstack([wd, tof]))
    plt.xlabel('Translational Position (x) [mm]')
    plt.ylabel('Wall Distance [mm]')
    legends = ['Left Side', 'Left Front', 'Right Front', 'Right Side', 'ToF']
    plt.grid()
    plt.tight_layout()
    fig_tof = plt.gcf()

    # save
    for ext in [
        # '.png',
        '.svg',
    ]:
        fig_v.savefig(filename + '_v' + ext)
        fig_xy.savefig(filename + '_xy' + ext)
        fig_ref.savefig(filename + '_ref' + ext)
        fig_tof.savefig(filename + '_tof' + ext)

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
    print("filename: ", filename)
    process(filename)
