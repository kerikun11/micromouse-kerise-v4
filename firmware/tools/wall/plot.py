#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ============================================================================ #
from numpy.core.fromnumeric import size
import serial  # pip install pyserial
import datetime
import os
import numpy as np
import matplotlib.pyplot as plt
import argparse


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


def process(filename, show):
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

    # plot [v_tra, v_rot]
    data = [v_tra, v_rot]
    ylabels = ['trans. vel. [mm/s]', 'rot. vel. [rad/s]']
    fig, axs = plt.subplots(len(data), 1, tight_layout=True, sharex=True)
    for i, ax in enumerate(axs):
        ax.plot(t, data[i])
        ax.set_ylabel(ylabels[i])
        ax.grid()
        ax.legend(['Reference', 'Estimated'])
    axs[-1].set_xlabel('Time [s]')
    plt.suptitle("Translational and Rotational Velocity")
    save_fig('v')
    # return plt.show()

    # plot xy
    fig, ax = plt.subplots(tight_layout=True)
    ax.plot(x, y)
    ax.grid(which='major', linestyle='-')
    ax.grid(which='minor', linestyle=':')
    ax.set_xticks(np.arange(-90*32, 90*32, 15))
    ax.set_xticks(np.arange(-90*32, 90*32, 5), minor=True)
    ax.set_yticks(ax.get_xticks())
    ax.set_yticks(ax.get_xticks(minor=True), minor=True)
    plt.axis('equal')
    plt.title('x-y shape')
    plt.xlabel('x [mm]')
    plt.ylabel('y [mm]')
    plt.legend(['Reference', 'Estimated'])
    save_fig('xy')
    # return plt.show()

    # plot ref
    plt.figure(tight_layout=True)
    plt.plot(x[:, 0], ref)
    plt.title('Reflector Raw Value')
    plt.xlabel('Translational Position [mm]')
    plt.ylabel('Reflector Raw Value')
    plt.legend(['Left Side', 'Left Front', 'Right Front', 'Right Side'])
    plt.grid()
    save_fig('ref')
    # return plt.show()

    # plot wd
    plt.figure(tight_layout=True)
    plt.plot(x[:, 0], wd)
    plt.title('Wall Distance [mm]')
    plt.xlabel('Translational Position [mm]')
    plt.ylabel('Wall Distance [mm]')
    plt.legend(['Left Side', 'Left Front', 'Right Front', 'Right Side'])
    plt.grid()
    save_fig('wd')
    # return plt.show()

    # plot ref and tof
    fig, axs = plt.subplots(2, 1, tight_layout=True, sharex=True)
    plt.suptitle("Reflector Raw Value and ToF Distance")
    titles = ['Reflector Raw Value', 'Front Wall Distance [mm]']
    ylabels = ['reflector value', 'wall distance [mm]']
    legends = ['L Side', 'L Front', 'R Front', 'R Side']
    data = [ref, tof]
    for i, ax in enumerate(axs):
        ax.plot(x[:, 0], data[i])
        ax.set_title(titles[i])
        ax.set_ylabel(ylabels[i])
        ax.grid()
        ax.legend(legends)
    ax.legend(['ToF'])
    plt.xlabel('Translational Position [mm]')
    save_fig('ref_tof')
    # return plt.show()

    # plot ref and wd
    fig, axs = plt.subplots(2, 1, tight_layout=True, sharex=True)
    plt.suptitle("Reflector Raw Value and Wall Distance")
    titles = ['Reflector Raw Value', 'Wall Distance [mm]']
    ylabels = ['reflector value', 'wall distance [mm]']
    legends = ['L Side', 'L Front', 'R Front', 'R Side']
    data = [ref, wd]
    for i, ax in enumerate(axs):
        ax.plot(x[:, 0], data[i])
        ax.set_title(titles[i])
        ax.set_ylabel(ylabels[i])
        ax.grid()
        ax.legend(legends)
    plt.xlabel('Translational Position [mm]')
    save_fig('ref_wd')
    # return plt.show()

    # plot side wd
    plt.figure(tight_layout=True)
    plt.plot(x[:, 0], wd[:, [0, 3]])
    plt.title('Side Wall Distance [mm]')
    plt.xlabel('Translational Position [mm]')
    plt.ylabel('Wall Distance [mm]')
    plt.legend(['L', 'R'])
    plt.grid()
    save_fig('side_wd')
    # return plt.show()

    # show
    if show:
        plt.show()


def save_fig(suffix, fig=None):
    if not fig:
        fig = plt.gcf()
    # save
    for ext in [
        # '.png',
        '.svg',
    ]:
        fig.savefig(os.path.splitext(filename)[0] + '_' + suffix + ext)


# parse args
parser = argparse.ArgumentParser()
parser.add_argument('files', help="csv data file list", nargs='*')
parser.add_argument("--port", "-p", help="serial port", default='/dev/ttyUSB0')
parser.add_argument("--baud", "-b", help="serial baudrate", default=2_000_000)
parser.add_argument("--show", "-s", help="show figure", type=int, default=1)
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
    process(filename, args.show)
