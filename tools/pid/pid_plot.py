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

parser = argparse.ArgumentParser()
parser.add_argument("--src")
args = parser.parse_args()

# ============================================================================ #
# file select
if args.src:
    filename = args.src

# ============================================================================ #
# serial read
if not args.src:
    datetime_string = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    filename = 'data/' + datetime_string + '.csv'
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    serial_port = '/dev/ttyUSB0'
    with serial.Serial(serial_port, 2_000_000, timeout=3) as ser:
        ser.flush()
        print('serial listening...')
        lines = ser.readlines()
        with open(filename, 'w') as f:
            for line in lines:
                # print(line.decode())
                f.write(line.decode())

# ============================================================================ #
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

# calculate input sum
u_tra = np.hstack((u_tra, np.sum(u_tra, axis=1).reshape(u_tra.shape[0], 1)))
u_rot = np.hstack((u_rot, np.sum(u_rot, axis=1).reshape(u_rot.shape[0], 1)))

# ============================================================================ #
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

# ============================================================================ #
# fit
fig.tight_layout()

# ============================================================================ #
# save
for ext in ['.png', '.svg']:
    fig.savefig(filename + ext)

# ============================================================================ #
# show
plt.show()
