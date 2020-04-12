#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ============================================================================ #
import numpy as np
import matplotlib.pyplot as plt
import argparse
from matplotlib.ticker import ScalarFormatter

parser = argparse.ArgumentParser()
parser.add_argument("--src")
args = parser.parse_args()

# ============================================================================ #
# file select
filename = './KERISEv4/14 tra tra FB rot FB/190811-154249.tab'
if args.src:
    filename = args.src

# ============================================================================ #
# load
raw = np.loadtxt(filename, delimiter='\t')
dt = 1e-3
t = dt * np.arange(raw.shape[0])
v_tra = raw[:, 0:2]
a_tra = raw[:, 2:4]
u_tra = raw[:, 4:9]
v_rot = raw[:, 9:11]
a_rot = raw[:, 11:13]
u_rot = raw[:, 13:18]

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

legends = ['FF', 'FB', 'FB p', 'FB i', 'FF+FB']
axs[int(axs.size/2)-1].legend(legends, ncol=5)
axs[-1].legend(legends, ncol=5)
legends = ['Reference', 'Estimated']
axs[0].legend(legends)
axs[int(axs.size/2)].legend(legends)

fig.tight_layout()

# ============================================================================ #
# show
plt.show()
