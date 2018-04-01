import visdom

import numpy as np
import matplotlib.pylab as plt
from mpl_toolkits.mplot3d import Axes3D

from udacidrone import Drone
from udacidrone.messaging import MsgID

viz = visdom.Visdom()
assert viz.check_connection()

d = Drone.read_telemetry_data('Logs\TLog.txt')
local = d['MsgID.LOCAL_POSITION']

north = np.array(local[1])
east = np.array(local[2])
height = np.array(local[3])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(east, north, height)
ax.set_xlabel('East')
ax.set_ylabel('North')

viz.matplot(plt)