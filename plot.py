import visdom
#import pandas as pd
import numpy as np
import matplotlib.pylab as plt
import collections

from udacidrone import Drone

# for matplotlib, projection='3d' we need to import Axes3d
from mpl_toolkits.mplot3d import Axes3D

# def read_using_pandas(log_file):
#     data = pd.read_csv(log_file, header = None, usecols=[0,2,3,4])

#     local_index = data[0] == 'MsgID.LOCAL_POSITION'
#     data = data[local_index]

#     # remove the first column that has msgId in it
#     data = data.drop(0, axis=1)
#     # specify names for each column
#     data.columns = ['north', 'east', 'height']

#     # change height from -ve to +ve and return as float
#     data = data.applymap(float)
#     data.height *= -1.0

#     return data

def read_local_position(log_file):
    data = Drone.read_telemetry_data(log_file)
    data = data['MsgID.LOCAL_POSITION']

    PlotData = collections.namedtuple('PltoData', ["north", "east", "height"])
    plot_data = PlotData(north = data[1], east = data[2], height = data[3] * -1.0)
    return plot_data

def create_plot(data):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(data.east, data.north, data.height)
    ax.set_xlabel('East')
    ax.set_ylabel('North')
    ax.set_zlabel('Height')

def show_as_matplotlib(data):
    create_plot(data)
    plt.show()

def show_visdom(data):
    viz = visdom.Visdom()
    create_plot(data)
    
    assert viz.check_connection()
    viz.matplot(plt)

data = read_local_position('Logs\TLog.txt')
#show_as_matplotlib(data)
show_visdom(data)