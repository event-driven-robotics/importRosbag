# -*- coding: utf-8 -*-

"""
Copyright (C) 2021 Event-driven Perception for Robotics
Authors: Sim Bamford
This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Intended as part of importRosbag.

The importTopic function receives a list of messages and returns
a dict with one field for each data field in the message, where the field
will contain an appropriate iterable to contain the interpretted contents of each message.
In some cases, static info is repeated in each message; in which case a field may not contain an iterable. 

This function imports the ros message type defined at:
http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
"""

#%%

from tqdm import tqdm
import numpy as np

from .common import unpackRosString, unpackRosTimestamp, unpackRosFloat64Array

def importTopic(msgs, **kwargs):
    '''
    ros message is defined here:
        http://docs.ros.org/en/api/sensor_msgs/html/msg/Illuminance.html
    the result is are np arrays of float64 for:
        illuminance (1 col)
    '''
    sizeOfArray = 1024
    tsAll = np.zeros((sizeOfArray), dtype=np.float64)
    illuminanceAll = np.zeros((sizeOfArray, 1), dtype=np.float64)
    for idx, msg in enumerate(tqdm(msgs, position=0, leave=True)):
        if sizeOfArray <= idx:
            tsAll = np.append(tsAll, np.zeros((sizeOfArray), dtype=np.float64))
            illuminanceAll = np.concatenate((illuminanceAll, np.zeros((sizeOfArray, 1), dtype=np.float64)))
            sizeOfArray *= 2
        data = msg['data']
        #seq = unpack('=L', data[0:4])[0]
        tsAll[idx], ptr = unpackRosTimestamp(data, 4)
        frame_id, ptr = unpackRosString(data, ptr)
        illuminanceAll[idx, :], ptr = unpackRosFloat64Array(data, 1, ptr)
        ptr += 8 # Skip the covariance matrix
    numEvents = idx + 1
    # Crop arrays to number of events
    tsAll = tsAll[:numEvents]
    illuminanceAll = illuminanceAll[:numEvents]
    outDict = {
        'ts': tsAll,
        'illuminance': illuminanceAll,
        }
    return outDict
