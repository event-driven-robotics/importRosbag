# -*- coding: utf-8 -*-

"""
Copyright (C) 2019 Event-driven Perception for Robotics
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
        http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
    the result is are np arrays of float64 for:
        rotQ (4 cols, quaternion)
        angV (3 cols)
        acc (3 cols)
        mag (3 cols)
        temp (1 cols) - but I'll probably ignore this to start with
    '''
    tsAll = []
    rotQAll = []
    angVAll = []
    accAll = []
    magAll = []
    data_start_idx = np.inf
    data_end_idx = -np.inf
    #tempAll = np.zeros((sizeOfArray, 1), dtype=np.float64)
    for idx, msg in enumerate(tqdm(msgs, position=0, leave=True)):
        # TODO: maybe implement kwargs['useRosMsgTimestamps']
        try:
            data = msg['data']
            #seq = unpack('=L', data[0:4])[0]
            ts_new, ptr = unpackRosTimestamp(data, 4)
            frame_id, ptr = unpackRosString(data, ptr)
            rotQ_new, ptr = unpackRosFloat64Array(data, 4, ptr)
            rotQAll.append(rotQ_new)
            ptr += 72 # Skip the covariance matrix
            angV_new, ptr = unpackRosFloat64Array(data, 3, ptr)
            angVAll.append(angV_new)
            ptr += 72 # Skip the covariance matrix
            acc_new, ptr = unpackRosFloat64Array(data, 3, ptr)
            accAll.append(acc_new)
            #ptr += 24
            #ptr += 72 # Skip the covariance matrix
            data_start_idx = min(idx, data_start_idx)
            data_end_idx = max(idx, data_end_idx)
        except KeyError:
            ts_new, _ = unpackRosTimestamp(msg['time'], 0)
        tsAll.append(ts_new)
    outDict = {
        'ts': tsAll,
        'rotQ': rotQAll,
        'angV': angVAll,
        'acc': accAll,
        'mag': magAll,
        'start_idx': data_start_idx,
        'end_idx': data_end_idx
        }
    return outDict
