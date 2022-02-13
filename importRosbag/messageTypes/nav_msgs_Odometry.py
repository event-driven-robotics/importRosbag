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
http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html  
"""

#%%

from tqdm import tqdm
import numpy as np

# Local imports

from .common import unpackRosUint32, unpackRosString, unpackRosTimestamp, unpackRosFloat64Array

def importTopic(msgs, **kwargs):
    sizeOfArray = 1024
    tsAll = np.zeros((sizeOfArray), dtype=np.float64)
    poseAll = np.zeros((sizeOfArray, 7), dtype=np.float64)
    twistAll = np.zeros((sizeOfArray, 6), dtype=np.float64)
    for idx, msg in enumerate(tqdm(msgs, position=0, leave=True)):
        if sizeOfArray <= idx:
            tsAll = np.append(tsAll, np.zeros((sizeOfArray), dtype=np.float64))
            poseAll = np.concatenate((poseAll, np.zeros((sizeOfArray, 7), dtype=np.float64)))
            twistAll = np.concatenate((twistAll, np.zeros((sizeOfArray, 6), dtype=np.float64)))
            sizeOfArray *= 2
        data = msg['data']
        seq, ptr = unpackRosUint32(data, 0)
        tsAll[idx], ptr = unpackRosTimestamp(data, ptr)
        frame_id, ptr = unpackRosString(data, ptr)
        child_frame_id, ptr = unpackRosString(data, ptr)
        poseAll[idx, :], ptr = unpackRosFloat64Array(data, 7, ptr)
        ptr += 8*36 # skip the covariance matrix
        twistAll[idx, :], ptr = unpackRosFloat64Array(data, 6, ptr)
    numEvents = idx + 1
    # Crop arrays to number of events
    tsAll = tsAll[:numEvents]
    
    poseAll = poseAll[:numEvents]
    twistAll = twistAll[:numEvents]
    point = poseAll[:, 0:3]
    rotation = poseAll[:, [6, 3, 4, 5]] # Switch quaternion form from xyzw to wxyz
    outDict = {
        'ts': tsAll,
        'point': point,
        'rotation': rotation,
        'twist': twistAll}
    return outDict
