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
http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
"""

#%%

from struct import unpack
from tqdm import tqdm
import numpy as np

from .common import unpackRosString, unpackRosUint8, unpackRosUint32, \
                    unpackRosFloat32, unpackRosTimestamp

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
    numEvents = 0
    sizeOfArray = 1024
    tsAll = np.zeros((sizeOfArray), dtype=np.float64)
    #tempAll = np.zeros((sizeOfArray, 1), dtype=np.float64)
    #for msg in tqdm(msgs, position=0, leave=True):
    for msg in msgs:
        
        data = msg['data']
        ptr = 0
        seq, ptr = unpackRosUint32(data, ptr)
        time, ptr = unpackRosTimestamp(data, ptr)
        frame_id, ptr = unpackRosString(data, ptr)
        height, ptr = unpackRosUint32(data, ptr)
        width, ptr = unpackRosUint32(data, ptr) 

        print('len')
        print(len(data))
        print(seq)
        print(time)
        print(frame_id)
        print(height)
        print(width)
        print()

        ptr +=4 # There's an IP address in there, not sure why
        name, ptr = unpackRosString(data, ptr)
        offset, ptr = unpackRosUint32(data, ptr)
        datatype, ptr = unpackRosUint8(data, ptr)
        count, ptr = unpackRosUint32(data, ptr)
        print('name')
        print(name)
        print('ptr')
        print(ptr)
        print('offset')
        print(offset)
        print(datatype)
        print(count)
        print()

        name, ptr = unpackRosString(data, ptr)
        offset, ptr = unpackRosUint32(data, ptr)
        datatype, ptr = unpackRosUint8(data, ptr)
        count, ptr = unpackRosUint32(data, ptr)
        print('name')
        print(name)
        print('ptr')
        print(ptr)
        print('offset')
        print(offset)
        print(datatype)
        print(count)
        print()

        name, ptr = unpackRosString(data, ptr)
        offset, ptr = unpackRosUint32(data, ptr)
        datatype, ptr = unpackRosUint8(data, ptr)
        count, ptr = unpackRosUint32(data, ptr)
        print('name')
        print(name)
        print('ptr')
        print(ptr)
        print('offset')
        print(offset)
        print(datatype)
        print(count)
        print()

        name, ptr = unpackRosString(data, ptr)
        offset, ptr = unpackRosUint32(data, ptr)
        datatype, ptr = unpackRosUint8(data, ptr)
        count, ptr = unpackRosUint32(data, ptr)
        print('name')
        print(name)
        print('ptr')
        print(ptr)
        print('offset')
        print(offset)
        print(datatype)
        print(count)
        print()


        isBigendian, ptr = unpackRosUint8(data, ptr)
        pointStep, ptr = unpackRosUint32(data, ptr)
        rowStep, ptr = unpackRosUint32(data, ptr)
        print(isBigendian)
        print(pointStep)
        print(rowStep)
        print('ptr')
        print(ptr)
        print()

        x, ptr = unpackRosFloat32(data, ptr)
        print(x)
        y, ptr = unpackRosFloat32(data, ptr)
        print(y)
        z, ptr = unpackRosFloat32(data, ptr)
        print(z)
        rgb, ptr = unpackRosFloat32(data, ptr)
        print(rgb)
        print()
        
        break

        numEvents += 1
    # Crop arrays to number of events
    tsAll = tsAll[:numEvents]
    outDict = {
        'ts': tsAll,

        }
    return outDict
