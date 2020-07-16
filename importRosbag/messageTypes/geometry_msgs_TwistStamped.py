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

The interpretMessages function receives a list of messages and returns
a dict with one field for each data field in the message, where the field
will contain an appropriate iterable to contain the interpretted contents of each message.

This function imports the ros message type defined at:
http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistStamped.html
"""

#%%

from tqdm import tqdm
import numpy as np

from .common import unpackRosString, unpackRosTimestamp, unpackRosFloat64Array

def importTopic(msgs, **kwargs):
    sizeOfArray = 1024
    ts = []
    linV = []
    angV = []
    data_start_idx = np.inf
    data_end_idx = -np.inf
    for idx, msg in enumerate(tqdm(msgs, position=0, leave=True)):
        try:
            data = msg['data']
            #seq = unpack('=L', data[0:4])[0]
            ts_new, ptr = unpackRosTimestamp(data, 4)
            frame_id, ptr = unpackRosString(data, ptr)
            linV_new, ptr = unpackRosFloat64Array(data, 3, ptr)
            linV.append(linV_new)
            angV_new, _ = unpackRosFloat64Array(data, 3, ptr)
            data_start_idx = min(idx, data_start_idx)
            data_end_idx = max(idx, data_end_idx)
            angV.append(angV_new)
        except KeyError:
            ts_new, _ = unpackRosTimestamp(msg['time'], 0)
        ts.append(ts_new)

    outDict = {
        'ts': ts,
        'linV': linV,
        'angV': angV,
        'start_idx': data_start_idx,
        'end_idx': data_end_idx}
    return outDict

