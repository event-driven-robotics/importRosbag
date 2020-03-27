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
https://github.com/uzh-rpg/rpg_dvs_ros/tree/master/dvs_msgs/msg
"""

#%%

from struct import unpack
from tqdm import tqdm
import numpy as np

# local imports

from .common import unpackRosString, unpackRosUint32

def importTopic(msgs, **kwargs):
    numEvents = 0
    sizeOfArray = 1024
    tsAll = np.zeros((sizeOfArray), dtype=np.float64)
    xAll = np.zeros((sizeOfArray), dtype=np.uint16)
    yAll = np.zeros((sizeOfArray), dtype=np.uint16)
    polAll = np.zeros((sizeOfArray), dtype=np.bool)
    for msg in tqdm(msgs, position=0, leave=True):
        # TODO: maybe implement kwargs['useRosMsgTimestamps']
        data = msg['data']
        #seq = unpack('=L', data[0:4])[0]
        #timeS, timeNs = unpack('=LL', data[4:12])
        frame_id, ptr = unpackRosString(data, 12)
        height, ptr = unpackRosUint32(data, ptr)
        width, ptr = unpackRosUint32(data, ptr) 
        numEventsInMsg, ptr = unpackRosUint32(data, ptr)
        while sizeOfArray < numEvents + numEventsInMsg:
            tsAll = np.append(tsAll, np.zeros((sizeOfArray), dtype=np.float64))
            xAll = np.append(xAll, np.zeros((sizeOfArray), dtype=np.uint16))
            yAll = np.append(yAll, np.zeros((sizeOfArray), dtype=np.uint16))
            polAll = np.append(polAll, np.zeros((sizeOfArray), dtype=np.bool))
            sizeOfArray *= 2
        for idx in range(numEventsInMsg):
            x, y, ts, tns, pol = unpack('=HHLL?', data[ptr + idx*13 : ptr + (idx + 1)*13])
            idxAll = idx + numEvents
            tsFloat = np.float64(ts)+np.float64(tns)*0.000000001 # It's possible that this will kilL the precision
            xAll[idxAll] = x
            yAll[idxAll] = y
            tsAll[idxAll] = tsFloat
            polAll[idxAll] = pol
        numEvents += numEventsInMsg
    # Crop arrays to number of events
    tsAll = tsAll[:numEvents]
    xAll = xAll[:numEvents]
    yAll = yAll[:numEvents]
    polAll = polAll[:numEvents]
    outDict = {
        'x': xAll,
        'y': yAll,
        'ts': tsAll,
        'pol': polAll,
        'dimX': width,
        'dimY': height}
    return outDict
