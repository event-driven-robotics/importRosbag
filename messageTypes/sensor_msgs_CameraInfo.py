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

http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
We assume that there will only be one camera_info msg per channel,
so the resulting dict is populated by the following fields:
    std_msgs/Header header
    uint32 height
    uint32 width
    string distortion_model - actually not storing this for now, just checking that it's "plumb_bob"
    <following as numpy matrices of the appropriate dimensions>
    float64[] D (distortion params)
    float64[9] K (Intrinsic camera matrix)
    float64[9] R (rectification matrix - only for stereo setup)
    float64[12] P (projection matrix)
    <ignoring the following for now:>
    uint32 binning_x
    uint32 binning_y
    sensor_msgs/RegionOfInterest roi

"""

#%%

import numpy as np

from .common import unpackRosString, unpackRosUint32, unpackRosFloat64Array

def unpackKRP(data, outDict, ptr):
    outDict['K'], ptr = unpackRosFloat64Array(data, 9, ptr)
    outDict['K'].reshape(3, 3)
    outDict['R'], ptr = unpackRosFloat64Array(data, 9, ptr)
    outDict['R'].reshape(3, 3)
    outDict['P'], ptr = unpackRosFloat64Array(data, 12, ptr)
    outDict['P'].reshape(3, 4)
    return ptr

def importTopic(msgs, **kwargs):

    outDict = {}
    data = msgs[0]['data'] # There is one calibration msg per frame. Just use the first one
    #seq = unpack('=L', data[0:4])[0]
    #timeS, timeNs = unpack('=LL', data[4:12])
    frame_id, ptr = unpackRosString(data, 12)
    outDict['height'], ptr = unpackRosUint32(data, ptr)
    outDict['width'], ptr = unpackRosUint32(data, ptr)
    outDict['distortionModel'], ptr = unpackRosString(data, ptr)
    ptr +=4 # There's an IP address in there, not sure why
    ptrAfterDistortionModel = ptr
    if outDict['distortionModel'] == 'plumb_bob':
        try:
            outDict['D'], ptr = unpackRosFloat64Array(data, 5, ptr)
            ptr = unpackKRP(data, outDict, ptr)
        except ValueError:
            # It can happen, if camera has no distortion, that D is left out
            ptr = ptrAfterDistortionModel
            outDict['D'] = np.zeros((5), dtype=np.float64)
            ptr = unpackKRP(data, outDict, ptr)
    elif outDict['distortionModel'] == 'Kannala Brandt4':
        outDict['D'], ptr = unpackRosFloat64Array(data, 4, ptr)
        ptr = unpackKRP(data, outDict, ptr)
    else:
        #print('Distortion model not supported:' + outDict['distortionModel'])
        #return None
        # simulated data doesn't have a distortion model name, so try again like this
        frame_id, ptr = unpackRosString(data, 12)
        outDict['height'], ptr = unpackRosUint32(data, ptr)
        outDict['width'], ptr = unpackRosUint32(data, ptr)
        outDict['D'], ptr = unpackRosFloat64Array(data, 5, ptr)
        outDict['K'], ptr = unpackRosFloat64Array(data, 9, ptr)
        outDict['K'].reshape(3, 3)
        outDict['R'], ptr = unpackRosFloat64Array(data, 9, ptr)
        outDict['R'].reshape(3, 3)
        outDict['P'], ptr = unpackRosFloat64Array(data, 12, ptr)
        outDict['P'].reshape(3, 4)
    # Ignore binning and ROI
    return outDict
