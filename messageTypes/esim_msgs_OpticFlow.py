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

https://github.com/uzh-rpg/rpg_esim/blob/master/event_camera_simulator/esim_msgs/msg/OpticFlow.msg

    <select>
http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html    
http://rpg.ifi.uzh.ch/davis_data.html

In particular ...

"""

#%%

from struct import unpack
from struct import error as structError
from tqdm import tqdm
import numpy as np
import string

def getOrInsertDefault(inDict, arg, default):
    # get an arg from a dict.
    # If the the dict doesn't contain the arg, return the default, 
    # and also insert the default into the dict
    value = inDict.get(arg, default)
    if value == default:
        inDict[arg] = default
    return value

def unpackHeader(headerLen, headerBytes):
    fields = {}
    ptr = 0
    while ptr < headerLen:
        fieldLen = unpack('=l', headerBytes[ptr:ptr+4])[0]
        ptr += 4
        #print(fieldLen)
        field = headerBytes[ptr:ptr+fieldLen]
        ptr += fieldLen
        #print(field)
        fieldSplit = field.find(b'\x3d')
        fieldName = field[:fieldSplit].decode("utf-8")
        fieldValue = field[fieldSplit+1:]
        fields[fieldName] = fieldValue
    return fields

def unpackRosUint32(data, ptr):
    return unpack('=L', data[ptr:ptr+4])[0], ptr+4

def unpackRosString(data, ptr):
    stringLen = unpack('=L', data[ptr:ptr+4])[0]
    ptr += 4
    outStr = data[ptr:ptr+stringLen].decode('utf-8')
    ptr += stringLen
    return outStr, ptr

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
    rotQAll = np.zeros((sizeOfArray, 4), dtype=np.float64)
    angVAll = np.zeros((sizeOfArray, 3), dtype=np.float64)
    accAll = np.zeros((sizeOfArray, 3), dtype=np.float64)
    magAll = np.zeros((sizeOfArray, 3), dtype=np.float64)
    #tempAll = np.zeros((sizeOfArray, 1), dtype=np.float64)
    for msg in tqdm(msgs, position=0, leave=True):
        if sizeOfArray < numEvents + 1:
            tsAll = np.append(tsAll, np.zeros((sizeOfArray), dtype=np.float64))
            rotQAll = np.concatenate((rotQAll, np.zeros((sizeOfArray, 4), dtype=np.float64)))
            angVAll = np.concatenate((angVAll, np.zeros((sizeOfArray, 3), dtype=np.float64)))
            accAll = np.concatenate((accAll, np.zeros((sizeOfArray, 3), dtype=np.float64)))
            magAll = np.concatenate((magAll, np.zeros((sizeOfArray, 3), dtype=np.float64)))
            sizeOfArray *= 2
        # TODO: maybe implement kwargs['useRosMsgTimestamps']
        data = msg['data']
        #seq = unpack('=L', data[0:4])[0]
        timeS, timeNs = unpack('=LL', data[4:12])
        tsAll[numEvents] = np.float64(timeS)+np.float64(timeNs)*0.000000001 
        frame_id, ptr = unpackRosString(data, 12)
        rotQAll[numEvents, :] = np.frombuffer(data[ptr:ptr+32], np.float64)
        ptr += 32
        ptr += 72 # Skip the covariance matrix
        angVAll[numEvents, :] = np.frombuffer(data[ptr:ptr+24], np.float64)
        ptr += 24
        ptr += 72 # Skip the covariance matrix
        accAll[numEvents, :] = np.frombuffer(data[ptr:ptr+24], np.float64)
        #ptr += 24
        #ptr += 72 # Skip the covariance matrix
        numEvents += 1
    # Crop arrays to number of events
    tsAll = tsAll[:numEvents]
    rotQAll = rotQAll[:numEvents]
    angVAll = angVAll[:numEvents]
    accAll = accAll[:numEvents]
    magAll = magAll[:numEvents]
    outDict = {
        'ts': tsAll,
        'rotQ': rotQAll,
        'angV': angVAll,
        'acc': accAll,
        'mag': magAll
        }
    return outDict

def interpretMsgsAsCam(msgs, **kwargs):
    '''
    camera info - i.e. results of calibration
    ros message is defined here:
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
    '''
    outDict = {}
    data = msgs[0]['data'] # There is one calibration msg per frame. Just use the first one
    #seq = unpack('=L', data[0:4])[0]
    #timeS, timeNs = unpack('=LL', data[4:12])
    frame_id, ptr = unpackRosString(data, 12)
    outDict['height'], ptr = unpackRosUint32(data, ptr)
    outDict['width'], ptr = unpackRosUint32(data, ptr)
    outDict['distortionModel'], ptr = unpackRosString(data, ptr)
    ptrAfterDistortionModel = ptr
    if outDict['distortionModel'] == 'plumb_bob':
        try:
            ptr +=4 # There's an IP address in there, not sure why
            outDict['D'] = np.frombuffer(data[ptr:ptr+40], dtype=np.float64)
            ptr += 40
            outDict['K'] = np.frombuffer(data[ptr:ptr+72], dtype=np.float64).reshape(3, 3)
            ptr += 72
            outDict['R'] = np.frombuffer(data[ptr:ptr+72], dtype=np.float64).reshape(3, 3)
            ptr += 72
            outDict['P'] = np.frombuffer(data[ptr:ptr+96], dtype=np.float64).reshape(3, 4)
        except ValueError:
            # 2020_02 Sim: When we run ESIM it is not outputting the D matrix, not sure why 
            # TODO: Either we modify the way we use ESIM, or else, if there's a reason for the absence of D, 
            # allow for that here in a less hacky way.
            ptr = ptrAfterDistortionModel + 4 # Allow for IP address
            outDict['D'] = np.zeros((5), dtype=np.float64)
            #ptr += 40
            outDict['K'] = np.frombuffer(data[ptr:ptr+72], dtype=np.float64).reshape(3, 3)
            ptr += 72
            outDict['R'] = np.frombuffer(data[ptr:ptr+72], dtype=np.float64).reshape(3, 3)
            ptr += 72
            outDict['P'] = np.frombuffer(data[ptr:ptr+96], dtype=np.float64).reshape(3, 4)
    elif outDict['distortionModel'] == 'Kannala Brandt4':
        ptr +=4 # There's an IP address in there, not sure why
        outDict['D'] = np.frombuffer(data[ptr:ptr+32], dtype=np.float64)
        ptr += 40
        outDict['K'] = np.frombuffer(data[ptr:ptr+72], dtype=np.float64).reshape(3, 3)
        ptr += 72
        outDict['R'] = np.frombuffer(data[ptr:ptr+72], dtype=np.float64).reshape(3, 3)
        ptr += 72
        outDict['P'] = np.frombuffer(data[ptr:ptr+96], dtype=np.float64).reshape(3, 4)
    else:
        #print('Distortion model not supported:' + outDict['distortionModel'])
        #return None
        # simulated data doesn't have a distortion model name, so try again like this
        frame_id, ptr = unpackRosString(data, 12)
        outDict['height'], ptr = unpackRosUint32(data, ptr)
        outDict['width'], ptr = unpackRosUint32(data, ptr)
        outDict['D'] = np.frombuffer(data[ptr:ptr+40], dtype=np.float64)
        ptr += 40
        outDict['K'] = np.frombuffer(data[ptr:ptr+72], dtype=np.float64).reshape(3, 3)
        ptr += 72
        outDict['R'] = np.frombuffer(data[ptr:ptr+72], dtype=np.float64).reshape(3, 3)
        ptr += 72
        outDict['P'] = np.frombuffer(data[ptr:ptr+96], dtype=np.float64).reshape(3, 4)
    #ptr += 96
    # Ignore binning and ROI
    return outDict
