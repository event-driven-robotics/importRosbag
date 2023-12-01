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

In this new version of the pointcloud2 extract, we can automated unpacking
all the fields that are included in pointcloud2 message in the format that is required.
"""

# %%


from tqdm import tqdm
import numpy as np
import time

from .common import unpackRosString, unpackRosUint8, unpackRosUint32, unpackRosTimestamp, unpackRosFloat32, \
    unpackRosUint16, unpackStringData


def importTopic(msgs, **kwargs):
    '''
    ros message is defined here:
        http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
    the result is a variable dimension dictionary with
        - timestamp
        - point structure (include all field that declares in pointcloud2 msg)
    '''
    disable_bar = kwargs.get('disable_bar')
    DTYPE_VAR = {
        '1': '=i1',  # INT8
        '2': '=u1',  # UINT8
        '3': '=i2',  # INT16
        '4': '=u2',  # UINT16
        '5': '=i4',  # INT32
        '6': '=u4',  # UINT32
        '7': '=f4',  # FLOAT32
        '8': '=f8',  # FLOAT64
    }
    dtype_header_1 = {
        'names': ['sequence_id', 'timestamp_seconds', 'timestamp_nseconds',  # List of Strings
                  'frame_len'],
        'formats': ['=u4','=u4','=u4','=u4'],                                # List of Strings
        'offsets': [0, 4, 8, 12],                                            # List of INT
        'itemsize': 16                                                       # INT
    }
    dtype_header_2 = {
        'names': ['height', 'width', 'fieldSize'],                          # List of Strings
        'formats': [ '=u4', '=u4', '=u4'],                                  # List of Strings
        'offsets': [0, 4, 8],                                               # List of INT
        'itemsize': 12                                                      # INT
    }
    dtype_header_3 = {
        'names': ['is_Bigendian', 'PointStep', 'RowStep', 'data_size'],     # List of Strings
        'formats': [ '=u1', '=u4', '=u4','=u4'],                            # List of Strings
        'offsets': [0, 1, 5, 9],                                            # List of INT
        'itemsize': 13                                                      # INT
    }
    dtype_fields = {
        'names': [],                                                        # List of Strings
        'formats': [],                                                      # List of Strings
        'offsets': [],                                                      # List of INT
        'itemsize': 0,                                                      # INT
        'count': []                                                         # List of INT
    }

    tsByMessage_2 = []
    pointsByMessage_2 = []

    for msg in tqdm(msgs, disable=disable_bar):
        data = msg['data']
        ptr = 0
        # Reading first header (with numpy)
        header_1 = np.frombuffer(data[ptr:ptr + dtype_header_1['itemsize']], dtype=dtype_header_1)
        timestamp = np.float64(header_1['timestamp_seconds'][0])+np.float64(header_1['timestamp_nseconds'][0])*0.000000001
        ptr += dtype_header_1['itemsize']
        # Unpacking frame_id with unknown dimension
        frame_len = header_1['frame_len'][0].astype(int)
        frame_id, ptr = unpackStringData(data, frame_len, ptr)
        # Reading second header
        header_2 = np.frombuffer(data[ptr:ptr + dtype_header_2['itemsize']], dtype=dtype_header_2)
        ptr += dtype_header_2['itemsize']

        #Assign variables
        height = header_2['height'][0]
        width = header_2['width'][0]
        fieldSize = header_2['fieldSize'][0]

        #Reading fields of pointcloud2
        if width > 0 and height > 0:
            for element in range(fieldSize):
                names, ptr = unpackRosString(data, ptr)
                offset = np.frombuffer(data[ptr:ptr+4], dtype='=u4')
                ptr += 4
                datatype = np.frombuffer(data[ptr:ptr+1], dtype='=u1')
                ptr += 1
                count = np.frombuffer(data[ptr:ptr + 4], dtype='=u4')
                ptr += 4
                #APPEND TO THE DICTIONARY ('dtype_fields')
                dtype_fields['names'].append(names)
                dtype_fields['offsets'].append(offset[0])
                dtype_fields['formats'].append(str(DTYPE_VAR[str(datatype[0])]))
                dtype_fields['count'].append(count[0])
            # Reading third header
            header_3 = np.frombuffer(data[ptr:ptr + dtype_header_3['itemsize']], dtype=dtype_header_3)
            ptr += dtype_header_3['itemsize']

            #Assign variables
            PointStep = header_3['PointStep'][0]
            dtype_fields['itemsize'] = PointStep
            numPoints = width * height

            #Read pointcloud2 data with datatype obtained before
            data_array = np.empty((numPoints, 1), dtype=dtype_fields)
            for x in range(width):
                for y in range(height):
                    data_array[x * height + y, :] = np.frombuffer(data[ptr:ptr+dtype_fields['itemsize']], dtype=dtype_fields)
                    ptr += PointStep

            pointsByMessage_2.append(data_array)
            tsByMessage_2.append(np.ones(numPoints, dtype=np.float64) * timestamp)  #Meter aquí la traspuesta de la matriz???
            #Reset dtype_fields variables for the next msg
            dtype_fields['names'] = []
            dtype_fields['offsets'] = []
            dtype_fields['formats'] = []
            dtype_fields['count'] = []

    if not pointsByMessage_2:  # None of the messages contained any points
        return None
    data_array = np.concatenate(pointsByMessage_2)
    timestamp = np.concatenate(tsByMessage_2)
    # Crop arrays to number of events
    outDict = {
        'ts': timestamp,
        'point': data_array,
        }
    return outDict
