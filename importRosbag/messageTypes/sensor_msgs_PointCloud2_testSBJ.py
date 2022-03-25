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

For simplicity, we're currently directly unpacking the format that we are 
encountering in the data, which is x,y,z,_,rgb,_,_,_ 
each as 32-bit little-endian floats
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
        http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
    the result is are np arrays of float64 for:
        rotQ (4 cols, quaternion)
        angV (3 cols)
        acc (3 cols)
        mag (3 cols)
        temp (1 cols) - but I'll probably ignore this to start with
    '''
    # tempAll = np.zeros((sizeOfArray, 1), dtype=np.float64)
    # for msg in tqdm(msgs, position=0, leave=True):
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
        'formats': ['=u4','=u4','=u4','=u4'],  # List of Strings
        'offsets': [0, 4, 8, 12],  # List of INT
        'itemsize': 16  # INT
    }
    dtype_header_2 = {
        'names': ['height', 'width', 'fieldSize'],
        'formats': [ '=u4', '=u4', '=u4'],  # List of Strings
        'offsets': [0, 4, 8],  # List of INT
        'itemsize': 12  # INT
    }
    dtype_header_3 = {
        'names': ['is_Bigendian', 'PointStep', 'RowStep', 'data_size'],
        'formats': [ '=u1', '=u4', '=u4','=u4'],  # List of Strings
        'offsets': [0, 1, 5, 9],  # List of INT
        'itemsize': 13  # INT
    }
    dtype_fields = {
        'names': [],     # List of Strings
        'formats': [],  # List of Strings
        'offsets': [],   # List of INT
        'itemsize': 0,
        'count': []     # List of INT
    }

    tsByMessage_2 = []
    pointsByMessage_2 = []

    '''    #Del codigo anterior
    tsByMessage = []
    pointsByMessage = []
    intensityByMessage = []
    timeByMessage = []
    reflectivityByMessage = []
    ringByMessage = []
    noiseByMessage = []
    rangesByMessage = []'''

    for msg in tqdm(msgs, disable=disable_bar):
        '''time_start = time.time()'''
        data = msg['data']
        ptr = 0
        pointer = 0
        # Made with numpy from buffer
        header_1 = np.frombuffer(data[pointer:pointer + dtype_header_1['itemsize']], dtype=dtype_header_1)
        timestamp = np.float64(header_1['timestamp_seconds'][0])+np.float64(header_1['timestamp_nseconds'][0])*0.000000001

        pointer += dtype_header_1['itemsize']
        frame_len = header_1['frame_len'][0].astype(int)
        frame_id, pointer = unpackStringData(data, frame_len, pointer)
        header_2 = np.frombuffer(data[pointer:pointer + dtype_header_2['itemsize']], dtype=dtype_header_2)
        pointer += dtype_header_2['itemsize']

        #Asignar las variables de array a independientes
        height = header_2['height'][0]
        width = header_2['width'][0]
        fieldSize = header_2['fieldSize'][0]

        #print('type of fieldSize: {},{}'.format(type(header_2['fieldSize'][0]),len(header_2['fieldSize'])))
        if width > 0 and height > 0:
            for element in range(fieldSize):
                name, pointer = unpackRosString(data, pointer)
                offset = np.frombuffer(data[pointer:pointer+4], dtype='=u4')
                pointer += 4
                datatype = np.frombuffer(data[pointer:pointer+1], dtype='=u1')
                pointer += 1
                count = np.frombuffer(data[pointer:pointer + 4], dtype='=u4')
                pointer += 4
                #APPEND TO THE DICTIONARY ('dtype_fields')
                dtype_fields['names'].append(name)
                dtype_fields['offsets'].append(offset[0])
                dtype_fields['formats'].append(str(DTYPE_VAR[str(datatype[0])])) #Pasarlo por el diccionario
                dtype_fields['count'].append(count[0])

            header_3 = np.frombuffer(data[pointer:pointer + dtype_header_3['itemsize']], dtype=dtype_header_3)
            pointer += dtype_header_3['itemsize']

            PointStep = header_3['PointStep'][0]
            dtype_fields['itemsize'] = PointStep
            numPoints = width * height
            data_array = np.empty((numPoints,1),dtype=dtype_fields)
            #array = np.frombuffer(data[pointer:pointer + dtype_fields['itemsize']], dtype=dtype_fields)
            for x in range(width):
                for y in range(height):
                    data_array[x * height + y, :] = np.frombuffer(data[pointer:pointer+dtype_fields['itemsize']], dtype=dtype_fields)
                    pointer += PointStep

            pointsByMessage_2.append(data_array)
            tsByMessage_2.append(np.ones(numPoints, dtype=np.float64) * timestamp)  #Meter aquí la traspuesta de la matriz???

        if not pointsByMessage_2:  # None of the messages contained any points
            return None
            #pass
        data_array = np.concatenate(pointsByMessage_2)
        timestamp = np.concatenate(tsByMessage_2)
        '''elapsed_time = time.time() - time_start
        print("Tiempo transcurrido: {} segundos".format(elapsed_time))'''
        # Crop arrays to number of events
        outDict = {
            'ts': timestamp,
            'point': data_array,
        }
        return outDict

        ''' #-----------------------------------------------------------------
        seq, ptr = unpackRosUint32(data, ptr)
        ts, ptr = unpackRosTimestamp(data, ptr)
        frame_id, ptr = unpackRosString(data, ptr)
        height, ptr = unpackRosUint32(data, ptr)
        width, ptr = unpackRosUint32(data, ptr)

        if width > 0 and height > 0:

            arraySize, ptr = unpackRosUint32(data, ptr)
            for element in range(arraySize):
                # Move through the field definitions - we'll ignore these
                # until we encounter a file that uses a different set
                name, ptr = unpackRosString(data, ptr)
                offset, ptr = unpackRosUint32(data, ptr)
                datatype, ptr = unpackRosUint8(data, ptr)
                count, ptr = unpackRosUint32(data, ptr)

            isBigendian, ptr = unpackRosUint8(data, ptr)
            pointStep, ptr = unpackRosUint32(data, ptr)
            rowStep, ptr = unpackRosUint32(data, ptr)

            numPoints = width * height
            points = np.empty((numPoints, 3), dtype=np.float32)
            intensity = np.empty((numPoints, 1), dtype=np.float32)
            time_arr = np.empty((numPoints, 1), dtype=np.uint32)
            reflectivity = np.empty((numPoints, 1), dtype=np.uint16)
            ring = np.empty((numPoints, 1), dtype=np.uint8)
            noise = np.empty((numPoints, 1), dtype=np.uint16)
            ranges = np.empty((numPoints, 1), dtype=np.uint32)

            arraySize, ptr = unpackRosUint32(data, ptr)
            # assert arraySize = width*height
            for x in range(width):
                for y in range(height):
                    points[x * height + y, :] = np.frombuffer(data[ptr:ptr + 12], dtype=np.float32)
                    intensity[x * height + y, 0] = np.frombuffer(data[ptr + 16:ptr + 20], dtype=np.float32)
                    # intensity, ptr_delete = unpackRosFloat32(data, ptr+12)
                    time_arr[x * height + y, 0] = np.frombuffer(data[ptr + 20:ptr + 24], dtype=np.uint32)
                    reflectivity[x * height + y, 0] = np.frombuffer(data[ptr + 24:ptr + 26], dtype=np.uint16)
                    ring[x * height + y, 0] = np.frombuffer(data[ptr + 26:ptr + 27], dtype=np.uint8)
                    noise[x * height + y, 0] = np.frombuffer(data[ptr + 27:ptr + 29], dtype=np.uint16)
                    ranges[x * height + y, 0] = np.frombuffer(data[ptr + 29:ptr + 33], dtype=np.uint32)

                    ptr += pointStep

            pointsByMessage.append(points)
            tsByMessage.append(np.ones(numPoints, dtype=np.float64) * ts)
            intensityByMessage.append(intensity)
            # intensityByMessage.append(np.ones(numPoints, dtype=np.float64) * intensity)
            timeByMessage.append(time_arr)
            reflectivityByMessage.append(reflectivity)
            ringByMessage.append(ring)
            noiseByMessage.append(noise)
            rangesByMessage.append(ranges)

    if not pointsByMessage:  # None of the messages contained any points
        return None
    points = np.concatenate(pointsByMessage)
    ts = np.concatenate(tsByMessage)
    intensity = np.concatenate(intensityByMessage)
    time_internal = np.concatenate(timeByMessage)
    reflectivity = np.concatenate(reflectivityByMessage)
    ring = np.concatenate(ringByMessage)
    noise = np.concatenate(noiseByMessage)
    ranges = np.concatenate(rangesByMessage)

    # Crop arrays to number of events
    outDict = {
        'ts': ts,
        'point': points,
        'intensity': intensity,
        'time': time_internal,
        'reflectivity': reflectivity,
        'ring': ring,
        'noise': noise,
        'ranges': ranges,
    }
    return outDict'''
