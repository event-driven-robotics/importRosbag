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

TTOOOOOODOOOOOO: Transform needs unwrap timestamps? Why?>

Unpacks a rosbag into its topics and messages
Uses the topic types to interpret the messages from each topic, 
yielding dicts for each topic containing iterators for each field.
By default unpacks all topics, but you can use any of the following keyword 
params to limit which topics are intepretted:
    - 'listTopics' = True - no unpacking - just returns a list of the topics contained in 
       the file and their associated types
    - 'importTopics' = <list of strings> - only imports the listed topics
    - 'importTypes' = <list of strings> - only imports the listed types


importRpgDvsRos function first uses importRosbag  function, 
which is a completely standard function to import a rosbag file,
resulting in a dictionary of conns (connections contained in the file), 
each containing 'msgs', a list of messages. 
This does not attempt interpretation of the messages. 
importRpgDvsRos then takes any connection which is of recognised types, and
uses only these messages to output an importedDict, in the format defined for importAe.

In particular, this supports the dvs_msgs/EventArray messages defined at:
http://rpg.ifi.uzh.ch/davis_data.html

It also supports some standard ros types:
    sensor_msgs/Image, 
    sensor_msgs/CameraInfo, (calibration)
    sensor_msgs/Imu 
    geometry_msgs/PoseStamped 
    
Return nested dicts of the form:
{
    info
    data
        0
            dvs
                "pol": numpy array of bool
                "x": numpy array of uint16
                "y": numpy array of uint16
                "ts": numpy array of float - seconds (basic format is int with unit increments of 80 ns) 
            frame ...
            imu ...
            etc ...
            
The function requires a template input to tell it which connections to map
to which channels and datatypes in the resulting dict. 
Here follow 2 example templates:
    
template = {
    'left': {
        'dvs': '/davis/left/events',
    }, 'right': {
        'dvs': '/davis/right/events',
        }
    }    
    
template = {
    'ch0': {
        'dvs': '/dvs/events',
        'frame': '/dvs/image_raw',
        'pose6q': '/optitrack/davis',
        'cam': '/dvs/camera_info',
        'imu': '/dvs/imu'
        }
    }    

Any connections which are not named in the template are not imported but simply listed.
To just inspect the connections, pass in an empty template (or just don't pass in the template parameter)
Use this to inspect a new .bag file before defining the import template.
"""

#%%

from struct import unpack
from struct import error as structError
from tqdm import tqdm
import numpy as np

# Local imports

from messageTypes.common import unpackHeader

from messageTypes.dvs_msgs_EventArray import importTopic as import_dvs_msgs_EventArray
from messageTypes.esim_msgs_OpticFlow import importTopic as import_esim_msgs_OpticFlow
from messageTypes.geometry_msgs_PoseStamped import importTopic as import_geometry_msgs_PoseStamped
from messageTypes.geometry_msgs_Transform import importTopic as import_geometry_msgs_Transform
from messageTypes.geometry_msgs_TwistStamped import importTopic as import_geometry_msgs_TwistStamped
from messageTypes.sensor_msgs_CameraInfo import importTopic as import_sensor_msgs_CameraInfo
from messageTypes.sensor_msgs_Image import importTopic as import_sensor_msgs_Image
from messageTypes.sensor_msgs_Imu import importTopic as import_sensor_msgs_Imu
from messageTypes.sensor_msgs_PointCloud2 import importTopic as import_sensor_msgs_PointCloud2

def importTopic(topic, **kwargs):
    msgs = topic['msgs']
    topicType = topic['type'].replace('/','_')
    if topicType == 'dvs_msgs_EventArray': return import_dvs_msgs_EventArray(msgs, **kwargs)
    if topicType == 'esim_msgs_OpticFlow': return import_esim_msgs_OpticFlow(msgs, **kwargs)
    if topicType == 'geometry_msgs_PoseStamped': return import_geometry_msgs_PoseStamped(msgs, **kwargs)
    if topicType == 'geometry_msgs_Transform': return import_geometry_msgs_Transform(msgs, **kwargs)
    if topicType == 'geometry_msgs_TwistStamped': return import_geometry_msgs_TwistStamped(msgs, **kwargs)
    if topicType == 'sensor_msgs_CameraInfo': return import_sensor_msgs_CameraInfo(msgs, **kwargs)
    if topicType == 'sensor_msgs_Image': return import_sensor_msgs_Image(msgs, **kwargs)
    if topicType == 'sensor_msgs_Imu': return import_sensor_msgs_Imu(msgs, **kwargs)
    if topicType == 'sensor_msgs_PointCloud2': return import_sensor_msgs_PointCloud2(msgs, **kwargs)
    return None

def readFile(filePathOrName):
    print('Attempting to import ' + filePathOrName + ' as a rosbag 2.0 file.')
    with open(filePathOrName, 'rb') as file:
        # File format string
        fileFormatString = file.readline().decode("utf-8")
        print('ROSBAG file format: ' + fileFormatString)
        if fileFormatString != '#ROSBAG V2.0\n':
            print('This file format might not be supported')
        eof = False
        conns = []
        chunks = []
        while not eof:
            # Read a record header
            try:
                headerLen = unpack('=l', file.read(4))[0]
            except structError:
                if len(file.read(1)) == 0: # Distinguish EOF from other struct errors 
                   # a struct error could also occur if the data is downloaded by one os and read by another.
                   eof = True
                   continue
            # unpack the header into fields 
            headerBytes = file.read(headerLen)
            fields = unpackHeader(headerLen, headerBytes)
            # Read the record data
            dataLen = unpack('=l', file.read(4))[0]
            data = file.read(dataLen)
            # The op code tells us what to do with the record
            op = unpack('=b', fields['op'])[0]
            fields['op'] = op
            if op == 2:
                # It's a message
                # AFAIK these are not found unpacked in the file
                #fields['data'] = data 
                #msgs.append(fields)
                pass
            elif op == 3:
                # It's a bag header - use this to do progress bar for the read
                chunkCount = unpack('=l', fields['chunk_count'])[0]
                pbar = tqdm(total=chunkCount, position=0, leave=True)
            elif op == 4:
                # It's an index - this is used to index the previous chunk
                conn = unpack('=l', fields['conn'])[0]
                count = unpack('=l', fields['count'])[0]
                for idx in range(count):
                    time, offset = unpack('=ql', data[idx*12:idx*12+12])
                    chunks[-1]['ids'].append((conn, time, offset))
            elif op == 5:
                # It's a chunk
                fields['data'] = data
                fields['ids'] = []
                chunks.append(fields)
                pbar.update(len(chunks))
            elif op == 6:
                # It's a chunk-info - seems to be redundant
                pass
            elif op == 7:
                # It's a conn
                # interpret data as a string containing the connection header
                connFields = unpackHeader(dataLen, data)
                connFields.update(fields) 
                connFields['conn'] = unpack('=l', connFields['conn'])[0]
                connFields['topic'] = connFields['topic'].decode("utf-8")
                connFields['type'] = connFields['type'].decode("utf-8")
                conns.append(connFields)
    return conns, chunks

#%% Break chunks into msgs

def breakChunksIntoMsgs(chunks):
    msgs = [] 
    print('Breaking chunks into msgs ...')           
    for chunk in tqdm(chunks, position=0, leave=True):
        for idx in chunk['ids']:
            ptr = idx[2]
            headerLen = unpack('=l', chunk['data'][ptr:ptr+4])[0]
            ptr += 4
            # unpack the header into fields 
            headerBytes = chunk['data'][ptr:ptr+headerLen]
            ptr += headerLen
            fields = unpackHeader(headerLen, headerBytes)
            # Read the record data
            dataLen = unpack('=l', chunk['data'][ptr:ptr+4])[0]
            ptr += 4
            fields['data'] = chunk['data'][ptr:ptr+dataLen]
            fields['conn'] = unpack('=l', fields['conn'])[0]
            msgs.append(fields)
    return msgs

def rekeyConnsByTopic(connDict):
    topics = {}
    for conn in connDict:
        topics[connDict[conn]['topic']] = connDict[conn]
    return topics


def importRosbag(filePathOrName, **kwargs):
    print('Importing file: ', filePathOrName) 
    conns, chunks = readFile(filePathOrName)
    # Restructure conns as a dictionary keyed by conn number
    connDict = {}
    for conn in conns:
        connDict[conn['conn']] = conn
        conn['msgs'] = []
    if kwargs.get('listTopics', False):
        topics = rekeyConnsByTopic(connDict)
        print('Topics in the file are (with types):')
        for topicKey, topic in topics.items():
            del topic['conn']
            del topic['md5sum']
            del topic['msgs']
            del topic['op']
            del topic['topic']
            topic['message_definition'] = topic['message_definition'].decode("utf-8")
            print('    ' + topicKey + ' --- ' + topic['type'])
        return topics
    msgs = breakChunksIntoMsgs(chunks)
    for msg in msgs:     
        connDict[msg['conn']]['msgs'].append(msg)
    topics = rekeyConnsByTopic(connDict)

    importedTopics = {}
    importTopics = kwargs.get('importTopics')
    importTypes = kwargs.get('importTypes')
    if importTopics is not None:
        for topicToImport in importTopics:
            for topicInFile in topics.keys():
                if topicInFile == topicToImport:
                    importedTopic = importTopic(topics[topicInFile], **kwargs)
                    if importedTopic is not None:
                        importedTopics[topicToImport] = importedTopic
                        del topics[topicInFile]            
    elif importTypes is not None:
        for typeToImport in importTypes:
            for topicInFile in list(topics.keys()):
                if topics[topicInFile]['type'] == typeToImport:
                    importedTopic = importTopic(topics[topicInFile], **kwargs)
                    if importedTopic is not None:
                        importedTopics[topicInFile] 
                        del topics[topicInFile]    
    else: # import everything
        for topicInFile in list(topics.keys()):
            importedTopic = importTopic(topics[topicInFile], **kwargs)
            if importedTopic is not None:
                importedTopics[topicInFile] = importedTopic
                del topics[topicInFile]

    if importedTopics:
        print('Topics imported are:')
        for topic in importedTopics.keys():
            print(topic)        

    if topics:
        print('Topics not imported are:')
        for topic in topics.keys():
            print(topic)     
    
        return importedTopics

    '''
    # post processing
    # jointly rezero for all channels
    if kwargs.get('zeroTimestamps', True):
        # Optional: start the timestamps at zero for the first event
        # This is done collectively for all the concurrent imports
        rezeroTimestampsForImportedDicts(outDict)
    # report the remaining topics
    remainingTopics = topics.keys()
    if remainingTopics:
        print('The following topics are present in the file but were not imported: ')
        for topic in remainingTopics:
            print(topic)
    # if cam and dvs exist in the same channel, apply height and width to dimY/X
    #for channelKey in outDict['data'].keys():
    #    utiliseCameraInfoWithinChannel(outDict['data'][channelKey])
    return outDict
    '''
    
    '''   

    numEvents = 0
    sizeOfArray = 1024
    tsAll = np.zeros((sizeOfArray), dtype=np.float64)
    chAll = np.zeros((sizeOfArray), dtype=np.uint8)
    xAll = np.zeros((sizeOfArray), dtype=np.uint16)
    yAll = np.zeros((sizeOfArray), dtype=np.uint16)
    polAll = np.zeros((sizeOfArray), dtype=np.bool)
    '''
        
#%% OBSOLETE:
    
'''
def utiliseCameraInfoWithinChannel(channelDict):
    if 'cam' in channelDict and 'height' in channelDict['cam']:
        if 'dvs' in channelDict and 'dimY' not in channelDict['dvs']:
            channelDict['dvs']['dimX'] = channelDict['cam']['width']
            channelDict['dvs']['dimY'] = channelDict['cam']['height']
'''            

'''
    #Handling template

    template = kwargs.get('template', {})
    # The template becomes the data branch of the importedDict
    outDict = {
        'info': kwargs,
        'data': {}
            }            
    for channelKey in template:
        print('Importing for channel : ', channelKey)
        channelKeyStripped = str(channelKey).translate(str.maketrans('', '', string.punctuation))
        outDict['data'][channelKeyStripped] = {}
        for dataType in template[channelKey]:
            topic = template[channelKey][dataType]
            topic = topics.pop(topic)
            msgs = topic['msgs']
            print('Importing for dataType "' + dataType + '"; there are ' + str(len(msgs)) + ' messages')
            if dataType == 'dvs':
                # Interpret these messages as EventArray
                outDict['data'][channelKeyStripped][dataType] = interpretMsgsAsDvs(msgs, **kwargs)
            elif dataType == 'pose6q':
                outDict['data'][channelKeyStripped][dataType] = interpretMsgsAsPose6q(msgs, **kwargs)
            elif dataType == 'frame':
                outDict['data'][channelKeyStripped][dataType] = interpretMsgsAsFrame(msgs, **kwargs)
            elif dataType == 'cam':
                outDict['data'][channelKeyStripped][dataType] = interpretMsgsAsCam(msgs, **kwargs)
            elif dataType == 'imu':
                outDict['data'][channelKeyStripped][dataType] = interpretMsgsAsImu(msgs, **kwargs)
            else:
                print('dataType "', dataType, '" not recognised.')
        if getOrInsertDefault(kwargs, 'zeroTimestamps', True):
            # Optional: start the timestamps at zero for the first event
            # This is done collectively for all the concurrent imports
            zeroTimestampsForAChannel(outDict['data'][channelKeyStripped])
'''

'''
def unpackRosUint32(data, ptr):
    return unpack('=L', data[ptr:ptr+4])[0], ptr+4

def unpackRosString(data, ptr):
    stringLen = unpack('=L', data[ptr:ptr+4])[0]
    ptr += 4
    outStr = data[ptr:ptr+stringLen].decode('utf-8')
    ptr += stringLen
    return outStr, ptr

'''