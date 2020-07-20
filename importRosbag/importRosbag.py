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

Unpacks a rosbag into its topics and messages
Uses the topic types to interpret the messages from each topic, 
yielding dicts for each topic containing iterables for each field.
By default unpacks all topics, but you can use any of the following keyword 
params to limit which topics are intepretted:
    - 'listTopics' = True - no unpacking - just returns a list of the topics contained in 
       the file and their associated types
    - 'importTopics' = <list of strings> - only imports the listed topics
    - 'importTypes' = <list of strings> - only imports the listed types

Message types supported are strictly those listed in the initial imports section
of this file. There are a selection of standard message types and a couple
related to event-based sensors. 
The method of importation is honed to the particular needs of
the author, sometimes ignoring certain fields, grouping data in particular ways 
etc. However it should serve as a model for anyone who wishes to import rosbags.
Although it's possible to import messages programmatically given only the 
message definition files, we have chosen not to do this, because if we did it
we would anyway need to take the resulting data and pick out the bits we wanted. 

"""

#%%

from struct import unpack
from struct import error as structError
from tqdm import tqdm

# Local imports

from .messageTypes.common import unpackHeader

from .messageTypes.dvs_msgs_EventArray import importTopic as import_dvs_msgs_EventArray
from .messageTypes.esim_msgs_OpticFlow import importTopic as import_esim_msgs_OpticFlow
from .messageTypes.geometry_msgs_PoseStamped import importTopic as import_geometry_msgs_PoseStamped
from .messageTypes.geometry_msgs_Transform import importTopic as import_geometry_msgs_Transform
from .messageTypes.geometry_msgs_TransformStamped import importTopic as import_geometry_msgs_TransformStamped
from .messageTypes.geometry_msgs_TwistStamped import importTopic as import_geometry_msgs_TwistStamped
from .messageTypes.sensor_msgs_CameraInfo import importTopic as import_sensor_msgs_CameraInfo
from .messageTypes.sensor_msgs_Image import importTopic as import_sensor_msgs_Image
from .messageTypes.sensor_msgs_Imu import importTopic as import_sensor_msgs_Imu
from .messageTypes.sensor_msgs_PointCloud2 import importTopic as import_sensor_msgs_PointCloud2
from .messageTypes.tf_tfMessage import importTopic as import_tf_tfMessage

def importTopic(topic, **kwargs):
    msgs = topic['msgs']
    topicType = topic['type'].replace('/','_')
    if topicType == 'dvs_msgs_EventArray': topicDict = import_dvs_msgs_EventArray(msgs, **kwargs)
    elif topicType == 'esim_msgs_OpticFlow': topicDict = import_esim_msgs_OpticFlow(msgs, **kwargs)
    elif topicType == 'geometry_msgs_PoseStamped': topicDict = import_geometry_msgs_PoseStamped(msgs, **kwargs)
    elif topicType == 'geometry_msgs_Transform': topicDict = import_geometry_msgs_Transform(msgs, **kwargs)
    elif topicType == 'geometry_msgs_TransformStamped': topicDict = import_geometry_msgs_TransformStamped(msgs, **kwargs)
    elif topicType == 'geometry_msgs_TwistStamped': topicDict = import_geometry_msgs_TwistStamped(msgs, **kwargs)
    elif topicType == 'sensor_msgs_CameraInfo': topicDict = import_sensor_msgs_CameraInfo(msgs, **kwargs)
    elif topicType == 'sensor_msgs_Image': topicDict = import_sensor_msgs_Image(msgs, **kwargs)
    elif topicType == 'sensor_msgs_Imu': topicDict = import_sensor_msgs_Imu(msgs, **kwargs)
    elif topicType == 'sensor_msgs_PointCloud2': topicDict = import_sensor_msgs_PointCloud2(msgs, **kwargs)
    elif topicType == 'tf_tfMessage': topicDict = import_tf_tfMessage(msgs, **kwargs)
    else: 
        return None
    if topicDict:
        topicDict['rosbagType'] = topic['type']
    return topicDict

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
        chunk_infos_idx = 0
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
            fields['data_file'] = filePathOrName
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
                fields['ids'] = []
                chunks.append(fields)
                pbar.update(len(chunks))
            elif op == 6:
                # It's a chunk-info
                chunks[chunk_infos_idx]['ver'] = unpack('<L', fields['ver'])[0]
                chunks[chunk_infos_idx]['chunk_pos'] = unpack('<Q', fields['chunk_pos'])[0]
                chunks[chunk_infos_idx]['start_time'] = unpack('<LL', fields['start_time'])
                chunks[chunk_infos_idx]['end_time'] = unpack('<LL', fields['end_time'])
                count = unpack('<L', fields['count'])[0]
                for i in range(count):
                    conn_id, msg_count = unpack('<LL', data[i*8:i*8+8]) # TODO I don't know what use to make of these
                chunk_infos_idx += 1 # Chunk infos appear at the end of the bag one per chunk
            elif op == 7:
                # It's a conn
                # interpret data as a string containing the connection header
                connFields = unpackHeader(dataLen, data)
                connFields.update(fields) 
                connFields['conn'] = unpack('=l', connFields['conn'])[0]
                connFields['topic'] = connFields['topic'].decode("utf-8")
                connFields['type'] = connFields['type'].decode("utf-8").replace('/', '_')
                conns.append(connFields)
    return conns, chunks

#%% Break chunks into msgs

def breakChunksIntoMsgs(chunks):
    msgs = []
    print('Breaking chunks into msgs ...')           
    for chunk in tqdm(chunks, position=0, leave=True):
        for idx in chunk['ids']:
            with open(chunk['data_file'], 'rb') as f:
                f.seek(chunk['chunk_pos'])
                headerLen = unpack('=l', f.read(4))[0]
                f.read(headerLen)
                data_len_with_header = unpack('<l', f.read(4))[0] - idx[2]
                f.read(idx[2])

                headerLen = unpack('=l', f.read(4))[0]
                # unpack the header into fields
                headerBytes = f.read(headerLen)
                fields = unpackHeader(headerLen, headerBytes)
                # Read the record data
                data_len = unpack('=l', f.read(4))[0]
                fields['conn'] = unpack('=l', fields['conn'])[0]
                fields['chunk_pos'] = chunk['chunk_pos']
                fields['start_time'] = chunk['start_time']
                fields['end_time'] = chunk['end_time']
                fields['data_pos'] = f.tell()
                fields['data_len'] = data_len
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
    return topics # TODO make it retrocompatible


def import_topics_at_time(topics, start_time, end_time):
    for topic in topics:
        with open(topics[topic]['data_file'], 'rb') as f:
            for idx, msg in enumerate(topics[topic]['msgs']):
                msg_start_time = msg['start_time'][0] + msg['start_time'][1] * 1e-9
                if end_time >= msg_start_time >= start_time:  # TODO check if condition is correct
                    if 'data' not in msg.keys():
                        f.seek(msg['data_pos'])
                        msg['data'] = f.read(msg['data_len'])
                elif 'data' in msg.keys():
                    del msg['data']
    return import_all_topics(topics, {})


def import_all_topics(topics, kwargs):
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
            typeToImport = typeToImport.replace('/', '_')
            for topicInFile in list(topics.keys()):
                if topics[topicInFile]['type'].replace('/', '_') == typeToImport:
                    importedTopic = importTopic(topics[topicInFile], **kwargs)
                    if importedTopic is not None:
                        importedTopics[topicInFile] = importedTopic
                        del topics[topicInFile]
    else:  # import everything
        for topicInFile in list(topics.keys()):
            importedTopic = importTopic(topics[topicInFile], **kwargs)
            if importedTopic is not None:
                importedTopics[topicInFile] = importedTopic
                # del topics[topicInFile]
    print()
    if importedTopics:
        print('Topics imported are:')
        for topic in importedTopics.keys():
            print(topic + ' --- ' + importedTopics[topic]['rosbagType'])
            # del importedTopics[topic]['rosbagType']
        print()
    if topics:
        print('Topics not imported are:')
        for topic in topics.keys():
            print(topic + ' --- ' + topics[topic]['type'])
        print()
    return importedTopics

