# importRosbag
Import rosbag data - standalone - no ROS installation required.

the importRosbag function unpacks a rosbag (path supplied by the filePathOrName parameter) into its topics and messages. 
It then uses the topic types to interpret the messages from each topic, yielding dicts for each topic, each containing an iterable for each field.
By default unpacks all topics, but you can use any of the following keyword 
params to limit which topics are intepretted:

* 'listTopics' = True - no unpacking - just returns a list of the topics contained in the file and their associated types
* 'importTopics' = <list of strings> - only imports the listed topics
* 'importTypes' = <list of strings> - only imports the listed types

Message types supported are strictly those listed in the initial imports section of this file.
There are a selection of standard message types and a couple related to event-based sensors. 
The method of importation is honed to the particular needs of the author, sometimes ignoring certain fields, grouping data in particular ways etc. 
For example, from the CameraInfo message type we import only a single message because we're not currently interested in autocalibration or its results.
However this code should serve as a model for anyone who wishes to import rosbags.
Although it's possible to import messages programmatically given only the message definition files, we have chosen not to do this, because if we did it we would anyway want to take the resulting data and pick out the bits we wanted. 

