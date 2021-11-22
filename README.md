RoboSherlock
------------

 What  | Status
  ---  |  ---
 **Master** | [![Build Status master](https://travis-ci.org/RoboSherlock/robosherlock.svg)](https://travis-ci.org/RoboSherlock/robosherlock)
**Kinetic** | [![Build Status kinetic](https://travis-ci.org/RoboSherlock/robosherlock.svg?branch=kinetic)](https://travis-ci.org/RoboSherlock/robosherlock)
**Melodic** | [![Build Status melodic](https://travis-ci.org/RoboSherlock/robosherlock.svg?branch=melodic)](https://travis-ci.org/RoboSherlock/robosherlock)
**Code coverage**  | [![codecov](https://codecov.io/gh/RoboSherlock/robosherlock/branch/master/graph/badge.svg)](https://codecov.io/gh/RoboSherlock/robosherlock)

 
------------

Core implementation for the RoboSherlock robotic perception framework based on the principles of unstructured information management.
For installation instructions and getting started see the project web page:  http://www.robosherlock.org


* live robot data `rosrun robosherlock runAAE _ae:=demo _vis:=true`

* record&playback mongo data for development
  -> Load the data into Mongo DB first by executing (WARNING: This will delete existing RS Mongo Data!): `rosrun robosherlock run _ae:=storage` While this program is running, open another terminal and playback the input rosbag file with: `rosbag play YOURBAGFILE.bag` After loading the data into the mongo database, you can stop the run _ae:=storage command.
