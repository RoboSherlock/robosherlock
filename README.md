RoboSherlock
------------

 What  | Status
  ---  |  ---
 **Noetic** | [![CI](https://github.com/abmuslim/robosherlock/actions/workflows/actions.yml/badge.svg?branch=noetic)](https://github.com/abmuslim/robosherlock/actions/workflows/actions.yml)
 
------------

Core implementation for the RoboSherlock robotic perception framework based on the principles of unstructured information management.
For installation instructions and getting started see the project web page:  http://www.robosherlock.org


* live robot data `rosrun robosherlock runAAE _ae:=demo _vis:=true`

* record&playback mongo data for development
  -> Load the data into Mongo DB first by executing (WARNING: This will delete existing RS Mongo Data!): `rosrun robosherlock run _ae:=storage` While this program is running, open another terminal and playback the input rosbag file with: `rosbag play YOURBAGFILE.bag` After loading the data into the mongo database, you can stop the run _ae:=storage command.

