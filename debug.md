Below is Debug procedure to follow when the ROS subscriber on the blender model side is not accepting values published from the ROS publisher that gets values from the dlib:

If ROS and blender model are not connected even after sourcing the devel/setup.bash, 
If after running the commands 'source devel/setup.bash' and 'roslaunch dlib_puppeteering dlib_camera.launch' the mirroring script is publishing data points from dlib but the model is not responding, then do the following.
Open a terminal from root (in order to be able to execute all commands). In order to do so:
First open a terminal (non root)
then enter on the opened terminal the command 'sudo -H gnome-terminal'.
This will open in turn open a new terminal with from root.
So in order to re-make the mirroring source, remove an existing 'devel' and 'build' folders within the home directory of the mirroring source folder.
Then from the new root terminal window cd to the mirroring source folder (such as dlib_ws) follow the necessary steps below to make it run successfully which is also already written the README.md of the app:

/home/meareg/dlib_ws# catkin_make
/home/meareg/dlib_ws# cd src
/home/meareg/dlib_ws/src# pip3 install -t ../devel/lib/python2.7/dist-packages/ ./blender_api_msgs
/home/meareg/dlib_ws/src# cd ..
/home/meareg/dlib_ws# source devel/setup.bash

Then on by opening a new terminal (non root, if file paths within the source code such as dlib's 'shape_predictor_68_face_landmarks.dat' are not saved under the root directory) enter:

roslaunch dlib_puppeteering dlib_camera

Then on another terminal or sub-terminal:
go to the blender_api folder within the mirroring source folder and enter: /home/meareg/dlib_ws/src/blender_api$ 'blender -y Sophia.blend -P autostart.py'

