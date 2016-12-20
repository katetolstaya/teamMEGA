# Team MEGA

Repos for Additional Components:
GUI https://github.com/katetolstaya/teammega_gui
Matlab Path Planner https://github.com/brent8149/Matlab_planner

Executing the Software

To run:

If you are using a simulator (jMavSim):

To use jmavsim:
cd ~/src/Firmware
make broadcast jmavsim

Otherwise, simply note the IP Address of your quadrotor on the network.

To run MAVROS: (Check the port / IP)
For simulator:
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
For Serial / USB:
roslaunch mavros px4.launch fcu_url:="serial:///dev/ttyACM0:921600"
For real drone over WIFI:
roslaunch mavros px4.launch fcu_url:="udp://:14550@192.168.4.1:14555"

Launch ROSbridge
roslaunch rosbridge_server rosbridge_websocket.launch

Launch web-app
open teammega_gui/index.html

Launch team_mega
rosrun teamMega teamMega

Launch matlab
matlab planner_sim.m

After the above commands have been executed, you will be able to click landing locations and obstacles in the GUI, and observe the planner creating appropriate trajectories for the system, and then successfully flying the quadrotors.

