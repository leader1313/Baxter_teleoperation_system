// How to start PickCandy with Baxter
    // Hardware
    Turn on the Baxter(wait for the Baxter's head LED to turn green)
    Plug in the Robotiq grippers

    // Terminal 1
    roscore

    // Terminal 2		// for robotiq gripper
    cd catkin_ws
    ./baxter.sh
    rosrun robotiq_c_rtu robotiq_c_rtu_node.py gripper_left /dev/ttyUSB0   

    // Terminal 3  // for robotiq gripper
    cd catkin_ws
    ./baxter.sh
    rosrun robotiq_c_rtu robotiq_c_rtu_node.py gripper_right /dev/ttyUSB1

    // Terminal 4
    cd catkin_ws
    ./baxter.sh
    rosrun baxter_tools tuck_arms.py -u
    cd ../pbl2018_baxter
    python opencampus_pickcandy.py


//How to connect unity
roslaunch rosbridge_server rosbridge_websocket.launch 

// How to finish PickCandy with Baxter
    rosrun baxter_tools tuck_arms.py -t
    Plug out the Robotiq grippers   
    Turn off the Baxter



// How to use robotiq gripper
    // initialize
    rosrun robotiq_c_rtu robotiq_c_rtu_node.py gripper_left /dev/ttyUSB0
    rosrun robotiq_c_rtu robotiq_c_rtu_node.py gripper_right /dev/ttyUSB1
    (Left and right can be swapped. Make sure.)

    // publish
    rostopic pub -1 /robotiq/device_name/command/speed std_msgs/Int32 -- 0
    rostopic pub -1 /robotiq/device_name/command/force std_msgs/Int32 -- 0
    rostopic pub -1 /robotiq/device_name/command/position std_msgs/Int32 -- 255



// How to use baxter camera
rosrun image_view image_view image:=/cameras/left_hand_camera/image
//How to compressed image
#right_hand_camera
rosrun image_transport republish raw in:=/cameras/right_hand_camera/image compressed out:=/cameras/right_hand_camera/image
#left_hand_camera
rosrun image_transport republish raw in:=/cameras/left_hand_camera/image compressed out:=/cameras/left_hand_camera/image



// How to display the image on Baxter face monitor
rosrun baxter_examples xdisplay_image.py --file=`rospack find baxter_examples`/share/images/nbf1.jpg
