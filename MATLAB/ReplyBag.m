bagFilePath = 'C:\Users\Owner\Documents\2016-09-12-13-20-38.bag';
disp('Loading . . .')
bag = rosbag(bagFilePath);
disp('Complete')

odomTopic = select(bag, 'Topic', '/odometry/filtered');
scanTopic = select(bag, 'Topic', '/scan');
handCameraTopic = select(bag, 'Topic', 'cameras/right_hand_camera/image');
headCameraTopic = select(bag, 'Topic', 'cameras/head_camera/image');
IRTopic = select = select(bag,'Topic', '/robot/analog_io/right_hand_range/value_uint32');
velCmdTopic = select(bag, 'Topic', '/mobility_base/cmd_vel');


