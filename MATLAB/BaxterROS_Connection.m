% Brendan Cook's Gazebo connection script

my_laptop_IP = '192.168.0.2';                             %IP address of the computer running MATLAB
global_MATLAB_node_name = '/matlab_global_node1';            %Unique name for the MATLAB node (optional)
Master_Node_Name = '192.168.0.101';                            %IP address of the virtual machine

rosshutdown;                                                %Shutdown any potentially running ROS

rosinit(Master_Node_Name, 'NodeHost', my_laptop_IP, ...
                     'NodeName', global_MATLAB_node_name);  %Initiate ROS node for MATLAB