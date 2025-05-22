import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pato/Desktop/HoverRobotNavigation/install/ros2_ps5_stereo'
