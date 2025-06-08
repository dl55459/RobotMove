import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rob/ros2_ws/src/prarob_vision/install/prarob_vision'
