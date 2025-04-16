import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/suba/my_ros2_ws/src/install/obstacle_av_custom'
