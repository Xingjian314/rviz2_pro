import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/liuxingjian/Desktop/proj/rviz2_pro/install/my_turtlebot3_launch'
