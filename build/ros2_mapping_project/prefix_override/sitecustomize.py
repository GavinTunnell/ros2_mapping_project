import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/team4/Desktop/ros2_mapping_project/install/ros2_mapping_project'
