import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/workspace/share/ros2-workspace/trung_test/ur3_needle_ws/install/ur3_needle_sim'
