import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/syafihidayat/Documents/robot_smoothing/install/robot_gui_pkg'
