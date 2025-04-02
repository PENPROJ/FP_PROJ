import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mrl-seuk/sitl_crazy/CrazySim/ros2_ws/install/crazyflie_mpc'
