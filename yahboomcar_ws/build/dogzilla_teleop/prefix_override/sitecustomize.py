import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/clp44/dogzilla/yahboomcar_ws/install/dogzilla_teleop'
