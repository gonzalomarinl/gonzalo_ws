import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/gonzalomarin/gonzalo_ws/install/sancho_navigation'
