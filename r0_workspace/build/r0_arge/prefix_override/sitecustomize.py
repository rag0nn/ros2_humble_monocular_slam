import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rag0n/Desktop/lab/r0_workspace/install/r0_arge'
