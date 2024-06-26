import subprocess, signal, os
import numpy as np
from copy import deepcopy 


from visualization_msgs.msg import Marker, MarkerArray

class GazeboInterface():
    def __init__(self, stage):
        self.stage = stage
        self.devnull = open(os.devnull, 'wb')
        self.gz_iface = subprocess.Popen("ros2 launch FYP_MK01" + self.stage + ".py headless:=False",
                                        stdout=subprocess.PIPE,
                                        shell=True,
                                        preexec_fn=os.setsid,
                                        stderr=self.devnull)
    def __del__(self):
        os.killpg(os.getpgid(self.gz_iface.pid), signal.SIGINT)

