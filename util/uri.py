import rtde_receive
import rtde_control
import util.robotiq_gripper as robotiq_gripper

HOST = "192.168.0.103"
GRIPPER_PORT = 63352

class RMPLAB_Uri(object):
    def __init__(self, host=None):
        if host is None:
            host = HOST
        self.host = host
        self.control = None
        self.recieve = None
        self.gripper = robotiq_gripper.RobotiqGripper()

    def connect(self, gripper_calibrate=True):
        self.control = rtde_control.RTDEControlInterface(self.host)
        self.recieve = rtde_receive.RTDEReceiveInterface(self.host)
        self.gripper.connect(self.host, GRIPPER_PORT)
        self.gripper.activate(gripper_calibrate)  