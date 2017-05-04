import pybullet as p
from simulation.interface.core import CtrlInterface

class IJoystick(CtrlInterface):

	def __init__(self, host, remote):
		# Default settings for camera
		super(IJoystick, self).__init__(host, remote)

	