from bullet.control.interface import CtrlInterface

class IVR(CtrlInterface):

	def __init__(self, remote):
		# Default settings for camera
		super(IVR, self).__init__(remote)

	def _remote_comm(self, pybullet, model):
		pass

	def _local_comm(self, pybullet, model):
		control_map = model.create_control_mappings()
		while True:
			events = pybullet.getVREvents()
			skip_flag = model.redundant_control()
			for e in (events):
				if skip_flag:
					if e[0] == model.controllers[1]:
						break
					model.control(e, control_map)
				else:
					model.control(e, control_map)

