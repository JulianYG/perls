from bullet.control.interface import CtrlInterface

class ICmd(CtrlInterface):

	def __init__(self, host, remote=False):
		super(ICmd, self).__init__(host, remote)
		self.socket = host

	def _remote_comm(self, model):
		pass

	def _local_comm(self, model):

		link_info = model.get_tool_link_states(-1)
		# Set same number of controllers as number of arms/grippers
		model.set_virtual_controller(range(len(link_info)))
		self.control_map = model.create_control_mappings()

		self.pos = [list(i[0]) for i in link_info]
		self.pseudo_event = {0: 0}

		while True:
			events = self.socket.read_msg()

			self._event_handler(events, model)	
			
			time.sleep(0.01)

			