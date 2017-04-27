from bullet.control.interface import CtrlInterface


class ICmd(CtrlInterface):

	def __init__(self, host, remote):
		super(IVR, self).__init__(host, remote)

	def client_communicate(self, agent):
		pass

	def server_communicate(self, agent, scene, task, gui=True):
		pass

	def local_communicate(self, agent, gui=True):

		link_info = agent.get_tool_link_states(-1)
		# Set same number of controllers as number of arms/grippers
		agent.set_virtual_controller(range(len(link_info)))
		self.control_map = agent.create_control_mappings()

		self.pos = [list(i[0]) for i in link_info]
		self.pseudo_event = {0: 0, 3: 0.0}

		while True:

			events = self.server.read_msg()
			self._event_handler(events, agent)
			time.sleep(0.01)

			