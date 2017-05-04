from simulation.interface.core import CtrlInterface


class ICmd(CtrlInterface):

	def __init__(self, host, remote):
		super(IVR, self).__init__(host, remote)

	def server_communicate(self, agent, scene, task, gui=True):
		
		self.socket.connect_with_client()

		while True:
			events = self.socket.listen_to_client()
			for event in events:
				event = eval(event)
				try:
					if isinstance(event, dict):
						if 'pose' in event:
							agent.reach(event['id'], 
								event[pose][0], 
								event[pose][1], 
								fixed=False, 
								expedite=True)

				except IllegalOperation as e:
					illegal_operation_handler(e, self.socket)
					continue
			if not gui:
				p.stepSimulation()


	# def local_communicate(self, agent, gui=True):

	# 	link_info = agent.get_tool_link_states(-1)
	# 	# Set same number of controllers as number of arms/grippers
	# 	agent.set_virtual_controller(range(len(link_info)))
	# 	self.control_map = agent.create_control_mappings()

	# 	self.pos = [list(i[0]) for i in link_info]
	# 	self.pseudo_event = {0: 0, 3: 0.0}

	# 	while True:

	# 		events = self.server.read_msg()
	# 		self._event_handler(events, agent)
	# 		time.sleep(0.01)

	# 		