from simulation.interface.core import CtrlInterface


class ICmd(CtrlInterface):

	def __init__(self, host, remote):
		super(ICmd, self).__init__(host, remote)

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

	def client_communicate(self):

		self.socket.connect_with_server()
		
		if not agent.controllers:
			tools = agent.get_tool_ids()
			agent.set_virtual_controller(range(len(tools)))

		control_map, obj_map = agent.create_control_mappings()
		# Let the socket know controller IDs
		self.socket.broadcast_to_server((CTRL_HOOK, agent.controllers))

		while True:
			# Send to server
			event = p.getKeyboardEvents()
			# Make fake button events
			event[6] = {1: 0}
			self.socket.broadcast_to_server(event)

			# Receive and render from server
			signal = self.socket.listen_to_server()
			for s in signal:
				s = eval(s)
				self._signal_loop(s, agent, control_map, obj_map)

			time.sleep(0.01)
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