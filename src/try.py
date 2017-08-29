# #!/usr/bin/env python
# import sys
# import termios
# import contextlib


# @contextlib.contextmanager
# def raw_mode(file):
#     old_attrs = termios.tcgetattr(file.fileno())
#     new_attrs = old_attrs[:]
#     new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
#     try:
#         termios.tcsetattr(file.fileno(), termios.TCSADRAIN, new_attrs)
#         yield
#     finally:
#         termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)


# def main():
#     print ('exit with ^C or ^D')
#     with raw_mode(sys.stdin):
#         try:
#             while True:
#                 ch = sys.stdin.read(1)
#                 if not ch or ch == chr(4):
#                     break
#                 print ('%02x' % ord(ch),)
#         except (KeyboardInterrupt, EOFError):
#             pass


# if __name__ == '__main__':
#     main()


import sys
import time
import openvr

vr_system = openvr.init(openvr.VRApplication_Scene)
vr_compositor = openvr.IVRCompositor()

# print(dir(vr_system))
x = 0
for i in range(100):
	# poses = vr_system.getDeviceToAbsoluteTrackingPose(
	#     openvr.TrackingUniverseStanding,
	#     0,
	#     openvr.k_unMaxTrackedDeviceCount)
	# pose = poses[5]
	# vr_compositor.waitGetPoses(poses, openvr.k_unMaxTrackedDeviceCount, None, 0)  
	# print(pose.mDeviceToAbsoluteTracking)
	event_struct = openvr.VREvent_t()

	event = vr_system.pollNextEvent(event_struct)

	state =  openvr.VRControllerState_t()

	result, x = vr_system.getControllerState(3, openvr.sizeof(openvr.VRControllerState_t))
	
	touchpad_x, touchpad_y = x.rAxis[0].x, x.rAxis[0].y # in range [-1, 1]
	gripper_press = x.rAxis[1].x # in range [0, 1], 0 is unpressed, 1 is pressed

	print("touchpad_x: {}".format(touchpad_x))
	print("touchpad_y: {}".format(touchpad_y))
	print("gripper_press: {}".format(gripper_press))

	if event:
		info = event_struct.data
		buttonID = info.controller.button # identifies button

		device_idx = event_struct.trackedDeviceIndex
		if event_struct.eventType == openvr.VREvent_ButtonPress:
			
			if buttonID == openvr.k_EButton_ApplicationMenu:
				print('should reset!')

			elif buttonID == openvr.k_EButton_Grip:
				print("Grip was pressed!")
			elif buttonID == openvr.k_EButton_SteamVR_Touchpad:
				print("Touchpad was pressed!")
			elif buttonID == openvr.k_EButton_SteamVR_Trigger:
				print("Trigger was pressed!")


		elif event_struct.eventType == openvr.VREvent_ButtonUnpress:

			if buttonID == openvr.k_EButton_System:
				print('sdfsdfoij')
			elif buttonID == openvr.k_EButton_Grip:
				print("Grip was unpressed!")
			elif buttonID == openvr.k_EButton_SteamVR_Touchpad:
				print("Touchpad was unpressed!")
			elif buttonID == openvr.k_EButton_SteamVR_Trigger:
				print("Trigger was unpressed!")

		elif event_struct.eventType == openvr.VREvent_ButtonTouch:

			if buttonID == openvr.k_EButton_ApplicationMenu:
				print('should resetsdfdsf!')
			elif buttonID == openvr.k_EButton_Grip:
				print("Grip was touched!")
			elif buttonID == openvr.k_EButton_SteamVR_Touchpad:
				print("Touchpad was touched!")
			elif buttonID == openvr.k_EButton_SteamVR_Trigger:
				print("Trigger was touched!")


		elif event_struct.eventType == openvr.VREvent_ButtonUntouch:
			if buttonID == openvr.k_EButton_Grip:
				print("Grip was untouched!")
			elif buttonID == openvr.k_EButton_SteamVR_Touchpad:
				print("Touchpad was untouched!")
			elif buttonID == openvr.k_EButton_SteamVR_Trigger:
				print("Trigger was untouched!")


	



	# connected, state = vr_system.getControllerState(1, openvr.sizeof(openvr.VRControllerState_t))
	# print()
	# y = int(state.unPacketNum)
	# if y != x:
	# print(dir(event))
		# y = x

	# print(vr_system.pollNextEvent(openvr.TrackingUniverseStanding))
	# print(pose.bDeviceIsConnected, dir(state.rAxis), state.ulButtonPressed, state.ulButtonTouched, state.unPacketNum)
	# print([vr_system.getTrackedDeviceClass(i) for i in range(16)])
	# print(dir(state), dir(pose), pose.mDeviceToAbsoluteTracking)


	sys.stdout.flush()
	time.sleep(0.2)

openvr.shutdown()