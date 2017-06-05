import numpy as np
import pybullet as p

def get_distance(posA, posB):
	return np.sqrt(np.sum((np.array(posA) - np.array(posB)) ** 2))

class Constant():

	WARNING_HOOK = -1
	VEL_CTRL = 0
	TORQ_CTRL = 1
	POS_CTRL = 2

	ARM = 3
	GRIPPER = 4
	CONSTRAINT = 5

	SHUTDOWN_HOOK = 6
	RESET_HOOK = 7
	START_HOOK = 8
	CTRL_HOOK = 9

	HOT_KEYS = [32, 49, 50, 111, 120, 121, 122, 65295, 65296, 65297, 65298]


class Key():

	X = 120
	Y = 121
	Z = 122
	O = 111
	G = 32

	@staticmethod
	def ONE(e):
		return e == 49

	@staticmethod
	def TWO(e):
		return e == 50

	@staticmethod
	def UP(e):
		return e == p.B3G_UP_ARROW

	@staticmethod
	def DOWN(e):
		return e == p.B3G_DOWN_ARROW

	@staticmethod
	def LEFT(e):
		return e == p.B3G_LEFT_ARROW

	@staticmethod
	def RIGHT(e):
		return e == p.B3G_RIGHT_ARROW

	@staticmethod
	def SPACE(e):
		return e == 32

	@staticmethod
	def PITCH_CCW(e):
		return e == p.B3G_UP_ARROW

	@staticmethod
	def PITCH_CW(e):
		return e == p.B3G_DOWN_ARROW

	@staticmethod
	def ROLL_CCW(e):
		return e == p.B3G_LEFT_ARROW

	@staticmethod
	def ROLL_CW(e):
		return e == p.B3G_RIGHT_ARROW

	# up: 65298 down: 65297 left: 65295 right: 65296
			# c: 99 r: 114 o: 111


