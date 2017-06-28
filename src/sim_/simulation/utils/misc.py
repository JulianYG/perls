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

	HOT_KEYS = [32, 49, 50, 120, 121, 122, 65295, 65296, 65297, 65298, 65306]


class Key():

	X = 120
	Y = 121
	Z = 122
	S = p.B3G_SHIFT
	G = 32
	R = 100
	CR = 102

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
	def SHIFT(e):
		return e == p.B3G_SHIFT
