import pybullet as p
import csv
import time

p.connect(p.SHARED_MEMORY)
p.setInternalSimFlags(0)  # don't load default robot assets etc
p.resetSimulation()

p.setGravity(0,0,-9.8)

p.loadURDF("table/table.urdf", 1.000000,-0.200000,0.000000,0.000000,0.000000,0.707107,0.707107)
p.loadURDF("jenga/jenga.urdf", 1.300000,0.200000,0.699990,-0.000005,0.707107,0.000006,0.707107)
p.loadURDF("jenga/jenga.urdf", 1.200000,0.200000,0.699990,-0.000005,0.707107,0.000006,0.707107)
p.loadURDF("jenga/jenga.urdf", 1.100000,0.200000,0.699990,-0.000005,0.707107,0.000006,0.707107)
p.loadURDF("rl/pole.urdf",0.80000,0.100000,0.699990,0.00000,0.0,0.00000,1)
p.loadURDF("rl/pole.urdf",0.80000,-0.200000,0.699990,0.000000,0.0,0.00000,1)
p.loadURDF("rl/pole.urdf",0.80000,-0.500000,0.699990,0.00000,0.0,0.00000,1)
ob_start = p.loadURDF("rl/torus_0.urdf",0.8,0.1,0.69999,1,0,0,1)
p.loadURDF("rl/torus_1.urdf",0.8,0.1,0.74999,1,0,0,1)
p.loadURDF("rl/torus_2.urdf",0.8,0.1,0.79999,1,0,0,1)
p.loadURDF("rl/torus_3.urdf",0.8,0.1,0.84999,1,0,0,1)
ob_end = p.loadURDF("rl/torus_4.urdf",0.8,0.1,0.89999,0,0,0,1)
p.loadURDF("plane.urdf",0,0,0,0,0,0,1)

robot = p.loadURDF('kuka_iiwa/model_vr_limits.urdf', 1.4,-0.2,0.6,0,0,0,1)
lower_limit = [-.967,-2.,-2.96,.19,-2.96,-2.09,-3.05]
upper_limit = []

gripper = p.loadSDF('gripper/wsg50_one_motor_gripper_new_free_base.sdf')
f = open('trajectory0113-111606.csv', 'r')
reader = csv.reader(f)
delay = 0
try:
	for row in reader:
		if int(row[0]) == -1:
			delay = float(row[1])
		else:
			time.sleep(delay)
			if int(row[0]) != 3:
				p.resetBasePositionAndOrientation(int(row[0]) - 4, (float(row[1]), float(row[2]), float(row[3])), \
					(float(row[4]), float(row[5]), float(row[6]), float(row[7])))
			else:
				eef_pos = (float(row[1]), float(row[2]), float(row[3]))
				eef_orien = (float(row[4]), float(row[5]), float(row[6]), float(row[7]))
				joint_pos = p.calculateInverseKinematics(robot, 6, eef_pos, eef_orien)

				for i in range(len(joint_pos)):
					p.resetJointState(robot, i, joint_pos[i])

except KeyboardInterrupt:
	p.disconnect()
	f.close()

