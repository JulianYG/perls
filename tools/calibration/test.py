

from camera_calibrator import RobotCalibrator as RC
from camera_calibrator import HybridCalibrator as HC
from camera import *


# USB Camera matrix 
_UK = np.array([
	[600.153387, 0, 315.459915], 
	[0, 598.015225, 222.933946], 
	[0,          0,          1]
	], np.float32)

# USB Camera Distortion
_UD = np.array([0.147084, -0.257330, 
	0.003032, -0.006975, 0.000000], np.float32)


# dc = DC(0, 'right_hand_camera', (9, 6), 
# 		0.026, '../calib_data', (640, 480), (752, 480), 
# 		calib_min=15)
# dc.calibrate()

# camera = RobotCamera('right_hand_camera')
# rc = RC(camera, (9,6 ), 0.026, '../calib_data')

# left_cam = UVCCamera(0)

left_cam = Kinect('hd', dimension=(1920, 1080))

# left_cam.snapshot(dict(
# 			board_size=(9, 6),
# 			num_of_points=54,
# 			directory='../calib_data'
# 			))


right_cam = RobotCamera('right_hand_camera')
hs = KinectRobotStereo(left_cam, right_cam)
# hr = UVCRobotStereo(left_cam, right_cam)

hc = HC(hs, (9,6 ), 0.026, '../calib_data', calib_min=2)
# hc = HC(hr, (9,6 ), 0.026, '../calib_data', calib_min=5)


hc.calibrate()