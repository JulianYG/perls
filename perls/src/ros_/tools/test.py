

from camera_calibration import DuoCalibrator as DC




# USB Camera matrix 
_UK = np.array([
	[600.153387, 0, 315.459915], 
	[0, 598.015225, 222.933946], 
	[0,          0,          1]
	], np.float32)

# USB Camera Distortion
_UD = np.array([0.147084, -0.257330, 
	0.003032, -0.006975, 0.000000], np.float32)


dc = DC(0, 'right_hand_camera', (9, 6), 
		0.026, '../calib_data', (640, 480), (752, 480), 
		calib_min=15)
dc.calibrate()
