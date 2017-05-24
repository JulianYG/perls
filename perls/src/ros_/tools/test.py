

from camera_calibration import DuoCalibrator as DC





dc = DC(0, 'right_hand_camera', (9, 6), 
		0.026, '../calib_data', (640, 480), (752, 480), 
		calib_min=15)
dc.calibrate()
