
import numpy as np
import cv2

KINECT_DEPTH_SHIFT = -22.84013555237548
GRIPPER_SHIFT = 0.0251


class KinectCamera(object):


    def __init__(self, intrinsics=None, distortion=None):

        import pylibfreenect2
        from pylibfreenect2 import Freenect2, SyncMultiFrameListener
        from pylibfreenect2 import FrameType, Registration, Frame
        from pylibfreenect2 import createConsoleLogger, setGlobalLogger
        from pylibfreenect2 import LoggerLevel
        from pylibfreenect2.libfreenect2 import IrCameraParams, ColorCameraParams

        try:
            from pylibfreenect2 import OpenCLPacketPipeline
            pipeline = OpenCLPacketPipeline()
        except:
            try:
                from pylibfreenect2 import OpenGLPacketPipeline
                pipeline = OpenGLPacketPipeline()
            except:
                from pylibfreenect2 import CpuPacketPipeline
                pipeline = CpuPacketPipeline()
            
        self._intrinsics_RGB = intrinsics
        self._distortion_RGB = distortion

        # Init kinect camera
        fn = Freenect2()
        num_devices = fn.enumerateDevices()
        if num_devices == 0:
            print("No device connected!")
            sys.exit(1)

        serial = fn.getDeviceSerialNumber(0)
        self._fn = fn
        self._serial = serial

        self._device = self._fn.openDevice(self._serial, pipeline=pipeline)

        self._listener = SyncMultiFrameListener(
            FrameType.Color | FrameType.Ir | FrameType.Depth)

        # Register listeners
        self._device.setColorFrameListener(self._listener)
        self._device.setIrAndDepthFrameListener(self._listener)
        self._device.start()

        # NOTE: must be called after device.start()
        if self._intrinsics_RGB is None:

            self._intrinsics_RGB = self._to_matrix(self._device.getColorCameraParams())

            self._registration = Registration(self._device.getIrCameraParams(), 
                self._device.getColorCameraParams())
        else:
            self._registration = Registration(
                            self._device.getIrCameraParams(),
                            self._device.getColorCameraParams())

        self._undistorted = Frame(512, 424, 4)
        self._registered = Frame(512, 424, 4)

        self._big_depth = Frame(1920, 1082, 4)
        self._color_depth_map = np.zeros((424, 512),  np.int32).ravel()
        print self._color_depth_map.shape


    def get_image(self):
        try:
            self._listener.release(self._frames)
        except:
            pass
        print("WAITING FOR FRAME")
        self._frames = self._listener.waitForNewFrame()
        print("GOT FRAME")

        color, ir, depth = \
            self._frames[pylibfreenect2.FrameType.Color],\
            self._frames[pylibfreenect2.FrameType.Ir], \
            self._frames[pylibfreenect2.FrameType.Depth]

        self._registration.apply(color, depth, 
            self._undistorted,
            self._registered, 
            bigdepth=self._big_depth, 
            color_depth_map=self._color_depth_map)


        color = cv2.flip(color.asarray(), 1)
        color = cv2.undistort(color, self._intrinsics_RGB, self._distortion_RGB)

        d = self._big_depth.asarray(np.float32)
        d = cv2.flip(d[1:-1], 1)
        # print(depth_map)
        self._listener.release(self._frames)
        # self._device.close()
        return color, d

