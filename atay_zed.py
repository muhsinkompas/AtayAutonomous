import cv2
import statistics
import numpy as np
import pyzed.sl as sl

def get_object_z(point_cloud, bounds,area_div = 2):
    '''
    Calculates the median z position of top slice(area_div) of point cloud
    in camera frame.
    Arguments:
        point_cloud: Point cloud data of whole frame.
        bounds: Bounding box for object in pixels.
            bounds[0]: x-min
            bounds[1]: y-min
            bounds[2]: x-max
            bounds[3]: y-max
    Return:
        z: Location of object in meters.
    '''

    z_vect = []

    x_center = int((bounds[0]+bounds[2])/2)
    y_center = int((bounds[1]+bounds[3])/2)

    for j in range(x_center - area_div, x_center + area_div):
        for i in range(y_center - area_div, y_center + area_div):
            z = point_cloud[i, j, 2]
            if not np.isnan(z) and not np.isinf(z):
                z_vect.append(z)
    try:
        z_median = statistics.median(z_vect)
    except Exception:

        z_median = -1
        pass

    return z_median

class ATAY_ZED:
    
    def __init__(self):
        
        self.zed = sl.Camera()
        self.init = sl.InitParameters()
        self.init.camera_resolution = sl.RESOLUTION.HD720
        self.init.depth_mode = sl.DEPTH_MODE.QUALITY
        self.init.coordinate_units = sl.UNIT.METER
        err = self.zed.open(self.init)
        if err != sl.ERROR_CODE.SUCCESS :
            print(repr(err))
            self.zed.close()
            exit(1)
        # Set runtime parameters after opening the camera
        self.runtime = sl.RuntimeParameters()
        self.runtime.sensing_mode = sl.SENSING_MODE.STANDARD
        self.image_size = self.zed.get_camera_information().camera_resolution
        self.image_width = self.image_size.width
        self.image_height = self.image_size.height
        # Declare your sl.Mat matrices
        self.cuda_image = sl.Mat(self.image_width, self.image_height, sl.MAT_TYPE.U8_C4)
        #self.depth_image_zed = sl.Mat(self.image_width, self.image_height, sl.MAT_TYPE.U8_C4)
        self.cuda_point_cloud = sl.Mat()
        self.image = np.zeros(shape=(self.image_height,self.image_width,4),dtype=np.uint8)
        self.point_cloud = np.zeros(shape=(self.image_height,self.image_width,3),dtype=np.uint8)
    
    def Grab(self):
        "ALLLLII"
        err = False
        if self.zed.grab(self.runtime) == sl.ERROR_CODE.SUCCESS :
            # Retrieve the left image
            self.zed.retrieve_image(self.cuda_image,sl.VIEW.LEFT,sl.MEM.CPU,self.image_size)
            # Retrieve the RGBA point cloud
            self.zed.retrieve_measure(self.cuda_point_cloud,sl.MEASURE.XYZ, sl.MEM.CPU,self.image_size)
            # To recover data from sl.Mat to use it with opencv, use the get_data() method
            # It returns a numpy array that can be used as a matrix with opencv
            self.image = self.cuda_image.get_data()
            self.point_cloud = self.cuda_point_cloud.get_data()
            err = True

        return err

    def Close(self):
        self.zed.close()
