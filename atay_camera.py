import os
import cv2
import json
import numpy as np
from threading import Thread


class Camera:

    def __init__(self,cam_id,cam_path):

        self.cam_id = cam_id
        self.cam_path = cam_path
        self.cam_name = "ALI"
        self.cam_h = 1
        self.cam_height = 360
        self.cam_width = 640
        self.cam_Pitch = 0
        self.cam_Roll = 0
        self.cam_Yaw = 0
        self.cam_intrinsic = np.array([[1,0,0],[0,1,0],[0,0,1]])
        self.stopped = False
        self.frame = []
    
    def print(self):
        print("\n#######################\n")
        print("Camera ID: " + str(self.cam_id))
        print("Camera Path: " + str(self.cam_path))
        print("Camera NAME: " + self.cam_name)
        print("Camera Resolution(HW): " + "(" + str(self.cam_height) +","+str(self.cam_width)+")")
        print("Camera Height: " + str(self.cam_h) + " m")
        print("Camera Angles(RPY): " + "Roll: " + str(self.cam_Roll) + " " + "Pitch: " + str(self.cam_Pitch) + " " + "Yaw: " + str(self.cam_Yaw))
        print("Camera Instrinsic Matrix: \n\n" + str(self.cam_intrinsic))
        print("\n#######################\n")

    def start(self):
        self.stream = cv2.VideoCapture(self.cam_path,cv2.CAP_V4L)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH,self.cam_width)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT,self.cam_height)
        (self.grabbed, self.frame) = self.stream.read()    
        Thread(target=self.get, args=()).start()
        

    def get(self):
        while not self.stopped:
            if not self.grabbed:
                self.stop()
            else:
                (self.grabbed, self.frame) = self.stream.read()

    def stop(self):
        self.stopped = True

def camera_define(cls_cameras,cfg="camera_cfg.json"):

    with open(cfg,'r') as f:
        data = json.load(f)
    
    for camera in data["Cameras"]:
        for c in cls_cameras :
            if c.cam_id == camera["cam_id"]:
                c.cam_name = camera["cam_name"]
                c.cam_h = camera["cam_h"]
                c.cam_height = camera["cam_height"]
                c.cam_width = camera["cam_width"]
                c.cam_Pitch = camera["cam_Pitch"]
                c.cam_Roll = camera["cam_Roll"]
                c.cam_Yaw = camera["cam_Yaw"]
                c.cam_intrinsic = np.array(camera["cam_intrinsic"])
                c.print()

def camera_detect(Camera_ID):
    """
    returns: 
        camera_id   = camera_ids[x][0]
        camera_path = camera_ids[x][1] 
    """

    camera_ids = []

    try:
        devices = os.listdir("/dev/v4l/by-id")
    except Exception:
        print("Not Found Camera")
        exit(-1)
        
  
    NumberofCamera = len(devices)
    print("Found " + str(NumberofCamera) + " Camera")

    for device in devices:
        for C_ID in Camera_ID:
            camera_id = ((device.split("-")[1]).split("_")[-1])
            if C_ID == camera_id:
                camera_ids.append([camera_id,"/dev/v4l/by-id/" + device])

    tmp = Camera_ID

    if len(Camera_ID) == len(camera_ids):
        print("All Cameras Were Found.")
        #print("eşit")
    else:
        #print("eşit değil")
        for CID in Camera_ID: #Fonksiyondan gelen
            for C_ID in camera_ids:# Bulunan
                if C_ID[0] == CID:
                    tmp.remove(C_ID[0])
        
        for not_found in tmp:
            print("ID: " + not_found + " " + "Camera Not Found")
        exit(-1)

    #print(camera_ids)

    return(camera_ids)