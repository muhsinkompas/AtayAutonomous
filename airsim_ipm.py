import cv2
import numpy as np
import math
import time
from numpy.linalg import inv

def ImToVehicleTransform(self):
    tform = tformToVeh(self)
    return tform

def tformToVeh(self):
    vehicleToImageTform = tformToIm(self)
    invtform = invertMK(vehicleToImageTform)
    return invtform

def tformToIm(self):
    camMatrix = rawtformToIm3D(self)
    cam1 = camMatrix[0,:]
    cam2 = camMatrix[1,:]
    cam3 = camMatrix[3,:]
    tform2D = np.vstack((cam1,cam2,cam3))
    tform = projective2dMK(tform2D)
    return tform

def rawtformToIm3D(self):
    translation = translationVect(self)
    rotation    = rotationMat(self)
    r = rotation
    t = translation
    tform = np.vstack((r,t))*self.IntrinsicMatrix
    return tform

def translationVect(self):
    rotationMat = rotZ(-self.Yaw)*rotX(90-self.Pitch)*rotZ(self.Roll)
    sl = self.SensorLocation
    translationInWorldUnits = np.hstack((sl[1], sl[0], self.height))
    translation = translationInWorldUnits*rotationMat
    return translation

def rotationMat(self):
    rotation = rotY(180)*rotZ(-90)*rotZ(-self.Yaw)*rotX(90-self.Pitch)*rotZ(self.Roll)
    return rotation

def projective2dMK(A):
    T = A
    return T

def invertMK(Vehicle):
    T = Tinv(Vehicle)
    invtform = T
    return invtform

def Tinv(VehicleT):
    Tinv = inv(VehicleT)
    return Tinv

def rotX(a):
    a = np.radians(a)
    R = np.matrix([[1,0,0],[0,math.cos(a),-math.sin(a)],[0,math.sin(a),math.cos(a)]])
    return R

def rotY(a):
    a = np.radians(a)
    R = np.matrix([[math.cos(a),0,math.sin(a)],[0,1,0],[-math.sin(a),0,math.cos(a)]])
    return R

def rotZ(a):
    a = np.radians(a)
    R = np.matrix([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])
    return R

def DefineRout(self,idealWorldLimits):
    outputresolutionscale = 1
    idealWorldLimitsX = np.array([idealWorldLimits[0,:]])
    idealWorldLimitsY = np.array([idealWorldLimits[1,:]])
    orx = outputresolutionscale
    ory = outputresolutionscale
    diffx = abs(idealWorldLimitsX[0,0]-idealWorldLimitsX[0,1])
    diffy = abs(idealWorldLimitsY[0,0]-idealWorldLimitsY[0,1])
    numCols = math.ceil(diffx/orx)
    numRows = math.ceil(diffy/ory)
    xNudge = (numCols*orx-diffx)/2
    yNudge = (numRows*ory-diffy)/2
    WorldLimitsOutX = idealWorldLimitsX+np.array([[-xNudge,xNudge]])
    WorldLimitsOutY = idealWorldLimitsY+np.array([[-yNudge,yNudge]])
    OutputImageSize = (numCols,numRows)
    #print(OutputImageSize)
    self.outputImageSize[0] = OutputImageSize[0]
    self.outputImageSize[1] = OutputImageSize[1]
    Rout = np.vstack((WorldLimitsOutX,WorldLimitsOutY))
    return Rout

def getSourceMappingInvertible2d(self,Rout,M,orx,ory):
    Sx = orx
    Sy = ory
    Tx = Rout[0,0]-orx*0.5
    Ty = Rout[1,0]-ory*0.5
    tIntrinsictoWorldOutput = np.array([[Sx,0,0],[0,Sy,0],[Tx,Ty,1]])
    M = inv(M)
    tComp = tIntrinsictoWorldOutput*M
    return tComp

def DefineWorldLimits(self,M):

    x1 = self.InputImageSize[1]
    y1 = self.InputImageSize[0]
    XWorldLimits = np.array([[0,x1]])+0.5
    YWorldLimits = np.array([[0,y1]])+0.5
    u = np.array([[XWorldLimits[0,0],np.mean(XWorldLimits),XWorldLimits[0,1]]])
    v = np.array([[YWorldLimits[0,0],np.mean(YWorldLimits),YWorldLimits[0,1]]])
    U = np.vstack((u,u,u))
    V = np.vstack((v,v,v))
    V = V.T
    x = M[0,0]*U+M[1,0]*V+M[2,0]
    y = M[0,1]*U+M[1,1]*V+M[2,1]
    z = M[0,2]*U+M[1,2]*V+M[2,2]
    X = x/z
    Y = y/z
    XWorldLimitsOut = np.array([[np.amin(X),np.amax(X)]])
    YWorldLimitsOut = np.array([[np.amin(Y),np.amax(Y)]])
    WorldLimitsOut = np.vstack((XWorldLimitsOut,YWorldLimitsOut))
    return WorldLimitsOut

def rotationmatrix(self):
    height = self.outputImageSize[0]
    width = self.outputImageSize[1]
    rotate_center = (height//2, width)

    self.rotation_mat = cv2.getRotationMatrix2D(rotate_center,self.Rotation_Angle, 1.)
    abs_cos = abs(self.rotation_mat[0,0])
    abs_sin = abs(self.rotation_mat[0,1])

    self.bound_w = int(height * abs_sin + width * abs_cos)
    self.bound_h = int(height * abs_cos + width * abs_sin)

    self.rotation_mat[0, 2] += self.bound_w//2 - rotate_center[0]
    self.rotation_mat[1, 2] += self.bound_h - rotate_center[1]


class IPM:

    def __init__(self ,Roll, Pitch, Yaw , SensorLocation, Rotation_Angle, IntrinsicMatrix, height, outImageSize=[None,640]):
        #self.image = image
        self.InputImageSize = [360,640]
        self.outImageSize = outImageSize
        self.Roll = Roll
        self.Pitch = Pitch
        self.Yaw = Yaw
        self.height = height
        self.Rotation_Angle = Rotation_Angle
        self.SensorLocation = SensorLocation

        self.IntrinsicMatrix = IntrinsicMatrix


        
        # self.FocalLength = np.array([391.9969,391.4284])
        # self.PrincipalPoint = np.array([320.5066, 182.5292])
        self.cameraimageSize = np.array([360,640])

        self.xmax = 2.09
        self.yshift = 1.5
        self.xmin = 0.25


        self.outView = np.array([self.xmin, self.xmax, -self.yshift, self.yshift])
        self.outputImageSize = self.InputImageSize.copy()
        rotationmatrix(self)
        self.Transform = IPM.DefineTform(self)
        print(self.Transform)

    def DefineTform(self):

        vehicleHomographyMK = ImToVehicleTransform(self)

        adjTform = np.array([[0, -1,  0],[-1,  0,  0],[0,  0,  1]])

        bevTform = vehicleHomographyMK*adjTform
        x0 = self.outView[1]-self.outView[0]
        y0 = self.outView[3]-self.outView[2]

        worldHW  = np.array([abs(x0),abs(y0)])
        reqImgHW = (self.outImageSize)
        nanIdxHW = np.array([ True, False])
        scale   = (reqImgHW[1]-1)/worldHW[1]
        scaleXY = np.array([scale, scale])
        dYdXVehicle = np.array([self.outView[3], self.outView[1]])
        tXY = np.array([scaleXY[0]*dYdXVehicle[0],scaleXY[1]*dYdXVehicle[1]])
        viewMatrix = np.array([[scaleXY[0],0,0],[0,scaleXY[1],0],[tXY[0]+1,tXY[1]+1,1]])
        T = bevTform*viewMatrix
        tform = projective2dMK(T)
        WorldLimits = DefineWorldLimits(self,tform)
        Rout = DefineRout(self,WorldLimits) #rout bulurken outimagesize'i revize et
        orx = 1
        ory = 1
        tform = getSourceMappingInvertible2d(self,Rout,tform,orx,ory)
        return tform

    def BirdEye(self,input_image):

        Gray_Image = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)
        Blur_Image = cv2.GaussianBlur(Gray_Image, (5, 5), 0)
        Median_Blur = cv2.medianBlur(Blur_Image,5)
        Sobel_Image = cv2.Sobel(Median_Blur, -1, 1, 1)
        th3 = cv2.adaptiveThreshold(Sobel_Image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,37,2)
        Result = cv2.warpPerspective(th3, M=self.Transform.T, dsize=(self.outputImageSize[0],self.outputImageSize[1]), flags=(cv2.WARP_INVERSE_MAP + cv2.INTER_CUBIC))
        #print(Result.shape)
        #Result = cv2.resize(Result,None,fx=0.5, fy=0.5)

        Result_RGB = cv2.warpPerspective(input_image, M=self.Transform.T, dsize=(self.outputImageSize[0],self.outputImageSize[1]), flags=(cv2.WARP_INVERSE_MAP + cv2.INTER_CUBIC))
        #Result_RGB = cv2.warpPerspective(input_image, M=self.Transform.T, dsize=(628,764), flags=(cv2.WARP_INVERSE_MAP + cv2.INTER_CUBIC))
        #Result_RGB = cv2.resize(Result_RGB,None,fx=0.5, fy=0.5)
        #Result_RGB = cv2.resize(Result_RGB,(1192, 839))
        
        #Result_RGB = cv2.warpAffine(Result_RGB, self.rotation_mat, (self.bound_h, self.bound_w)) 
        return Result_RGB,Result
