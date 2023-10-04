import cv2
import time
import numpy as np
import atay_carcontrol
from multiprocessing import Process, Pipe
from airsim_steerangle import CalculateSteerAngle,CalculateRoute
import airsim_yolocontroller 
import threading

steerAngle = 0
#OD_status = 0
#LD_status = 0

class m_status:
    def __init__(self):
        self.LD_status = 1
        self.OD_status = 1




def LaneDetection(conn, mk):
    
    global M_Result 
    global M_RGB_Result 
    from atay_camera import Camera,camera_detect,camera_define
    import airsim_ipm
    def Lane_Fitting(Lane,pol_order):
        return np.poly1d(np.polyfit(Lane[:,0],Lane[:,1] , pol_order))
    LaneOffsetA = 400
    LaneOffsetU = 90
    Left_ID  = "2793557F"
    Right_ID = "398D6D4F"
    Camera_ID = [Left_ID,Right_ID]

    camera_ids = camera_detect(Camera_ID) #Retuns ID and Path (Oreder of Entry)(first output index first input index)
    camera_id_list = []
    for cc in range(len(camera_ids)):
	    camera_id_list.append(camera_ids[cc][0])
    Left_Camera = Camera(cam_id=camera_ids[camera_id_list.index(Left_ID)][0],cam_path=camera_ids[camera_id_list.index(Left_ID)][1])
    Right_Camera = Camera(cam_id=camera_ids[camera_id_list.index(Right_ID)][0],cam_path=camera_ids[camera_id_list.index(Right_ID)][1])
        
    
    camera_define([Left_Camera,Right_Camera])
    Right_Camera.start()
    Left_Camera.start()
    
    IPM_Left  = airsim_ipm.IPM(Roll = Left_Camera.cam_Roll,  Pitch = Left_Camera.cam_Pitch,Yaw =  Left_Camera.cam_Yaw, SensorLocation = [0.1,0],Rotation_Angle = 0, IntrinsicMatrix = Left_Camera.cam_intrinsic, height = Left_Camera.cam_h)
    IPM_Right  = airsim_ipm.IPM(Roll = Right_Camera.cam_Roll,  Pitch = Right_Camera.cam_Pitch,Yaw =  Right_Camera.cam_Yaw, SensorLocation = [0,0],Rotation_Angle = 0, IntrinsicMatrix = Right_Camera.cam_intrinsic, height = Right_Camera.cam_h)

    RC_Frame = Right_Camera.frame
    LC_Frame = Left_Camera.frame

    IPM_Left_out_RGB , IPM_Left_out_Result = IPM_Left.BirdEye(input_image = LC_Frame)
    IPM_Right_out_RGB , IPM_Right_out_Result = IPM_Right.BirdEye(input_image = RC_Frame)
    outputoffset_L = 60
    outputoffset_R = 60
    outputoffset_H = 464
    outputoffset = outputoffset_L+outputoffset_R
    y_offset_L = 0
    y_offset_R = 0
    
    r_out_y = IPM_Right_out_RGB.shape[0]
    r_out_x = IPM_Right_out_RGB.shape[1]
    l_out_y = IPM_Left_out_RGB.shape[0]
    l_out_x = IPM_Left_out_RGB.shape[1]


    #RGB_Result = np.zeros((min(r_out_y,l_out_y),r_out_x+l_out_x-outputoffset,3), np.uint8)
    RGB_Result = np.zeros((min(r_out_y,l_out_y),r_out_x+l_out_x-outputoffset-outputoffset_H,3), np.uint8)
    Result = np.zeros((min(r_out_y,l_out_y),r_out_x+l_out_x-outputoffset-outputoffset_H), np.uint8)


    rgb_out_y = RGB_Result.shape[0]
    rgb_out_x = RGB_Result.shape[1]   
    
    RGB_Result[y_offset_R:,0:r_out_x-outputoffset_L-outputoffset_H,] = IPM_Right_out_RGB[r_out_y-rgb_out_y+y_offset_R:r_out_y,outputoffset_H:r_out_x-outputoffset_L,]
    RGB_Result[y_offset_L:,r_out_x-outputoffset_L-outputoffset_H:,] = IPM_Left_out_RGB[l_out_y-rgb_out_y+y_offset_L:l_out_y,outputoffset_R:,]
    Result[y_offset_R:,0:r_out_x-outputoffset_L-outputoffset_H] = IPM_Right_out_Result[r_out_y-rgb_out_y+y_offset_R:r_out_y,outputoffset_H:r_out_x-outputoffset_L]
    Result[y_offset_L:,r_out_x-outputoffset_L-outputoffset_H:] = IPM_Left_out_Result[l_out_y-rgb_out_y+y_offset_L:l_out_y,outputoffset_R:]
    ########################################################INITT############################################################
    pol_order = 2       #input('polinom derecesini gir :')# pol order daha sonra duzeltebiliriz.
    offset_x_ust = 10      #alttan verilecek ofsett. bunu sonra duzeltecegiz
    offset_x_alt = 33      #alttan verilecek ofsett. bunu sonra duzeltecegiz
    InputImage_height = Result.shape[0]
    InputImage_width = Result.shape[1]
    #height_h = InputImage_height//2
    #width_h = InputImage_width//2
    ref_Right = np.ones((1,InputImage_height),dtype=int)*660  #3.2m yol genişliği için 800px
    ref_Left = np.ones((1,InputImage_height),dtype=int)*280   #3.2m için 180
    error_distance = 30 #yatay arama mesafesi. hata cok buyukse nan alacak
    vertical_section_distance = 2  #dikey olarak kesit araligi
    section_max = (InputImage_height-offset_x_alt)//vertical_section_distance    #maksimum kesit sayisi.
    ##########################################################################################################################
    
    while True:
        
        RC_Frame = Right_Camera.frame
        
        LC_Frame = Left_Camera.frame

        IPM_Left_out_RGB , IPM_Left_out_Result= IPM_Left.BirdEye(input_image = LC_Frame)
        IPM_Right_out_RGB , IPM_Right_out_Result= IPM_Right.BirdEye(input_image = RC_Frame)
        #print("################################")
        #print(IPM_Right_out_RGB.shape)
        #print(IPM_Left_out_RGB.shape)
        

        #RGB_Result[y_offset_R:,0:r_out_x-outputoffset_L,] = IPM_Right_out_RGB[r_out_y-rgb_out_y+y_offset_R:r_out_y,0:r_out_x-outputoffset_L,]
        #RGB_Result[y_offset_L:,r_out_x-outputoffset_L:,] = IPM_Left_out_RGB[l_out_y-rgb_out_y+y_offset_L:l_out_y,outputoffset_R:,]
        RGB_Result[y_offset_R:,0:r_out_x-outputoffset_L-outputoffset_H,] = IPM_Right_out_RGB[r_out_y-rgb_out_y+y_offset_R:r_out_y,outputoffset_H:r_out_x-outputoffset_L,]
        RGB_Result[y_offset_L:,r_out_x-outputoffset_L-outputoffset_H:,] = IPM_Left_out_RGB[l_out_y-rgb_out_y+y_offset_L:l_out_y,outputoffset_R:,]
        Result[y_offset_R:,0:r_out_x-outputoffset_L-outputoffset_H] = IPM_Right_out_Result[r_out_y-rgb_out_y+y_offset_R:r_out_y,outputoffset_H:r_out_x-outputoffset_L]
        Result[y_offset_L:,r_out_x-outputoffset_L-outputoffset_H:] = IPM_Left_out_Result[l_out_y-rgb_out_y+y_offset_L:l_out_y,outputoffset_R:]
        
        e_Left = np.empty((0,2), int)
        e_Right = np.empty((0,2), int)
        ################################################################################################################################
        for i in range(1,section_max):
            
            ##################### FOR Left Lane #####################

            [H_Row] = np.where(Result[offset_x_ust+vertical_section_distance*i-1,:] == 255)
            #[H_Row] = np.where(L_Result[offset_x_ust+vertical_section_distance*i-1,:] == 255)

            if H_Row.size == 0:
                L_tmp_Var = np.nan
            else:
                M = abs(H_Row-ref_Left[0,vertical_section_distance*(i)])
                I=np.argmin(M)
            
                if M[I]>error_distance:
                    L_tmp_Var= np.nan
                else:
                    L_tmp_Var = H_Row[I]

            if ~np.isnan(L_tmp_Var):
                e_Left= np.append(e_Left,np.array([[offset_x_ust+vertical_section_distance*i,L_tmp_Var]]),axis=0)

            ##################### FOR Right Lane #####################
            
            #[H_Row] = np.where(R_Result[offset_x_ust+vertical_section_distance*i-1,:] == 255)

            if H_Row.size == 0:
                R_tmp_Var = np.nan
            else:
                M = abs(H_Row-ref_Right[0,vertical_section_distance*(i)])
                I=np.argmin(M)
            
                if M[I]>error_distance:
                    R_tmp_Var= np.nan
                else:
                    R_tmp_Var = H_Row[I]

            if ~np.isnan(R_tmp_Var):
                e_Right = np.append(e_Right,np.array([[offset_x_ust+vertical_section_distance*i,R_tmp_Var]]),axis=0)
    
        if e_Left.shape[0] == 0 and  e_Right.shape[0] == 0:

            sag_serit = []
            sol_serit = []
            serit_y_eksen = []
            #print("Şerit Yok")
            #ref_Right = np.ones((1,InputImage_height),dtype=int)*(55+width_h)
            #ref_Left = np.ones((1,InputImage_height),dtype=int)*350

        elif e_Left.shape[0] == 0:

            sag_serit = []
            sol_serit = []
            serit_y_eksen = []
            #print("Sol Şerit Yok")
            #ref_Left = np.ones((1,InputImage_height),dtype=int)*350
            R_p = Lane_Fitting(e_Right,pol_order)

            for xp in range(InputImage_height-LaneOffsetA,InputImage_height-LaneOffsetU,1):
                ref_Right[0,xp] = int(R_p(xp))
                sag_serit.append(R_p(xp))
                serit_y_eksen.append(xp)
                #ref_Left[0,xp] = int(L_p(xp))
                #cv2.circle(RGB_Result,(int(L_p(xp)),xp),3,(255,0,0),-1)
                cv2.circle(RGB_Result,(int(R_p(xp)),xp),3,(0,255,0),-1)

            #ref_Left = ref_Right - 110

        elif e_Right.shape[0] == 0:

            sag_serit = []
            sol_serit = []
            serit_y_eksen = []
            #print("Sağ Şerit Yok")
            #ref_Right = np.ones((1,InputImage_height),dtype=int)*(55)
            #ref_Left = np.ones((1,InputImage_height),dtype=int)*350
            
            L_p = Lane_Fitting(e_Left,pol_order)

            for xp in range(InputImage_height-LaneOffsetA,InputImage_height-LaneOffsetU,1):
                #ref_Right[0,xp] = int(R_p(xp))
                ref_Left[0,xp] = int(L_p(xp))
                sol_serit.append(L_p(xp))
                serit_y_eksen.append(xp)
                cv2.circle(RGB_Result,(int(L_p(xp)),xp),3,(255,0,0),-1)
                #cv2.circle(RGB_Result,(int(R_p(xp)),xp),3,(0,255,0),-1)

            #ref_Right = ref_Left + 110
            
        else:

            sag_serit = []
            sol_serit = []
            serit_y_eksen = []

            L_p = Lane_Fitting(e_Left,pol_order)
            R_p = Lane_Fitting(e_Right,pol_order)
            

            
            for xp in range(InputImage_height-LaneOffsetA,InputImage_height-LaneOffsetU,1):

                ref_Right[0,xp] = int(R_p(xp))
                ref_Left[0,xp] = int(L_p(xp))
                sag_serit.append(R_p(xp))
                sol_serit.append(L_p(xp))
                serit_y_eksen.append(xp)
                
                cv2.circle(RGB_Result,(int(L_p(xp)),xp),3,(255,0,0),-1)
                cv2.circle(RGB_Result,(int(R_p(xp)),xp),3,(0,255,0),-1)

        ################################################################################################################################
        #IPM_Right_out_RGB.shape IPM_Left_out_RGB.shape
        #cv2.imshow('Left birdeyeRGB', IPM_Left_out_RGB)
        #cv2.imshow(Left_Camera.cam_name+'birdeyeGRY', IPM_Left_out_Result)
        #cv2.imshow('Right birdeyeRGB', IPM_Right_out_RGB)
        #cv2.imshow('Right birdeyeGRY', IPM_Right_out_Result)
        
        mk.LD_status = conn.recv()
        print("received LD")
        
        cv2.imshow('birdeyeGRY', Result)
        cv2.imshow('birdeyeRGB', RGB_Result)
        
        
        #conn.send([sol_serit,sag_serit,serit_y_eksen,RGB_Result])
        conn.send([sol_serit,sag_serit,serit_y_eksen])
        print("LD passed")
        # LD_stat = 0
        # mk.LD_status = 0
        # iii = 0
        # while (LD_stat == 0):
        #     iii = iii+1
        #     LD_stat = mk.LD_status




        #conn.send([True,True]) #Bunu sonra veri gondermek icin ac

        if ((cv2.waitKey(1) == ord("q")) or (Right_Camera.stopped or Left_Camera.stopped)):
            
            if Right_Camera.stopped:
                Left_Camera.stop()
            elif Left_Camera.stopped:
                Right_Camera.stop()
            else:
                Right_Camera.stop()
                Left_Camera.stop()
            break
    Left_Camera.stop()
    Right_Camera.stop()
    cv2.destroyAllWindows()


def ObjectDetection(conn, mk):
    
    import pyzed.sl as sl
    import pycuda.autoinit
    from atay_yolo_classes import get_cls_dict
    from atay_yolo import TrtYOLOv3, YOLO_Visulation
    import atay_zed as zed
    
    atay_zed = zed.ATAY_ZED()


    cv2.namedWindow("YOLO Image", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("YOLO Image", 960, 540) 

    trt_yolov3 = TrtYOLOv3("alatay-yolov3-tiny.trt")
    cls_dict = get_cls_dict()
    key = ' '
    while key != 113 :
        
        err = atay_zed.Grab()
        
        if err :
            boxes, confs, clss = trt_yolov3.detect(atay_zed.image, 0.5)
            Yolo_Image,ObjDetections = YOLO_Visulation(atay_zed,[boxes, confs, clss],cls_dict)
            
            
            
            
            mk.OD_status = conn.recv()
            print("received YOLO")
            conn.send(ObjDetections)
            print("YOLO passed")
            cv2.imshow("YOLO Image", Yolo_Image)
            cv2.waitKey(1)
            #OD_stat = mk.OD_status
            #OD_stat = 0
            # mk.OD_status = 0
            # i = 0
            # while (OD_stat == 0):
            #     i = i+1
            #     OD_stat = mk.OD_status
                
        else:
            print("zed Error")

            #time.sleep(0.05)
            

    atay_zed.Close()
    cv2.destroyAllWindows()
    
    print("\nFINISH")

mk = m_status()

if __name__ == '__main__':
    
    AtayEV = atay_carcontrol.AtayEV()
    YOLOController = airsim_yolocontroller.YOLOController(AtayEV,StopTime=30,TrafficSignMinDis = 6.0 ,MinNumberOfDetections=3)
    
    ODRecv_conn, ODSend_conn = Pipe()
    LDRecv_conn, LDSend_conn = Pipe()
    mk.OD_status = 1
    mk.LD_status = 1
    p1 = Process(target = LaneDetection,args=(LDSend_conn,mk,))
    p2 = Process(target = ObjectDetection,args=(ODSend_conn,mk,))

    print("######### Initializing and Starting Processes #########")


    p1.start()
    p2.start()
    time.sleep(6)

    print("######### Started Processes #########")
    AtayEV.speed = 1
    AtayEV.steerangle = 0
    can_send_thread = threading.Thread(target = AtayEV.can_send)
    can_send_thread.start()
    

    try:
        while True:
            lss= 1
            odd = 1
            LDRecv_conn.send(lss)
            ODRecv_conn.send(odd)
            
            sol_serit,sag_serit,serit_y_eksen = LDRecv_conn.recv()
            Object_Info = ODRecv_conn.recv()
            

            print(sol_serit[1],sag_serit[1],serit_y_eksen[1])
            print(Object_Info)
            #GB = YOLOController.YProcess(RGBDepth_Frame,Object_Info)
            GB = YOLOController.YProcess(Object_Info)
            
            Route = CalculateRoute(sag_serit,sol_serit,serit_y_eksen,AtayEV.LaneInfo)
            AtayEV.steerangle = CalculateSteerAngle(Route,AtayEV.steerangle)
            print("###########################")
            print("SteeringAngle:", AtayEV.steerangle)
            print("###########################")
            print("Speed:",AtayEV.speed)
            print("###########################")
            #print("Lane Info: ",AtayEV.Lane_Info)
            print("###########################")
            AtayEV.update_info(steerangle = AtayEV.steerangle)

            print("Object Info: ",Object_Info)



    except KeyboardInterrupt:
        p1.terminate()
        p2.terminate()
        can_send_thread.kill()
        
        cv2.destroyAllWindows()
        pass
    
    can_send_thread.join()   
    p1.join()
    p2.join()

    print("\n")
    print("################################################################")
    print("##            Terminated Child Process and Threads            ##")            
    print("##                       Good BYEEEEE...                      ##")
    print("################################################################")
    print("\n")