import numpy as np


######fonksiyon asagidaki gibi import edilecek
#from airsim_steerangle import CalculateSteerAngle,CalculateRoute

######aci hesabi icin asagidaki cagrilacak
#sol_serit,sag_serit,serit_y_eksen, RGB_Result = LCrecv_conn.recv()
#Route = CalculateRoute(sag_serit,sol_serit,serit_y_eksen,AlatayEV.LaneInfo)
#AlatayEV.steerAngle = CalculateSteerAngle(Route,AlatayEV.steerAngle)

def CalculateRoute(sag_serit,sol_serit,serit_y_eksen,LaneInfo):

    #414 mp 350lp 470rp
    LLane = LaneInfo[0]
    RLane = LaneInfo[1]

    if LLane and not(RLane):
        sag_serit = []
    elif not(LLane) and RLane:
        sol_serit = []

    RLexist = len(sag_serit) != 0
    LLexist = len(sol_serit) != 0

    mid_y_eksen = np.asmatrix(serit_y_eksen)*1.0

    # Eger iki serit varsa
    if RLexist and LLexist:
        #print(len(sag_serit))
        #print(len(sol_serit))
        #print(len(serit_y_eksen))
        orta_serit = np.asmatrix((np.asarray(sag_serit)+np.asarray(sol_serit))/2)
    
    # Eger sadece sol serit varsa
    elif not(RLexist) and LLexist:

        orta_serit = np.asmatrix(np.asarray(sol_serit)+400) #3.2m için 400
        #orta_serit = np.asmatrix(np.asarray(sol_serit)+40)

    # Eger sadece sag serit varsa
    elif RLexist and not(LLexist):

        orta_serit = np.asmatrix(np.asarray(sag_serit)-314) #3.2m için -314
        #orta_serit = np.asmatrix(np.asarray(sag_serit)-50)
    
    # GOTE GELDIK
    else:
        orta_serit = np.empty(0)
        mid_y_eksen = np.empty(0)
    #print(orta_serit.dtype)
    #print(mid_y_eksen.dtype)
    #print(type(orta_serit))
    #print(type(mid_y_eksen))
    points = np.hstack((orta_serit.T,mid_y_eksen.T))
    #print(points.shape)
    #print("mp")
    return points

def CalculateSteerAngle(points,SteerAngle):
    camera_to_origin = 56
    refmidpoint = 487
    cps = points.size!=0
    sigma = 0.298
    mu = 0
    max_steering_angle = 20
    def moving_average(x, w):
        x=np.squeeze(np.asarray(x.T))
        x = np.pad(x,(0, w-1), 'constant', constant_values=(0))
        return np.convolve(x, np.ones(w), 'valid') / w
    #herhangi bir trafik isareti yok
    #serit var ise 
    if cps:
        wc = np.ones((points.shape[0],1)).T
        RefMidPoints = refmidpoint*np.ones((points.shape[0],1))
        np.flip(points)
        np.flip(points,axis=0)
        d = points[:,0]-RefMidPoints
        wp = wc*d/points.shape[0]
        #mp = np.array([wp[0,0],(points[points.shape[0]-1,1]+points[0,1])/2])
        #angle = np.degrees(np.arctan(mp[0]/mp[1]))
        #print(angle)
        angle = np.degrees(np.arctan((points[:,0]-RefMidPoints)/(points[:,1]+camera_to_origin)))
        #print(angle)
        if angle.size>3:
            angle = moving_average(angle,3)
        bins = np.arange(start=0, stop=points.shape[0],step = 1)/points.shape[0]
        weightco = (1/(sigma * np.sqrt(2 * np.pi)) * np.exp( - (bins - mu)**2 / (2 * sigma**2)))
        wangle=np.dot(angle.T,weightco)/np.sum(weightco)
        #print(angle)
        #print(wangle)

        # if wangle[0,0] >= max_steering_angle:
        #     wangle[0,0] = max_steering_angle
        # elif wangle[0,0] <= -max_steering_angle:
        #     wangle[0,0] = -max_steering_angle
        
        # return wangle[0,0]
        if wangle >= max_steering_angle:
            wangle = max_steering_angle
        elif wangle <= -max_steering_angle:
            wangle= -max_steering_angle
        
        return wangle
    else:

        return SteerAngle
