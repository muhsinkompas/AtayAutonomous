import airsim_timer
import cv2

class YOLOController:

    def __init__(self,AtayEV,StopTime,TrafficSignMinDis,MinNumberOfDetections = 3,ListSize = 4):
        
        if MinNumberOfDetections > ListSize:
            print("List size must be greater than Minimum Number OF Detection")
            ListSize = MinNumberOfDetections+1
        
        self.MinNumberOfDetections = MinNumberOfDetections
        self.ListSize = ListSize
        self.TrafficSignsList = []
        self.GenerateList()
        self.car = AtayEV
        self.StopTime = StopTime
        self.StopTimer = airsim_timer.StopTimer(self.StopTime,self.car)
        self.TrafficSignMinDis = TrafficSignMinDis 
        self.GB = False
    
    def GenerateList(self):
        emptyKW = "AMA"
        for _ in range(self.ListSize):
            self.TrafficSignsList.append(emptyKW)
    
    def POPList(self):
        self.TrafficSignsList.pop(0)

    def AppendList(self,Element):
        self.TrafficSignsList.append(Element)
    
    def UpdateList(self,Detection):
        self.POPList()
        self.AppendList(Detection)
    
    def NumberOFElement(self,Element):
        return self.TrafficSignsList.count(Element)

    def ALI(self,Element):
        return (self.NumberOFElement(Element) >= self.MinNumberOfDetections)

    def YProcess(self,TrafficSignsDetections):
        
        self.GB = False
        for TrafficSignsDetection in TrafficSignsDetections:
            TrafficSignNAME = TrafficSignsDetection[0]
            TrafficSignDis  = TrafficSignsDetection[1]
            
            if (TrafficSignNAME == "Kirmizi_Isik") and (TrafficSignDis <= self.TrafficSignMinDis):
                self.UpdateList(TrafficSignNAME)
                if self.ALI(TrafficSignNAME):
                    self.car.update_info(speed=0)  #update_info(steerangle=i)
                    self.car.update_info(brake=1)   
            
            elif (TrafficSignNAME == "Yesil_Isik") and (TrafficSignDis <= self.TrafficSignMinDis):
                self.UpdateList(TrafficSignNAME)
                if self.ALI(TrafficSignNAME):
                    self.car.update_info(speed=1)
                    self.car.update_info(brake=0) 
            
            elif (TrafficSignNAME == "Sol_Yasak") and (TrafficSignDis <= self.TrafficSignMinDis):
                self.UpdateList(TrafficSignNAME)
                if self.ALI(TrafficSignNAME):
                    self.car.LaneInfo = [False,  True]
            
            elif (TrafficSignNAME == "Sag_Yasak") and (TrafficSignDis <= self.TrafficSignMinDis):
                self.UpdateList(TrafficSignNAME)
                if self.ALI(TrafficSignNAME):
                    self.car.LaneInfo = [True,  False]
            
            #elif (TrafficSignNAME== "Girilmez") and (TrafficSignDis <= self.TrafficSignMinDis):
            elif (TrafficSignNAME == "Girilmez") and (TrafficSignDis <= 6):
                #self.car.LaneInfo = [True,  False]
                self.GB = True

            elif (TrafficSignNAME == "Durak") and (TrafficSignDis <= self.TrafficSignMinDis):
                self.UpdateList(TrafficSignNAME)
                #print("Number of Durak Signs:" + " " + str(YOLOController.NumberOFElement(TrafficSignsDetection[0])))
                if self.ALI(TrafficSignNAME) and not(self.StopTimer.TimerEnable):
                    self.StopTimer.Count()
            
            elif (TrafficSignNAME == "Park") and (TrafficSignDis <= 4):
                self.UpdateList(TrafficSignNAME)
                #print("Number of Durak Signs:" + " " + str(YOLOController.NumberOFElement(TrafficSignsDetection[0])))
                if self.ALI(TrafficSignNAME) and not(self.StopTimer.TimerEnable):
                    self.car.update_info(speed=0)
                    self.car.update_info(brake=1) 
            elif (TrafficSignNAME == "Dur") and (TrafficSignDis <= 6):
                self.UpdateList(TrafficSignNAME)
                #print("Number of Durak Signs:" + " " + str(YOLOController.NumberOFElement(TrafficSignsDetection[0])))
                if self.ALI(TrafficSignNAME) and not(self.StopTimer.TimerEnable):
                    self.car.update_info(speed=0)
                    self.car.update_info(brake=1)
            
            #cv2.rectangle(RGBDepth_Frame, TrafficSignsDetection[2][0], TrafficSignsDetection[2][1], (0, 255, 0), 1)
            #cv2.putText(RGBDepth_Frame, TrafficSignNAME + " " + str(round(TrafficSignDis,2)) + "m" , (TrafficSignsDetection[2][0][0], TrafficSignsDetection[2][0][1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, [0, 255, 0], 2)

        return self.GB
