import can
import numpy as np
import time



class AtayEV:
    
    def __init__(self):
        self.Jetson_CAN_ID = 0x0301
        self.status = 0
        self.reverse = 0
        self.throttle = 0
        self.steerangle = 0
        self.brake = 0
        self.armed = 0
        self.maxsteerangle = 20
        self.minsteerangle = -20
        self.maxspeed = 70
        self.minspeed = 51
        self.maxsocketval = 100
        self.speed = 0
        self.nominalspeed = 59
        self.maxControlOutput = 30
        self.maxSteerAngle = 20
        ##########  Left , Right ##############
        self.LaneInfo = [True, True]
        #self.speedCoefficient = (self.maxspeed-0)/(self.maxsocketval-0)
        self.speedCoefficient = (self.maxspeed-self.minspeed)/(self.maxsocketval-(-1*self.maxsocketval))
        self.steeringCoefficient = (self.maxsteerangle-self.minsteerangle)/(self.maxsocketval-(-1*self.maxsocketval))
        self.start = 0
        while self.start == 0:
            try:
                self.bus = can.Bus(interface="socketcan",channel="can0")
                print("Can Started")
                self.start = 1
            except:
                print("Waiting For Can Start")


    def update_info(self, brake = None, steerangle = None, speed = None): #bu fonksiyon üzerinden update edilecek. 
        if brake is not None:                                           #can_send uzerinden surekli veri basilacak.
            self.brake = brake
        if steerangle is not None:
            self.steerangle = steerangle
        if speed is not None:
            self.speed = speed
    
    def can_send(self): #Can bus'a veri gonderdigimiz kisim buraya koyulacak
        # while (True):
        #     if self.speed == 1:
        #         self.throttle = self.nominalspeed
        #     elif self.speed ==0:
        #         self.throttle = 0
            
        #     #status = self.status
        #     #brake = self.brake
        #     steerangle = self.steerangle + self.maxSteerAngle
        #     #thorttle = self.throttle
        start2=0
        sayac = 0
        while self.status!=1: #Veri gondermeden once status'u kontrol etmek icin
            while start2 == 0:
                sayac = sayac+1
                print("Waiting for Can Start2:",sayac)
                #msg = self.bus.recv(None)
                start2 = 1
                try:
                    msg = self.bus.recv(timeout = 1.0)
                except:
                    start2 = 0
                    pass
            if msg.arbitration_id==0x0302:
                self.status = msg.data[0]
            start2 = 0
            print("status error")
            #msg = self.bus.recv(None)
            
        print("break")
        time.sleep(0.10)
        self.bus.flush_tx_buffer()
        while True:
            if self.speed == 1:
                self.throttle = self.nominalspeed
            elif self.speed ==0:
                self.throttle = 0
            steerangle = self.steerangle + self.maxSteerAngle
            msg = self.bus.recv(timeout = 2.0)
            try:
                if msg.arbitration_id==0x0302:
                    self.status = msg.data[0]
            except:
                print("failed receive 302")
                self.status = 0
            tmp_throttle = self.throttle
            tmp_steerangle = steerangle
            tmp_brake = self.brake
            tmp_armed = self.armed
            if tmp_armed == 0:
                
                # if tmp_throttle < 0:
                #     tmp_throttle = 0
                #tmp_throttle = tmp_throttle+self.maxsocketval
                #tmp_throttle = map(tmp_throttle,self.speedCoefficient)
                #print(tmp_throttle)
                #tmp_throttle = tmp_throttle + self.minspeed 
                if tmp_steerangle<0:
                    tmp_steerangle=-1*tmp_steerangle
                if tmp_brake == 1:
                    tmp_throttle = 0
                elif self.reverse ==1:
                    tmp_brake = 2

                #disp(tmp_armed,tmp_brake,tmp_steerangle,tmp_throttle)
                #print(tmp_armed,tmp_brake,tmp_steerangle,tmp_throttle)

            else:
                tmp_throttle = 0
                tmp_brake = 0
                #tmp_steerangle = map(tmp_steerangle,self.steeringCoefficient)
                #tmp_steerangle = tmp_steerangle + self.maxsteerangle 
                #print(tmp_steerangle)
                print("Not Armed")
            message = can.Message(arbitration_id = self.Jetson_CAN_ID,extended_id=False, data = [int(tmp_brake), int(tmp_steerangle), int(tmp_throttle)])
            
            if self.status == 1:
                print("send msg")
                print(tmp_steerangle)
                # try:
                #self.bus.send(message)
                #time.sleep(0.05)
                #self.bus.send(message)
                self.bus.stop_all_periodic_tasks()
                self.bus.send_periodic(message,0.02)
                # except can.CanError:
                #     self.bus.flush_tx_buffer()
                #     print("FLUSHH")

                self.status = 0
                time.sleep(0.10)
                #print(chr(27)+'[2j')
                #print('\033c')
                #print('\x1bc')
            # else:
            #     hata_sayisi=0
            #     while self.status != 1:
            #         hata_sayisi = hata_sayisi+1
            #         print(hata_sayisi)
            #         if hata_sayisi==20:
            #             down_cmd='sudo ip link set down can0'
            #             up_cmd='sudo ip link set up can0'
            #             try:
            #                 os.system(down_cmd)
            #             except can.CanError:
            #                 print("down")
            #             time.sleep(0.10)
            #             os.system(up_cmd)
            #             print("canbus restarted")
            #             time.sleep(0.10)
            #             self.bus = can.Bus(interface="socketcan",channel="can0")
            #             hata_sayisi=0
            #         time.sleep(0.10)
            #         msg= self.bus.recv(None)
            #         if msg.arbitration_id==0x0302:
            #             self.status = msg.data[0]
            #             break
            #         time.sleep(0.10)
            self.status = 0
    
    def MapSteerAngle(self,steerAngle):
        #Maksimum maxSteerAngle kontor et!
        in_min = (-1*self.maxSteerAngle)
        in_max = self.maxSteerAngle
        out_max = 1
        out_min = -1
        x = (steerAngle - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        self.steerAngle = x
