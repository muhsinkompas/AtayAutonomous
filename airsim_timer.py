import threading as th

#burada yer alan car.speed ve car.brake kisimlarini duzelt.
#timer kodu yolocontroller.py kodunda cagirilacak.


class StopTimer:

    def __init__(self,StopTime,car):
        self.StopTime = StopTime
        self.EscapeTime = StopTime
        self.TimerEnable = False
        self.car = car
        #self.WaitTimer = th.Timer(self.StopTime,self.TimerFunc)
        #self.EscapeTimer = th.Timer(self.EscapeTime,self.EscapeTimerFunc)

    def TimerFunc(self):
        self.car.update_info(speed=1)
        self.car.update_info(brake=0)
        self.EscapeTimer.start()
        #time.sleep(2)
    
    def EscapeTimerFunc(self):
        self.TimerEnable = False
        print("Escape Manoeuvre in Progress...")
    
    def KillAllThreads(self):

        if self.WaitTimer.is_alive():
            self.WaitTimer.cancel()
        
        if self.EscapeTimer.is_alive():
            self.EscapeTimer.cancel()


    def Count(self):
        ###### Define Thread Timer ######
        self.WaitTimer = th.Timer(self.StopTime,self.TimerFunc)
        self.EscapeTimer = th.Timer(self.EscapeTime,self.EscapeTimerFunc)
        #################################
        self.car.update_info(speed=0)
        self.car.update_info(brake=1)
        self.TimerEnable = True
        print("Stopping at Station for "+ str(self.StopTime)+" "+ "Seconds...")
        self.WaitTimer.start()

