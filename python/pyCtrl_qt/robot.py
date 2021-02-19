import sys
from threading import Thread
import time
from robot_arm_ik import robot_arm_ik
from getchar import getChar

_STEP = 50
_CMD = 0
demo_show = 0

p_home = [0,460,300]

class Robot():
    def __init__(self, transport=None, data_send=None, log_console=None):
        self.ik = robot_arm_ik() # support postion to angle

        self.transport = transport
        self.data_send = data_send
        self.log_console = log_console

        self.anglePerStep = 360/(64*200*10) # 64细分，1.8度步进角，10:1减速器
        self.LanglePerStep = 360/(64*200) # 64细分，1.8度步进角

        self.mStop = True # stop
        self.nStop = True # stop
        self.oStop = True # stop
        self.lStop = True # stop
        self.lLimitStopEvent = False

        self.curX = 0
        self.curY = 0
        self.curZ = 0

        self.targetX = 0
        self.targetY = 0
        self.targetZ = 0

        self.targetAngle = [0,0,0]
        self.currentAngle = [0,0,0]
        self.m1Max = 90
        self.m1Min = -90
        self.m2Max = 90
        self.m2Min = 17.0
        self.m3Max = 60 #40
        self.m3Min = -78.0
        self.angleMax = [self.m1Max, self.m2Max, self.m3Max]
        self.angleMin = [self.m1Min, self.m2Min, self.m3Min]
        
        self.deltM2M3Min = 2
        self.deltM2M3Max = 130

        self.motoResetState = [False, False, False]

        self._running = True
        self.stateUpdateTime = time.time()

        self.log_print("init robot done")

    def log_print(self, data):
        if self.log_console is None:
            print(data)
        else:
            self.log_console(data+"\n")

    def getMotoStopVol(self, moto):
        if moto == 'M':
            return self.mStop
        if moto == 'N':
            return self.nStop
        if moto == 'O':
            return self.oStop
        if moto == 'L':
            return self.lStop
    
    def setMotoStopVol(self, moto, vol):
        if moto == 'M':
            self.mStop = vol
        if moto == 'N':
            self.nStop = vol
        if moto == 'O':
            self.oStop = vol
        if moto == 'L':
            self.lStop = vol

    def rxDataHandle(self, data):
        data_list = data.split('\\n')
        for str_data in data_list:
            if "STOPPED" in str_data: # move to target stop
                if "M.STATUS" in str_data:
                    self.setMotoStopVol('M',True)
                elif "N.STATUS" in str_data:
                    self.setMotoStopVol('N',True)
                elif "O.STATUS" in str_data:
                    self.setMotoStopVol('O',True)
                elif "L.STATUS" in str_data:
                    self.setMotoStopVol('L',True)
                if self.isStop():
                    self.curX = self.targetX
                    self.curY = self.targetY
                    self.curZ = self.targetZ
                    self.currentAngle = self.targetAngle[:]
    
            elif 'stop' in str_data: # limited stop
                self.log_print(str_data)
                if "M.stop" in str_data:
                    self.setMotoStopVol('M',True)
                    if "limit" in str_data:
                        self.motoResetState[0] = True
                elif "N.stop" in str_data:
                    self.setMotoStopVol('N',True)
                    if "limit" in str_data:
                        self.motoResetState[1] = True
                elif "O.stop" in str_data:
                    self.setMotoStopVol('O',True)
                    if "limit" in str_data:
                        self.motoResetState[2] = True
                elif "L.stop" in str_data:
                    self.setMotoStopVol('L',True)
                    if "limit" in str_data:
                        self.lLimitStopEvent = True
                        self.log_print("L LIMITED STOP")
                if self.isStop():
                    self.curX = self.targetX
                    self.curY = self.targetY
                    self.curZ = self.targetZ
                    self.currentAngle = self.targetAngle[:]
            elif "Error" in str_data:
            # else:
                self.log_print(str_data)

    def rxTask(self):
        while True:
            while self.transport.in_waiting == 0:
                if self._running == False:
                    return
                pass
            data_all = str(self.transport.readall())
            self.rxDataHandle(deta_all)

    def cmdSend(self, data):
        if self.data_send is None:
            self.transport.write(data.encode('UTF-8'))
        else:
            self.data_send(data)

    def angleToStep(self, m1Angle, m2Angle, m3Angle):
        mSteps = m1Angle / self.anglePerStep
        nSteps = m2Angle / self.anglePerStep
        oSteps = m3Angle / self.anglePerStep
        return mSteps, nSteps, oSteps

    def xyzToStep(self,x,y,z): # unit: mm
        armAngle = [0,0,0]
        if False == self.ik.postion_to_angle([x,y,z], armAngle):
            self.log_print("Error: xyzToStep postion1")
            return (0,0,0)

        m1, m2, m3 = self.ik.armAngle_to_motorAngle(armAngle)
        # self.log_print(m1,m2,m3)

        # if m1 < self.m1Min or m1 > self.m1Max:
        #     self.log_print("Error: xyzToStep postion2")
        #     return (0,0,0)
        # if m2 < self.m2Min or m2 > self.m2Max:
        #     self.log_print("Error: xyzToStep postion3")
        #     return (0,0,0)
        # if m3 < self.m3Min or m3 > self.m3Max:
        #     self.log_print("Error: xyzToStep postion4")
        #     return (0,0,0)
        # if (m2 - m3) < self.deltM2M3Min or (m2 - m3) > self.deltM2M3Min:
        #     self.log_print("Error: xyzToStep postion5")
        #     return (0,0,0)
        self.targetAngle = [m1, m2, m3]
        return self.angleToStep(m1, m2, m3)

    def goPostion(self, xyz, style = 0):
        if self.targetX == xyz[0] and self.targetY == xyz[1] and self.targetZ == xyz[2]:
            return

        mSteps, nSteps, oSteps = self.xyzToStep(xyz[0], xyz[1], xyz[2])
        if (mSteps + nSteps + oSteps) == 0:
            return

        self.targetX = xyz[0]
        self.targetY = xyz[1]
        self.targetZ = xyz[2]
        if style == 0: # 先移动底盘，后移动大、小臂
            self.setMotoStopVol('M', False)
            self.cmdSend("setM:{:}\n".format(mSteps))
            while not self.isStop():
                time.sleep(3)
            self.setMotoStopVol('N', False)
            self.setMotoStopVol('O', False)
            self.cmdSend("setN:{:}setO:{:}\n".format(nSteps, oSteps))
            while not self.isStop():
                time.sleep(3)
        if style == 1:
            self.setMotoStopVol('N', False)
            self.setMotoStopVol('O', False)
            self.cmdSend("setN:{:}setO:{:}\n".format(nSteps, oSteps))
            while not self.isStop():
                time.sleep(3)
            self.setMotoStopVol('M', False)
            self.cmdSend("setM:{:}\n".format(mSteps))
            while not self.isStop():
                time.sleep(3)

    def mvPostion(self, xyz):
        self.goPostion([xyz[0] + self.curX, xyz[1] + self.curY, xyz[2] + self.curZ])
    
    def lenToAngle(self, len):
        return len/(165/360)

    def mvL(self, len):
        self.mvAngle('L', self.lenToAngle(len))
        while not self.lStop:
            time.sleep(2)

    # moto = 'M', 'N', 'O'
    # angle = -180~180
    def mvAngle(self, moto, angle, wait = True):
        self.setMotoStopVol(moto, False)
        anglePerStep = self.anglePerStep
        if moto == 'L':
            anglePerStep = self.LanglePerStep
        self.cmdSend("add{:}:{:}\n".format(moto, angle / anglePerStep))
        if wait != True:
            return
        while not self.isStop():
            time.sleep(2)
   
    def setX(self, x):
        self.goPostion([x, self.curY, self.curZ])

    def pick(self):
        self.cmdSend("setP.pump:0\n")
    
    def release(self):
        self.cmdSend("setP.pump:1\n")

    def reset(self):
        self.log_print("reset step1")
        self.release()
        
        # 远离限位器移动一点距离，防止已经运动到限位器位置
        self.mvAngle('M', -3, False)
        self.mvAngle('N', -5, False)
        self.mvAngle('O', -3)
        
        self.setMotoStopVol('M', False)
        self.setMotoStopVol('N', False)
        self.setMotoStopVol('O', False)
        time.sleep(1)
        

        self.log_print("reset step2")
        self.mvAngle('M', 180, False)
        self.mvAngle('N', 180, False)
        self.mvAngle('O', 180)
        self.setMotoStopVol('N', False)
        self.setMotoStopVol('M', False)
        self.setMotoStopVol('O', False)
        time.sleep(1)

        self.log_print("reset step3")
        self.cmdSend("setM.currentPosition:{:}\n".format(74 / self.anglePerStep))
        self.cmdSend("setN.currentPosition:{:}\n".format(90 / self.anglePerStep))
        self.cmdSend("setO.currentPosition:{:}\n".format(40 / self.anglePerStep))
        time.sleep(0.5)
        self.goPostion(p_home)
        
        while not self.isStop():
            time.sleep(2)
        self.log_print("reset done")

    def isStop(self):
        if self._running == False:
            return True

        if self.mStop  and self.nStop and self.oStop:
            return True
        elif self.stateUpdateTime +2 <= int(time.time()):
                if self.getMotoStopVol('M') == False:
                    self.cmdSend("getM.status\n")
                if self.getMotoStopVol('N') == False:
                    self.cmdSend("getN.status\n")
                if self.getMotoStopVol('O') == False:
                    self.cmdSend("getO.status\n")
                if self.getMotoStopVol('L') == False:
                    self.cmdSend("getL.status\n")
                self.stateUpdateTime = int(time.time())
        return False
