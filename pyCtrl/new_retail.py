import sys
from threading import Thread
import time
import serial
from robot_arm_ik import robot_arm_ik
from getchar import getChar

COM_PORT = 'COM7'

_STEP = 50
_CMD = 0
demo_show = 0

p_home = [0,460,300]
p0 = [-428, 142, 255]
p1 = [385, 420, 68]
p2 = [160, 575, 68]
p3 = [-55, 580, 68]
p4 = [-240, 520, 68]
postion_list = [p0,p1,p2,p3,p4]

class Robot():
    def __init__(self, transport=None):
        self.ik = robot_arm_ik() # support postion to angle

        self.transport = transport
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
    
    def rxTask(self):
        while True:
            while self.transport.in_waiting == 0:
                if self._running == False:
                    return
                pass
            data_all = str(self.transport.readall())
            data_list = data_all.split('\\n')
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
                    print(str_data)
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
                            print("L LIMITED STOP")
                    if self.isStop():
                        self.curX = self.targetX
                        self.curY = self.targetY
                        self.curZ = self.targetZ
                        self.currentAngle = self.targetAngle[:]
                # else "Error" in str_data:
                else:
                    print(str_data)

    def cmdSend(self, data):
        self.transport.write(data.encode('UTF-8'))

    def angleToStep(self, m1Angle, m2Angle, m3Angle):
        mSteps = m1Angle / self.anglePerStep
        nSteps = m2Angle / self.anglePerStep
        oSteps = m3Angle / self.anglePerStep
        return mSteps, nSteps, oSteps

    def xyzToStep(self,x,y,z): # unit: mm
        armAngle = [0,0,0]
        if False == self.ik.postion_to_angle([x,y,z], armAngle):
            print("Error: xyzToStep postion1")
            return (0,0,0)

        m1, m2, m3 = self.ik.armAngle_to_motorAngle(armAngle)
        # print(m1,m2,m3)

        # if m1 < self.m1Min or m1 > self.m1Max:
        #     print("Error: xyzToStep postion2")
        #     return (0,0,0)
        # if m2 < self.m2Min or m2 > self.m2Max:
        #     print("Error: xyzToStep postion3")
        #     return (0,0,0)
        # if m3 < self.m3Min or m3 > self.m3Max:
        #     print("Error: xyzToStep postion4")
        #     return (0,0,0)
        # if (m2 - m3) < self.deltM2M3Min or (m2 - m3) > self.deltM2M3Min:
        #     print("Error: xyzToStep postion5")
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
        print("reset step1")
        self.release()
        # 远离限位器移动一点距离，防止已经运动到限位器位置
        self.mvAngle('M', -3, False)
        self.mvAngle('N', -5, False)
        self.mvAngle('O', -3)
        self.setMotoStopVol('M', False)
        self.setMotoStopVol('N', False)
        self.setMotoStopVol('O', False)
        time.sleep(1)

        print("reset step2")
        self.mvAngle('M', 180, False)
        self.mvAngle('N', 180, False)
        self.mvAngle('O', 180)
        self.setMotoStopVol('N', False)
        self.setMotoStopVol('M', False)
        self.setMotoStopVol('O', False)
        time.sleep(1)

        print("reset step3")
        self.cmdSend("setM.currentPosition:{:}\n".format(74 / self.anglePerStep))
        self.cmdSend("setN.currentPosition:{:}\n".format(90 / self.anglePerStep))
        self.cmdSend("setO.currentPosition:{:}\n".format(40 / self.anglePerStep))
        time.sleep(0.5)
        self.goPostion(p_home)
        while not self.isStop():
            time.sleep(2)
        print("reset done")

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

class new_retail():
    def __init__(self, robot = None):
        self.robot = robot

    def go_home(self):
        self.robot.goPostion(p_home)
        time.sleep(1)

    def pick_from_postion(self, p):
        pre_postion = [p[0]* 0.8, p[1]* 0.8, p[2]+20]
        print("goto pre posion")
        self.robot.goPostion(pre_postion, style=1) # 先动大小臂
        print("goto target postion")
        self.robot.goPostion(p, style=1) # 先动大小臂
        print("pick")
        self.robot.pick()
        time.sleep(0.5)
        print("move O and N 3 degree")
        self.robot.mvAngle('O', 2, False)
        self.robot.mvAngle('N', 1)
        # time.sleep(0.1)
        # print("goto pre posion")
        # self.robot.goPostion(pre_postion, style=1) # 先动大小臂
        print("go home")
        self.robot.goPostion(p_home, style=1) # 先动大小臂

    def put_to_postion(self, p):
        pre_postion = [p[0]* 0.8, p[1]* 0.8, p[2]+20]
        print("goto pre posion")
        self.robot.goPostion(pre_postion, style=1) # 先动大小臂
        print("goto target postion")
        self.robot.goPostion(p, style=1) # 先动大小臂
        print("release")
        self.robot.release()
        time.sleep(0.2)
        print("move O and N 3 degree")
        self.robot.mvAngle('O', 2, False)
        self.robot.mvAngle('N', 1)
        # time.sleep(0.2)
        # print("goto pre posion")
        # self.robot.goPostion(pre_postion, style=1) # 先动大小臂
        print("go home")
        self.robot.goPostion(p_home, style=1) # 先动大小臂

    def wait_box(self):
        self.robot.setMotoStopVol('L', False)
        self.robot.lLimitStopEvent = False
        while self.robot.lLimitStopEvent == False and self.robot._running:
            time.sleep(0.5)
        current_time = int(time.time())
        while current_time > int(time.time()) - 4:
            current_time = int(time.time())
            print("move line")
            self.robot.mvL(550)

    def demo1(self):
        i = 1
        pic_p = postion_list[0]
        print("demo1")
        for p in postion_list[1:]:
            print("waiting...")
            self.wait_box()
            print("go home")
            self.robot.goPostion(p_home)
            print("pick from p[{:}]".format(i))
            self.pick_from_postion(pic_p)
            time.sleep(0.2)
            print("pick from p[{:}]".format(i))
            self.put_to_postion(p)
            i += 1
    
def key_cmd_list():
    print("Z - exit     R - reset")
    print("P - pick     O - put")
    print("A - M0+      D - M0- 底座")
    print("W - M1+      S - M1- 大臂")
    print("Q - M2+      E - M2- 小臂")
    print("M - L+       N - L-  传送带")
    print("T - demo    ")
    print("B - go home  H - help")

def key_cb(key):
    global _STEP
    global _CMD
    global demo_show

    print(key.upper())
    if key == b'Z' or key == B'\x03': robot._running = False
    if key == b'R': 
        robot.reset() 
        print("reset done")
    if key == b'H': key_cmd_list()
    if key == b'P': robot.pick()
    if key == b'O': robot.release()
    if key == b'A': robot.mvAngle('M', _STEP * self.anglePerStep, False) # M 底盘
    if key == b'D': robot.mvAngle('M', -_STEP * self.anglePerStep, False)
    if key == b'W': robot.mvAngle('N', _STEP * self.anglePerStep, False) # N 大臂
    if key == b'S': robot.mvAngle('N', -_STEP * self.anglePerStep, False)
    if key == b'Q': robot.mvAngle('O', _STEP * self.anglePerStep, False) # O 小臂
    if key == b'E': robot.mvAngle('O', -_STEP * self.anglePerStep, False)
    if key == b'M': robot.mvL(600)
    if key == b'N': robot.mvL(-600)
    if key == b'B': robot.goPostion(p_home)
    if key == b'+' or key == b'=': 
        _STEP = _STEP + 10 if _STEP < 100  else 100
        print("_STEP:",_STEP)
    if key == b'-' or key == b'_': 
        _STEP = _STEP - 10 if _STEP > 20  else 10
        print("_STEP:",_STEP)
    if key == b'1': _CMD = 1
    if key == b'2': _CMD = 2
    if key == b'3': _CMD = 3
    if key == b'4': _CMD = 4
    if key == b'5': _CMD = 5
    if key == b'T': demo_show = 1

def key_scan():
    while robot._running == True:
        key_cb(getChar().upper())
        pass

transport = serial.Serial(port=COM_PORT, baudrate = 115200, timeout=0.1)
robot = Robot(transport)

if __name__ == '__main__':
    robotRxTask = Thread(target = robot.rxTask)
    keyTask = Thread(target = key_scan)
    print("start")
    
    robotRxTask.start()
    keyTask.start()

    key_cmd_list()
    print("\r\n\r\n警告: 启动前确保，限位器没有动作，且远离其他结构件！\r\n\r\n")

    demo = new_retail(robot)

    while demo.robot._running == True:
        if demo_show == 1:
            demo_show = 0
            demo.demo1()
        time.sleep(1)

    print("exit")
    if transport.isOpen():
        transport.flush()
        transport.close()
