import sys
from threading import Thread
import time
import serial
from robot_key_ctrl import ROBOT_KeyCtrl
# from getchar import getChar

COM_PORT = "COM13"

_STEP = 10
_CMD = 0
demo_sel = 0

BOARD_Z_MV_LEN = 18
BOARD_HOME_X = -55
BOARD_HOME_Y = -32
BOARD_HOME_Z = -17

CARRY_Z_MV_LEN = 35
CARRY_HOME_X = -25
CARRY_HOME_Y = -340
CARRY_HOME_Z = -17
CARRY_WIDTH = 30

CHESS_WIDTH = 281.5/12
CHESS_LENGHT = 272.5/12

MAX_LEN     = 500
MAX_WIDTH   = 500
MAX_HIGH    = 100

chess_list = [
    [[7,'G'],[6,'F']], # Black, White
    [[7,'F'],[8,'G']],
    [[7,'H'],[7,'I']],
    [[7,'E'],[7,'D']],
    [[6,'G'],[5,'H']],
    [[5,'F'],[4,'E']],
    [[8,'I'],[9,'J']],
    [[8,'F'],[9,'E']],
    [[9,'G'],[10,'H']],
    [[6,'D'],[5,'C']],
    [[8,'E'],[8,'J']],
    [[7,'J'],[9,'K']],
    [[10,'L'],[9,'I']],
    [[11,'G'],[9,'L']],
    [[9,'H'],[9,'M']],
]

class Postion():
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

class Robot():
    def __init__(self, transport=None):
        self.transport = transport
        self.xStepLen = 40/(32*200) # mm 步进电机每转一圈，皮带前进40mm, 32细分，1.8度步进角
        self.yStepLen = 40/(32*200) # mm 步进电机每转一圈，皮带前进40mm, 32细分，1.8度步进角
        self.zStepLen = 8/(32*200)  # mm T8丝杆导程8，即每圈8mm

        self.curPostion = Postion(0, 0, 0)
        self.targetPostion = Postion(0, 0, 0)

        self.xState = 0 # stop
        self.yState = 0 # stop
        self.zState = 0 # stop

        self._running = True
        self.stateUpdateTime = int(time.time())
    
    def rxTask(self):
        while True:
            while self.transport.in_waiting == 0:
                if self._running == False:
                    return
                pass
            data = str(transport.readall())
            data_list = data.split('\\n')
            for str_data in data_list:
                # print(str_data)
                if "STOPPED" in str_data: # moveto target stop
                    if "X.STATUS" in str_data:
                        self.xState = 0
                    elif "Y.STATUS" in str_data:
                        self.yState = 0
                    elif "Z.STATUS" in str_data:
                        self.zState = 0
                elif 'stop' in str_data: # limited stop
                    if "X.stop" in str_data:
                        self.xState = 0
                    elif "Y.stop" in str_data:
                        self.yState = 0
                    elif "Z.stop" in str_data:
                        self.zState = 0
                elif 'Error' in str_data:
                    print(str_data)
                    self._running = False

    def cmdSend(self, data):
        self.transport.write(data.encode('UTF-8'))

    def xyzToStep(self,x,y,z): # unit: mm
        xsteps = int(x / self.xStepLen)
        ysteps = int(y / self.yStepLen)
        zsteps = int(z / self.zStepLen)
        return xsteps, ysteps, zsteps

    def stepToXyz(self, xsteps, ysteps, zsteps):
        x = xsteps * self.xStepLen
        y = ysteps * self.yStepLen
        z = zsteps * self.zStepLen
        return x, y, z # unit: mm
    
    def waitStop(self):
        self.stateUpdateTime = int(time.time())
        while not self.isStop():
            time.sleep(0.5)

    def goPostion(self, xyz):
        xsteps, ysteps, zsteps = self.xyzToStep(xyz[0], xyz[1], xyz[2])
        self.xState = 1
        self.yState = 1
        self.zState = 1
        self.cmdSend("setX:{:}setY:{:}setZ:{:}\n".format(xsteps, ysteps, zsteps))
        self.waitStop()

        # self.xState = 1
        # self.cmdSend("setX:{:}\n".format(xsteps))
        # self.waitStop()
        # self.yState = 1
        # self.cmdSend("setY:{:}\n".format(ysteps))
        # self.waitStop()
        # self.zState = 1
        # self.cmdSend("setZ:{:}\n".format(zsteps))
        # self.waitStop()

    def mvPostion(self, xyz):
        xsteps, ysteps, zsteps = self.xyzToStep(xyz[0], xyz[1], xyz[2])
        self.xState = 1
        self.yState = 1
        self.zState = 1
        self.cmdSend("addX:{:}addY:{:}addZ:{:}\n".format(xsteps, ysteps, zsteps))
        self.waitStop()

    def mvZ(self, z):
        self.zState = 1
        self.cmdSend("addZ:{:}\n".format(int(z / self.zStepLen)))
        self.waitStop()
    
    def mvY(self, y):
        self.yState = 1
        self.cmdSend("addY:{:}\n".format(int(y / self.yStepLen)))
        self.waitStop()

    def mvX(self, x):
        self.xState = 1
        self.cmdSend("addX:{:}\n".format(int(x / self.xStepLen)))
        self.waitStop()

    def pick(self):
        self.cmdSend("setP.pump:0\n")
    
    def release(self):
        self.cmdSend("setP.pump:1\n")
    
    def lock(self):
        self.cmdSend("setX.stepperen:0\n")
        self.cmdSend("setY.stepperen:0\n")
        self.cmdSend("setZ.stepperen:0\n")

    def unlock(self):
        self.cmdSend("setX.stepperen:1\n")
        self.cmdSend("setY.stepperen:1\n")
        self.cmdSend("setZ.stepperen:1\n")

    def reset(self):
        self.release()
        chess.robot.unlock()
        print("reset step1")
        self.goPostion([-5, -5, -5])
        print("reset step2")
        self.goPostion([MAX_WIDTH, MAX_LEN, MAX_HIGH]) # 移动到不可达位置，使得必然触发限位器动作
        print("go reset postion done")
        time.sleep(1)
        
        self.cmdSend("setX.currentPosition:0\n")
        self.cmdSend("setY.currentPosition:0\n")
        self.cmdSend("setZ.currentPosition:0\n")
        time.sleep(1)
    
    def isStop(self):
        if self._running == False:
            return True
        
        if self.xState == 0 and self.yState == 0 and self.zState == 0:
            return True

        if self.stateUpdateTime +5 <= int(time.time()):
            if self.xState == 1:
                self.cmdSend("getX.status\n")
            if self.yState == 1:
                self.cmdSend("getY.status\n")
            if self.zState == 1:
                self.cmdSend("getZ.status\n")
            self.stateUpdateTime = int(time.time())
            # print("check stop cmd: ", self.stateUpdateTime, ":", self.xState, self.yState, self.zState)
        return False

class Chess():
    def __init__(self, robot = None): # width,length mm
        self.chessBoardWidth = CHESS_WIDTH
        self.chessBoardLength = CHESS_LENGHT
        self.chessCarryWidth = CARRY_WIDTH
        self.robot = robot
        self.boardHome = [BOARD_HOME_X, BOARD_HOME_Y, BOARD_HOME_Z]
        self.carryHome = [CARRY_HOME_X, CARRY_HOME_Y, CARRY_HOME_Z]
    
    def chessCarryRowColToXyz(self, row, col):
        postion = self.carryHome[:]
        postion[0] += -row*self.chessCarryWidth
        postion[1] += -col*self.chessCarryWidth
        postion[2] += 0
        return postion

    def chessBoardRowColToXyz(self, row, col):
        postion = self.boardHome[:]
        postion[0] += -row*self.chessBoardWidth
        postion[1] += -col*self.chessBoardLength
        postion[2] += 0
        return postion

    # sel = 0, get chess from chessCarry
    # sel = 1, get chess from chessBoard
    def getChess(self, row, col, sel = 0):
        # 1 移动到取棋子位置
        postion = [0,0,0]
        z_len = 0
        if sel == 0:
            postion = self.chessCarryRowColToXyz(row,col)[:]
            z_len = CARRY_Z_MV_LEN
        if sel == 1:
            postion = self.chessBoardRowColToXyz(row,col)[:]
            z_len = BOARD_Z_MV_LEN
        print("get chess from row:{:} col:{:} x:{:}, y:{:}".format(row, col, postion[0], postion[1]))
        self.robot.goPostion(postion)
        
        # 2 吸取棋子 // pick
        print("move to chess")
        self.robot.mvZ(-z_len)
        print("pick up")
        self.robot.pick()
        time.sleep(1)
        self.robot.mvZ(+z_len)
        print("get chess done")

    def goHome(self):
        print("go home...")
        self.robot.goPostion(self.carryHome)

    # sel = 0, get chess from chessCarry
    # sel = 1, get chess from chessBoard
    def putChess(self, row, col, sel = 0):
        # 3 移动到目标位置
        postion = [0,0,0]
        z_len = 0
        if sel == 0:
            postion = self.chessCarryRowColToXyz(row,col)[:]
            z_len = CARRY_Z_MV_LEN
        if sel == 1:
            postion = self.chessBoardRowColToXyz(row,col)[:]
            z_len = BOARD_Z_MV_LEN
        print("get chess from row:{:} col:{:} x:{:}, y:{:}".format(row, col, postion[0], postion[1]))
        self.robot.goPostion(postion)

        # 4 放置棋子
        print("release chess")
        self.robot.mvZ(-z_len)
        self.robot.release()
        time.sleep(0.2)
        self.robot.mvZ(z_len)


def exit():
    if transport.isOpen():
        transport.flush()
        transport.close()

def demo1():
    robot.unlock()
    time.sleep(1)
    chess.goHome()
    time.sleep(2)

    print("start demo")
    step_count = 0
    for step in chess_list:
        print (step_count)
        chess.getChess(row = int(step_count/4), col = step_count % 4, sel = 0)
        chess.putChess(row = step[0][0]-1, col = ord(step[0][1])-ord('A'), sel = 1) # black
        chess.getChess(row = int(step_count/4) + 6, col = step_count % 4, sel = 0)
        chess.putChess(row = step[1][0]-1, col = ord(step[1][1])-ord('A'), sel = 1) # white
        step_count = step_count + 1
        if robot._running == False:
            exit()
    
    print("pick all chess back")
    step_count = 0
    for step in chess_list:
        print (step_count)
        chess.getChess(row = step[0][0]-1, col = ord(step[0][1])-ord('A'), sel = 1) # black
        chess.putChess(row = int(step_count/4), col = step_count % 4, sel = 0)
        chess.getChess(row = step[1][0]-1, col = ord(step[1][1])-ord('A'), sel = 1) # white
        chess.putChess(row = int(step_count/4) + 6, col = step_count % 4, sel = 0)
        step_count = step_count - 1
        if robot._running == False:
            exit()
    
    print("stop demo")
    chess.goHome()
    robot.luck()
    time.sleep(1)

def key_cmd_list():
    print("Z - exit     R - reset")
    print("P - pick     O - put")
    print("A - x+       D - x-")
    print("W - y+       S - y-")
    print("Q - z+       E - z-")
    print("L - Lock     U - Unlock")
    print("1 - demo1")
    print("2 - demo2 for test")
    print("B - go home  H - help")


def key_cb(key):
# def loop(key):
    global _STEP
    global _CMD
    global demo_sel

    if key == 0:
        return

    print('handle', key)
    if key == 'Z' or key == '\x03': 
        print('exit...')
        robot._running = False
    if key == 'R': 
        robot.reset()
        print("1 - demo1")
        print("2 - demo2 for test")
    if key == 'H': key_cmd_list()
    if key == 'P': robot.pick()
    if key == 'O': robot.release()
    if key == 'A': robot.mvX(_STEP)
    if key == 'D': robot.mvX(-_STEP)
    if key == 'W': robot.mvY(_STEP)
    if key == 'S': robot.mvY(-_STEP)
    if key == 'Q': robot.mvZ(_STEP)
    if key == 'E': robot.mvZ(-_STEP)
    if key == 'B': chess.goHome()
    if key == 'L': robot.lock()
    if key == 'U': robot.unlock()
    if key == '+' or key == '=': 
        _STEP = _STEP + 10 if _STEP < 100  else 100
        print("_STEP:",_STEP)
    if key == '-' or key == '_': 
        _STEP = _STEP - 10 if _STEP > 20  else 10
        print("_STEP:",_STEP)
    if key == '1': demo_sel = 1
    if key == '2': demo_sel = 2
    if key == '0': demo_sel = 0

# last_key = 0
# def key_cb(key):
#     global last_key
#     last_key = key.upper()
#     print('input', last_key)

# def key_scan():
#     while robot._running == True:
#         key_cb(getChar().upper().decode())
#         pass

# keyTask = Thread(target = key_scan)
# keyTask.start()


transport = serial.Serial(port=COM_PORT, baudrate = 115200, timeout = 0.1)
robot = Robot(transport)
robotRxTask = Thread(target = robot.rxTask)
chess = Chess(robot = robot)

if __name__ == '__main__':
    robotRxTask.start()

    key_ctr= ROBOT_KeyCtrl(key_cb)

    key_cmd_list()

    while robot._running == True:
        if demo_sel == 1:
            demo_sel = 0
            print("start demo1")
            demo1()
        if demo_sel == 2:
            demo_sel = 0
            print("start demo2")
            demo2()
        pass
    chess.robot.lock()
    exit()