import sys
import serial
import serial.tools.list_ports
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtCore import QTimer
from ui_robot_app import Ui_RobotApp
from robot import Robot

class Pyqt5_Serial(QtWidgets.QWidget, Ui_RobotApp):
    def __init__(self):
        super(Pyqt5_Serial, self).__init__()
        self.setupUi(self)
        self.init()
        self.setWindowTitle("kup_robot调试工具")
        self.ser = serial.Serial()
        self.port_check()

        # 接收数据和发送数据数目置零
        self.data_num_received = 0
        self.label_rx_cnt.setText(str(self.data_num_received))
        self.data_num_sended = 0
        self.label_tx_cnt.setText(str(self.data_num_sended))

        # 机械臂相关
        self.robot = Robot(transport=self.ser, data_send=self.data_send, log_console=self.log)

        # 默认按钮状态
        self.open_button.setEnabled(True)
        self.close_button.setEnabled(False)
        self.groupBox_moto_ctrl.setEnabled(False)
        self.groupBox_xyz_ctrl.setEnabled(False)

    def init(self):
        # 串口检测按钮
        self.s1__box_1.clicked.connect(self.port_check)

        # 打开串口按钮
        self.open_button.clicked.connect(self.port_open)

        # 关闭串口按钮
        self.close_button.clicked.connect(self.port_close)

        # 发送数据按钮
        self.s3__send_button.clicked.connect(lambda:self.data_send(self.s3__send_text.toPlainText()))

        # 定时发送数据
        self.timer_send = QTimer()
        self.timer_send.timeout.connect(self.data_send)
        self.timer_send_cb.stateChanged.connect(self.data_send_timer)

        # 定时器接收数据
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.data_receive)

        # 清除发送窗口
        self.s3__clear_button.clicked.connect(self.send_data_clear)

        # 清除接收窗口
        self.s2__clear_button.clicked.connect(self.receive_data_clear)

        # 电机控制
        self.pushButton_sub_1.clicked.connect(lambda:self.moto_ctrl(self.lineEdit_name_1.text(), -int(self.lineEdit_step_1.text())))
        self.pushButton_sub_2.clicked.connect(lambda:self.moto_ctrl(self.lineEdit_name_2.text(), -int(self.lineEdit_step_2.text())))
        self.pushButton_sub_3.clicked.connect(lambda:self.moto_ctrl(self.lineEdit_name_3.text(), -int(self.lineEdit_step_3.text())))
        self.pushButton_sub_4.clicked.connect(lambda:self.moto_ctrl(self.lineEdit_name_4.text(), -int(self.lineEdit_step_4.text())))
        self.pushButton_sub_5.clicked.connect(lambda:self.moto_ctrl(self.lineEdit_name_5.text(), -int(self.lineEdit_step_5.text())))

        self.pushButton_add_1.clicked.connect(lambda:self.moto_ctrl(self.lineEdit_name_1.text(), int(self.lineEdit_step_1.text())))
        self.pushButton_add_2.clicked.connect(lambda:self.moto_ctrl(self.lineEdit_name_2.text(), int(self.lineEdit_step_2.text())))
        self.pushButton_add_3.clicked.connect(lambda:self.moto_ctrl(self.lineEdit_name_3.text(), int(self.lineEdit_step_3.text())))
        self.pushButton_add_4.clicked.connect(lambda:self.moto_ctrl(self.lineEdit_name_4.text(), int(self.lineEdit_step_4.text())))
        self.pushButton_add_5.clicked.connect(lambda:self.moto_ctrl(self.lineEdit_name_5.text(), int(self.lineEdit_step_5.text())))

        # self.pushButton_reset.clicked.connect(lambda:self.robot.reset()) # not support yet
        self.pushButton_pick_up.clicked.connect(lambda:self.robot.pick())
        self.pushButton_release.clicked.connect(lambda:self.robot.release())

    def moto_ctrl(self, moto_name, step):
        self.log("tx: {:}:{:}\n".format(moto_name, step))
        self.robot.mvAngle(moto_name, step*self.robot.anglePerStep, False)

    # 串口检测
    def port_check(self):
        # 检测所有存在的串口，将信息存储在字典中
        self.Com_Dict = {}
        port_list = list(serial.tools.list_ports.comports())
        self.s1__box_2.clear()
        for port in port_list:
            self.Com_Dict["%s" % port[0]] = "%s" % port[1]
            self.s1__box_2.addItem(port[0])
        if len(self.Com_Dict) == 0:
            self.state_label.setText(" 无串口")

    # 打开串口
    def port_open(self):
        self.ser.port = self.s1__box_2.currentText()
        self.ser.baudrate = int(self.s1__box_3.currentText())
        self.ser.bytesize = int(self.s1__box_4.currentText())
        self.ser.stopbits = int(self.s1__box_6.currentText())
        self.ser.parity = self.s1__box_5.currentText()

        try:
            self.ser.open()
        except:
            QMessageBox.critical(self, "Port Error", "串口打开错误！")
            return None

        # 打开串口接收定时器，周期为2ms
        self.timer.start(10)

        if self.ser.isOpen():
            self.open_button.setEnabled(False)
            self.close_button.setEnabled(True)
            self.groupBox_moto_ctrl.setEnabled(True)
            self.groupBox_xyz_ctrl.setEnabled(True)

    # 关闭串口
    def port_close(self):
        self.timer.stop()
        self.timer_send.stop()
        try:
            self.ser.close()
        except:
            pass

        self.open_button.setEnabled(True)
        self.close_button.setEnabled(False)
        self.groupBox_moto_ctrl.setEnabled(False)
        self.groupBox_xyz_ctrl.setEnabled(False)

        self.lineEdit_resend_period.setEnabled(True)
        # 接收数据和发送数据数目置零
        self.data_num_received = 0
        self.label_rx_cnt.setText(str(self.data_num_received))
        self.data_num_sended = 0
        self.label_tx_cnt.setText(str(self.data_num_sended))

    # 发送数据
    def data_send(self, data):
        if self.ser.isOpen():
            input_s = data
            if input_s != "":
                # 非空字符串
                if self.hex_send.isChecked():
                    # hex发送
                    input_s = input_s.strip()
                    send_list = []
                    while input_s != '':
                        try:
                            num = int(input_s[0:2], 16)
                        except ValueError:
                            QMessageBox.critical(self, 'wrong data', '请输入十六进制数据，以空格分开!')
                            return None
                        input_s = input_s[2:].strip()
                        send_list.append(num)
                    input_s = bytes(send_list)
                else:
                    # ascii发送
                    input_s = (input_s + '\r\n').encode('utf-8')

                num = self.ser.write(input_s)
                self.data_num_sended += num
                self.label_tx_cnt.setText(str(self.data_num_sended))
        else:
            pass

    def log(self, log):
        self.write_data_to_rx_box("LOG: " + log)

    def write_data_to_rx_box(self, data):
        self.s2__receive_text.insertPlainText(data)
        # 获取到text光标
        textCursor = self.s2__receive_text.textCursor()
        # 滚动到底部
        textCursor.movePosition(textCursor.End)
        # 设置光标到text中去
        self.s2__receive_text.setTextCursor(textCursor)

    # 接收数据
    def data_receive(self):
        try:
            num = self.ser.inWaiting()
        except:
            self.port_close()
            return None
        if num > 0:
            data = self.ser.read(num)
            num = len(data)

            # 统计接收字符的数量
            self.data_num_received += num
            self.label_rx_cnt.setText(str(self.data_num_received))
            
            # 串口数据处理
            self.robot.rxDataHandle(str(data))

            # hex显示
            if self.hex_receive.checkState():
                out_s = ''
                for i in range(0, len(data)):
                    out_s = out_s + '{:02X}'.format(data[i]) + ' '
                self.write_data_to_rx_box(out_s)
            else:
                # 串口接收到的字符串为b'123',要转化成unicode字符串才能输出到窗口中去
                self.write_data_to_rx_box(data.decode('utf-8'))
        else:
            pass

    # 定时发送数据
    def data_send_timer(self):
        if self.timer_send_cb.isChecked():
            self.timer_send.start(int(self.lineEdit_resend_period.text()))
            self.lineEdit_resend_period.setEnabled(False)
        else:
            self.timer_send.stop()
            self.lineEdit_resend_period.setEnabled(True)

    # 清除显示
    def send_data_clear(self):
        self.s3__send_text.setText("")

    def receive_data_clear(self):
        self.s2__receive_text.setText("")


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    myshow = Pyqt5_Serial()
    myshow.show()
    sys.exit(app.exec_())

