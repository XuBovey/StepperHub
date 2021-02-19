# 五子棋控制接口

配合[STM32F405_Stepper](https://github.com/XuBovey/StepperHubForStm32F405)，控制十字滑台完成五子棋的棋子移动。

## 使用说明

需根据识别到的串口进行修改.
``` python
transport = serial.Serial(port="COM10", baudrate = 115200)
```

## 控制原理

启动前需确认各轴限位器没有触发，TB板绿色LED灯处于闪烁状态。
启动后：
1. 机械臂运动到限位器位置，进行复位归零
2. 移动到home位置，用于取棋子的位置
3. 搬移棋子到指定位置


