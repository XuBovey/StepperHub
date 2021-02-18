# 2020-10-20

## 机械臂TB板接口规划

步进电机控制IO分配表：

|电机编号|步进控制 STP|方向控制 DIR|使能 EN|限位|公共端COM|
|---|---|---|---|---|---|
|X轴 M底盘|PA7/**X8** TIM8CH1N|PB10/**Y9**|~~PB11/**Y10**~~|PB14/**Y7**|5V|
|Y轴 N大臂|PA5/**X6** TIM2CH1|PA4/**X5**|~~PA6/**X7**~~|PB12/**Y5**|5V|
|Z轴 O小臂|PB13/**Y6** TIM1CH1N|PB6/**X9**|~~PB7/**X10**~~|PB15/**Y8**|5V|
|L轴 L同步带|PA6/**X7** TIM13CH1|PB11/**Y10**||PB7/**X10**|5V|

其他功能IO分配表：

|功能|管脚分配|
|---|---|
|吸盘|PC5/**X12** (接口板丝印X4)|
|RX|PA3/**X4**|
|TX|PA2/**X3**|
|舵机|PB8/**Y3** (接口板丝印X3)，TIM10CH1|

限位器控制放在中断函数中执行，当产生中断时停止执行。

控制电机移动到限位器位置后应设置当前位置为0点

需修改限位器接线为默认常闭状态

## 状态灯

绿色：串口读取工作

黄色：Y轴电机动作

红色：X轴电机动作

蓝色：Z轴电机动作

## stepperhub增补stepper_state结构体

增加代码如下：

```c
// stepperController.h
typedef HAL_StatusTypeDef (*funPwmCtrl)(TIM_HandleTypeDef * STEP_TIMER, uint32_t  STEP_CHANNEL);
typedef struct {
    char name;

    // ...

    // start fun
    funPwmCtrl start;

    // stop fun
    funPwmCtrl stop;
} stepper_state;
```

这样可使驱动支持STM32上CH*N之类的PWM输出，因需用到`stm32f4xx_hal_tim_ex.h`中的库函数。

## 电机控制命令

参考根目录readme.md

格式：

```
<command><stepper>[.parameter][:value]

<command>     : add | set | reset | get
<stepper>     : X | Y | Z  (or whatever single-letter names will be added in the future)
[.parameter]  : parameter name (the field of the stepper_state structure)
[:value]      : any 32-bit signed integer value (-2147483648 .. 2147483647)
```

## 电机转速计算

电机转速由PWM频率控制，其配置由函数`SetStepTimerByCurrentSPS`完成。其最主要的部分是:

``` c
uint32_t timerTicks = STEP_TIMER_CLOCK / stepper -> currentSPS;
```

STEP_TIMER_CLOCK为定时器时钟，程序中配置为84MHz。`currentSPS`为当前脉冲频率，除法的结果就是定时器计数周期数。而`currentSPS`是在`minSPS`和`maxSPS`之间的。

速度变化的快慢取决于`accelerationSPS`即加速度，每次修改`currentSPS`的步进值。

`currentSPS`是控制电机的脉冲，电机转速的转换还要考虑步距角和电机驱动的细分，例如当前采用的电机步距角是1.8度，即每圈需要200个步进，细分为32是把每个步进细分为32个脉冲，所以电机完整转一圈需要200*32=6400个脉冲，电机的转速就是currentSPS*60/6400rpm

## 限位器控制方法

限位器的状态由定时器5的回调函数中查询并发现边沿信号。信号触发时设置当前位置为目的位置，使电机制动，为了达到迅速制动的目的，还修改了`Stepper_PulseTimerUpdate`函数的`SS_RUNNING_FORWARD`和`SS_RUNNING_BACKWARD`状态的处理。增加代码段：

```c
      // handle stop from limited switch.
      if (stepper->currentPosition == stepper->targetPosition) {
          stepper->status = SS_STOPPED;
          stepper->stop(stepper->STEP_TIMER, stepper->STEP_CHANNEL);
          stepper -> currentSPS = stepper -> minSPS;
          kprintf("limited %c.stop:%ld\r\n", stepper->name, stepper->currentPosition);
          break;
      }
```

## 其中：

- **get** - returns the current value of the [parameter] 
- **add** - adds [value] to the current [parameter] value
- **set** - sets new [value] to the [parameter]
- **reset** - resets the [parameter] to its factory default (may used with ".all")
