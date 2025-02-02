# 电机控制系统仿真实验报告

<center><div style='height:2mm;'></div><div style="font-family:华文楷体;font-size:14pt;">姓名 卢裕锐  </div></center>
<center><span style="font-family:华文楷体;font-size:12pt">哈尔滨工程大学创梦之翼战队，电控组，邮箱 tianle4@outlook.com<br /></span>


## 八字方针：建模 分析 设计 校验

### 一、建模

#### 1.1 确定传递函数形式

#### ![0](picture/transfer function.jpg)

#### <font color=Green>1.2 确定传递函数参数</font>

<font color=Blue>运用实验法，向系统添加已知的输入，通过研究其响应确定传递函数参数。在这里填入传递函数参数，具体推导方法写在第五节。</font>

### 二、分析

<font color=Blue>利用根轨迹，伯德图等方法，分析开环系统的性能，如响应速度，超调大小，稳定性，稳态误差等。</font>

#### <font color=Green>2.1 系统开环传递函数根轨迹</font>

<font color=Blue>放一系列开环系统根轨迹的图片并做一定分析</font>

#### <font color=Green>2.2 系统开环传递函数伯德图</font>

<font color=Blue>放一系列开环系统伯德图并做一定分析</font>

### 三、设计

#### 3.1 PID控制器的数学表达式

C(s) = K+ $\frac{Ki}{s}$ + K~d~s

#### 3.2 控制器C语言代码

```c
void PID_Init(pid_t *pid, float Kp, float Ki, float Kd)//pid初始化
{
    pid->err = 0;
    pid->err_sum = 0;
    pid->err_difference = 0;
    pid->last_err = 0;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->output_filter = 0;
    pid->alpha = 0.1f;  // 滤波系数
}

float PID_Calculate(pid_t *pid, float target, float current)
{
    pid->err = target - current;
    pid->err_sum += pid->err;
    pid->err_difference = pid->err - pid->last_err;
    pid->last_err = pid->err;
    
    if(pid->err_sum > 1000) pid->err_sum = 1000;
    if(pid->err_sum < -1000) pid->err_sum = -1000;
    
    float output = (pid->Kp * pid->err) + (pid->Ki * pid->err_sum) + (pid->Kd * pid->err_difference);
    
    pid->output_filter = pid->alpha * output + (1 - pid->alpha) * pid->output_filter;//低通滤波
    
    return pid->output_filter;
}
```

#### <font color=Green>3.3 加入控制器后的simulink仿真结构图</font>

<font color=Blue>在这里放一张simulink仿真结构图并做说明。</font>

#### <font color=Green>3.4 simulink仿真结果</font>

<font color=Blue>把simulink仿真结果放在这里并做说明。</font>

### 四、校验（最重要）

#### 4.1 速度闭环

##### （1）系统框图

![2](picture/velocity_control.jpg)

##### （2）关键代码

```c
void PID_Init(pid_t *pid, float Kp, float Ki, float Kd)//pid初始化
{
    pid->err = 0;
    pid->err_sum = 0;
    pid->err_difference = 0;
    pid->last_err = 0;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->output_filter = 0;
    pid->alpha = 0.1f;  // 滤波系数
}

	Input = PID_Calculate(&pid_velocity, current_reference, Motor.Velocity);
    Motor_Simulation(&Motor, Input, dt);
  	Motor.Velocity = Get_Motor_Velocity(&Motor);

float PID_Calculate(pid_t *pid, float target, float current)
{
    pid->err = target - current;
    pid->err_sum += pid->err;
    pid->err_difference = pid->err - pid->last_err;
    pid->last_err = pid->err;
    
    if(pid->err_sum > 1000) pid->err_sum = 1000;
    if(pid->err_sum < -1000) pid->err_sum = -1000;
    
    float output = (pid->Kp * pid->err) + (pid->Ki * pid->err_sum) + (pid->Kd * pid->err_difference);
    
    pid->output_filter = pid->alpha * output + (1 - pid->alpha) * pid->output_filter;//低通滤波
    
    return pid->output_filter;
}
```

##### （3）阶跃响应时域图

<font color=Blue>放一张图，应至少包含期望速度与速度闭环控制系统实际速度两条曲线。并对比分析结果。</font>

![2](picture/step.png)

0 - 500ms上升速度较慢，上升时间长，动态响应慢。达到500ms后存在超调，随后趋于稳定，稳态误差为零

##### （4） 斜坡响应时域图

![2](picture/ramp.png)

##### （5）频率响应时域图

![2](picture/frequency.png)

#### 4.2 角度闭环

##### （1）系统框图

![3](picture/angle_control.jpg)

##### （2）关键代码

```c
single_output = PID_Calculate(&pid_angle, current_reference, Motor_Single.Angle);//单
Motor_Simulation(&Motor_Single, single_output, dt);
Motor_Single.Angle = Get_Motor_Angle(&Motor_Single);

cascade_outer_output = PID_Calculate(&pid_angle, current_reference,Motor_Cascade.Angle);//串
cascade_inner_output = PID_Calculate(&pid_angle2, cascade_outer_output, Motor_Cascade.Velocity);
Motor_Simulation(&Motor_Cascade, cascade_inner_output, dt);
Motor_Cascade.Angle = Get_Motor_Angle(&Motor_Cascade);

float PID_Calculate(pid_t *pid, float target, float current)//pid运算
{
    pid->err = target - current;
    pid->err_sum += pid->err;
    pid->err_difference = (pid->err - pid->last_err);
    pid->last_err = pid->err;

    if(pid->err_sum > 1000) pid->err_sum = 1000;
    if(pid->err_sum < -1000) pid->err_sum = -1000;
    
    float output = (pid->Kp * pid->err) + 
                   (pid->Ki * pid->err_sum) + 
                   (pid->Kd * pid->err_difference);
    
    pid->output_lvbo = pid->lvbo * output + (1 - pid->lvbo) * pid->output_lvbo;
    
    return pid->output_lvbo;//滤波输出
}
```

##### （3）阶跃响应时域图

![3](picture/angle_step.png)

图中：蓝线为串级pid，绿线为单极pid

单级pid的响应更为快速，但超调也更为明显，相对串级pid来说不太稳定。而串级pid响应速度较慢，但相对稳定，振荡较小。

##### （4）频率响应时域图

![3](picture/angle_frequency.png)

图中：淡蓝线为理想曲线，绿线为单级，蓝线为串级

单级pid响应曲线的幅值响应较大，波动明显，同时滞后较大，动态响应差；而串级pid响应曲线相对稳定，滞后也更小，动态响应好

##### （5）抗干扰性能时域图

<font color=Blue>放一张图，应至少包含期望角度，单级PID实际角度，串级PID实际角度三条曲线。并于无干扰的控制情况对比分析结果。</font>

![3](picture/disturbance.png)

### 五、扩展内容

### <font color=Green>1、推导该电机系统的传递函数</font>

<font color=Blue>描述建立传递函数的方法、自己的思路以及结果</font>

<font color=Blue>附上设计过程中重要的截图</font>

### <font color=Green>2、复合控制</font>

<font color=Blue>自行查阅资料，采用能想到的方法设计更好的控制器（PID优化，前馈等），从而提高系统的性能。</font>

#### <font color=Green>2.1 控制器设计</font>

<font color=Blue>在这里详细介绍自己选择的控制器。并贴上系统框图</font>

#### <font color=Green>2.2 关键代码</font>

```
//此处粘贴代码
```

#### <font color=Green>2.3 效果展示</font>

（1）角度闭环阶跃响应

<font color=Blue>放一张图，应至少包含期望角度，用传统PID闭环控制的实际曲线和用改进控制器的实际曲线三条曲线。并对比分析结果。</font>

（2）角度闭环频率响应

<font color=Blue>放一张图，应至少包含期望角度，用传统PID闭环控制的实际曲线和用改进控制器的实际曲线三条曲线。并对比分析结果。</font>

（3）角度闭环抗干扰性能

<font color=Blue>放一张图，应至少包含期望角度，用传统PID闭环控制的实际曲线和用改进控制器的实际曲线三条曲线。并对比分析结果。</font>

### <font color=Green>3、滤波</font>

<font color=Blue>在Matlab仿真中给系统的反馈信号叠加噪声，并自行设计滤波器滤除噪声，比较滤波前后的控制效果。</font>
