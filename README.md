# 26H 车载平衡滚球运动控制系统

本工程是 MSPM0G3507 上的 H 题控制程序，当前主线目标是完成“循线小车 + 车载滚球平衡控制”的全部题目要求。

题目原文位于：

- `D:\codexProjects\electronicCompetition_test\题目\H题_车载平衡滚球运动控制系统.pdf`

工程状态、题目要求和下一步路线见：

- [H_REQUIREMENTS.md](H_REQUIREMENTS.md)

## 当前固件状态

- 主控：MSPM0G3507。
- 驱动：后驱二轮小车，TB6612 A 通道为右后轮，B 通道为左后轮。
- 启停：B15 启动循迹，B5 停止并通过串口输出本次运行时间。
- 串口调参：UART1，PB6=TX，PB7=RX，9600 8N1。
- 备用串口：UART2，PA23=TX，PA24=RX，115200 8N1。
- 循迹：8 路灰度传感器 OUT1~OUT8。
- OLED：I2C0，PA28=SDA，PA31=SCL；当前 `OLED_RUNTIME_ENABLE=0`，因为已确认旧 OLED 模块故障。
- 步进电机 PWM：PA30，TIMG6 CCP1；方向/休眠/复位等控制脚已预留。

## 串口命令

常用命令以换行结束，大小写均可：

- `R`：启动。
- `S`：停止并刹车。
- `T120`：设置速度闭环目标编码器计数。
- `B255`：设置基础 PWM。
- `P1.2 I0.003 D0`：设置左右轮速度闭环 PID。
- `LP2.45 LD1.0 LC200`：设置循迹 PD 和差速限幅。
- `BR-14`：设置右轮补偿。

## 构建

在 `C:\Users\29543\workspace_ccstheia\PID\Debug` 下执行：

```powershell
& 'E:\CCS\CCS21.0\ccs\utils\bin\gmake.exe' clean all
```

代码修改后必须完整 build，并确认没有 error 和 warning。
