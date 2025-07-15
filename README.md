# RobStride 电机控制库使用说明 / RobStride Motor Control Library README

## 一、项目简介 Project Introduction

本工程为灵足时代 RS01/RobStride 电机 STM32 控制例程，核心为 `RobStride.h`/`RobStride01.cpp` 电机类库，支持灵足私有协议与MIT协议两种控制方式，实现了电机各主流控制模式（使能、失能、位置/速度/电流/运控/MIT模式、参数设置、零点、协议切换等）。

> This project is a STM32 control demo for RS01/RobStride motors from Lingzu Era. The core is the `RobStride.h`/`RobStride01.cpp` motor library, supporting both Lingzu private and MIT protocols, implementing enable/disable, position/speed/current/torque/MIT control, parameter settings, zeroing, and protocol switching.

---

## 二、使用说明 Getting Started

- **硬件平台 Hardware Board**：正点原子精英版 STM32F103ZET6 开发板 (Waveshare/ALIENTEK Elite STM32F103ZET6 Board, 实测通过)
- `RobStride.h`, `RobStride01.cpp` 添加到项目
- 需包含 `main.h`, `can.h`, `gpio.h`
- CAN总线已初始化，波特率与电机设置一致

### 2. 主循环用法 Main Loop Usage

核心主循环推荐如下结构（建议通过变量`mode`实现控制场景切换，每个`case`对应一种典型功能，具体见 main.c 示例）：

```c
uint8_t mode = 0; // 外部赋值，决定当前控制功能

while (1) {
    switch(mode) {
        case 0: RobStride_01.Enable_Motor(); break;
        case 1: RobStride_01.Disenable_Motor(1); break;
        case 2: RobStride_01.RobStride_Motor_move_control(5, 0, 0, 0.0, 0.0); break;
        case 3: RobStride_01.RobStride_Motor_Pos_control(2.0, 2); HAL_Delay(5); break;
        case 4: RobStride_01.RobStride_Motor_Speed_control(3.5, 5.0); HAL_Delay(5); break;
        case 5: RobStride_01.RobStride_Motor_current_control(1.2); HAL_Delay(5); break;
        // ... 其他case见main.c
        // MIT协议专用接口见后面注意事项
        default: break;
    }
    mode = 23;
    HAL_Delay(50);
}
```

### 3. CAN 回调函数 CAN Rx Callback

CAN数据接收后需调用解析函数：

```c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RXHeader,RxData) == HAL_OK) {
        if (RXHeader.IDE == CAN_ID_EXT)
            RobStride_01.RobStride_Motor_Analysis(RxData, RXHeader.ExtId);
        else
            RobStride_01.RobStride_Motor_Analysis(RxData, RXHeader.StdId);
    }
}
```

---

## 三、主要接口说明 Main API List

| 功能     | 接口名 / Function Name                                     | 简述 / Description     |
| ------ | ------------------------------------------------------- | -------------------- |
| 使能电机   | `Enable_Motor()`                                        | 进入运行状态               |
| 失能电机   | `Disenable_Motor(clear_err)`                            | 停止运行，`clear_err=1`清错 |
| 运控模式   | `RobStride_Motor_move_control(t, angle, speed, kp, kd)` | 力矩/速度/角度复合控制         |
| PP位置模式 | `RobStride_Motor_Pos_control(speed, angle)`             | 插补位置控制               |
| 速度模式   | `RobStride_Motor_Speed_control(speed, limit_cur)`       | 恒速控制                 |
| 电流模式   | `RobStride_Motor_current_control(current)`              | 恒流控制                 |
| 零点设置   | `Set_ZeroPos()`                                         | 设置当前角度为零点            |
| 读取参数   | `Get_RobStride_Motor_parameter(addr)`                   | 读取参数                 |
| 设置参数   | `Set_RobStride_Motor_parameter(addr, val, mode)`        | 设置参数                 |
| 协议切换   | `RobStride_Motor_MotorModeSet(F_CMD)`                   | 切换私有/Canopen/MIT协议   |

#### MIT协议相关

| 功能           | 接口名                                                                                                    |
| ------------ | ------------------------------------------------------------------------------------------------------ |
| MIT使能/失能     | `RobStride_Motor_MIT_Enable() / RobStride_Motor_MIT_Disable()`                                         |
| MIT综合控制      | `RobStride_Motor_MIT_Control(angle, speed, kp, kd, torque)`                                            |
| MIT位置/速度控制   | `RobStride_Motor_MIT_PositionControl(pos, speed)` / `RobStride_Motor_MIT_SpeedControl(speed, cur_lim)` |
| MIT清错        | `RobStride_Motor_MIT_ClearOrCheckError(cmd)`                                                           |
| MIT零点设置      | `RobStride_Motor_MIT_SetZeroPos()`                                                                     |
| MIT设置ID/运行模式 | `RobStride_Motor_MIT_SetMotorId(id)` / `RobStride_Motor_MIT_SetMotorType(type)`                        |
| MIT协议切换      | `RobStride_Motor_MIT_MotorModeSet(type)`                                                               |

---

## 四、注意事项 Important Notes

- **协议切换**：如需切换协议，请首先通过 `RobStride_Motor_MotorModeSet(0x**)` 或 `RobStride_Motor_MIT_MotorModeSet(0x**)` 进行协议切换，并断电重启电机，方可使用。
- **MIT接口仅在 MIT 协议下有效**，普通模式下禁止误用。
- **标准帧 CAN ID 范围为 0x00\~0x7FF**，建议不超过此范围。
- **参数地址与说明请参考 RS01 电机说明书**。
- 本库中所有 `Enable_Motor`/`Disenable_Motor` 接口均能自动兼容当前协议，但请优先按协议规范分别调用 MIT/私有接口，避免新手混淆。
- 控制参数如角度单位为弧度，速度单位为 rad/s，电流单位为 A。

---

## 五、常见问题 FAQ

- **Q：PP位置模式电机使能但不转？**\
  A：务必调用 `RobStride_Motor_Pos_control(speed, angle)` 前设置合适的目标速度（如 2\~5 rad/s），并检查速度参数是否通过 `0x7018` 写入。延时设置亦需充分。
- **Q：MIT接口下ID发错？**\
  A：请保证 CAN\_ID 不高于 0x7F，若出现 0x47F 这类问题请检查 ID 赋值与掩码（详见代码实现）。
- **Q：电机无响应？**\
  A：请检查 CAN 硬件连通性、波特率、电源，及主控和电机 ID/协议是否一致。

---

## 六、参考资料 Reference

- 《RS01电机通信协议说明书》
- 本库源码及 main.c 示例

---

## 七、联系方式 Contact

如有问题欢迎联系作者或灵足时代技术支持。

---

