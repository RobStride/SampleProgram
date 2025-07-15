# RobStride Motor Control Library README

## 1. Project Introduction

This project provides a STM32 (HAL) control demo for the Lingzu Era RS01/RobStride motors, based on the `RobStride.h`/`RobStride01.cpp` motor library. It supports both Lingzu private and MIT protocols, enabling all major control modes (enable, disable, position/speed/current/torque/MIT mode, parameter setting, zeroing, protocol switching, etc.).

---

## 2. Getting Started

- **Hardware Board:** ALIENTEK (Waveshare) Elite STM32F103ZET6 Development Board (tested stable)
- Project requires STM32CubeMX/Keil or any STM32 HAL-based project
- Add `RobStride.h` and `RobStride01.cpp` to your project
- Include `main.h`, `can.h`, `gpio.h`
- CAN bus must be initialized and the baud rate must match the motor setting

### Main Loop Example

It is recommended to use a variable `mode` to switch between different control functions. Each `case` corresponds to a typical function. For details, see the example `main.c`.

```c
uint8_t mode = 0; // Set by external input, determines the current function
while (1) {
    switch(mode) {
        case 0: RobStride_01.Enable_Motor(); break;
        case 1: RobStride_01.Disenable_Motor(1); break;
        case 2: RobStride_01.RobStride_Motor_move_control(5, 0, 0, 0.0, 0.0); break;
        case 3: RobStride_01.RobStride_Motor_Pos_control(2.0, 2); HAL_Delay(5); break;
        case 4: RobStride_01.RobStride_Motor_Speed_control(3.5, 5.0); HAL_Delay(5); break;
        case 5: RobStride_01.RobStride_Motor_current_control(1.2); HAL_Delay(5); break;
        // ... see main.c for more cases
        // MIT protocol dedicated APIs see below
        default: break;
    }
    HAL_Delay(50);
}
```

### CAN Rx Callback Example

After receiving CAN data, call the analysis function:

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

## 3. Main API List

| Feature          | Function Name                                           | Description                           |
| ---------------- | ------------------------------------------------------- | ------------------------------------- |
| Enable motor     | `Enable_Motor()`                                        | Enable motor, enter running state     |
| Disable motor    | `Disenable_Motor(clear_err)`                            | Disable, `clear_err=1` to clear error |
| Torque control   | `RobStride_Motor_move_control(t, angle, speed, kp, kd)` | Torque/Speed/Angle composite          |
| PP Position Mode | `RobStride_Motor_Pos_control(speed, angle)`             | Position interpolation mode           |
| Speed Mode       | `RobStride_Motor_Speed_control(speed, limit_cur)`       | Constant speed control                |
| Current Mode     | `RobStride_Motor_current_control(current)`              | Constant current control              |
| Zeroing          | `Set_ZeroPos()`                                         | Set current angle as zero             |
| Parameter read   | `Get_RobStride_Motor_parameter(addr)`                   | Read parameter                        |
| Parameter write  | `Set_RobStride_Motor_parameter(addr, val, mode)`        | Write parameter                       |
| Protocol switch  | `RobStride_Motor_MotorModeSet(F_CMD)`                   | Switch to private/Canopen/MIT         |

**MIT Protocol Specific**

| Feature               | Function Name                                                                                          |
| --------------------- | ------------------------------------------------------------------------------------------------------ |
| MIT Enable/Disable    | `RobStride_Motor_MIT_Enable()` / `RobStride_Motor_MIT_Disable()`                                       |
| MIT Composite Control | `RobStride_Motor_MIT_Control(angle, speed, kp, kd, torque)`                                            |
| MIT Pos/Speed Control | `RobStride_Motor_MIT_PositionControl(pos, speed)` / `RobStride_Motor_MIT_SpeedControl(speed, cur_lim)` |
| MIT Error Clear       | `RobStride_Motor_MIT_ClearOrCheckError(cmd)`                                                           |
| MIT Zeroing           | `RobStride_Motor_MIT_SetZeroPos()`                                                                     |
| MIT ID/Mode Set       | `RobStride_Motor_MIT_SetMotorId(id)` / `RobStride_Motor_MIT_SetMotorType(type)`                        |
| MIT Protocol Switch   | `RobStride_Motor_MIT_MotorModeSet(type)`                                                               |

---

## 4. Important Notes

- **Protocol switching:** To switch protocol, first use `RobStride_Motor_MotorModeSet(0x**)` or `RobStride_Motor_MIT_MotorModeSet(0x**)`, then power-cycle (reboot) the motor before using new protocol-related functions.
- **Standard CAN frame ID range is 0x00\~0x7F**; do not exceed this range.
- **Parameter addresses and details are subject to the RS01 motor protocol manual.**
- All `Enable_Motor`/`Disenable_Motor` functions auto-adapt to protocol but you should use the appropriate APIs as per protocol when possible.
- Control parameters: angle in radians, speed in rad/s, current in Amps.

---

## 5. FAQ

- **Q: PP position mode enabled but no movement?**\
  A: Make sure to set an appropriate target speed (e.g. 2\~5 rad/s) with `RobStride_Motor_Pos_control`, and check if speed is set via `0x7018`. Use sufficient delays after mode switching.
- **Q: Wrong CAN ID in MIT commands?**\
  A: Make sure `CAN_ID` does not exceed 0x7F; if you see IDs like 0x47F, check your ID assignment and masking in code.
- **Q: No response from motor?**\
  A: Check CAN bus wiring, baudrate, power supply, and make sure the master and motor IDs/protocol match.

---

## 6. References

- RS01 Motor Protocol Manual
- This library source code and main.c example

---

## 7. Contact

For support, contact the author or RobStride technical support.

---

