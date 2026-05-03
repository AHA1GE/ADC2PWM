# ADC2PWM

> 无刷风扇/电调控制器固件（支持 AVR、CH32、ESP32）

## 项目简介

ADC2PWM 是一款将模拟电压（ADC）输入转换为 PWM 信号输出的通用固件，适用于无刷风扇、舵机等场景。支持多平台（AVR、CH32、ESP32），具备 OLED 屏幕显示、低电压保护、软启动等功能。

---

## 功能特性

- 支持 AVR、CH32、ESP32 多平台
- 启动自检与安全锁定
- ADC 输入软启动、PWM 输出限幅
- OLED 屏幕实时显示电池电压、油门、PWM 值
- 低电压自动降功率/锁定
- 状态机架构，易于扩展

---

## 硬件连接说明

| 信号       | AVR/Arduino | CH32V003 | ESP32      | 说明            |
| ---------- | ----------- | -------- | ---------- | --------------- |
| PWM_OUT    | D2          | PD4      | 用户自定义 | PWM 输出        |
| ADC_IN     | A0          | PA2      | 用户自定义 | 油门/电位器输入 |
| BATT_SENSE | A1          | PD6      | 用户自定义 | 电池电压检测    |
| LED        | D3          | PD1      | 用户自定义 | 状态指示灯      |
| OLED_SDA   | -           | PC1      | -          | OLED 屏幕 SDA   |
| OLED_SCL   | -           | PC2      | -          | OLED 屏幕 SCL   |

> 具体引脚请参考 ADC2PWM.ino 中的宏定义，部分平台需手动指定。

---

## 编译方法

### 1. AVR/Arduino

- 使用 Arduino IDE 或 PlatformIO 打开 `ADC2PWM/ADC2PWM.ino`
- 选择对应开发板（如 Arduino Nano/Uno）
- 安装依赖库：U8x8lib（如需 OLED）
- 编译并上传

### 2. CH32V003

- 使用 Arduino IDE 或 PlatformIO 打开 `ADC2PWM/ADC2PWM.ino`
- 安装 WCH 开发板支持
- 选择 CH32V00X 开发板
- 打开 `ADC2PWM/` 文件夹，确保已包含 OLED-Basic-Lib
- 编译并上传

### 3. ESP32

- 使用 Arduino IDE 或 PlatformIO 打开 `ADC2PWM/ADC2PWM.ino`
- 选择对应 ESP32 开发板
- 安装依赖库：U8x8lib（如需 OLED）
- 根据实际连接修改引脚定义
- 编译并上传

---

## 依赖库

- [U8x8lib](https://github.com/olikraus/u8g2)（OLED 显示，Arduino/ESP32）

## 依赖库（已修改）

- [Servo](https://github.com/arduino-libraries/Servo)（增加 CH32 平台简易支持）
- [OLED-Basic-Lib](https://github.com/bdth-7777777/OLED-Basic-Lib)（为 CH32V003 精简代码）

---

## 参考与鸣谢

- Servo: Michael Margolis
- OLED-Basic-Lib: bdth-7777777

---

## License

本项目遵循 LGPL 及各依赖库协议，详见源代码注释。
