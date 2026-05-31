# CH32V003 ADC2UVW Pinout

## MCU Target

**CH32V003F4P6** (TSSOP20). All pins listed are available.

## Pin Assignment Table

| Pin | Net / Signal     | Alt Function | I/O | Connected To             | Internal Peripheral | Pull / State | Notes                                                                                                              |
| --- | ---------------- | ------------ | --- | ------------------------ | ------------------- | ------------ | ------------------------------------------------------------------------------------------------------------------ |
| PD4 | LED              | -            | O   | Status LED (active-low)  | GPIO                | pull-up      | Open-drain sink to GND. `LED_ACTIVE_HIGH=0`                                                                        |
| PD6 | BATT_ADC         | ADC6         | I   | Battery voltage divider  | ADC                 | —            | 4.7 kΩ to GND, 10 kΩ to VBAT. Divider ratio 14.7:4.7. vRef = 5 V. Formula: `vBatt = adc × 14.7 / 4.7 × 5 / 1023`   |
| PA1 | DRV8311_ISEN_U   | ADC1         | I   | DRV8311 current sense U  | ADC                 | —            | Phase U current feedback from DRV8311 CSA (current shunt amplifier)                                                |
| PA2 | THROTTLE_ADC     | ADC0         | I   | Potentiometer / throttle | ADC                 | —            | Primary control input. 0–1023 mapped to PWM_MIN_US–PWM_MAX_US. Must drop ≤ `ADC_ARM_THRESHOLD` (10) to arm the ESC |
| PD0 | DRV8311_nFAULT   | —            | I   | DRV8311 fault output     | GPIO                | —            | Active-low fault indicator from DRV8311. Monitored via `drv8311_get_status()`                                      |
| PC0 | DRV8311_nSLEEP   | —            | O   | DRV8311 sleep control    | GPIO                | —            | High = awake, Low = sleep. Toggled by `drv8311_nsleep_ctrl()`                                                      |
| PD3 | DRV8311_PWM_SYNC | T2CH2        | O   | DRV8311 PWM sync input   | TIM2                | —            | Synchronisation signal from MCU timer to DRV8311 PWM generator                                                     |
| PD2 | DRV8311_ISEN_W   | ADC3         | I   | DRV8311 current sense W  | ADC                 | —            | Phase W current feedback from DRV8311 CSA                                                                          |
| PC7 | DRV8311_MISO     | —            | I   | DRV8311 SPI data out     | GPIO (bit-bang SPI) | —            | SPI master-in-slave-out. DRV8311 → MCU register read data                                                          |
| PC6 | DRV8311_MOSI     | —            | O   | DRV8311 SPI data in      | GPIO (bit-bang SPI) | —            | SPI master-out-slave-in. MCU → DRV8311 register write data                                                         |
| PC5 | DRV8311_SCK      | —            | O   | DRV8311 SPI clock        | GPIO (bit-bang SPI) | —            | SPI serial clock. Protocol supports standard SPI and tSPI (with device ID + extended header)                       |
| PC4 | DRV8311_ISEN_V   | ADC2         | I   | DRV8311 current V        | ADC                 | —            | Phase V current feedback from DRV8311 CSA                                                                          |
| PC3 | DRV8311_CS       | —            | O   | DRV8311 SPI chip select  | GPIO                | —            | Active-low SPI chip select for DRV8311                                                                             |
| PC2 | OLED_SCL         | —            | O   | SSD1306 OLED I²C clock   | GPIO (bit-bang I²C) | —            | Hardcoded in `OLED_driver.c` as `GPIOC, GPIO_Pin_2`. Open-drain bit-banged I²C                                     |
| PC1 | OLED_SDA         | —            | I/O | SSD1306 OLED I²C data    | GPIO (bit-bang I²C) | —            | Hardcoded in `OLED_driver.c` as `GPIOC, GPIO_Pin_1`. Open-drain bit-banged I²C                                     |

## DRV8311 SPI Interface Summary

| Signal | MCU Pin | Direction (MCU → DRV8311) |
| ------ | ------- | ------------------------- |
| SCK    | PC5     | OUT                       |
| MOSI   | PC6     | OUT                       |
| MISO   | PC7     | IN                        |
| CS     | PC3     | OUT                       |

SPI is bit-banged via `drv8311_instance_t.interface.spi_trans()` callback. Data is transmitted big-endian, 3 bytes per frame (SPI mode) or 4 bytes (tSPI mode). Register addresses and bitfields are defined in `drv8311_reg.h`.

## OLED I²C Interface Summary

| Signal | MCU Pin | Notes                                                      |
| ------ | ------- | ---------------------------------------------------------- |
| SDA    | PC1     | Bit-banged, open-drain via `GPIO_ResetBits`/`GPIO_SetBits` |
| SCL    | PC2     | Bit-banged, open-drain via `GPIO_ResetBits`/`GPIO_SetBits` |

Visible area: 88×48 pixels. Logical framebuffer: 128×56 pixels. X-offset 40, Y-offset 8. Driven by `OLED-Basic-Lib`.
