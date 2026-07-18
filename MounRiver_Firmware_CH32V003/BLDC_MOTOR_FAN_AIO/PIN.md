# CH32V003 BLDC_MOTOR_FAN_AIO Pinout

## MCU Target

**CH32V003F4P6** (TSSOP20). All pins listed are available.

## Pin Assignment

PD4 ADC_W A7 10K to phase W, 10k to GND
PD5 ADC_U A5 10K to phase U, 10k to GND
PD6 ADC_V A6 10K to phase V, 10k to GND
PD7 ESC_ERROR GPI
PA1 NEUTRAL A1 20K to each phase, no other connection
PA2 ADC_BATT A0
PD0 Btn_Up GPI
PC0 Btn_Down GPI
PD3 ESC_CURR_W A4
PD2 ESC_CURR_V A3
PD1 SWD/LED GPO(init at last, with a small delay, although not necessary)
PC7 DRV8311_MISO
PC6 DRV8311_MOSI
PC5 DRV8311_SCK
PC4 ESC_CURR_U A2
PC3 DRV8311_CS
PC2 OLED_SCL
PC1 OLED_SDA
