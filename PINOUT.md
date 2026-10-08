# Pinout

Source of truth: the top of `src/main.ino`. Do not change the map.

## Motor driver (TB6612FNG)

| TB6612FNG | GPIO | Macro | Function |
|---|---:|---|---|
| PWMA | 25 | `PIN_PWMA` | left motor speed |
| AIN1 | 26 | `PIN_AIN1` | left motor direction |
| AIN2 | 27 | `PIN_AIN2` | left motor direction |
| STBY | 5 | `PIN_STBY` | enable |
| BIN1 | 33 | `PIN_BIN1` | right motor direction |
| BIN2 | 4 | `PIN_BIN2` | right motor direction |
| PWMB | 32 | `PIN_PWMB` | right motor speed |
| VCC | 3V3 | | logic |
| GND | GND | | common ground |
| AO1/AO2 | | | left BO motor |
| BO1/BO2 | | | right BO motor |

## IR sensors (`IR_PIN[7] = {14, 13, 16, 17, 18, 19, 23}`, left to right)

| Sensor | GPIO | Weight in `W[]` | Status |
|---|---:|---:|---|
| IR1 | 14 | -3 | **DEFECTIVE: DO NOT USE** (reserved; still scanned by the sketch, M1) |
| IR2 | 13 | -2 | active |
| IR3 | 16 | -1 | active |
| IR4 | 17 | 0 | active, **centre / reference, must remain GPIO17** |
| IR5 | 18 | +1 | active |
| IR6 | 19 | +2 | active |
| IR7 | 23 | +3 | active |

Physical order, left to right: IR2, IR3, IR4, IR5, IR6, IR7.

## ToF (I2C)

| Signal | GPIO |
|---|---:|
| SDA | 21 |
| SCL | 22 |

## On-board

| Function | GPIO |
|---|---:|
| BOOT button (start/stop) | 0 |
| LED | 2 |

## GPIO map by number

| GPIO | Use |
|---:|---|
| 0 | BOOT button |
| 2 | LED |
| 4 | BIN2 |
| 5 | STBY |
| 13 | IR2 |
| 14 | IR1 (defective, unused) |
| 16 | IR3 |
| 17 | IR4 (centre) |
| 18 | IR5 |
| 19 | IR6 |
| 21 | SDA |
| 22 | SCL |
| 23 | IR7 |
| 25 | PWMA |
| 26 | AIN1 |
| 27 | AIN2 |
| 32 | PWMB |
| 33 | BIN1 |
