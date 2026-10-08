# Bill of materials

No prices are listed. Specifications are given only where the project information or the
code states them; everything else is marked **USER MUST VERIFY**.

| Component | Quantity | Purpose | Specification | Notes |
|---|---:|---|---|---|
| ESP32 Dev Module | 1 | controller | Arduino IDE board "ESP32 Dev Module"; core 2.x or 3.x | uses the on-board BOOT button (GPIO0) and LED (GPIO2) |
| TB6612FNG motor driver (breakout) | 1 | drives both motors | dual H-bridge; 3.3 V logic as used here | `STBY` wired to GPIO5 |
| BO geared DC motor | 2 | differential drive | rated voltage and current: **USER MUST VERIFY** | one per side |
| IR line sensor modules | 6 working (7 positions) | line detection | digital DO output; 3-pin modules with a pot; IR4 is a 4-pin module | IR1 (GPIO14) defective, not used; supply and DO level: **USER MUST VERIFY** |
| VL53-series ToF sensor (breakout) | 1 | obstacle distance | VL53L0X-compatible interface, I2C | supply range and pull-ups: **USER MUST VERIFY** |
| Battery | 1 | main power | 7.4 V nominal | capacity and discharge rating not specified |
| "Total 360" buck converter | 1 | regulated low-voltage supply for ESP32 and sensors | output voltage and current rating: **USER MUST VERIFY** | set and measure the output before connecting the ESP32 |
| Custom chassis | 1 | frame | about 250 mm long, about 190 mm wheel span | dimensions drive the geometry constants |
| Wheels | 2 | drive | diameter: **USER MUST MEASURE** | diameter affects speed (`C = pi D`) |
| Caster (or skid) | 1 | passive support | | must not catch on the line |
| Wiring | as needed | | thick leads for battery and motor, thin for logic | |
| Connectors | as needed | | | secure battery connector; avoid loose logic connectors |
| Mounting hardware | as needed | rigid sensor bar and ToF mounts | | constant sensor height |
| Gyro | 0 | not used | | available during development; the firmware does not use it |
