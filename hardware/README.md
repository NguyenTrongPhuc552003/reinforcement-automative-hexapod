## System Components

### 1. **BeagleBone Black (BBB)**
BeagleBone Black is used as the central processing unit, running Linux and handling all real-time control logic. It communicates with:
- **PCA9685 servo drivers** via I2C (SCL on P9.19, SDA on P9.20)
- **Ultrasonic sensor** via GPIO (TRIG on P8.11, ECHO on P8.12)
- **IMU (MPU6050)** via I2C

It also generates PWM signals and controls other peripherals (e.g., thermal fan).

---

### 2. **Servo Driver Modules**
Two PCA9685 chips are used as PWM generators to drive 16 of the 18 required servos (expandable). Each driver:
- Communicates with the BBB over I2C
- Outputs 8 independent PWM signals for servo motors
- Includes address pins (A0–A5) to support multiple drivers on the same I2C bus
- Contains decoupling capacitors and pull-up resistors for signal stability

---

### 3. **Ultrasonic Sensor (HC-SR04)**
The HC-SR04 sensor is used to detect obstacles and measure distance:
- Powered by +5V with decoupling capacitors (10uF and 100nF)
- Connected to BBB via:
  - TRIG pin (output from BBB)
  - ECHO pin (input to BBB, with level shifting for 3.3V compatibility)
- The data is used to decide turning or stopping actions

---

### 4. **IMU Sensor (MPU6050)**
- Provides orientation and acceleration feedback
- Connected via I2C (shared with other peripherals)
- Powered by +5V with local decoupling capacitors

---

### 5. **Power Supply Circuit**
- Accepts external battery input (+BATT) and generates regulated +5V and +3.3V
- Features buck converter (TPS54331DR) with appropriate filter networks
- Provides stable voltage rails to all system modules

---

### 6. **Level Shifting Circuit**
- Used to safely interface 5V logic (from HC-SR04 ECHO) with 3.3V logic of the BBB
- Implemented using MOSFET-based bidirectional level shifters
- Ensures reliable operation and protection of BBB GPIO pins

---

### 7. **Thermal Fan Driver**
- Controlled via PWM from BBB (PWM1A)
- Includes MOSFET switching and resistive control for speed regulation
- Provides thermal management for the electronics chassis if needed

---

### 8. **Servo Connectors Board**
- Groups and distributes servo outputs from the PCA9685 modules
- Adds additional bypass capacitors for local noise suppression
- Organized with labeled connectors for easy assembly

---

## Files and Schematics
The full KiCad schematic set includes:
- `Overview Schematic`
- `BBBlack.kicad_sch`
- `ServoDrivers.kicad_sch`
- `HC-SR04.kicad_sch`
- `IMU.kicad_sch`
- `PowerSupply.kicad_sch`
- `LevelShifting.kicad_sch`
- `ThermalFan.kicad_sch`
- `ConnectorServo.kicad_sch`

---

## Key Features
- 18 DOF hexapod locomotion with coordinated servo control
- Real-time obstacle detection using HC-SR04
- Modular design with clean separation of power, control, and sensing
- Compatible with Linux-based development on BeagleBone Black

---