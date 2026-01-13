# Servo position 
   FRONT
 [LF]   [RF]
 [LB]   [RB]
   USB (Tail)


# MPU6050 connection 
MPU VCC → 3.3V or 5V
MPU GND → GND
MPU SDA → A4
MPU SCL → A5


# IR receivers connection
IR OUT → D12
IR VCC → 5V
IR GND → GND


# HC-05 connection
HC-05 TX → Nano RX (D0)
HC-05 RX → Nano TX (D1) 
HC-05 VCC → 5V
HC-05 GND → GND


# Servo connection 
LF_HIP  = 2,   // Left Front Hip (Pitch)
LF_KNEE = 3,   // Left Front Knee (Pitch)

RF_HIP  = 4,   // Right Front Hip
RF_KNEE = 5,   // Right Front Knee

LB_HIP  = 6,   // Left Back Hip
LB_KNEE = 7,   // Left Back Knee

RB_HIP  = 8,   // Right Back Hip
RB_KNEE = 9,   // Right Back Knee


# Servo Orientation 
Upper Joint: Hip Pitch
90° = leg vertical
120° = forward
60° = backward
Lower Joint: Knee Pitch
90° = straight
120° = bent
60° = extended


# Mechanical model 
Leg DOF:        2 (hip pitch + knee pitch)
Foot style:     Flat pad
Stance:         Rectangular
Dimensions:     40mm + 55mm
Height/clear:   ~95-100mm
Drive style:    Tank turn + trot
Controller:     Arduino Nano (USB tail)
Servos:         SG90 x 8
IMU:            MPU6050 (pitch-only correction)
Power:          2S → BEC 5V
Comms:          IR + HC05 + Serial fallback
