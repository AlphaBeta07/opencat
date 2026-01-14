# Servo position 
   FRONT  
 [LF]   [RF]  
 [LB]   [RB]  
   USB (Tail)  

# Servos Connection
FL Hip  → D2  
FL Knee → D3  
FR Hip  → D4  
FR Knee → D5  
BL Hip  → D6  
BL Knee → D7  
BR Hip  → D8  
BR Knee → D9  
  
# MPU6050 Conenction
SDA → A4  
SCL → A5  
VCC → 5V  
GND → GND  
  
# HC-05 Connection
TX → D0 (Nano RX)  
RX → D1 (Nano TX via divider)  
VCC → 5V  
GND → GND  
  
# Initial Position before attaching servo horn
![Initial Position](initial_pos.png)

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
Foot style:     round pad  
Stance:         Rectangular  
Dimensions:     40mm + 55mm  
Height/clear:   ~95-100mm  
Drive style:    Tank turn + trot  
Controller:     Arduino Nano (USB tail)  
Servos:         SG90 x 8  
IMU:            MPU6050 (pitch-only correction)  
Power:          2S → BEC 5V / power bank  
Comms:          HC05 + Serial fallback  
