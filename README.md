Servo position 
   FRONT
 [LF]   [RF]
 [LB]   [RB]
   USB (Tail)

MPU6050 connection 
MPU VCC → 3.3V or 5V
MPU GND → GND
MPU SDA → A4
MPU SCL → A5

Servo connection
RF Upper  → D3
RF Lower  → D5

LF Upper  → D6
LF Lower  → D9

RB Upper  → D10
RB Lower  → D11

LB Upper  → D12
LB Lower  → D13

IR receivers connection
IR OUT → D12
IR VCC → 5V
IR GND → GND

HC-05 connection
HC-05 TX → Nano RX (D0)
HC-05 RX → Nano TX (D1) 
HC-05 VCC → 5V
HC-05 GND → GND

overall connection
          ┌──────────────────────────────┐
          │        ARDUINO NANO          │
          │                               │
   D0 <---┤ RX       TX ---> D1 (HC-05)   │
   D2     ┤                               
   D3 --->┤ RF Upper Servo                │
   D4     ┤
   D5 --->┤ RF Lower Servo
   D6 --->┤ LF Upper Servo
   D7     ┤ 
   D8 --->┤ LB Upper Servo (if IR on D12)
   D9 --->┤ LF Lower Servo
  D10 --->┤ RB Upper Servo
  D11 --->┤ RB Lower Servo
  D12 --->┤ IR Receiver OUT
  D13 --->┤ LB Lower Servo
          │                               │
   A4 ----┤ SDA (MPU6050)
   A5 ----┤ SCL (MPU6050)
          │                               │
   5V ----┤ +5 From BEC / USB
  GND ----┤ Common Ground
          └───────────────────────────────┘



| Leg | Joint | Servo Pin | PWM Timer    |
| --- | ----- | --------- | ------------ |
| RF  | Upper | D3        | PWM (Timer2) |
| RF  | Lower | D5        | PWM (Timer0) |
| LF  | Upper | D6        | PWM (Timer0) |
| LF  | Lower | D9        | PWM (Timer1) |
| RB  | Upper | D10       | PWM (Timer1) |
| RB  | Lower | D11       | PWM (Timer2) |
| LB  | Upper | D8        | digital      |
| LB  | Lower | D13       | digital      |

Servo Orientation 
Upper Joint: Hip Pitch
90° = leg vertical
120° = forward
60° = backward
Lower Joint: Knee Pitch
90° = straight
120° = bent
60° = extended
