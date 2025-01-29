
#include <MsTimer2.h>
//The speed PID control is realized by counting the speed code plate
#include <BalanceCar.h>
#include <KalmanFilter.h>
#include "Adeept_Distance.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// #include "MPU6050.h"
#include "Wire.h"

/**********************Instantiate an  object**********************/
MPU6050         mpu;                                    //Instantiate an MPU6050 object named mpu
BalanceCar      balancecar;
KalmanFilter    kalmanfilter;
Adeept_Distance Dist;
/*************************END**************************/

/***********************Pin definition***********************/
// The ultrasonic module controls the pins
#define TRIG            3
#define ECHO            5
// RGB Color lamp control pin
#define RLED            A0
#define GLED            A1
#define BLED            A2
// The buzzer controls the pin
#define BUZZER          11
// MPU6050 Gyroscope control pin
#define MPU_SCL         5
#define MPU_SDA         4
// TB6612 Chip control pins
#define TB6612_STBY     8
#define TB6612_PWMA     10
#define TB6612_PWMB     9
#define TB6612_AIN1     12
#define TB6612_AIN2     13
#define TB6612_BIN1     7
#define TB6612_BIN2     6
// Motor encoder controls pins
#define MOTOR1          2
#define MOTOR2          4
/*************************END**************************/

/***********************Enumeration variable***********************/
enum COLOR
{
    RED = 0,        // red
    GREEN,          // green
    BLUE,           // blue
    YELLOW,         // yellow
    PURPLE,         // purple
    CYAN,           // cyan
    WHITE,          // white
    ALL_OFF         // off(black)
};
/*************************END**************************/

/***********************Variable definitions***********************/
byte TX_package[4] = {0xA5, 0, 0, 0x5A};                    // Packet header(0xA5) + original data (n*byte) + inspection(1byte) + Package the tail(0x5A)
byte RX_package[11] = {0};
int Serialcount = 0;
char x_axis = 0;                                            // Store variables along the X-axis
char y_axis = 0;                                            // Store variables on the Y axis
byte klaxon = 0;                                            // The default storage rate is1 ~ 255
byte S_Button = 0;                                          // Store a clockwise rotation variable
byte N_Button = 0;                                          // Store counterclockwise rotation variables
byte mode1_Button = 0;
byte mode2_Button = 0;
byte mode3_Button = 0;
byte mode1_var = 0;
byte mode2_var = 0;
byte mode3_var = 0;


// Ultrasonic detection of distance variables
int UT_distance = 0;
int detTime = 0; 

// Pulse calculation
int lz = 0;
int rz = 0;
int rpluse = 0;
int lpluse = 0;
int sumam;

// Kalman_Filte
float Q_angle = 0.001, Q_gyro = 0.005;                      // Angular data confidence, angular velocity data confidence
float R_angle = 0.5 , C_0 = 1;
float timeChange = 5;                                       // Filter sampling interval in milliseconds
float dt = timeChange * 0.001;                              // Note: Dt is the filter sampling time

// Angle data
float Q;
float Angle_ax;                                             // The Angle of tilt calculated by acceleration
float Angle_ay;
float K1 = 0.05;                                            // The weight of the accelerometer
float angle0 = -0.17;                                       // Angle of mechanical balance
int slong;

//          p:20      i:0.0     d:0.58
double kp = 23, ki = 0.0, kd = 0.48;                        // Parameters that you need to modify
//          p:5.5            i:0.1098           d:0.0
double kp_speed = 5.52, ki_speed = 0.1098, kd_speed = 0.0;  // Parameters that you need to modify
//          p:10            i:0          d:0.1
double kp_turn = 10, ki_turn = 0, kd_turn = 0.09;           // Rotary PID setting

int16_t ax, ay, az, gx, gy, gz;

int front = 0;                                              // Forward variables
int back = 0;                                               // Back variables
int turnl = 0;                                              // Turn left sign
int turnr = 0;                                              // Turn right
int spinl = 0;                                              // Rotate the left flag
int spinr = 0;                                              // Rotate the flag right

// Steering PID parameters
double setp0 = 0, dpwm = 0, dl = 0;                         // Angle balance, PWM poor, dead zone,PWM1,PWM2

// Turn and rotate parameters
int turncount = 0;                                          // Calculate the steering intervention time
float turnoutput = 0;

double Setpoint;                                            // Angle DIP set point, input, output
double Setpoints, Outputs = 0;                              // speed DIP set point, input, output

int speedcc = 0;
volatile long count_right = 0;                              // The volatile LON type is used to ensure that the external interrupt pulse meter values are valid when used in other functions
volatile long count_left = 0;
/*************************END**************************/

/*********************************************************
Function name: Pin_Init()
Function Initialize pin high/low level
Function parameters: None
The function returns: none
*********************************************************/
void Timer2Isr()
{
    sei();                                                  // Enable global variables
    countpluse();                                           // Pulse superposition subfunction
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);           // IIC obtains MPU6050 six-axis data ax ay az gx gy gz
    kalmanfilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);                                   //Get Angle and Kaman filter
    angleout();                                             // Angle loop PD control
    speedcc++;
    if (speedcc >= 8)                                       // 40 ms into the speed loop control
    {
        Outputs = balancecar.speedpiout(kp_speed, ki_speed, kd_speed, front, back, setp0);
        speedcc = 0;
    }
    turncount++;
    if (turncount > 4)                                      // 40 ms into the rotation control
    {
        turnoutput = balancecar.turnspin(turnl, turnr, spinl, spinr, kp_turn, kd_turn, kalmanfilter.Gyro_z);                              //Rotate the subfunction
        turncount = 0;
    }
    balancecar.posture++;
    balancecar.pwma(Outputs, turnoutput, kalmanfilter.angle, kalmanfilter.angle6, turnl, turnr, spinl, spinr, front, back, kalmanfilter.accelz, TB6612_AIN1, TB6612_AIN2, TB6612_BIN1, TB6612_BIN2, TB6612_PWMA, TB6612_PWMB);                            //小车总PWM输出
    detTime++;
    if(detTime >= 4)                                        // 40 ms an ultrasound measurement
    {
        detTime = 0;
        UT_distance = Dist.getDistanceCentimeter();
    }
}


ISR(PCINT2_vect)
{
    count_right ++;
}   //Right speed dial count
void Code_left() 
{
  count_left ++;
}   //Left speed gauge count

void setup()
{
    Pin_Config();                                           // Module pin configuration
    Pin_Init();                                             // Module pin initialization
    Dist.begin(ECHO, TRIG);                                 // Add the ultrasonic module
    Wire.begin();                                           // Join the I2C bus sequence
    Serial.begin(9600);                                     // Initialize the baud rate of the serial port to 9600
    mpu.initialize();                                       // Initialize the MPU6050
    delay(1500);                                            // Wait for the system to stabilize
    balancecar.pwm1 = 0;
    balancecar.pwm2 = 0;
    //5ms timed interrupt Settings use timer2  
    MsTimer2::set(5, Timer2Isr);
    MsTimer2::start();
}

void loop()
{
    attachInterrupt(0, Code_left, CHANGE);                  // Enable external interrupt 0               
    attachPinChangeInterrupt(MOTOR2);                       // Pin D4 is interrupted externally
    TX_Information(UT_distance);                            // Send ultrasonic data
    RX_Information();                                       // Receive Bluetooth data

    if(mode1_var == 1)
    {
        mode1();
    }
    else if(mode2_var == 1)
    {
        mode2();
    }
    else if(mode3_var == 1) 
    {
        mode3();
    }
}


void angleout()
{
    balancecar.angleoutput = kp * (kalmanfilter.angle + angle0) + kd * kalmanfilter.Gyro_x;//PD Angle loop control
}

void countpluse()
{
    lz = count_left;
    rz = count_right;

    count_left = 0;
    count_right = 0;

    lpluse = lz;
    rpluse = rz;

    if ((balancecar.pwm1 < 0) && (balancecar.pwm2 < 0))                     //When the moving direction of the trolley is judged to be backward (PWM motor voltage is negative), the pulse number is negative
    {
        rpluse = -rpluse;
        lpluse = -lpluse;
    }
    else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 > 0))                //The number of pulses is negative when the moving direction of the trolley is judged to be forward (I.E. the voltage of the PWM motor is positive)
    {
        rpluse = rpluse;
        lpluse = lpluse;
    }
    else if ((balancecar.pwm1 < 0) && (balancecar.pwm2 > 0))                //The number of pulses is negative when the moving direction of the trolley is judged to be forward (I.E. the voltage of the PWM motor is positive)
    {
        rpluse = rpluse;
        lpluse = -lpluse;
    }
    else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 < 0))                //The number of left rotation and right pulse is negative and the number of left pulse is positive
    {
        rpluse = -rpluse;
        lpluse = lpluse;
    }

    // Bring up the judgment
    balancecar.stopr += rpluse;
    balancecar.stopl += lpluse;

    // Every 5ms when the interrupt enters, the pulse number is superimposed
    balancecar.pulseright += rpluse;
    balancecar.pulseleft += lpluse;
    sumam = (balancecar.pulseright + balancecar.pulseleft) * 4;
}

/*********************************************************
Function name: mode1()
Function Function: Control the car direction and speed through Bluetooth
Function parameters: None
The function returns: none
*********************************************************/
void mode1()
{
    RGB(GREEN);
    if(klaxon > 0)                                              // speaker switch
    {
        digitalWrite(BUZZER, HIGH);
    }
    else
    {
        digitalWrite(BUZZER, LOW);   
    }

    if(N_Button > 0)                                            // Rotate counterclockwise
    {
        spinr = 1;
    }
    else if(S_Button > 0)                                       // Rotate clockwise
    {
        spinl = 1;
    }
    else if(x_axis >= -30 && x_axis <= 30 && y_axis > 30)       // Forward motion
    {
        ResetCarState();
        back = -30;
    }
    else if (x_axis >= -30 && x_axis <= 30 && y_axis < -30)     // Move backward
    {
        ResetCarState();
        front = 30;
    }
    else if (y_axis >= -30 && y_axis <= 30 && x_axis < -30)     // right
    {
        turnr = 1;
    }
    else if (y_axis >= -30 && y_axis <= 30 && x_axis > 30)      // left
    {
        turnl = 1;
    }
    else                                                        // stop
    {
        RGB(RED);
        ResetCarState();
    }
}

/*********************************************************
Mode2 ()
Function Function: Obstacle avoidance mode
Function parameters: None
The function returns: none
*********************************************************/
void mode2()
{
    if(UT_distance < 8)
    {
        ResetCarState();
        front = 15;
        RGB(RED);
    }
    else if(UT_distance < 15)
    {
        RGB(BLUE);
        turnr = 1;
        delay(200);
    }
    else
    {
        RGB(GREEN);
        ResetCarState();
        back = -15;
    }  
}

/*********************************************************
Mode3 ()
Function Function: follow mode
Function parameters: None
The function returns: none
*********************************************************/
void mode3()
{
    if(UT_distance >= 20 && UT_distance < 35)
    {
        ResetCarState();
        back = -15;
        RGB(GREEN);
    }
    else if(UT_distance < 20)
    {
        ResetCarState();
        front = 15;
        RGB(BLUE);
    }
    else
    {
        RGB(RED);
        ResetCarState();
    }  
}



void ResetCarState()
{
    turnl = 0; 
    turnr = 0;  
    front = 0; 
    back = 0; 
    spinl = 0; 
    spinr = 0; 
    turnoutput = 0;
}

/*********************************************************
Function name: RX_Information()
Function Function: Receives data packets through Bluetooth
Function parameters: None
The function returns: none
*********************************************************/
void RX_Information(void)
{
    if(Serial.available() > 0)
    {
        delay(1);                                           // delay 1 ms
        if(Serial.readBytes(RX_package, 11))
        {
            if (RX_package[0] == 0xA5 && RX_package[10] == 0x5A)     // 只验证了包头与包尾，暂未验证检验码
            {
                Serialcount = 0;
                x_axis = RX_package[1];                     // The X-axis value
                y_axis = RX_package[2];                     // The Y-axis value
                klaxon = RX_package[3];                     // the horn
                N_Button = RX_package[4];                   // Rotate counterclockwise
                S_Button = RX_package[5];                   // Rotate clockwise
                // mode1_Button = RX_package[6];
                // mode2_Button = RX_package[7];
                // mode3_Button = RX_package[8];
                if(RX_package[6] > 0)
                {
                    ResetCarState();
                    RGB(GREEN);
                    mode1_var = 1;
                    mode2_var = 0;
                    mode3_var = 0;
                }
                else if(RX_package[7] > 0)
                {
                    ResetCarState();
                    RGB(BLUE);
                    mode1_var = 0;
                    mode2_var = 1;
                    mode3_var = 0;
                }
                else if (RX_package[8] > 0)
                {
                    ResetCarState();
                    RGB(PURPLE);
                    mode1_var = 0;
                    mode2_var = 0;
                    mode3_var = 1;
                }
                
            }
            else
            {
                Serialcount++;
                return;
            }
        }
    }
    else
    {
        Serialcount++;
        if(Serialcount > 300)
        {
            klaxon = 0;
            x_axis = 0;
            y_axis = 0;
            N_Button = 0;
            S_Button = 0;
        }
    }
}

/*********************************************************
Function name: TX_Information()
Function Function: Sends data packets through Bluetooth
Function argument: dat, the data to be sent
The function returns: none
*********************************************************/
void TX_Information(byte dat)
{
    if(dat>127) dat = 127;
    TX_package[1] = dat;
    TX_package[2] = TX_package[1];                          // Check the sum                             
    Serial.write(TX_package, 4);                            // Send the packet
}

/*********************************************************
Function name: Pin_Init()
Function Initialize pin high/low level
Function parameters: None
The function returns: none
*********************************************************/
void Pin_Init()
{   
    digitalWrite(BUZZER, LOW);                              // The buzzer controls the pin output low level
    
    digitalWrite(TB6612_STBY, HIGH);                        // TB6612 enable control pin output high level
    digitalWrite(TB6612_PWMA, LOW);                         // TB6612 PWMA control pin output low level
    digitalWrite(TB6612_PWMB, LOW);                         // 
    digitalWrite(TB6612_AIN1, LOW);                         // 
    digitalWrite(TB6612_AIN2, HIGH);                        // 
    digitalWrite(TB6612_BIN1, HIGH);                        // 
    digitalWrite(TB6612_BIN2, LOW);                         //
}

/*********************************************************
Function name: attachPinChangeInterrupt()
The D4 pin is set as an external interrupt
Function parameters: pin, 4
The function returns: none
*********************************************************/
void attachPinChangeInterrupt(int pin)
{
    pinMode(pin, INPUT_PULLUP);
    cli();
    PCMSK2 |= bit(PCINT20);
    PCIFR |= bit(PCIF2);
    PCICR |= bit(PCIE2); 
    sei();
}

/*********************************************************
Function name: Pin_Config()
The pin I/O function configures pin I/O mode
Function parameters: None
The function returns: none
*********************************************************/
void Pin_Config()
{
    pinMode(TRIG, OUTPUT);                                  // Ultrasonic Trig control pin configuration output
    pinMode(ECHO, INPUT);                                   // Ultrasonic Echo control pin configuration input

    pinMode(RLED, OUTPUT);                                  // RGB color lights red control pin configuration output
    pinMode(GLED, OUTPUT);                                  // RGB color light green control pin configuration output
    pinMode(BLED, OUTPUT);                                  // RGB color light blue control pin configuration output
    
    pinMode(BUZZER, OUTPUT);                                // Buzzer control pin configuration output
    
    pinMode(TB6612_STBY, OUTPUT);                           // TB6612 Enable control pin configuration output
    pinMode(TB6612_PWMA, OUTPUT);                           // TB6612 PWMA control pin configuration output
    pinMode(TB6612_PWMB, OUTPUT);                           // TB6612 PWMB controls pin configuration output
    pinMode(TB6612_AIN1, OUTPUT);                           // 
    pinMode(TB6612_AIN2, OUTPUT);                           // 
    pinMode(TB6612_BIN1, OUTPUT);                           // 
    pinMode(TB6612_BIN2, OUTPUT);                           // 
    
    pinMode(MOTOR1, INPUT);                                 // Code motor 1 control pin configuration input
    pinMode(MOTOR2, INPUT);                                 // Code motor 2 control pin configuration input
}

/*********************************************************
Function name: RGB()
Function function: control RGB output various colors
Function parameters: color, various color reference enumeration@color
The function returns: none
*********************************************************/
void RGB(enum COLOR color)
{   
    switch (color)
    {
    case RED:
        digitalWrite(RLED, LOW);
        digitalWrite(GLED, HIGH);
        digitalWrite(BLED, HIGH);
        break;
    case GREEN:
        digitalWrite(RLED, HIGH);
        digitalWrite(GLED, LOW);
        digitalWrite(BLED, HIGH);
        break;
    case BLUE:
        digitalWrite(RLED, HIGH);
        digitalWrite(GLED, HIGH);
        digitalWrite(BLED, LOW);
        break;
    case YELLOW:
        digitalWrite(RLED, LOW);
        digitalWrite(GLED, LOW);
        digitalWrite(BLED, HIGH);
        break;
    case PURPLE:
        digitalWrite(RLED, LOW);
        digitalWrite(GLED, HIGH);
        digitalWrite(BLED, LOW);
        break;
    case CYAN:
        digitalWrite(RLED, HIGH);
        digitalWrite(GLED, LOW);
        digitalWrite(BLED, LOW);
        break;
    case WHITE:
        digitalWrite(RLED, LOW);
        digitalWrite(GLED, LOW);
        digitalWrite(BLED, LOW);
        break;
    default:
        digitalWrite(RLED, HIGH);
        digitalWrite(GLED, HIGH);
        digitalWrite(BLED, HIGH);
        break;
    }
}
