/*
   Code Made By Jonathan Mathews
   This code is to meant to level an aircraft in flight.
   Using 2 MPU-6050s and a PID conroller the program will output repective error for pitch, roll, and yaw on The Serial Moniter.
   Each axis of rotation has a unique PID value and is mapped to specific servos to move the controll surfaces.
*/

//Libaries
#include <Wire.h>
#include <LiquidCrystal.h>
#include <Servo.h>
#include "SR04.h"

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(13, 12, 8, 9, 10, 11); //(rs, en, d4, d5, d6, d7)

//Servos
#define DEFAULT_PULSE_WIDTH  1500
Servo right_aileron , left_aileron , right_rudder, left_rudder, elevator, land_right, land_left;
//Specific Servo limits in MicroSeconds | Here for refrence mostly
int RA[] = {2050, 1460, 850};                              //Right Aileron    |Down, Level, Up|
int LA[] = {825, 1400, 1950};                              //Left Aileron     |Down, Level, Up|
int RR[] = {1000, 1550, 2100};                             //Right Rudder     |Out,  Level, In|
int LR[] = {2100, 1600, 1000};                             //Left Rudder      |Out,  Level, In|
int EV[] = {2000, 1430, 850};                              //Evevator         |Down, Level, Up|


//Ultrasonic sensor
#define TRIG_PIN 3                                        //Define Trig Pin
#define ECHO_PIN 2                                        //Define Echo Pin 
SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);                     //Declare Sensor
int distance;                                             //Int for Distance(cm)

//IMU Addresses
const int IMU1 = 0x68;                                    
const int IMU2 = 0x69;                                    //The Power Pin is routed to ADO instead of VCC to change address

//Variables for IMU
int gyro_x, gyro_y, gyro_z, gyro_x2, gyro_y2, gyro_z2;
long gyro_x_cal, gyro_y_cal, gyro_z_cal, gyro_x_cal2, gyro_y_cal2, gyro_z_cal2;
long acc_x, acc_y, acc_z, acc_x2, acc_y2, acc_z2, acc_total_vector;
float angle_pitch, angle_pitch2, angle_roll, angle_roll2, angle_yaw, angle_yaw2;
float avg_pitch, avg_roll, avg_yaw;

// Setup timers and temp variables
long loop_timer;
int temperature, temperature2;

// Display counter
int displaycount = 0;

//Pid Time
float elapsedTime, time, timePrev;                        //Variables for time control

// PID Constants
float desired_pitch = -5;                                //Desired Pitch  |Can be changed|
float desired_roll = 0;                                  //Desired Roll   |Can be changed|
float desired_yaw = 0;                                   //Desired Yaw    |Can be changed|


/*
 * The following variables are for the PID constants. Each axis of Rotation
 * has it's own PID loop essentally.
 */

//Pitch
float kp_pitch = 7;
float ki_pitch = 4;
float kd_pitch = 3;
float PID_p_pitch, PID_i_pitch, PID_d_pitch, PID_total_pitch;

//Roll
float kp_roll = 5;
float ki_roll = 3;
float kd_roll = 3;
float PID_p_roll, PID_i_roll, PID_d_roll, PID_total_roll;

//Yaw
float kp_yaw = 3;
float ki_yaw = 3;
float kd_yaw = 4;
float PID_p_yaw, PID_i_yaw, PID_d_yaw, PID_total_yaw;

//Variables For PID value caculation
float pitch_diference, pitch_previous_error, pitch_error, roll_diference, roll_previous_error, roll_error, yaw_diference, yaw_previous_error, yaw_error; 
int period = 60;                                         //Refresh rate period of the loop is 60ms

//Place holder values   |These are used when you don't know thw upper and lower bound of a servo|
int servo_upper_bound, servo_lower_bound;


void setup() {

  // LEDs
  pinMode(39, OUTPUT);                                   //Front Lights
  pinMode(41, OUTPUT);                                   //Middle Lights
  pinMode(43, OUTPUT);                                   //Back Lights

  digitalWrite(39, HIGH);                                //Turn Front Lights ON
  delay(1200);                                           //Wait 1.2 Seconds
  digitalWrite(41, HIGH);                                //Turn Middle Lights ON
  delay(1200);                                           //Wait 1.2 Seconds
  digitalWrite(43, HIGH);                                //Turn Back Lights ON
  delay(1200);                                           //Wait 1.2 Seconds

  //Declare Servo Pins |Can be changed|
  right_aileron .attach(26);
  left_aileron .attach(27);
  right_rudder.attach(28);
  left_rudder.attach(29);
  elevator.attach(24);
  land_left.attach(30);
  land_right.attach(31);

  land_left.writeMicroseconds(500);                      //Sets Landing Gear Up
  land_right.writeMicroseconds(500);                         


  //Start I2C
  Wire.begin();

  //LCD commands 
  lcd.begin(16, 2);                                      //Starts LCD
  lcd.clear();                                           //Clears LCD Display
  lcd.setCursor(0, 0);                                   //Printing on first row
  lcd.print("Calibrating");                              //Prints on Display
  lcd.setCursor(0, 1);                                   //Printing on second row

  setup_mpu_6050_registers                              //Setup the registers of the MPU-6050s

  for (int cal_int = 0; cal_int < 2000 ; cal_int ++) {                  //Run this code 2000 times To get average Offset
    if (cal_int % 125 == 0)lcd.print(".");                              //Prints a dot on the LCD every 125 values
    read_mpu_6050_data();
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;

    gyro_x_cal2 += gyro_x2;
    gyro_y_cal2 += gyro_y2;
    gyro_z_cal2 += gyro_z2;

    delay(3);                                                           
  }
  gyro_x_cal /= 2000;                                                   //Divide the gyro values by 2000 to get the avarage
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;

  gyro_x_cal2 /= 2000;
  gyro_y_cal2 /= 2000;
  gyro_z_cal2 /= 2000;

  // Start Serial Monitor
  Serial.begin(115200);

  // Init Timer
  loop_timer = micros();

  //Timer for PID
  time = millis();
}

void loop() {

  //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - Angle Calculation - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -//
  /*
   * The following are steps taken in order to turn the raw sensor values into usable information.
   */


  read_mpu_6050_data();                                                 //Read the raw acc and gyro data from the MPU-6050

  gyro_x -= gyro_x_cal;                                                 //Offset
  gyro_y -= gyro_y_cal;                                                 //Offset
  gyro_z -= gyro_z_cal;                                                 //Offset

  gyro_x2 -= gyro_x_cal2;                                               //Offset
  gyro_y2 -= gyro_y_cal2;                                               //Offset
  gyro_z2 -= gyro_z_cal2;                                               //Offset

  //Gyro angle calculations
  angle_pitch += gyro_x * 0.0000713;                                    //Pitch Angle
  angle_pitch2 += gyro_x2 * 0.0000713;                                  //Pitch Angle

  angle_roll += gyro_y * 0.0000713;                                     //Roll Angle
  angle_roll2 += gyro_y2 * 0.0000713;                                   //Roll Angle

  angle_yaw += gyro_z * 0.0000713;                                      //Yaw Angle
  angle_yaw2 += gyro_z2 * 0.0000713;                                    //Yaw Angle

  avg_pitch = ((angle_pitch + angle_pitch2) / 2);                       //Avg Pitch
  avg_roll = ((angle_roll + angle_roll2) / 2);                          //Avg Roll
  avg_yaw = ((angle_yaw + angle_yaw2) / 2);                             //Avg Yaw

                                                                        //Place IMUs level and not down the values that are seen. Use these values for manual calibration
  avg_pitch -= 0;                                                       //Manual Offset
  avg_roll -= 0;                                                        //Manual Offset
  avg_yaw -= 0;                                                         //Manual Offset

  //- - - - - - - - - - - - - - - - -  - - - - - - - - - - - - - - - - - - Print Information - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -//

  // Print to Serial Monitor
  Serial.print("| Pitch: "); Serial.print(avg_pitch);
  Serial.print(" | ROll: "); Serial.print(avg_roll);
  Serial.print(" | YAW: "); Serial.print(avg_yaw);
  Serial.print(" | Distance: "); Serial.println(distance);

  // Increment the display counter
  displaycount = displaycount + 1;
  avg_yaw -= 0;                                                         //Manual Offset
                                                                          
  if (displaycount > 50) {                                              //Values are printed with a gap inbetween so the LCD is not overwhelmed

    distance = sr04.Distance();                                         //Check to whether or not to deploy landing gear
    if (distance < 7) {                                                 //This placed in this if statment not everwhelm the arudunio. As The ultrasonic sensor is not that percise
         land_right.writeMicroseconds(2000);                            //we only need to check whether if we are near the ground. Checking for this contantly would be 
         land_left.writeMicroseconds(2000);                             //waste of processing power
    }
 
    lcd.clear();                                                        //Print Information on LCD
    lcd.setCursor(0, 0);
    lcd.print("P:"); lcd.print(avg_pitch); lcd.print("|R:"); lcd.print(avg_roll);
    lcd.setCursor(0, 1);
    lcd.print("Y:"); lcd.print(avg_yaw); lcd.print(" |D:"); lcd.print(distance);
    displaycount = 0;

  }
  //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  PID MATH!!! - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -//

  /*
    if (millis() > time + period)
    {
      time = millis();*/

  //All the Errors
  pitch_error = desired_pitch - avg_pitch;   //Serial.print(" | Pitch Error: "); Serial.print(pitch_error);        |Pitch Error Calulation and Uncomment to Serial Print|
  roll_error = desired_roll - avg_roll;      //Serial.print(" | Roll Error: "); Serial.print(roll_error);          |Pitch Error Calulation and Uncomment to Serial Print|
  yaw_error = desired_yaw - avg_yaw;         //Serial.print(" | Yaw Error: "); Serial.print(yaw_error);            |Pitch Error Calulation and Uncomment to Serial Print|


  //Pitch Correction - - - - - - - - - - - - - - - - - - - - - -
  PID_p_pitch = kp_pitch * pitch_error;                          //Proportional Value

  pitch_diference = (pitch_error - pitch_previous_error);        //Derivative Path
  if (-1.2 < (pitch_diference) && (pitch_diference) < 1.2) {     //Check for high frequancy noise
    PID_d_pitch += 0;}                                           //Blocking noise
  else {
    PID_d_pitch = kd_pitch * (pitch_diference / period);}        //Derivative Value                    

  if (700 <= PID_i_pitch) && (PID_i_pitch >= 1900)) {            //Integral Path 
    PID_i_pitch += 0;}                                           //Clamps intergal if saturating 
  else {
    PID_i_pitch = PID_i_pitch + ki_pitch * (pitch_error);}       //Integral Value

  PID_total_pitch = PID_p_pitch + PID_i_pitch + PID_d_pitch;     //Total PID value for Pitch
  pitch_previous_error = pitch_error;                            //Sets error to previous error

  float PID_elevator = map(PID_total_pitch,-60,60,2000,750);     //Maps PID_pitch value to Servo Limits
  elevator.writeMicroseconds(PID_elevator);                      //Tells Servo To move

  //Roll Correction - - - - - - - - - - - - - - - - - - - - - -
  PID_p_roll = kp_roll * roll_error;                                                   //Proportional Path

  roll_diference = (roll_error - roll_previous_error);                                 //Derivative Path

  if (-1.2 < (roll_diference) && (roll_diference) < 1.2) {                             //Blocking noise |Using the d/dx of roll to check for high frequancy noise|
    PID_d_pitch += 0;
    PID_d_roll += 0;
  }
  else {
    PID_d_roll = kd_roll * (roll_diference / period);
  }

  if (750 <= PID_i_roll && PID_i_roll >= 2200) {                                       //Intergal Vlaue |Checks for saturation|
    PID_i_pitch += 0;                                                                  //Clamps intergal path if system is saturating
  }
  else {
    PID_i_roll = PID_i_roll + ki_roll * (roll_error);                                  //Calucatles Integral Value
  }

  PID_total_roll = PID_p_roll + PID_i_roll + PID_d_roll;                               //Total PID value for Roll

  roll_previous_error = roll_error;                                                    //Sets error to previous error

  if (PID_total_roll < -65) {
    PID_total_pitch = -65;
  }
  if (PID_total_roll > 65) {
    PID_total_pitch = 65;
  }

  float PID_left_aileron = map(PID_total_roll, -65, 65,  2200, 725)                    //Maps PID_roll value to Servos Limits
  float PID_right_aileron = map(PID_total_roll, -65, 65,  2250, 750);                  


  //Yaw Correction - - - - - - - - - - - - - - - - - - - - - -
  PID_p_yaw = kp_yaw * yaw_error;                                                      //Proportional Value

  yaw_diference = (yaw_error - yaw_previous_error);                                    //Derivitive Value
  
  if (-1.2 < (yaw_diference) && (yaw_diference) < 1.2) {                               //Blocking noise |Using the d/dx of yaw to check for high frequancy noise|
    PID_d_yaw += 0;
  }
  else {
    PID_d_yaw = kd_yaw * (yaw_diference / period);
  }

  if (750 <= PID_i_yaw && PID_i_yaw >= 2400) {                                         //Intergal Vlaue |Checks for saturation|
    PID_i_yaw += 0;                                                                    //Clamps intergal path if system is saturating
  }
  else {
    PID_i_yaw = PID_i_yaw + ki_yaw * (yaw_error);                                      //Calucatles Integral Value
  }

  PID_total_yaw = PID_p_yaw + PID_i_yaw + PID_d_yaw;                                   //Total PID value for Roll

  if (PID_total_roll < -40) {
    PID_total_pitch = -40;
  }
  if (PID_total_roll > 40) {
    PID_total_pitch = 40;
  }

  float PID_left_rudder = map(PID_total_yaw, -40, 40,  2500, 825);                     //Maps PID_roll value to Servos Limits
  float PID_right_rudder = map(PID_total_yaw, -40, 40,  2500, 700);     


  //Moving Controll Surfaces
  elevator.writeMicroseconds(PID_elevator);
  right_aileron .writeMicroseconds(PID_right_aileron );
  left_aileron .writeMicroseconds(PID_left_aileron );
  left_rudder.writeMicroseconds(PID_left_rudder);
  right_rudder.writeMicroseconds(PID_right_rudder);

  }//End of Void Loop

void setup_mpu_6050_registers() {
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Accessing register 6B - Power Management(Sec.4.28)
  Wire.write(0x00);                                                    //SLEEP to 0 - Required
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);                                                    //Register 1C - Acccel ConFig(Sec.4.5)
  Wire.write(0x10);                                                    //Sets the accel to +/- 8g
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);                                                    //Accessing register 1B - Gyroscope ConFig(Sec. 4.4)
  Wire.write(0x08);                                                    //Sets the gyro to +/- 500deg./s
  Wire.endTransmission();


  //Activate the MPU-6050 TWO
  Wire.beginTransmission(0x69);                                        // Second IMU adrress is 0x69
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(0x69);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x69);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

}

void read_mpu_6050_data() {
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68, 14);                                          //Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);                                       //Retreive the Sensor values
  acc_x = Wire.read() << 8 | Wire.read();                              
  acc_y = Wire.read() << 8 | Wire.read();                      
  acc_z = Wire.read() << 8 | Wire.read();                          
  temperature = Wire.read() << 8 | Wire.read();                    
  gyro_x = Wire.read() << 8 | Wire.read();                            
  gyro_y = Wire.read() << 8 | Wire.read();                             
  gyro_z = Wire.read() << 8 | Wire.read();                           

  Wire.beginTransmission(0x69);                                        //Same thing for Second IMU
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x69, 14);
  while (Wire.available() < 14);
  acc_x2 = Wire.read() << 8 | Wire.read();
  acc_y2 = Wire.read() << 8 | Wire.read();
  acc_z2 = Wire.read() << 8 | Wire.read();
  temperature2 = Wire.read() << 8 | Wire.read();
  gyro_x2 = Wire.read() << 8 | Wire.read();
  gyro_y2 = Wire.read() << 8 | Wire.read();
  gyro_z2 = Wire.read() << 8 | Wire.read();

}
