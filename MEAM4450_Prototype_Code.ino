#include <Servo.h>

//PIN ASSIGNMENT
#define TOPL_PIN 1                //top-left LDR
#define TOPR_PIN 0                //top-right LDR
#define BOTL_PIN 3                //bottom-left LDR
#define BOTR_PIN 2                //bottom-right LDR
#define SERVO_UD1_PIN 4           //left up_down servo
#define SERVO_UD2_PIN 5           //right up_down servo
#define SERVO_LR_PIN 6            //left_right servo
#define PANEL1_PIN A4
#define PANEL2_PIN A5

int topl, topr, botl, botr;
int updown_pos, leftright_pos;
const int threshold_value = 20;   //measurement sensitivity
const int servo_speed = 2;

//declare two servos
Servo servo_updown1;
Servo servo_updown2;
Servo servo_leftright;

void setup()
{
  updown_pos = 90;
  leftright_pos = 90;
  topl = 0;
  topr = 0;
  botl = 0;
  botr = 0;
  
  Serial.begin(9600);                                 //open serial port, set baud rate to 9600 bps
  Serial.println("CLEARDATA");                        //clear all existing data
  Serial.println("LABEL,t,voltage,current,power");    //define column headings (PLX-DAQ command)
  
  servo_updown1.attach(SERVO_UD1_PIN);                //servos for up-down movement
  servo_updown2.attach(SERVO_UD2_PIN);
  servo_leftright.attach(SERVO_LR_PIN);               //servo for left-right movement
}

void loop()
{
  float volt = analogRead(PANEL1_PIN)*5.0/1023;
  float voltage = 2*volt;           //Volt=(R1/R1+R2)*Voltage / R1=R2=10 Ohms  => voltage=2*volt
  float current = voltage/20;       //I=voltage/(R1+R2)
  float power = voltage*current;
  Serial.print("DATA,TIME,");       //PLX-DAQ command
  Serial.print(voltage);            //send the voltage to serial port
  Serial.print(",");
  Serial.print(current);            //send the current to serial port
  Serial.print(",");
  Serial.println(power);            //send the power to serial port

  delay(50); // Wait for 50 milliseconds
  automaticsolartracker();
}

void automaticsolartracker(){
  //capture analog values of each LDR
  topl = analogRead(TOPL_PIN);
  topr = analogRead(TOPR_PIN);
  botl = analogRead(BOTL_PIN);
  botr = analogRead(BOTR_PIN);

  //calculate averages
  int avgtop = (topl + topr) / 2;
  int avgbot = (botl + botr) / 2;
  int avgleft = (topl + botl) / 2;
  int avgright = (topr + botr) / 2;
  
  //calculate differences
  int diffelev = avgtop - avgbot;           //up-down difference
  int diffazi = avgright - avgleft;         //left-right difference
  //Serial.println(diffazi);

  //up-down movement of solar tracker
  if (abs(diffelev) >= threshold_value){  //change position only if light difference exceeds threshold_value
    if (diffelev > 0)
      updown_pos = updown_pos + servo_speed > 180 ? 180 : updown_pos + servo_speed;
    else
      updown_pos = updown_pos - servo_speed < 0 ? 0 : updown_pos - servo_speed;
  }
  
  //left-right movement of solar tracker
  if (abs(diffazi) >= threshold_value) {    //change position only if light difference exceeds threshold_value
    if (diffazi > 0)
      leftright_pos = leftright_pos + servo_speed > 180 ? 180 : leftright_pos + servo_speed;
    else
      leftright_pos = leftright_pos - servo_speed < 0 ? 0 : leftright_pos - servo_speed;
  }
  servo_updown1.write(updown_pos);
  servo_updown2.write(180 - updown_pos);
  servo_leftright.write(leftright_pos);
}
