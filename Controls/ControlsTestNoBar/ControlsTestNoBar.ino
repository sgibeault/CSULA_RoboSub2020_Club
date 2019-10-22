/* Robosub 2018-2019 Stabilization Code 
 *  Current Revision: 060819
 *  Joseph Daniel Iorio
 *  Sidra Gibeault
 *  
 *  -- Revision log --
 *  040619: Joseph - PID gains tuned during 04/06/19 pool testing. Depth control implemented. 
 *  052219: Joseph - Integrated new IMU (Adafrtuit/Bosch BNO055)
 *  060119: Sidra - Added manual control code for dynamic ROV movement
 *  061419: Sidra - Fixed issue with barometer code
 *  
 */


#include <Wire.h>
#include <Servo.h>
#include "MS5837.h"
#include <Adafruit_BNO055.h>
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

int control = 0;

MS5837 sensor;

Servo thruster_frontright;
Servo thruster_frontleft;
Servo thruster_backright;
Servo thruster_backleft;
Servo thruster_sideright;
Servo thruster_sideleft;

float elapsedTime, time, timePrev;
float PIDY, PIDX, PIDZ, pwmFrontRight, pwmFrontLeft, pwmBackRight, pwmBackLeft, pwmSideLeft, pwmSideRight, errorY, errorX, errorZ, previous_errorY, previous_errorX, previous_errorZ, current_depth, PID_depth, depth_error, previous_depth_error;
float pid_pY=0;
float pid_iY=0;
float pid_dY=0;
float pid_pX=0;
float pid_iX=0;
float pid_dX=0;
float pid_pZ=0;
float pid_iZ=0;
float pid_dZ=0;
float pid_p_depth = 0;
float pid_d_depth = 0;
float pid_i_depth = 0;
float y_angle = 0;
float x_angle = 0;
float z_angle = 0;

/////////////////PID GAINS/////////////////
double kp=3.5;// 3.5 WORKED
double ki=0.0000005;// 0 WOKRED
double kd=4;// 4 WORKED
///////////////////////////////////////////////

double throttle=1500; //Motor throttle --> Linearized control about this nominal thruster setting (0 thrust since we don't want thrusters constantly running)
float desired_angle = 0; //Setpoint angle
float desired_depth = 1; //Setpoint depth


void setup() {
  
  Wire.begin(); //begin the wire comunication
  Serial.begin(9600);
  thruster_frontright.attach(5); //attatch the front to pin 3
  thruster_frontleft.attach(6);  //attatch the backthruster to pin 5
  thruster_backright.attach(7); //attach the right thruster to pin 6
  thruster_backleft.attach(9); //attach the left thruster to pin 9
  thruster_sideright.attach(10); // etc.
  thruster_sideleft.attach(11); // etc.
  
  time = millis(); //Time is in miliseconds
  thruster_frontright.writeMicroseconds(1500); 
  thruster_frontleft.writeMicroseconds(1500);
  thruster_backright.writeMicroseconds(1500); 
  thruster_backleft.writeMicroseconds(1500);
  //delay(5000); /* Start up delay */

  /* Initialise the IMU */
  
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);

  /* Set up Barometer */

//  sensor.init();
//  sensor.setModel(MS5837::MS5837_30BA);
//  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
   
}//end of setup void

void loop() {

    timePrev = time;  // the previous time is stored before the actual time read
    time = millis();  // actual time read
    elapsedTime = (time - timePrev) / 1000; 

/////////////////////////////I M U/////////////////////////////////////
    
    Wire.beginTransmission(0x28);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    y_angle = euler.y();
    x_angle = euler.z();
    z_angle = euler.x(); //Note: IMU interprets vertical axis as x-axis, but we'll refer to it as z-axis 

/*Calculate error between measured and desired angle*/

errorY =  y_angle - desired_angle;
errorX = x_angle - desired_angle;
errorZ = z_angle - desired_angle;


if (errorZ<-180){
  errorZ+=360;
}
else if (errorZ>180){
  errorZ-=360;
}
    
/* Calculate proportional PID term */

pid_pY = kp*errorY;
pid_pX = kp*errorX;
pid_pZ = kp*errorZ;

/* Calculate the integral PID term, only when error is +/- 10 degrees */

if(-10 <errorY <10)
{
  pid_iY = pid_iY+(ki*errorY);  
}

if(-10 <errorX <10)
{
  pid_iX = pid_iX+(ki*errorX);  
}

/*Calculate derivative PID term */

pid_dY = kd*((errorY - previous_errorY)/elapsedTime);
pid_dX = kd*((errorX - previous_errorX)/elapsedTime);
pid_dZ = kd*((errorZ - previous_errorZ)/elapsedTime);

/*Sum all PID terms */

PIDY = pid_pY + pid_dY;
PIDX = pid_pX + pid_dX;
PIDZ = pid_pZ + pid_dZ;

/*Place bounds on PID terms */

//if(PIDY < -100)
//{
//  PIDY=-100;
//}
//if(PIDY > 100)
//{
//  PIDY=100;
//}
//
//if(PIDX < -100)
//{
//  PIDX=-100;
//}
//if(PIDX > 100)
//{
//  PIDX=100;
//}

/*Calculate PWM signal --> Throttle + PID terms = Motor Mixing Algorithm */

/*
Layout for how the PID terms interact with front/back/left/right thrusters
pwmFront = throttle + PIDX;
pwmBack = throttle - PIDX;
pwmRight = throttle - PIDY;
pwmLeft = throttle + PIDY;
*/

/* Barometer  */

// Wire.beginTransmission(0x76); // begin comm with barometer
// sensor.read(); // get data from barometer
// current_depth = sensor.depth(); // create current depth variable
// depth_error = desired_depth - current_depth; // calculate the error between current depth and desired depth
//  //Serial.print("Depth: "); 
//  //Serial.print(sensor.depth()); 
//  //Serial.println(" m");

/* Barometer PID terms */

//pid_p_depth = kp*depth_error;
//
//
//if(-.01 <depth_error <.01)
//{
//  pid_i_depth = pid_i_depth+(ki*depth_error);
//}
//
//pid_d_depth = kd*((depth_error - previous_depth_error)/elapsedTime);


 
//PID_depth = pid_p_depth + pid_d_depth;

/* Motor Mixing Algorithm */

//Front Thrusters
//pwmFrontRight = throttle + PIDX - PIDY + PID_depth;
//pwmFrontLeft = throttle + PIDX + PIDY + PID_depth;

pwmFrontRight = throttle + PIDX + PIDY + PID_depth; 
pwmFrontLeft = throttle - PIDX + PIDY + PID_depth;

//Back Thrusters
//pwmBackRight = throttle - PIDX - PIDY + PID_depth;
//pwmBackLeft = throttle - PIDX + PIDY + PID_depth;

pwmBackRight = throttle + PIDX - PIDY + PID_depth;
pwmBackLeft = throttle - PIDX - PIDY + PID_depth;


//Side Thrusters


pwmSideRight = throttle;// + PIDZ;
pwmSideLeft = throttle;// - PIDZ;



/*Place bounds on PWM signal */


//Front
if(pwmFrontRight < 1525)
{
  pwmFrontRight= 1500;
}
if(pwmFrontRight > 1560)
{
  pwmFrontRight=1560;
}

if(pwmFrontLeft < 1525)
{
  pwmFrontLeft= 1500;
}
if(pwmFrontLeft > 1560)
{
  pwmFrontLeft=1560;
}
//Back
if(pwmBackRight < 1525)
{
  pwmBackRight= 1500;
}
if(pwmBackRight > 1560)
{
  pwmBackRight=1560;
}

if(pwmBackLeft < 1525)
{
  pwmBackLeft= 1500;
}
if(pwmBackLeft > 1575)
{
  pwmBackLeft=1560;
}
if(pwmSideRight > 1560)
{
  pwmSideRight=1560;
}
if(pwmSideRight < 1525)
{
  pwmSideRight=1500;
}
if(pwmSideLeft > 1560)
{
  pwmSideLeft=1560;
}
if(pwmSideLeft < 1525)
{
  pwmSideLeft=1500;
}

//////////////////////////////////////////////////////

// CASE STATEMENTS FOR MANUAL CONTROL

  if (Serial.available() > 0) {
                // read the incoming byte:
                control = Serial.read();

                // say what you got:
                if ((control >= 49 && control <=57) || (control >= 97 && control <= 119)){
                Serial.print("I received: ");
                Serial.println(control, DEC);
                }
                else{
                  Serial.println("Invalid key pressed.");
                }
        }

  if ((control <= 57 && control >= 49) || (control <= 119 && control >= 97)) {
     switch(control) {
        case 97: //a : turn left
          thruster_frontright.writeMicroseconds(pwmFrontRight); 
          thruster_frontleft.writeMicroseconds(pwmFrontLeft);
          thruster_backright.writeMicroseconds(pwmBackRight); 
          thruster_backleft.writeMicroseconds(pwmBackLeft);
          thruster_sideright.writeMicroseconds(pwmSideRight-100);
          thruster_sideleft.writeMicroseconds(pwmSideLeft+100);
          break;
        case 115: //s : go backwards
          thruster_frontright.writeMicroseconds(pwmFrontRight); 
          thruster_frontleft.writeMicroseconds(pwmFrontLeft);
          thruster_backright.writeMicroseconds(pwmBackRight); 
          thruster_backleft.writeMicroseconds(pwmBackLeft);
          thruster_sideright.writeMicroseconds(pwmSideRight+100);
          thruster_sideleft.writeMicroseconds(pwmSideLeft+100);
          break;
        case 100: //d : turn right
          thruster_frontright.writeMicroseconds(pwmFrontRight); 
          thruster_frontleft.writeMicroseconds(pwmFrontLeft);
          thruster_backright.writeMicroseconds(pwmBackRight); 
          thruster_backleft.writeMicroseconds(pwmBackLeft);
          thruster_sideright.writeMicroseconds(pwmSideRight+100);
          thruster_sideleft.writeMicroseconds(pwmSideLeft-100);
          break;
        case 119: //w : go forward
          thruster_frontright.writeMicroseconds(pwmFrontRight); 
          thruster_frontleft.writeMicroseconds(pwmFrontLeft);
          thruster_backright.writeMicroseconds(pwmBackRight); 
          thruster_backleft.writeMicroseconds(pwmBackLeft);
          thruster_sideright.writeMicroseconds(pwmSideRight-100);
          thruster_sideleft.writeMicroseconds(pwmSideLeft-100);
          break;
        case 101: //e : go up
          thruster_frontright.writeMicroseconds(pwmFrontRight-100); 
          thruster_frontleft.writeMicroseconds(pwmFrontLeft-100);
          thruster_backright.writeMicroseconds(pwmBackRight-100); 
          thruster_backleft.writeMicroseconds(pwmBackLeft-100);
          thruster_sideright.writeMicroseconds(pwmSideRight);
          thruster_sideleft.writeMicroseconds(pwmSideLeft);
          break;
        case 99: //c : submerge
          thruster_frontright.writeMicroseconds(pwmFrontRight+100); 
          thruster_frontleft.writeMicroseconds(pwmFrontLeft+100);
          thruster_backright.writeMicroseconds(pwmBackRight+100); 
          thruster_backleft.writeMicroseconds(pwmBackLeft+100);
          thruster_sideright.writeMicroseconds(pwmSideRight);
          thruster_sideleft.writeMicroseconds(pwmSideLeft);
          break;
        case 103: //g : the two right thrusters are on (roll)
          thruster_frontright.writeMicroseconds(pwmFrontRight+100); 
          thruster_frontleft.writeMicroseconds(pwmFrontLeft-100);
          thruster_backright.writeMicroseconds(pwmBackRight+100); 
          thruster_backleft.writeMicroseconds(pwmBackLeft-100);
          thruster_sideright.writeMicroseconds(pwmSideRight);
          thruster_sideleft.writeMicroseconds(pwmSideLeft);
          break;
        case 104: //h : the two left thrusters are on (roll)
          thruster_frontright.writeMicroseconds(pwmFrontRight-100); 
          thruster_frontleft.writeMicroseconds(pwmFrontLeft+100);
          thruster_backright.writeMicroseconds(pwmBackRight-100); 
          thruster_backleft.writeMicroseconds(pwmBackLeft+100);
          thruster_sideright.writeMicroseconds(pwmSideRight);
          thruster_sideleft.writeMicroseconds(pwmSideLeft);
          break;
        case 105: //i : the two front thrusters are on (pitch)
          thruster_frontright.writeMicroseconds(pwmFrontRight+100); 
          thruster_frontleft.writeMicroseconds(pwmFrontLeft+100);
          thruster_backright.writeMicroseconds(pwmBackRight-100); 
          thruster_backleft.writeMicroseconds(pwmBackLeft-100);
          thruster_sideright.writeMicroseconds(pwmSideRight);
          thruster_sideleft.writeMicroseconds(pwmSideLeft);
          break;
        case 106: //j : the two back thrusters are on (pitch)
          thruster_frontright.writeMicroseconds(pwmFrontRight-100); 
          thruster_frontleft.writeMicroseconds(pwmFrontLeft-100);
          thruster_backright.writeMicroseconds(pwmBackRight+100); 
          thruster_backleft.writeMicroseconds(pwmBackLeft+100);
          thruster_sideright.writeMicroseconds(pwmSideRight);
          thruster_sideleft.writeMicroseconds(pwmSideLeft);
          break;
        case 111: //o : shut off all motors
          thruster_frontright.writeMicroseconds(pwmFrontRight); 
          thruster_frontleft.writeMicroseconds(pwmFrontLeft);
          thruster_backright.writeMicroseconds(pwmBackRight); 
          thruster_backleft.writeMicroseconds(pwmBackLeft);
          thruster_sideright.writeMicroseconds(pwmSideRight);
          thruster_sideleft.writeMicroseconds(pwmSideLeft);
          break;
        default:
          thruster_frontright.writeMicroseconds(pwmFrontRight); 
          thruster_frontleft.writeMicroseconds(pwmFrontLeft);
          thruster_backright.writeMicroseconds(pwmBackRight); 
          thruster_backleft.writeMicroseconds(pwmBackLeft);
          thruster_sideright.writeMicroseconds(pwmSideRight);
          thruster_sideleft.writeMicroseconds(pwmSideLeft);
          break;
       }
  }

  else{

          thruster_frontright.writeMicroseconds(pwmFrontRight); 
          thruster_frontleft.writeMicroseconds(pwmFrontLeft);
          thruster_backright.writeMicroseconds(pwmBackRight); 
          thruster_backleft.writeMicroseconds(pwmBackLeft);
          //thruster_sideright.writeMicroseconds(pwmSideRight);
          //thruster_sideleft.writeMicroseconds(pwmSideLeft);
    
  }

//////////////////////////////////////////////////////


//Store errors for next loop

previous_errorY = errorY; 
previous_errorX = errorX;
previous_errorZ = errorZ;
previous_depth_error = depth_error;

//Serial.println(current_depth);

//Serial.print(z_angle);
//Serial.print(' ');
//Serial.print(x_angle);
//Serial.print(' ');
//Serial.println(y_angle);

}
