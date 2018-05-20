#include <Servo.h>

/* Servo control for AL5D arm */
 
/* Arm dimensions( mm ) */
#define BASE_HGT 88.75      //base height (mm)
#define HUMERUS 96.00      //shoulder-to-elbow "humerus bone" (mm)
#define ULNA 90.6        //elbow-to-wrist "ulna bone" (mm)
#define GRIPPER 96      //wrist-gripper length (mm)

 
#define ftl(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))  //float to long conversion
 
/* Servo names/numbers */
/* Base servo HS-485HB */
#define BAS_SERVO 0
/* Shoulder Servo HS-5745-MG */
#define SHL_SERVO 1
/* Elbow Servo HS-5745-MG */
#define ELB_SERVO 2
/* Wrist servo HS-645MG */
#define WRI_SERVO 3
/* Gripper servo HS-422 */
#define GRI_SERVO 5

Servo bas;
Servo shl;
Servo elb;
Servo wri;
Servo gri;


/*NEEED TO RESOLVE THESE PINS PROPERLY.....*/

//available: 3 4 5 7 8 9 11 
// order of pins from top to bottom is 8 11 4 9 7 5 3

int baspin = 8;
int shlpin = 12;
int elbpin = 4;
int wripin = 9;
int grippin = 7;
int speedpin = 5;
int turnpin = 3;
//int ledPin = 13;

float basValue = 1500;
float wriValue = 1500;
float elbValue = 1500;
float shlValue = 1500;
float griValue = 1500;
 
/* pre-calculations */
float hum_sq = HUMERUS*HUMERUS;
float uln_sq = ULNA*ULNA;
                     //ServoShield object
                     


float xaccel;
float yaccel;
float zaccel;
float armxpos;
float armypos;
float armzpos;

bool haveall3accels = false;
bool haveall3armpos = false;
bool gotvoicecommand = false;
String voicecommand = "";

const char onCommand[] = "on";
const char offCommand[] = "off";
const char rightturnCommand[] = "turn right";
const char lefttturnCommand[] = "turn left";

String rcvd = "";

String readString;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //pinMode(ledPin, OUTPUT);
  pinMode(speedpin,OUTPUT);
  pinMode(turnpin,OUTPUT);
  bas.attach(baspin);
  shl.attach(shlpin);
  elb.attach(elbpin);
  wri.attach(wripin);

  shl.writeMicroseconds(shlValue);
  bas.writeMicroseconds(basValue);
  //delay(2000);
  elb.writeMicroseconds(elbValue);
  //delay(2000);
  wri.writeMicroseconds(wriValue);
  
  gri.writeMicroseconds(griValue);

  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available()) {
    delay(10);  //small delay to allow input buffer to fill

    char c = Serial.read();  //gets one byte from serial buffer
    if (c == ',') {
      break;
    }  //breaks out of capture loop to print readstring
    readString += c; 
    
  } //makes the string readString  

  if (readString.charAt(0) == 'X')
    {
      gotvoicecommand = false;
      xaccel = readString.substring(1).toFloat();
    }
    else if (readString.charAt(0) == 'Y')
    {
      gotvoicecommand = false;
      yaccel = readString.substring(1).toFloat();
    }
    else if (readString.charAt(0) == 'Z')
    {
      gotvoicecommand = false;
      zaccel = readString.substring(1).toFloat();
      haveall3accels = true;
    }
    else if (readString.charAt(0) == 'V')
    {
      gotvoicecommand = true;
      voicecommand = readString.substring(1);
    }
    else if (readString.charAt(0) == 'B')
    {
      gotvoicecommand = false;
      armypos = readString.substring(1).toFloat();
      //Serial.print("ypos:" + String(armypos));
    }
    else if (readString.charAt(0) == 'L')
    {
      gotvoicecommand = false;
      armxpos = readString.substring(1).toFloat();
      //Serial.print(" xpos:" + String(armxpos));
    }
    else if (readString.charAt(0) == 'D')
    {
      gotvoicecommand = false;
      armzpos = readString.substring(1).toFloat();
      //Serial.println(" zpos:" + String(armzpos));
      haveall3armpos = true;
    }
    
    //Serial.println(readString); //prints string to serial port out

    readString=""; //clears variable for new input
    
    if (gotvoicecommand)
    {
      Serial.println("got voice command");
      //const char copy[voicecommand.length()];
      const char * copy = voicecommand.c_str();
      Serial.println(copy);
      //voicecommand.toCharArray(copy, voicecommand.length());
      if(!strcmp(onCommand, copy))
      {
        //Serial.println("inside on");
        analogWrite(speedpin,30);
      }
      else if(!strcmp(offCommand,copy))
      {
        analogWrite(speedpin,0);
      }
      else if(!strcmp(rightturnCommand,copy))
      {
        analogWrite(turnpin,240);
      }
      else if(!strcmp(lefttturnCommand,copy))
      {
        analogWrite(turnpin,140);
      }
      else
      {
        int toindex = voicecommand.indexOf("towards");
        int mvindex = voicecommand.indexOf("move", toindex);
        float xcoord = (float)voicecommand.substring(toindex+3, mvindex-1).toInt();
        voicecommand = voicecommand.substring(mvindex);
    
        toindex = voicecommand.indexOf("towards");
        mvindex = voicecommand.indexOf("move", toindex);
        float ycoord = (float)voicecommand.substring(toindex+3, mvindex-1).toInt();
        voicecommand = voicecommand.substring(mvindex);
    
        toindex = voicecommand.indexOf("towards");
        mvindex = voicecommand.indexOf("tilt", toindex);
        float zcoord = (float)voicecommand.substring(toindex+3, mvindex-1).toInt();
        //Serial.println(voicecommand);
        toindex = voicecommand.indexOf("by");
        float angle = (float)voicecommand.substring(toindex+3).toInt();  
        Serial.print("xcoord: " + String(xcoord) + " ycoord: " + String(ycoord) + " zcoord: " + String(zcoord));
        set_arm(xcoord, ycoord, zcoord, angle);      
      }
      gotvoicecommand = false;
    }
    else
    {
      if (haveall3accels)
      {
        //do turning:
        if ((xaccel < 10.0 && xaccel > 5.0) && (yaccel > -8.0 && yaccel < 8.0) && (zaccel > -2.8 && zaccel < 2.8))
        {
          analogWrite(speedpin, 30);
          if (yaccel > -1 && yaccel < 1)
          {
            analogWrite(turnpin, 191);
          }   
          else if (yaccel > 1)
          {
            //analogWrite(speedpin, 30);
            analogWrite(turnpin, (int) map(yaccel, 1, 6, 191, 245));
          }
          else
          {
            analogWrite(turnpin, (int) map(yaccel, -1, -6, 191, 130));
          }
        }
        else if (xaccel > 8.5 && zaccel > -3)
        {
          analogWrite(speedpin, 30);  // analogRead values go from 0 to 1023, analogWrite values from 0 to 255
        }
        //increase speed over a finite range
        else if (xaccel < 8.5 && zaccel > 0 && abs(yaccel) < 1)
        {
          analogWrite(speedpin, (int) map(xaccel, 8.49, 0, 30, 60));  // analogRead values go from 0 to 1023, analogWrite values from 0 to 255   
        }
        //stop driving
        else {
          analogWrite(speedpin, 0);    
        }

        //do turning:
        if ((xaccel < 10.0 && xaccel > 5.0) && (yaccel > -8.0 && yaccel < 8.0) && (zaccel > -2.8 && zaccel < 2.8))
        {
          if (yaccel > -1 && yaccel < 1)
          {
            analogWrite(turnpin, 191);
          }   
          else if (yaccel > 1)
          {
            //analogWrite(speedpin, 30);
            analogWrite(turnpin, (int) map(yaccel, 1, 6, 191, 245));
          }
          else
          {
            analogWrite(turnpin, (int) map(yaccel, -1, -6, 191, 130));
          }
        }
      }
      else if (haveall3armpos)
      {
        set_arm(armxpos, armypos, armzpos, 0);
      }
    }
    delay(50);      
}

void set_arm( float x, float y, float z, float grip_angle_d )
{
  float grip_angle_r = radians( grip_angle_d );    //grip angle in radians for use in calculations
  /* Base angle and radial distance from x,y coordinates */
  float bas_angle_r = atan2( x, y );
  float rdist = sqrt(( x * x ) + ( y * y ));
  /* rdist is y coordinate for the arm */
  y = rdist;
  /* Grip offsets calculated based on grip angle */
  float grip_off_z = ( sin( grip_angle_r )) * GRIPPER;
  float grip_off_y = ( cos( grip_angle_r )) * GRIPPER;
  /* Wrist position */
  float wrist_z = ( z - grip_off_z ) - BASE_HGT;
  float wrist_y = y - grip_off_y;
  /* Shoulder to wrist distance ( AKA sw ) */
  float s_w = ( wrist_z * wrist_z ) + ( wrist_y * wrist_y );
  float s_w_sqrt = sqrt( s_w );
  /* s_w angle to ground */
  //float a1 = atan2( wrist_y, wrist_z );
  float a1 = atan2( wrist_z, wrist_y );
  /* s_w angle to humerus */
  float a2 = acos((( hum_sq - uln_sq ) + s_w ) / ( 2 * HUMERUS * s_w_sqrt ));
  /* shoulder angle */
  float shl_angle_r = a1 + a2;
  float shl_angle_d = degrees( shl_angle_r );
  /* elbow angle */
  float elb_angle_r = acos(( hum_sq + uln_sq - s_w ) / ( 2 * HUMERUS * ULNA ));
  float elb_angle_d = degrees( elb_angle_r );
  float elb_angle_dn = -( 180.0 - elb_angle_d );
  /* wrist angle */
  float wri_angle_d = ( grip_angle_d - elb_angle_dn ) - shl_angle_d;
 
  /* Servo pulses */
  float bas_servopulse = (ftl(1500.0 - (( degrees( bas_angle_r )) * 10.55 )));
  float shl_servopulse = (ftl(1500.0 - (( shl_angle_d - 90) * 10.55 )));
  float elb_servopulse =(ftl(1500.0 + (( elb_angle_d - 90.0 ) * 10.55 )));
  float wri_servopulse = (ftl(1500 + ( wri_angle_d  * 10.55 )));
 
  /* Set servos */
  bas.writeMicroseconds(ftl( bas_servopulse ));
  wri.writeMicroseconds(ftl( wri_servopulse ));
  shl.writeMicroseconds(ftl( shl_servopulse ));
  elb.writeMicroseconds(ftl( elb_servopulse ));
 
}

void move_gradually(long basFinal, long wriFinal, long shlFinal, long elbFinal) {

  for (int i = basValue + (basFinal - basValue)/10; i <= basFinal ; i = i + (basFinal - basValue)/10) {
    bas.writeMicroseconds(i); 
  }

  for (int i = shlValue + (shlFinal - shlValue)/10; i <= shlFinal ; i = i + (shlFinal - shlValue)/10) {
    shl.writeMicroseconds(i); 
  }

  for (int i = elbValue + (elbFinal - elbValue)/10; i <= elbFinal ; i = i + (elbFinal - elbValue)/10) {
    elb.writeMicroseconds(i); 
  }

  for (int i = wriValue + (wriFinal - wriValue)/10; i <= wriFinal ; i = i + (wriFinal - wriValue)/10) {
    wri.writeMicroseconds(i); 
  }
}
