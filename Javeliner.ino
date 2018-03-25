/*
 * OK that's how it's going to work
 * 
 * when new message is received (hopefully containing coordinates)
 * The coordinates are fed to program and car drives
 * when the car STOPS
 * the serial buffer is flushed, and the wait for a new set of coordinates is on!
 */




#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <HMC5883L.h>
#include <TinyGPS++.h>

//serial for gps module

static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device is Serial2


//vars for gps navigation

float lon = 0; //current longitude in decimal degrees (Y) in decimal degrees
float lat = 0; //current latitude in decimal degrees (X) in decimal degrees

float latT = 0;//42.35611343383789062500; //Target latitude (X) in decimal degrees - will be 23 characters long when received from other device
float lonT = 0;//-71.09935760498046875000; //Target longitude (Y) in decimal degrees

float Dlon = 0; //difference between lon and lonT in decimal degrees
float Dlat = 0; //difference between lat and latT in decimal degrees

float headingT = 0; //heading to target from current position in decimal degrees
float headingError = 0; // difference between heading T and heading in decimal degrees

float distance = 0; //distance in KM between robot and endpoint

float lastDistance = 0; // the distace in the last run. will be used to make sure there are no funky things going on




//things for magnetometer ----- X axis is the direction of the robot-----


/* Setup compass */
HMC5883L compass;

float angle = 0; //angle reading from sensor

//vars for servo

Servo servo;
#define servoPin 6
int servoCenter = 91; // check!
int servoAngle = 0;

//vars for ESC

#define escPin 7

Servo esc;

//stuff for hc-12 communication

#include <SoftwareSerial.h>
SoftwareSerial HC12(10, 11); // HC-12 TX Pin, HC-12 RX Pin
String inBuffer = "";    // string to hold input
char inChar ;
long outInt = 0;
float outFloat;

//stuff for program flow control

boolean gotCoordinates = false;


//================SETUP=========================

void setup()  
{
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Welcome to Javeliner!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  Serial2.begin(GPSBaud);

// Serial port to HC12
    HC12.begin(9600);               








  // Initialize Initialize HMC5883L
  Serial.println("Initialize HMC5883L");
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }



  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0);

  delay(1000);








  
   Serial.println("ending setup");

  
  //attach servo
  pinMode(servoPin, OUTPUT);
  servo.attach(servoPin);
  servo.write(servoCenter);
  delay(1000);
  servo.write(servoCenter+30);
  delay(1000);
  servo.write(servoCenter-30);
  delay(1000);
  servo.write(servoCenter); 

  //attach esc
   delay(1000);
   esc.attach(escPin);

   escStart();

   Serial.println("ending setup");
}

//================END SETUP==============================

//function to start esc

 void escStart(){
  
  for (int pos = 90; pos <= 100; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    esc.write(pos);              // tell servo to go to position in variable 'pos'
    delay(150);                       // waits 15ms for the servo to reach the position
  }
  //run just a little
  esc.write(120);
  delay(500);
  esc.write(90);
  }

//function to calculate distance between coordinates

float calc_dist(float flat1, float flon1, float flat2, float flon2)
{
float dist_calc=0;
float dist_calc2=0;
float diflat=0;
float diflon=0;

//variables to radians
flat1 *= (2*PI)/360 ;
flon1 *= (2*PI)/360 ;
flat2 *= (2*PI)/360 ;
flon2 *= (2*PI)/360 ;

//I've to spplit all the calculation in several steps. If i try to do it in a single line the arduino will explode.
diflat=radians(flat2-flat1);
flat1=radians(flat1);
flat2=radians(flat2);
diflon=radians((flon2)-(flon1));

dist_calc = (sin(diflat/2.0)*sin(diflat/2.0));
dist_calc2= cos(flat1);
dist_calc2*=cos(flat2);
dist_calc2*=sin(diflon/2.0);
dist_calc2*=sin(diflon/2.0);
dist_calc +=dist_calc2;

dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));

dist_calc*=6371000.0; //Converting to meters
//Serial.println(dist_calc);
return dist_calc;
}




uint32_t timer = millis();


//===============LOOP==================================

void loop(){                     // run over and over again



// wait for message

    Serial.println("Waiting for coordinates...");
    //HC12.flush();
    gotCoordinates = false;
    
    inBuffer = "";    // erase the char and string to hold input
    bool start = false;

 while (HC12.available()==0) {}             // wait until HC-12 has data

  // Read serial input:
 
     while (HC12.available()) {             // If HC-12 has data
    inChar = HC12.read();          // Store each icoming byte from HC-12
    delay(5);
    //Serial.println(inChar);
    // Reads the data between the start "s" and end marker "e"
    if (start == true) {
      if (inChar != 'e') {
        inBuffer += char(inChar);    // Add each byte to inBuffer string variable
      }
      else {
        start = false;

  // Converts the string into integer
    outInt = inBuffer.toInt();

     outFloat = outInt/10000000.0;
    //Serial.print(outFloat,8);
    
    //Serial.println("");
        
      }
    }
    // Checks whether the received message statrs with the start marker "s"
    else if ( inChar == 'a') {
      start = true; // If true start reading the message
    }
  }

 //the first number we get is the latitude
 latT = outFloat;





 //erase important variables
 //HC12.flush();
   inBuffer = "";    // erase the char and string to hold input
    start = false;
   // outFloat = 0;
   // outInt= 0;
 





 //now should be another message with the longitude (don't forget to add delay beween sending lat and lon)
 //delay(300);






 
 while (HC12.available()==0) {}             // wait until HC-12 has data

  // Read serial input:
 
     while (HC12.available()) {             // If HC-12 has data
    inChar = HC12.read();          // Store each icoming byte from HC-12
    delay(5);
    // Reads the data between the start "s" and end marker "e"
    if (start == true) {
      if (inChar != 'e') {
        inBuffer += char(inChar);    // Add each byte to inBuffer string variable

      }
      else {
        start = false;

  // Converts the string into integer
    outInt = inBuffer.toInt();

     outFloat = outInt/10000000.0;
    //Serial.print(outFloat,8);
    
    //Serial.println("");
        
      }
    }
    // Checks whether the received message statrs with the start marker "s"
    else if ( inChar == 'b') {
      start = true; // If true start reading the message
    }
  }

 //the second number we get is the longitude
 lonT = outFloat;

//Display the coordinated we got:

Serial.print("Remote Lat: ");Serial.println(latT,8);
Serial.print("Remote Lon: ");Serial.println(lonT,8);

    //make sure that our coordinates are not 0.000
    if (lonT==latT){
      delay(500);
      Serial.println("SHIT it's the same!");
      gotCoordinates = false;
  
     }
     else{gotCoordinates = true; esc.attach(escPin);
       //run just a little
  esc.write(120);
  delay(500);
  esc.write(90);
}







while (gotCoordinates == true){// after we got our message = start navigating
  Serial.println("Moving to navigation");


Serial.println("Waiting for GPS");

//if we got all the way here, and the gps is still not locked, go back to beginning
//if (Serial2.available() == 0){gotCoordinates = false; esc.write(90);Serial.println("noserial2");}

  while (Serial2.available() > 0)
    if (gps.encode(Serial2.read()))
         if (gps.location.isValid()){

      //store vars
      lat = gps.location.lat();
      lon = gps.location.lng();

      //display the location in decimal degrees
      
      Serial.print("Location (in degrees): LAT:");
      Serial.print(lat, 20);
      Serial.print(", LON:"); 
      Serial.println(lon, 20);

      //find the heading REQUIRED to get to the target

      Dlon = lonT - lon;
      Dlat = latT - lat;

      headingT = (atan2(Dlon, Dlat))*(180/PI); // heading in decimal degrees

      //debugging - heading to target
      //Serial.println("Degrees Target heading: "); Serial.println(headingT);

      
      //correct for negative degrees

      if (headingT < 0) {headingT += 360;}
      else if(headingT >360){headingT -= 360;}


      Serial.print("Heading to target: "); Serial.println(headingT);
     
      //get magnetometer data:
     
       Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  //in boston = -14° 38' 
  float declinationAngle = (-14 + (38.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  float headingDegrees = heading * 180/M_PI;

 // if (headingDegrees > 180){ headingDegrees = (headingDegrees -360)

    //display data
    Serial.print("Heading Degrees"); Serial.println(headingDegrees); //0-360 degrees


    //calculate error in heading (-360) to 360 degrees
    headingError = headingT - headingDegrees;

    //make headingError be in the range of -180 to 180
    
    if (headingError < -180){headingError += 360;}
    else if(headingError > 180){headingError -= 360;}

    //display error

    Serial.print("Heading Error:"); Serial.println(headingError);

    //write the current numbers that are stored in var distance into lastDistance
    lastDistance = distance;

    
    //calculate distance to target IN DEGREES!!!!!!! function turns to radians
    
    distance = calc_dist(lat,lon,latT,lonT);
    Serial.print("Distance to target: "); Serial.println(distance);

    //print the difference in the last distance to the current distance, if too big we got something funky going on and can't be near target - -DEBUGGING
    Serial.print("Difference in distace from last one: "); Serial.println(lastDistance - distance);


    
    //steer in direction

    if (headingError > 160 || headingError < -160){// if the robot is oriented in the opposite direction
      
      //turn around and then calculate again


      servoAngle = 130;
      delay(500);
      
      }//endif
    else{//the robot is facing more or less the correct direction - drive there
      
      
      servoAngle = int(lround(servoCenter - headingError*0.7));

      if (servoAngle > 110){servoAngle = 110;}
      else if (servoAngle < 55){servoAngle = 55;}

      servo.write(servoAngle);

     
      }//endelse

      //check if we got to the target - near 5 meters of it and break out of while loop to check for more data

     if(distance < 0.08){esc.write(90); delay(2000); gotCoordinates = false; Serial.println("we are here");}//maybe add break
      

      if ( gotCoordinates== true){ esc.write(140);} // start driving-- make sure we have coordinates
      //debugging for servo
      Serial.print("SERVO:"); Serial.println(servoAngle);

    }//end if fix

  }//end while got coordinates
  }//endloop



