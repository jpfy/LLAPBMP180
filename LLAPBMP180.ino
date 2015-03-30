

/* 
Phils first version of LLAPBMP sensor with rfu368
Using SFE_BMP180 library

Current issues are:
   1. Temp readings show .xx not xx.xx
   2. Barometric readings show xx.xx not xxxx
   3. LLAP not working properly
   
Impression is that varibles are currently in wrong class.
That I am not showing propper LLAP info


SFE_BMP180 library example sketch

If you want to measure altitude, you will need to know the pressure
at a baseline altitude. This can be average sealevel pressure, or
a previous pressure reading at your altitude, in which case
subsequent altitude readings will be + or - the initial baseline.
This is done with the altitude() function.

Hardware connections:

- (GND) to GND
+ (VDD) to 3.3V

(WARNING: do not connect + to 5V or the sensor will be damaged!)

You will also need to connect the I2C pins (SCL and SDA) to your
Arduino. The pins are different on different Arduinos:

Any Arduino pins labeled:  SDA  SCL
Uno, Redboard, Pro:        A4   A5
Mega2560, Due:             20   21
Leonardo:                   2    3

Leave the IO (VDDIO) pin unconnected. This pin is for connecting
the BMP180 to systems with lower logic levels such as 1.8V

Have fun! -Your friends at SparkFun.

The SFE_BMP180 library uses floating-point equations developed by the
Weather Station Data Logger project: http://wmrx00.sourceforge.net/

Our example code uses the "beerware" license. You can do anything
you like with this code. No really, anything. If you find it useful,
buy me a beer someday.

V10 Mike Grusin, SparkFun Electronics 10/24/2013
*/

// Your sketch must #include this library, and the Wire library.
// (Wire is a standard library included with Arduino.):
#include <LLAPSerial.h>
#include <SFE_BMP180.h>
#include <Wire.h>

// You will need to create an SFE_BMP180 object, here called "pressure":

SFE_BMP180 pressure;

#define ALTITUDE 0 // Altitude of SparkFun's HQ in Boulder, CO. in meters
#define DEVICEID "BM"	// this is the LLAP device ID
void setup()
{
  	Serial.begin(115200);
	pinMode(8,OUTPUT);		// switch on the radio
	digitalWrite(8,HIGH);
	pinMode(4,OUTPUT);		// switch on the radio
	digitalWrite(4,LOW);	// ensure the radio is not sleeping
	delay(1000);				// allow the radio to startup
	LLAP.init(DEVICEID);
  LLAP.sendMessage(F("STARTED"));
// Initialize the sensor (it is important to get calibration values stored on the device).

  if (pressure.begin())
  LLAP.sendMessage(F("WORKING"));
//   Serial.println("BMP180 init success");
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.
LLAP.sendMessage(F("ERROR"));
  //  Serial.println("BMP180 init fail\n\n");
    while(1); // Pause forever.
  }
}

void loop()
{
 
  if (LLAP.bMsgReceived) {
		Serial.print(F("msg:"));
		Serial.println(LLAP.sMessage); 
		LLAP.bMsgReceived = false;	// if we do not clear the message flag then message processing will be blocked
	}

  char status;
  double T,P,p0,a;

  // Loop here getting pressure readings every 10 seconds.

  // If you want sea-level-compensated pressure, as used in weather reports,
  // you will need to know the altitude at which your measurements are taken.
  // We're using a constant called ALTITUDE in this sketch:
  
 // Serial.println();
 // Serial.print("provided altitude: ");
 // Serial.print(ALTITUDE,0);
 // Serial.print(" meters, ");
 // Serial.print(ALTITUDE*3.28084,0);
 // Serial.println(" feet");
  
  // If you want to measure altitude, and not pressure, you will instead need
  // to provide a known baseline pressure. This is shown at the end of the sketch.

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Print out the measurement:
      	/*	if (isnan(T) || isnan(P)) {
			LLAP.sendMessage(F("ERROR"));
		} else {
  */
  LLAP.sendIntWithDP("Tmp",T,2);
			//delay(100);
		//	LLAP.sendIntWithDP("",t,1);
		}
      //Serial.print("temperature: ");
      //Serial.print(T,2);
      //Serial.print(" deg C, ");
      //Serial.print((9.0/5.0)*T+32.0,2);
      //Serial.println(" deg F");
      
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
        /*  if (isnan(T) || isnan(P)) {
			LLAP.sendMessage(F("ERROR"));
		} else {
  */
			LLAP.sendIntWithDP("Pre",P,2);
			//delay(100);
		//	LLAP.sendIntWithDP("",t,1);
		}
          // Print out the measurement:
          //Serial.print("absolute pressure: ");
          //Serial.print(P,2);
          //Serial.print(" mb, ");
          //Serial.print(P*0.0295333727,2);
          //Serial.println(" inHg");

          // The pressure sensor returns abolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sealevel function and your current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
          // Result: p0 = sea-level compensated pressure in mb

          p0 = pressure.sealevel(P,ALTITUDE); // we're at 1655 meters (Boulder, CO)
         // Serial.print("relative (sea-level) pressure: ");
         // Serial.print(p0,2);
         // Serial.print(" mb, ");
         // Serial.print(p0*0.0295333727,2);
         // Serial.println(" inHg");

          // On the other hand, if you want to determine your altitude from the pressure reading,
          // use the altitude function along with a baseline pressure (sea-level or other).
          // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
          // Result: a = altitude in m.

         a = pressure.altitude(P,p0);
         // Serial.print("computed altitude: ");
         // Serial.print(a,0);
         // Serial.print(" meters, ");
         // Serial.print(a*3.28084,0);
         // Serial.println(" feet");
//        }
//        else Serial.println("error retrieving pressure measurement\n");
//      }
//      else Serial.println("error starting pressure measurement\n");
//    }
//    else Serial.println("error retrieving temperature measurement\n");
//  }
//  else Serial.println("error starting temperature measurement\n");

  delay(30000);  // Pause for 30 seconds.
}}}
