/* LLAP180 by Phil Young
My attempt at using the BMP180 with the LLAP protocol
Designed for use on the XinoRF and RFu-328 from Ciseco
However should work with any Arduino

Uses the Ciseco LLAPSerial Library
https://github.com/CisecoPlc/LLAPSerial
Uses the SFE_BMP180 Library
https://github.com/sparkfun/BMP180_Breakout
Based on a mash up of the SFE_BMP180 example sketch and the
LLAP_DHT22 example sketch
*/
/*

General instructions from SFE_BMP180 Example
https://www.sparkfun.com/products/11824

Like most pressure sensors, the BMP180 measures absolute pressure.
This is the actual ambient pressure seen by the device, which will
vary with both altitude and weather.

Before taking a pressure reading you must take a temparture reading.
This is done with startTemperature() and getTemperature().
The result is in degrees C.

Once you have a temperature reading, you can take a pressure reading.
This is done with startPressure() and getPressure().
The result is in millibar (mb) aka hectopascals (hPa).

If you'll be monitoring weather patterns, you will probably want to
remove the effects of altitude. This will produce readings that can
be compared to the published pressure readings from other locations.
To do this, use the sealevel() function. You will need to provide
the known altitude at which the pressure was measured.

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
// Do not remove the include below
#include "llap180.h"
// Your sketch must #include this library, and the Wire library.
// (Wire is a standard library included with Arduino.):
#include <LLAPSerial.h>
#include <SFE_BMP180.h>
#include <Wire.h>

// You will need to create an SFE_BMP180 object, here called "pressure":

SFE_BMP180 pressure;

#define ALTITUDE 0 // Altitude of SparkFun's HQ in Boulder, CO. in meters
#define DEVICEID "DR"	// this is the LLAP device ID

//Battery read
#define BATTERY_READ_INTERVAL 10
int batteryCountDown = BATTERY_READ_INTERVAL;

int readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
//  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
//  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
//  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return (int)result; // Vcc in millivolts
}

/////////////////////////////////////////////////////////
// SRF AT command handling
/////////////////////////////////////////////////////////

uint8_t setupSRF()	// set Sleep mode 2
{
  if (!enterCommandMode())	// if failed once then try again
	{
		if (!enterCommandMode()) return 1;
	}
  //if (!sendCommand("ATSD49E30")) return 2;	// 5 minutes
  //if (!sendCommand("ATSD4E20")) return 2;	// 20 seconds
 // if (!sendCommand("ATSD1388")) return 2;	// 5 seconds
 // if (!sendCommand("ATSD3E8")) return 2;	// 1 second
  	 if (!sendCommand("ATSDAFC8")) return 2; //45s
  if (!sendCommand("ATSM3")) return 3;
  if (!sendCommand("ATDN")) return 3;
  return 5;
}
//End of sleep mode

uint8_t enterCommandMode()
{
  delay(1200);
  Serial.print("+++");
  delay(500);
  while (Serial.available()) Serial.read();  // flush serial in
  delay(500);
  return checkOK(500);
}

uint8_t sendCommand(char* lpszCommand)
{
  Serial.print(lpszCommand);
  Serial.write('\r');
  return checkOK(100);
}

uint8_t checkOK(int timeout)
{
  uint32_t time = millis();
  while (millis() - time < timeout)
  {
    if (Serial.available() >= 3)
    {
      if (Serial.read() != 'O') continue;
      if (Serial.read() != 'K') continue;
      if (Serial.read() != '\r') continue;
      return 1;
    }
  }
  return 0;
}


void setup()
{
  	Serial.begin(115200);
  	LLAP.init(DEVICEID);
	pinMode(8,OUTPUT);		// switch on the radio
	digitalWrite(8,HIGH);
	pinMode(4,OUTPUT);		// switch on the radio
	digitalWrite(4,LOW);	// ensure the radio is not sleeping
	delay(400);				// allow the radio to startup
	  batteryCountDown = BATTERY_READ_INTERVAL;
	  // Wait for it to be initialised
	  delay(200);
	  // set up sleep mode 3 (low = awake)
	  uint8_t val;
	  while ((val = setupSRF()) != 5)
	  {
		  LLAP.sendInt("ERR",val); // Diagnostic
		  delay(5000);	// try again in 5 seconds
	  }

	  pinMode(9,OUTPUT);
	  digitalWrite(9,LOW);	// No voltage to the sensor

		LLAP.sendMessage(F("STARTED"));  // send the usual "started message
	}

void loop()
{
	  if( --batteryCountDown <= 0 ) {
		    int mV = readVcc();
		    LLAP.sendIntWithDP("B", mV, 3 );
		    batteryCountDown = BATTERY_READ_INTERVAL;
		    delay(20);
		  }
		digitalWrite(9,HIGH);
		delay(500);// provide voltage for the sensor
//
//  if (LLAP.bMsgReceived) {
//		Serial.print(F("msg:"));
//		Serial.println(LLAP.sMessage);
//		LLAP.bMsgReceived = false;	// if we do not clear the message flag then message processing will be blocked
//	}

  char status;
  double T,P,p0,a;
  char TempString[10]; //Array for conversion Double to String
  char PressureArray[10]; //Array for conversion Double to String

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
      		if (isnan(T) || isnan(P)) {
			LLAP.sendMessage(F("ERROR"));
		} else {
			//Converts double to array then string then adds to T
			dtostrf(T,5,2,TempString);
				String ST;
			ST = "T" + String(TempString);
			LLAP.sendMessage(ST);
		}

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
          if (isnan(T) || isnan(P)) {
			LLAP.sendMessage(F("ERROR"));
		} else {
			//Convert double to string
			dtostrf(P,7,2,PressureArray);
			String Pres;
			Pres = "P"+ String(PressureArray);
			LLAP.sendMessage(Pres);
		}

          // The pressure sensor returns abolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sealevel function and your current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
          // Result: p0 = sea-level compensated pressure in mb

          p0 = pressure.sealevel(P,ALTITUDE); // we're at 1655 meters (Boulder, CO)

          // On the other hand, if you want to determine your altitude from the pressure reading,
          // use the altitude function along with a baseline pressure (sea-level or other).
          // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
          // Result: a = altitude in m.

         // a = pressure.altitude(P,p0);
        }
        //Need to change these error messages at some point
      //  else Serial.println("error retrieving pressure measurement\n");
      }
    //  else Serial.println("error starting pressure measurement\n");
    }
   // else Serial.println("error retrieving temperature measurement\n");
  }
//  else Serial.println("error starting temperature measurement\n");

//  delay(30000);  // Pause for 30 seconds.
}

