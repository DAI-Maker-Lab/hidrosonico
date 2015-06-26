/*	Hidrosónico is a stream gauge using a MaxSonar HRXL MB7369 sonar 
rangefinder, a Seeeduino Stalker v3 Arduino-compatible microcontroller platform, 
and an Adafruit FONA 800 GSM module. It reads the distance between the sensor 
(specifically from where the housing meets the threading) and the water's 
surface directly below and sends this data at specified intervals to cloud-based 
storage, SMS, and/or email as determined by the user. It was designed to aid in 
collection of hydrological data and for flood early warning in developing 
countries. This version integrates a rain gauge at the request of the recipient. 

The code provided sends data to data.sparkfun.com as the cloud service but it 
could be easily adapted to any other provider. Please note the places in the 
code that are marked for user-specific input.

This is very much a beta unit and we will be making changes as a result of the 
data and experiences we are collecting from the two pilot installations in 
Honduras (at Chinda and Corquín).

This code uses snippets of the Adafruit example code for its FONA library 
(https://github.com/adafruit/Adafruit_FONA_Library) and snippets from the 
Sparkfun example weather station code (https://github.com/sparkfun/
Weather_Shield) for the rain gauge, both provided under open source licenses or 
as public domain. 

The remainder of this code is released under the MIT License. You are free to use 
and change this code as you like, provided you retain attribution to Development
Alternatives, Inc. Non-code portions of the project are made available under
the Creative Commons-Attribution (CC BY) license.

If you use it, please let us know (robert_ryan-silva[at]dai.com), and if you 
improve it, please share! */

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
//	These variables must be customized for each unit.

const int HEIGHT = 491;		//	Height of top of octagonal gasket, in cm

#define SPARKFUN_PUBLIC_KEY "XXXXXXXXXXXXXXXXXXXX"
#define SPARKFUN_PRIVATE_KEY "XXXXXXXXXXXXXXXXXXXX"

/*	We use pulse.to to distribute SMS messages without having to reprogram the
unit every time someone wants to be added or dropped from the SMS distribution
list. Pulse.to is not a great provider for this in lots of ways, so don't take
that as a recommendation. */
#define MOBILE_GATEWAY "XXXXXXXXX"

/*	Variables for sending emails must also be customized -- search for "SMTP
server". If you use pulse.to for your SMS forwarder, you'll need to change those
values, too. */

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

//	Include programming libraries into the sketch

/*	SRAM is at a real premium in this sketch. If you make any changes, you may
want to use MemoryFree.h to check free SRAM, particularly if the sketch is
behaving oddly. */

//#include <MemoryFree.h>

#include <Adafruit_FONA.h>      //  For Adafruit FONA module
#include <SoftwareSerial.h>		//  For serial communication with FONA
#include <Wire.h>				//  I2C library for communication with RTC
#include <Sleep_n0m1.h>			//  Sleep library
#include <DS1337.h>				//  For the Stalker's real-time clock (RTC)
#include <Time.h>				//  For time functions

//	Define pins for various functions
#define rtcInt 0		//  Interrupt number associated with rtcPin
#define rainInt 1		//  Interrupt number associated with rainPin
#define rtcPin 2		//  Interrupt pin from RTC
#define rainPin 3   	//  Set rain gauge pin
#define pwPin 7		    //  Pulse width data pin from sonar
#define FONA_RST 8 	    //  FONA reset pin
#define FONA_PS 9	    //  Power status pin
#define FONA_KEY 10	    //  On/off pin
#define FONA_TX 12	    //  Receive from FONA
#define FONA_RX 13	    //  Send to FONA

//	Define variables that the sketch will use
int streamHeight;

byte arraysize = 7; 		//  Array for median sonar values (must be odd)
int rangevalue[] = { 0, 0, 0, 0, 0, 0, 0 };		//	Initial array for sonar

byte currentHour;		//  We will create variables to hold integer values
byte currentMinute;		//  for the time and date from the RTC
byte currentDay;
byte currentMonth;
int currentYear;

unsigned int action = 0;	//	The upload number
int failedUploads = 0;		//	We keep track of failed uploads for debugging

boolean rainySeason;		//  A flag to indicate rainy or dry season
boolean dataSent;

volatile unsigned long raintime, rainlast, raininterval;
volatile unsigned long dailyRain = 0;  	//  Rain in um so far today in local time
volatile unsigned long hourlyRain = 0;	//  Rain in um over the last hour

//  Configure the software serial interface to FONA
SoftwareSerial fonaSerial = SoftwareSerial(FONA_TX, FONA_RX);
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

DS1337 RTC;				//  Create the DS1337 object
Sleep sleep;			//  Create the sleep object



static void rainIRQ()
{
	raintime = millis();  				//  Set raintime to current time
	raininterval = raintime - rainlast;	//  Calculate interval between events

	//  Ignore switch-bounce glitches less than 10ms after initial edge
	if (raininterval > 10)
	{
		/* For propellerhead reasons, we want to avoid using floating point
		numbers if we can. So we will increment the rain counts in micrometers
		and then later divide them by 1,000 to get mm.*/
		dailyRain += 279;      	//  Each dump is 2.794mm of rain
		hourlyRain += 279;  	//  Increase this hour's amount of rain
		rainlast = raintime; 	//  Set up for next event
		Serial.println(F("Rain event."));
	}
}



static void rtcIRQ()
{
	//	This is a blank interrupt -- we're just using it to wake up.
}



void setup()
{
	Serial.begin(115200);		//  Begin the Serial interface

	Serial.println(F("Setup begins."));

	ADCSRA = 0;  			//  Disable analog-digital converter for power savings
	Wire.begin();			//  Begin the I2C interface
	RTC.begin();			//  Begin the RTC

	Serial.println(F("Hidrosonico online."));

	pinMode(pwPin, INPUT);	        //  Sonar pulse width pin
	pinMode(FONA_KEY, OUTPUT);      //  The Key pin from the FONA
	pinMode(FONA_PS, INPUT);        //  The Power Status pin from the FONA
	pinMode(rainPin, INPUT_PULLUP); //  The signal pin from the rain gauge

	delay(1000);		        //  I expect I put this here for a reason

	//	Try up to five times to boot the FONA module.

	byte attempts = 0;
	boolean booted = false;
	while(attempts < 5 && booted == false)
	{
		attempts++;
		if(bootFona() == false)		//  Start the FONA
		{
			booted = false;
		}
		else
		{
			booted = true;
		}
	}

	if(booted == false)
	{
		Serial.println(F("FONA boot failed. Check connections."));
	}

	ATcommand(F("ATZ"));			//  Revert to last saved settings

	/*	We will set the RTC's time from the mobile network. This capability is
	disabled by factory default. We will check to see if it is enabled; if not,
	we will enable it, save that as the new default, and restart the FONA so
	the new setting can take effect.*/

	fona.enableGPRS(true);

	clockSet();	//  Set the RTC
	stopFona();	//  Turn off the FONA to save power

	DateTime now = RTC.now(); 			//  Get the current date/time
	Serial.print(F("The current RTC time is "));
	Serial.print(now.hour(), DEC);
	Serial.print(F(":"));
	Serial.println(now.minute(), DEC);

	RTC.enableInterrupts(EveryMinute);	 //  RTC will interrupt every minute
	RTC.clearINTStatus();                //  Clear any outstanding interrupts

	attachInterrupt(rainInt, rainIRQ, FALLING);
	attachInterrupt(rtcInt, rtcIRQ, FALLING);
	interrupts();					//  Activate interrupts

	//Serial.print(F("Free SRAM: "));
	//Serial.println(freeMemory());

	Serial.println(F("Setup complete."));
}



void loop()
{
	DateTime now = RTC.now(); 			//  Get the current date/time

	Serial.print(now.hour());			//  For debugging, print the time
	Serial.print(F(":"));
	Serial.println(now.minute());

	Serial.flush();			//  Give Serial a chance to catch up before sleep

	//	If this is a fifteenth minute, take and upload a reading.

	if(now.minute() % 15 == 0)
	{
		currentHour = (byte)now.hour();		//  Cast the time and date values
		currentMinute = (byte)now.minute();	//  as integers so we can manipulate
		currentDay = (byte)now.date();		//  them more easily later.
		currentMonth = (byte)now.month();
		currentYear = (int)now.year();

		//	If the current month is May-December, raise the rainySeason flag
		if(currentMonth >= 5)
		{
			rainySeason = true;
		}
		else
		{
			rainySeason = false;
		}

		action++;

		bootFona();			//  Start the Fona

		takeReading();	//  Take a reading

		Serial.print(F("streamHeight: "));
		Serial.println(streamHeight);

		//	We will attempt to upload data to the cloud. If the attempt fails,
		//	we will restart the Fona and try again for up to three attempts.

		byte attempts = 0;		//	Reset the counter
		dataSent = false;		//	Reset the flag

		while(dataSent == false && attempts <= 2)
		{
			attempts++;		//	Increment attempt counter

			if(cloudUpload() == false)	//	If upload appears to fail...
			{
				Serial.println(F("Upload failed. Restarting Fona."));
				stopFona();			//  ...shut the FONA off.
				delay(5000);        //  Give the FONA a moment
				bootFona();         //  Reboot.
				dataSent = false;	//  Ensure flag is down
				failedUploads++;	//  Increment fail counter
			}

			else
			{
				dataSent = true;
			}
		}

		if(mailCall() == true)	//	If it's time to send mail...
		{
			sendMail();			//	...send it.
		}

		if(SMScall() == true)	//	If it's time to send an SMS...
		{
			sendSMS();			//	...send it.
		}

		if(currentMinute == 0)	//	At the top of the hour, after upload...
		{
			hourlyRain = 0;		//	...zero out the hour's rain total
		}

		if(currentHour == 0 && currentMinute == 0)	//	At midnight, after upload...
		{
			dailyRain = 0;      //	...zero out the day's rain total

			/* We're going to reset the time again, because the RTC loses a lot
			more time than the datasheet claims. This is risky, though; not all
			mobile networks' time information is universally reliable. */
			clockSet();
		}

		stopFona();		          //	Shut down the FONA to save power

		//Serial.print(F("Free SRAM: "));
		//Serial.println(freeMemory());

		Serial.println(F("Data send routine complete."));
	}

	RTC.clearINTStatus();				//	Clear the last loop's RTC interrupt
	sleep.pwrDownMode(); 				//	Set sleep mode to Power Down
	Serial.print(F("Sleeping..."));
	Serial.flush();						//	Let Serial catch up
	sleep.sleepInterrupt(0, FALLING);	//	Sleep

	//	When the RTC interrupt triggers, the sketch will resume the loop here
	//	and so restart back at the beginning of the loop.
}



boolean bootFona()
{
	// Power up the FONA if it needs it
	if (digitalRead(FONA_PS) == LOW)
	{
		Serial.print(F("Powering FONA on..."));
		while (digitalRead(FONA_PS) == LOW)
		{
			digitalWrite(FONA_KEY, LOW);
			delay(500);
		}
		digitalWrite(FONA_KEY, HIGH);
		Serial.println(F(" done."));
		delay(500);
	}

	// Start the FONA
	Serial.print(F("Initializing FONA..."));
	fonaSerial.begin(4800);

	// See if the FONA is responding
	if (! fona.begin(fonaSerial))
	{
		Serial.println(F("Couldn't find FONA"));
		return false;
		while (1);
	}
	else
	{
		Serial.println(F("FONA is OK"));

		//	Wait for a valid network
		Serial.print(F("Waiting for GSM network..."));
		while (1)
		{
			uint8_t network_status = fona.getNetworkStatus();
			if (network_status == 1 || network_status == 5) break;
			delay(250);
		}
		Serial.println(F(" done."));

		delay(5000);

		uint8_t rssi = fona.getRSSI();

		if (rssi > 5)
		{
			if (!fona.enableGPRS(true))
			{
				if(fona.GPRSstate() == 1)
				{
					Serial.println(F("GPRS is on."));
				}
				else
				{
					Serial.println(F("Failed to turn GPRS on."));
				}
			}
		}
		else
		{
			Serial.println(F("Can't transmit, network signal strength is poor."));
		}
		return true;
	}
}



void stopFona()
{
	delay(5000);				//	Shorter delays yield unpredictable results

	if (!fona.enableGPRS(false))
	{
		if(fona.GPRSstate() == 1)
		{
			Serial.println(F("Failed to turn GPRS off."));
		}
		else
		{
			Serial.println(F("GPRS is off."));
		}
	}

	delay(500);

	Serial.println(F("Turning off Fona "));
	while(digitalRead(FONA_PS) == HIGH)
	{
		digitalWrite(FONA_KEY, LOW);
	}
	digitalWrite(FONA_KEY, HIGH);

	delay(4000);
}



boolean cloudUpload()
{
	char url[210];
	uint16_t statuscode;
	int16_t length;
	unsigned int vbat = 0;		//  Battery voltage

	fona.getBattVoltage(&vbat);		//	Read the battery voltage

	//	Here's where we'll convert the rain counts to mm for uploading.

	//	We're tracking raw rain dumps for calibration confirmation purposes.
	int rawDumps = (dailyRain / 279);
	int dailyRainmm = (round(dailyRain / 1000));	//	Convert um to mm
	int hourlyRainmm = (round(hourlyRain / 1000));

	//	Generate the URL for transmission to Sparkfun

	/* We'll add a trailing zero for minutes less than ten so 6:00 doesn't
	show up as 6:0 */
	if(currentMinute < 10)
	{
		sprintf (url, "http://data.sparkfun.com/input/%s?private_key=%s&accion=%d&altura=%d&bateria=%d&hora=%d:0%d&lluvia_diaria=%d&lluvia_hora=%d&raw_dumps=%d&upload_failures=%d",
		         SPARKFUN_PUBLIC_KEY, SPARKFUN_PRIVATE_KEY, action, streamHeight, vbat, currentHour, currentMinute, dailyRainmm, hourlyRainmm, rawDumps, failedUploads);
	}
	else
	{
		sprintf (url, "http://data.sparkfun.com/input/%s?private_key=%s&accion=%d&altura=%d&bateria=%d&hora=%d:%d&lluvia_diaria=%d&lluvia_hora=%d&raw_dumps=%d&upload_failures=%d",
		         SPARKFUN_PUBLIC_KEY, SPARKFUN_PRIVATE_KEY, action, streamHeight, vbat, currentHour, currentMinute, dailyRainmm, hourlyRainmm, rawDumps, failedUploads);
	}

	Serial.print(F("Sending: "));
	Serial.println(url);

	if (fona.HTTP_GET_start(url, &statuscode, (uint16_t *)&length))
	{
		while (length > 0)
		{
			while (fona.available())
			{
				char c = fona.read();

				// Serial.write is too slow, we'll write directly to Serial register!
				loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
				UDR0 = c;

				length--;
				if (! length) break;
			}
		}
		fona.HTTP_GET_end();

		if(statuscode == 200)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		Serial.println(F("Failed to send GPRS data."));
		return false;
	}

	Serial.println(F("cloudUpload complete."));
}



void takeReading()
{
	long pulse;				//  The pulse return length for the sonar
	long pulseMode;			//	Mode of pulse readings

	Serial.print(F("Taking readings..."));

	//	Fill the array with readings
	for(int readingCount = 0; readingCount < arraysize; readingCount++)
	{
		/*
			The MaxSonar measures the distance of objects by bouncing a
			superaudible pulse off the object and measuring the time of flight
			(TOF) between the emission of the pulse and its return. For the
			MB7369, 1 microsecond TOF = 1mm of distance. For more see:
			http://www.maxbotix.com/articles/085-pt5.htm#codes
		*/

		Serial.print(F("Reading "));
		Serial.print(readingCount);
		Serial.print(F(": "));

		pulse = pulseIn(pwPin, HIGH);		//	Returns length of pulse in us

		Serial.println(pulse);

		rangevalue[readingCount] = pulse;
		delay(10);							//	Short delay before next pulse
	}

	//	Take mode of readings to smooth out any errors or noise
	pulseMode = mode(rangevalue, arraysize);

	Serial.print(F("pulseMode: "));
	Serial.println(pulseMode);

	streamHeight = HEIGHT - (pulseMode / 10);	//	The pulse wavelength is 1mm

	Serial.println(F("done."));
	delay(5000);
}



int mode(int * x, int n)
{
	//	We'll calculate the mode of the readings in hopes of error reduction.
	int i = 0;
	int count = 0;
	int maxCount = 0;
	int mode = 0;

	int bimodal;
	int prevCount = 0;
	while(i < (n - 1))
	{
		prevCount = count;
		count = 0;
		while(x[i] == x[i + 1])
		{
			count++;
			i++;
		}
		if(count > prevCount & count > maxCount)
		{
			mode = x[i];
			maxCount = count;
			bimodal = 0;
		}
		if(count == 0)
		{
			i++;
		}
		if(count == maxCount) 		//	If the dataset has 2 or more modes
		{
			bimodal = 1;
		}
		if(mode == 0 || bimodal == 1) 	//	Return the median if no mode
		{
			mode = x[(n / 2)];
		}
		return mode;
	}
}



boolean sendMail()
{
	char emailSubject[49];
	int dailyRainmm = (round(dailyRain / 1000));	//	Convert um to mm

	//	If the current minute is less than ten, we'll render it with a leading
	//	zero (6:00 instead of 6:0)
	if(currentMinute < 10)
	{
		sprintf(emailSubject, "AT+SMTPSUB=\"%d:0%d: Altura: %dcm Lluvia: %mm\"", currentHour, currentMinute, streamHeight, dailyRainmm);
	}
	else
	{
		sprintf(emailSubject, "AT+SMTPSUB=\"%d:%d: Altura: %dcm Lluvia: %mm\"", currentHour, currentMinute, streamHeight, dailyRainmm);
	}

	ATcommand(F("AT+EMAILCID=1"));		//	Set bearer profile
	ATcommand(F("AT+EMAILTO=30"));		//	Set server timeout

	//      The following four lines must be customized for your SMTP server:

	ATcommand(F("AT+SMTPSRV=\"INSERT.YOUR.SMTP.SERVER.HERE\",587"));
	ATcommand(F("AT+SMTPAUTH=1,\"YOUR.SMTP.USERNAME.HERE\",\"YourPassword\""));
	ATcommand(F("AT+SMTPFROM=\"YOUR.FROM.ADDRESS\",\"Your From Name\""));
	ATcommand(F("AT+SMTPRCPT=0,0,\"YOUR.TO.ADDRESS\",\"Your To Name\""));
	ATcommand(emailSubject);

	if(ATcommand("AT+SMTPSEND", "+SMTPSEND: 1") == false)
	{
		return false;
	}
	else
	{
		return true;
	}
	delay(500);
}



boolean sendSMS()
{
	char sms[51];
	int dailyRainmm = (round(dailyRain / 1000));	//	Convert um to mm

	//	If the current minute is less than ten, we'll render it with a leading
	//	zero (6:00 instead of 6:0)
	if(currentMinute < 10)
	{
		sprintf(sms, "@(your pulse.to group) %d:0%d: Altura: %dcm Lluvia: %dmm", currentHour, currentMinute, streamHeight, dailyRainmm);
	}
	else
	{
		sprintf(sms, "@(your pulse.to group) %d:%d: Altura: %dcm Lluvia: %dmm", currentHour, currentMinute, streamHeight, dailyRainmm);
	}

	/* If pulse.to would fix its Twitter interface, this could work instead:

	if (!fona.sendSMS(40404, sms)) {
	        Serial.println(F("Failed"));
	}
	else
	{
	        Serial.println(F("Sent."));
	}*/

	if (!fona.sendSMS(MOBILE_GATEWAY, sms))
	{
		Serial.println(F("Failed"));
		return false;
	}
	else
	{
		Serial.println(F("Sent"));
		return true;
	}
}



boolean ATcommand(char * command)
{
	flushFona();
	fona.println(command);
	Serial.print(command);
	if(fonaParse("OK") == false)
	{
		return false;
	}
	else
	{
		return true;
	}
}



boolean ATcommand(const __FlashStringHelper * command)
//	We can save quite a bit of SRAM by allowing ATcommand arguments to be
//	stored as flash strings
{
	flushFona();
	fona.println(command);
	Serial.print(command);
	if(fonaParse("OK") == false)
	{
		return false;
	}
	else
	{
		return true;
	}
}



boolean ATcommand(char * command, char * reply)
{
	flushFona();
	fona.println(command);
	Serial.print(command);
	if(fonaParse(reply) == false)
	{
		return false;
	}
	else
	{
		return true;
	}
}



void flushFona()
{
	// Read all available serial input to flush pending data.
	while(fona.available())
	{
		fona.read();
	}
}



boolean fonaParse(char * expected)
{
	char content[127];
	char character = -1;
	byte index = 0;             	   		//	Index into array

	delay(1000);							//	Let FONA catch up

	while(fona.available() > 0)
	{
		if(index < 126) 					// 	Array size minus one
		{
			character = fona.read(); 		// 	Read a character
			content[index] = character; 	// Store it
			index++;						// Increment where to write next
			content[index] = '\0'; 			// Null terminate the string
		}
	}

	Serial.println(content);				//	For debugging

	if (strstr(content, expected)  == NULL) //	If expected reply not found
	{
		return false;
	}
	else
	{
		return true;
	}
}



boolean mailCall()
{
	/*	Mail and SMS schedules vary depending on whether it is rainy or dry
	season. This function checks to see if it is time to send emails.*/

	//	During dry season we send one message per day, at 6:00
	if(rainySeason == false && currentMinute == 0 && (currentHour == 6))
	{
		return true;
	}
	else
	{
		//	During rainy season, we send a message every six hours, at 0:00, 6:00,
		//	12:00 and 18:00.
		if(rainySeason == true && currentMinute == 0 && (currentHour == 0 ||
		        currentHour == 6 || currentHour == 12 || currentHour == 18))
		{
			return true;
		}
		else
		{
			return false;
		}
	}
}



boolean SMScall()
{
	/*	Mail and SMS schedules vary depending on whether it is rainy or dry
	season. This function checks to see if it is time to send SMS messages.*/

	//	During dry season we send one message per day, at 7:00
	if(rainySeason == false && currentMinute == 0 && (currentHour == 7))
	{
		return true;
	}
	else
	{
		//	During rainy season, we send a message every six hours, at 7:00,
		//	13:00 and 19:00.
		if(rainySeason == true && currentMinute == 0 && (currentHour == 7 ||
		        currentHour == 13 || currentHour == 19))
		{
			return true;
		}
		else
		{
			return false;
		}
	}
}



boolean clockSet()
{
	//	This routine sets the clock based on the mobile provider's network time.

	/*	Network timestamping is disabled on the FONA by default. We will check
	to see if it is enabled, and if not -- for example if we're using this FONA
	for the first time -- we'll enable it. */
	if(ATcommand("AT+CLTS?", "+CLTS: 1") == false)	//	If timestamp disabled...
	{
		ATcommand(F("AT+CLTS=1"));	//	...enable it...
		ATcommand(F("AT&W"));		//	...save as the new default...
		stopFona();					//	...turn off the FONA and...
		bootFona();					//	...restart.
	}

	flushFona();						//	Flush any stray input
	fona.enableNetworkTimeSync(true);   //  ...try to get the time from the mobile network
	delay(5000);						//	Let FONA catch up
	fona.println(F("AT+CCLK?"));		//	Query FONA's clock for network time
	int netYear = fona.parseInt();		//	Get the results
	byte netMonth = fona.parseInt();
	byte netDay = fona.parseInt();
	byte netUTCHour = fona.parseInt();
	byte netMinute = fona.parseInt();
	byte netSecond = fona.parseInt();	//	Our seconds may lag slightly

	//    Set to Honduras time -- Honduras doesn't have Daylight Savings, so this is easy
	byte netHour;

	if(netUTCHour - 6 < 0)
	{
		netHour = (netUTCHour - 6 + 24);    //    Yes, that's 18. But this is easier to follow.
	}
	else
	{
		netHour = (netUTCHour - 6);
	}

	//	When the time and date check fails, it usually returns the date as 1/1/2004
	if(netYear >= 15)
	{
		DateTime dt(netYear, netMonth, netDay, netHour, netMinute, netSecond, 0);
		RTC.adjust(dt); 				//	Adjust date-time as defined above
	}

	char theDate[17];
	sprintf(theDate, "%d/%d/%d %d:%d", netMonth, netDay, netYear, netHour, netMinute);
	Serial.println(theDate);

	delay(500);							//	Give FONA time to catch up
	return true;
}
