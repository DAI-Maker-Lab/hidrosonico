#define VERSION "0.4-150918.1105"

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
//	These variables must be customized for each unit.

#define UNIT_NAME "HidrosonicoXXXX"          //    Individual name of unit

/* We use the last four digits of the unit's phone number for the unit name, 
in order to more easily keep track of which unit is which.*/

const int HEIGHT = 0;		//	Height of top of octagonal gasket, in cm

//	http://data.sparkfun.com/hidrosonico4631
#define SPARKFUN_PUBLIC_KEY "enter_public_key_here"
#define SPARKFUN_PRIVATE_KEY "enter_private_key_here"

#define TWITTER_GATEWAY "40404"       //      Twitter SMS gateway 

#define UTCoffset -6                  //      Local time offset from UTC. 

/* If your locality implements daylight savings time, this is considerably more
complicated and you'll need changes in the code to accommodate it (provided you
are concerned with local time for things like a midnight reset of rain totals --
if not, you may not care much).*/

const int yellowLevel = 100;                //      Yellow alert level

/* This is the alert level at which we start sending more regular messages.*/

/* We use pulse.to so that we can send SMS messages to changing lists of 
stakeholders without having to change any parameters in the firmware. We'd
love to hear about alternatives!*/

#define MOBILE_GATEWAY "17043237775"         //      pulse.to gateway
const char* pulseTo = "@new_pulsegroup";     //      pulse.to group name

//      Insert your SMTP server settings below

#define SMTPServer "AT+SMTPSRV=\"mail.server.com\",587"
#define SMTPAuthorization "AT+SMTPAUTH=1,\"hidrosonico@domain.com\",\"password\""
#define SMTPFromLine "AT+SMTPFROM=\"hidrosonico@domain.com\",\"Hidrosonico XXXX\""
#define SMTPRecipient "AT+SMTPRCPT=0,0,\"mailgroup@domain.com\",\"Grupo Hidrosonico\""

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

/*	Hidrosónico is a stream gauge using a MaxSonar HRXL MB7369 sonar 
rangefinder, a Seeeduino Stalker v3 Arduino-compatible microcontroller platform, 
and an Adafruit FONA 800 GSM module. It reads the distance between the sensor 
(specifically from where the housing meets the threading) and the water's 
surface directly below and sends this data at specified intervals to cloud-based 
storage, SMS, Twitter and/or email as determined by the user. It was designed to 
aid in collection of hydrological data and for flood early warning in developing 
countries. This version integrates a rain gauge at the request of the recipient. 

The code provided sends data to data.sparkfun.com as the cloud service but it 
could be easily adapted to another provider. 

Please note the section at the beginning of the code that calls for user- and 
unit-specific input.

This is a beta unit and we are actively making changes as a result of 
the data and experiences we are collecting from existing pilot installations 
in Honduras.

This code uses snippets of the Adafruit example code for its FONA library 
(https://github.com/adafruit/Adafruit_FONA_Library) and snippets from the 
Sparkfun example weather station code (https://github.com/sparkfun/Weather_Shield) 
for the rain gauge, both provided under open source licenses or as public domain. 

The remainder of this code is released under the MIT License. You are free to use 
and change this code as you like, provided you retain attribution to Development
Alternatives, Inc. Non-code portions of the project are made available under
the Creative Commons-Attribution (CC BY) license.

If you use it, please let us know (robert_ryan-silva[at]dai.com), and if you 
improve it, please share! */

//	Include programming libraries into the sketch
#include <Adafruit_FONA.h>      //  For Adafruit FONA module
#include <SoftwareSerial.h>	//  For serial communication with FONA
#include <Wire.h>		//  I2C library for communication with RTC
#include <Sleep_n0m1.h>		//  Sleep library
#include <DS1337.h>		//  For the Stalker's real-time clock (RTC)

//	Define pins for various functions
#define rtcInt 0		//  Interrupt number associated with rtcPin
#define rainInt 1		//  Interrupt number associated with rainPin
#define rtcPin 2		//  Interrupt pin from RTC
#define rainPin 3   	        //  Set rain gauge pin

#define pwPin 7  		//  Pulse width data pin from sonar
#define rangePin 8              //  Sonar ranging start/stop pin

#define beePin 5                //  Pin to turn XBee on/off

#define FONA_RST 4 	        //  FONA reset pin
#define FONA_PS 6	        //  Power status pin
#define FONA_KEY 9	        //  On/off pin
#define FONA_TX 11	        //  Receive from FONA
#define FONA_RX 12	        //  Send to FONA

//	Define variables that the sketch will use
int streamHeight = 0;

/* We will create variables to hold  values for the time and date from the RTC. */

byte currentHour;		
byte currentMinute;		  
byte currentDay;
byte currentMonth;
int currentYear;

boolean yellow = false;        //  This cycle's yellow alert
boolean lastYellow = false;    //  Last cycle's yellow alert
boolean alertToday = false;    //  Whether an alert has been sent today
boolean readSuccess = false;   //  Whether the unit is returning a plausible reading

//  For debugging, we'll track the clock acquisition method -- see clockSet()

char method = 0;                

unsigned int action = 0;        //  Tracking the reading/upload number
int failedUploads = 0;

volatile unsigned long raintime, rainlast, raininterval;
volatile float dailyRain = 0;  	//  Rain in mm so far today in local time
volatile float hourlyRain = 0;	//  Rain in mm over the last hour

int pulse;			//  The pulse return length for the sonar
int pulseMode;                  //  The mode of pulses
byte arraysize = 7; 		//  Array for median sonar values (must be odd)
int rangevalue[] = { 0, 0, 0, 0, 0, 0, 0 };	//	Initial array for sonar

//  Configure the software serial interface to FONA

SoftwareSerial fonaSerial = SoftwareSerial(FONA_TX, FONA_RX);
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

DS1337 RTC;			//  Create the DS1337 real-time clock (RTC) object
Sleep sleep;			//  Create the sleep object



static void rainIRQ()
{
	raintime = millis();  			//  Set raintime to current time
	raininterval = raintime - rainlast;	//  Calculate interval between events
	if (raininterval > 10)  		//  Ignore switch-bounce glitches less
		                                //	than 10mS after initial edge
	{
		dailyRain += .2794;      //  Each dump is .2794mm of rain
		hourlyRain += .2794;  	//  Increase this hour's amount of rain
		rainlast = raintime; 	//  Set up for next event
		Serial.println(F("Rain event."));  //  For debugging
	}
}



static void rtcIRQ()
{
	//	This is a blank interrupt -- we're just using it to wake up.
}



void setup()
{
	Serial.begin(57600);		//  Begin the Serial interface -- using XBee baud rate

	Serial.println(F("Setup begins."));

	ADCSRA = 0;  			//  Disable analog-digital converter for power savings
	Wire.begin();			//  Begin the I2C interface
	RTC.begin();			//  Begin the RTC

	Serial.print(F("Hidrosonico online. Version "));
        Serial.println(VERSION);

	pinMode(pwPin, INPUT);	        //  Sonar pulse width pin
        pinMode(rangePin, OUTPUT);      //  Sonar range start/stop pin
        pinMode(beePin, OUTPUT);        //  Pin controlling power to XBee
	pinMode(FONA_KEY, OUTPUT);      //  The Key pin from the FONA
	pinMode(FONA_PS, INPUT);        //  The Power Status pin from the FONA
	pinMode(rainPin, INPUT_PULLUP); //  The signal pin from the rain gauge

        /*  We experimented with turning ranging off between readings, but it was unclear
        that it was saving much power and it seemed to be interfering with some of the readings.
        It may be worth testing in a future version, but for now we'll just leave it on.*/

//        digitalWrite(rangePin, LOW);    //  Turn ranging off
        digitalWrite(rangePin, HIGH);    //  Turn ranging on
        
	delay(1000);		        

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
                        Serial.println(F("FONA booted successfully."));
		}
	}
	
        if(booted == false)
        {
                Serial.println(F("FONA boot failed. Check connections."));
        }

//	fona.sendCheckReply(F("ATZ"), F("OK"));		//  Revert to last saved settings

	clockSet();	//  Set the RTC
	stopFona();	//  Turn off the FONA to save power

	DateTime now = RTC.now(); 		 //  Get the current date/time from RTC
	Serial.print(F("The current RTC time is "));
	Serial.print(now.hour(), DEC);
	Serial.print(F(":"));
	Serial.println(now.minute(), DEC);

	RTC.enableInterrupts(EveryMinute);	 //  RTC will interrupt every minute
	RTC.clearINTStatus();                    //  Clear any outstanding interrupts

	attachInterrupt(rainInt, rainIRQ, FALLING);
	attachInterrupt(rtcInt, rtcIRQ, FALLING);
	interrupts();					//  Activate interrupts

	Serial.println(F("Setup complete."));
}



void loop()
{
	DateTime now = RTC.now(); 	//  Get the current date/time

	Serial.print(now.hour());       //  For debugging, print the time
	Serial.print(F(":"));
	Serial.println(now.minute());

	Serial.flush();			//  Give Serial a chance to catch up before sleep

	//	If this is a fifteenth minute, take and upload a reading.

	if(now.minute() % 15 == 0)
	{       
                boolean dataSent = false;               //  Put the flag down
  
		currentHour = now.hour();      
		currentMinute = now.minute();  
		currentDay = now.date();       
		currentMonth = now.month();
		currentYear = now.year();

		action++;

                //  We'll turn the XBee on only from 9-6 to save power
                if (currentHour > 9 && currentHour < 14)
                {
                        digitalWrite (beePin, LOW);
                }
                else
                {
                        digitalWrite (beePin, HIGH);
                }

		bootFona();     //  Start the Fona

                /* One of the original field test units, in Corquín, Honduras, sometimes returned spurious 
                negative streamHeights. The non-negative results seemed to be accurate. While we think this was
                a hardware problem, we'll go ahead and retry the reading up to 3 times if the result is a
                negative streamHeight to try to reduce any recurrence. */

                byte readAttempts = 0;  //  Keep track of attempts at reading a non-negative result
                readSuccess = false;    //  Put the flag down
                
                //  We'll make up to three attempts to get a non-negative reading
                while(readAttempts < 3 && readSuccess == false)
		{
                        takeReading();	//  Take a reading
                        readAttempts++;
                }
                
                readAttempts = 0;      //    Reset the counter

		Serial.print(F("streamHeight: "));
		Serial.println(streamHeight);

                if(streamHeight > yellowLevel)    //    Are we at yellow alert?
                {
                        yellow = true;
                }
                else
                {
                        yellow = false;
                }

		//	We will attempt to upload data to the cloud. If the attempt fails,
		//	we will restart the Fona and try again for up to three attempts.

		byte sendAttempts = 0;		//	Reset the counter
		dataSent = false;		//	Reset the flag

		while(dataSent == false && sendAttempts <= 2)
		{
			sendAttempts++;		//	Increment attempt counter

			if(cloudUpload() == false)	//  If upload appears to fail...
			{
				Serial.println(F("Upload failed. Restarting Fona."));
				stopFona();		//  ...shut the FONA off.
				delay(5000);            //  Give the FONA a moment
				bootFona();             //  Reboot.
				dataSent = false;	//  Ensure flag is down
				failedUploads++;	//  Increment fail counter
			}

			else
			{
				dataSent = true;
			}
		}

		/*if(mailCall() == true)		        //	If it's time to send messages...
		{
			sendMail();			//	...send them.
                        sendSMS();
		}*/

		if(currentMinute == 0)	        //	At the top of the hour, after upload...
		{
			hourlyRain = 0;		//	...zero out the hour's rain total
		}

		if(currentHour == 0 && currentMinute == 0)	//	At midnight, after upload...
		{
			dailyRain = 0;          //	...zero out the day's rain total...
                        alertToday = false;     //      ...reset the alert flag
                        clockSet();             //      ...and set the time again -- the RTC loses more time
                                                //      than the datasheet claims.
		}

		stopFona();		        //	Shut down the FONA to save power
                
                lastYellow = yellow;
        }

	RTC.clearINTStatus();			//	Clear the last loop's RTC interrupt
	sleep.pwrDownMode(); 			//	Set sleep mode to Power Down
	Serial.print(F("Sleeping..."));
	Serial.flush();				//	Let Serial catch up before sleep
	sleep.sleepInterrupt(0, FALLING);	//	Sleep

	/* When the RTC interrupt triggers, the sketch will resume the loop here and so go 
        back to the beginning of the loop. */
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
        unsigned int vbat = 0;		//  Battery voltage as read by FONA
        
	fona.getBattVoltage(&vbat);	//  Read the battery voltage

        int rawDumps = (dailyRain / .2794);    

        /* We'll tell FONA to get ready for an HTTP GET command. We could use fona.HTTP_GET_START and build a url 
        with sprintf, but this uses a lot less SRAM.*/
        
	fona.sendCheckReply(F("AT+SAPBR=2,1"), F("OK"));
	fona.sendCheckReply(F("AT+SAPBR=1,1"), F("OK"));
	fona.sendCheckReply(F("AT+HTTPINIT"), F("OK"));
	fona.sendCheckReply(F("AT+HTTPPARA=\"CID\",1"), F("OK"));

        //  Now we'll construct the URL to transmit. 
        fona.print(F("AT+HTTPPARA=\"URL\",\"http://data.sparkfun.com/input/"));
        fona.print(SPARKFUN_PUBLIC_KEY);
        fona.print(F("?private_key="));
        fona.print(SPARKFUN_PRIVATE_KEY);
        fona.print(F("&1_numero="));
        fona.print(action);
        fona.print(F("&2_altura="));
        fona.print(streamHeight);
        fona.print(F("&3_lluvia_diaria="));
        fona.print(dailyRain, 1);
        fona.print(F("&4_lluvia_hora="));
        fona.print(hourlyRain, 1);
        fona.print(F("&5_bateria="));
        fona.print(vbat);
        fona.print(F("&6_hora_sistema="));
        fona.print(currentHour);
        fona.print(F(":"));
        
        if(currentMinute < 10)      //    Print a leading zero when necessary (i.e., 6:00 instead of 6:0)
        {
              fona.print(F("0"));
        }
        
        fona.print(currentMinute);
        fona.print(F("&7_clock_method="));
        fona.print(method);
        fona.print(F("&8_raw_dumps="));
        fona.print(rawDumps);
        fona.print(F("&9_upload_failures="));
        fona.print(failedUploads);
        fona.println(F("\""));


//    ...and Serial print the same thing:

        Serial.print(F("AT+HTTPPARA=\"URL\",\"http://data.sparkfun.com/input/"));
        Serial.print(SPARKFUN_PUBLIC_KEY);
        Serial.print(F("?private_key="));
        Serial.print(SPARKFUN_PRIVATE_KEY);
        Serial.print(F("&1_numero="));
        Serial.print(action);
        Serial.print(F("&2_altura="));
        Serial.print(streamHeight);
        Serial.print(F("&3_lluvia_diaria="));
        Serial.print(dailyRain, 1);
        Serial.print(F("&4_lluvia_hora="));
        Serial.print(hourlyRain, 1);
        Serial.print(F("&5_bateria="));
        Serial.print(vbat);
        Serial.print(F("&6_hora_sistema="));
        Serial.print(currentHour);
        Serial.print(F(":"));
        
        if(currentMinute < 10)      //    Print a leading zero when necessary (i.e., 6:00 instead of 6:0)
        {
              Serial.print(F("0"));
        }
        
        Serial.print(currentMinute);
        Serial.print(F("&7_clock_method="));
        Serial.print(method);
        Serial.print(F("&8_raw_dumps="));
        Serial.print(rawDumps);
        Serial.print(F("&9_upload_failures="));
        Serial.print(failedUploads);
        Serial.println(F("\""));
        
	flushFona();
	fona.sendCheckReply(F("AT+HTTPACTION=0"), F("OK"));

	fona.println("AT+HTTPREAD");

	int dataLength = fona.parseInt();      //  HTTPREAD returns the length of the data read and then the data
	int successFail = fona.parseInt();     //  data.sparkfun.com returns "1 success" on a successful upload

	if(successFail = 1)
	{
		Serial.println(F("Upload succeeded."));
		return true;
	}
	else
	{
		Serial.println(F("Upload failed."));
		return false;
	}

	fona.HTTP_GET_end();
}



void takeReading()
{

	Serial.print(F("Taking readings..."));

//        digitalWrite(rangePin, HIGH);    //    Start ranging - future implementation
//
//        /* The Maxbotix only needs 20us to start ranging, but does some filtering based on 
//        adjacent readings, so we'll let it run for five seconds before we start querying it. */
//
//        delay(5000);        
 
	//	Fill the array with readings
	for(int readingCount = 0; readingCount < arraysize; readingCount++)
	{
		/* The MaxSonar measures the distance of objects by bouncing a
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
		delay(10);				//	Short delay before next pulse reading
	}

//        digitalWrite(rangePin, LOW);          //    Turn sonar off - future implementation
        digitalWrite(rangePin, HIGH);          //    We'll leave ranging on
        
	//	Take mode of readings to smooth out any errors or noise
	pulseMode = mode(rangevalue, arraysize);

	Serial.print(F("pulseMode: "));
	Serial.println(pulseMode);

	streamHeight = HEIGHT - (pulseMode / 10);

        if(streamHeight < 0)
        {
                readSuccess = false;
        }
        else
        {
                readSuccess = true;
        }

	Serial.println(F("done."));
	delay(5000);
}



int mode(int * x, int n)    //  Calculate the mode of an array of readings
{
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
	char emailSubject[59];
        char* yellowAlert;
        int dailyRainmm = (round(dailyRain), 0);	//    Remove irrelevant decimal places

        if(yellow == true)
        {
                yellowAlert = "AMARILLA";
        }
        else
        {
                yellowAlert = "";
        }


	//	If the current minute is less than ten, we'll render it with a leading
	//	zero (6:00 instead of 6:0)
	if(currentMinute < 10)
	{
		sprintf(emailSubject, "AT+SMTPSUB=\"%d:0%d: %s Altura: %dcm Lluvia: %dmm\"", currentHour, currentMinute, yellowAlert, streamHeight, dailyRainmm);
	}
	else
	{
		sprintf(emailSubject, "AT+SMTPSUB=\"%d:%d: %s Altura: %dcm Lluvia: %dmm\"", currentHour, currentMinute, yellowAlert, streamHeight, dailyRainmm);
	}

	fona.sendCheckReply(F("AT+EMAILCID=1"), F("OK"));	//	Set bearer profile
	fona.sendCheckReply(F("AT+EMAILTO=30"), F("OK"));	//	Set server timeout

        //      The following four lines use the data you put in the #define statements at the top of the sketch:

	fona.sendCheckReply(F(SMTPServer), F("OK"));
	fona.sendCheckReply(F(SMTPAuthorization), F("OK"));
	fona.sendCheckReply(F(SMTPFromLine), F("OK"));
	fona.sendCheckReply(F(SMTPRecipient), F("OK"));

	fona.sendCheckReply(emailSubject, "OK");

	if(fona.sendCheckReply(F("AT+SMTPSEND"), F("OK")) == false)
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
        int dailyRainmm = (round(dailyRain), 0);	//    Remove irrelevant decimal places

        if(smsStart(MOBILE_GATEWAY) == false)
        {
                Serial.println(F("SMS failed."));
        }
        else
        {
                fona.print(pulseTo);
                fona.print(F(" "));
                fona.print(currentHour);
                fona.print(F(":"));
                
                if(currentMinute < 10)
                {
                        fona.print(F("0"));
                }
                
                fona.print(currentMinute);
                
                if(yellow == true)
                {
                        fona.print(F(" AMARILLA"));
                }
                
                fona.print(F(" Altura: "));
                fona.print(streamHeight);
                fona.print(F("cm Lluvia: "));
                fona.print(dailyRainmm);
                fona.println(F("mm"));
                
                smsEnd();
        }
                
        /* Let's try Twitter. Many networks, including Tigo in Honduras enable tweeting by
        texting to 40404.*/
        
        delay(10000);

        if(smsStart(TWITTER_GATEWAY) == false)
        {
                Serial.println(F("Tweet failed."));
        }
        else
        {
                fona.print(currentHour);
                fona.print(F(":"));
                
                if(currentMinute < 10)
                {
                        fona.print(F("0"));
                }
                
                fona.print(currentMinute);
                
                if(yellow == true)
                {
                        fona.print(F(" AMARILLA"));
                }
                
                fona.print(F(" Altura: "));
                fona.print(streamHeight);
                fona.print(F("cm Lluvia: "));
                fona.print(dailyRainmm);
                fona.println(F("mm"));
                
                smsEnd();
        }
}



boolean smsStart(const char* SMSrecipient)
{
        fona.sendCheckReply(F("AT+CMGF=1"), F("OK"));
        fona.print(F("AT+CMGS=\""));
        fona.print(SMSrecipient);
        if(fona.sendCheckReply(F("\""), F("> ")) == false)
        {
                return false;
        }
        else
        {
                return true;
        }
}



boolean smsEnd()
{
         fona.print("\x1A"); 
}
 
  
  
  boolean mailCall()
{
	/*	Mail and SMS schedules vary depending on whether the water has exceeded the yellow
        alert level.  */
        
        /*  If this reading is yellow and the last one is not, we'll send an alert provided we haven't
        sent one today already. When the levels are marginal, we could have lots of cases where it goes
        back and forth between yellow and sub-yellow, so we don't want to send more than one alert per day.*/
        
        if(lastYellow == false && yellow == true && alertToday == false)
        {
                return true;
                alertToday = true;
        }
        
        //	At yellow or above, we send four messages per day
	if(streamHeight >= yellow && (currentMinute == 0 && (currentHour == 0 ||
		        currentHour == 6 || currentHour == 12 || currentHour == 18)))
	{
		return true;
	}
	else
	{
		//	Below yellow alert, we send just one message at noon
		if(streamHeight < yellow && (currentMinute == 0 && currentHour == 12))
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
    	int netYear;	
	byte netMonth;
	byte netDay;
	byte netHour;
	byte netMinute;
	byte netSecond;
        byte netOffset;

        char theDate[17];

        fona.enableRTC(1);            //    Enable reading of time from cell network
        Serial.println(F("Attempting to get local timestamp...."));
        
        delay(15000);                   //    Give it (a lot of) time to catch up

        Serial.println(F("AT+CCLK?"));
        fona.println(F("AT+CCLK?"));    //    Query FONA's clock for network time              
        netYear = fona.parseInt();      //    Capture the results - network year is 2-digit
        netMonth = fona.parseInt();   
	netDay = fona.parseInt();
	netHour = fona.parseInt();
	netMinute = fona.parseInt();
	netSecond = fona.parseInt();	//    Our seconds may lag slightly
        netOffset = fona.parseInt();    //    This tells us the time zone the clock is using

        //  If necessary, adjust UTC to local time
        if((netOffset / 4) != UTCoffset)
        {
                netHour = (netHour - (netOffset / 4));                

                //  Adjust UTC to local time
                if((netHour + UTCoffset) < 0)
                {
                        netHour = (24 + netHour + UTCoffset);
                        netDay = (netDay - 1);
                }
                else
                {
                        if((netHour + UTCoffset) > 23)
                        {
                                netHour = (netHour + UTCoffset - 24);
                                netDay = (netDay + 1);
                        }
                        else
                        {
                                netHour = (netHour + UTCoffset);
                        }
                }
        }
        
        sprintf(theDate, "%d/%d/%d %d:%d", netYear, netMonth, netDay, netHour, netMinute);
        Serial.print(F("Network timestamp: "));
	Serial.println(theDate);

        /*  This timestamp function is network-dependent; not all cellular networks implement it. In fact,
        experience indicates it may be tower-dependent, because on the Tigo network in Honduras, it works in
        some locations and not in others. If we succeed in getting the time this way, great. If the 2-digit year 
        comes back as before 15 or after 50*, we take that as an indication of failire and try another method.
        The most common failure mode we've encountered is for the date to come back as 1 January 2004.
        
        * If you're still using this in 2051, yay! But it's time to update the code. :) */

        if(netYear < 15 || netYear > 50)      
        {
                Serial.println(F("Network timestamp failed."));
                Serial.println(F("Attempting to get time from GSM location service..."));
                flushFona();
                fona.println(F("AT+CIPGSMLOC=2,1"));    //	Query GSMLOC for time     
                delay(15000);                           //      Let the network catch up         
                int throwAway = fona.parseInt();        //      We don't need these ints
                //throwAway = fona.parseInt();
                //throwAway = fona.parseInt();
                netYear = fona.parseInt();		//	Get the results -- GSMLOC year is 4-digit
	        netMonth = fona.parseInt();
	        netDay = fona.parseInt();
	        netHour = fona.parseInt();              //      GSMLOC is supposed to get UTC
	        netMinute = fona.parseInt();
	        netSecond = fona.parseInt();	        //	Our seconds may lag slightly
                
                //  Adjust UTC to local time
                if((netHour + UTCoffset) < 0)
                {
                        netHour = (24 + netHour + UTCoffset);
                        netDay = (netDay - 1);
                }
                else
                {
                        if((netHour + UTCoffset) > 23)
                        {
                                netHour = (netHour + UTCoffset - 24);
                                netDay = (netDay + 1);
                        }
                        else
                        {
                                netHour = (netHour + UTCoffset);
                        }
                }
               
	        sprintf(theDate, "%d/%d/%d %d:%d", netYear, netMonth, netDay, netHour, netMinute);
	        Serial.print(F("GSMLOC time: "));
                Serial.println(theDate);
  
                if(netYear < 2015 || netYear > 2050 || netHour > 23)   //    If it still doesn't work...
                       {
                             Serial.println(F("GSM location service failed."));
                             fona.enableNTPTimeSync(true, F("0.daimakerlab.pool.ntp.org"));  //    Get time from NTP pool
                             Serial.println(F("Attempting to enable NTP sync."));
                             
                             delay(15000);                      //      Give it (a lot of) time to catch up
                             
                             fona.println(F("AT+CCLK?"));	//	Query FONA's clock for NTP time              
                             netYear = fona.parseInt();		//	Capture the results
                             netMonth = fona.parseInt();
                             netDay = fona.parseInt();
                             netHour = fona.parseInt();         //      We asked NTP for UTC
                             netMinute = fona.parseInt();
                             netSecond = fona.parseInt();	//	Our seconds may lag slightly
        
                
                              //  Adjust UTC to local time
                              if((netHour + UTCoffset) < 0)
                              {
                                      netHour = (24 + netHour + UTCoffset);
                                      netDay = (netDay - 1);
                              }
                              else
                              {
                                      if((netHour + UTCoffset) > 23)
                                      {
                                              netHour = (netHour + UTCoffset - 24);
                                              netDay = (netDay + 1);
                                      }
                                      else
                                      {
                                              netHour = (netHour + UTCoffset);
                                      }
                              }
        
                             sprintf(theDate, "%d/%d/%d %d:%d", netYear, netMonth, netDay, netHour, netMinute);
                             Serial.print(F("NTP time: "));
                             Serial.println(theDate);
                             
                             method = 'N';
                       }
                       else method = 'G';
        }
        else method = 'C';

        if((netYear >= 15 && netYear < 50) || (netYear >= 2015 && netYear < 2050))
        {
                Serial.println(F("Adjusting RTC."));
                DateTime dt(netYear, netMonth, netDay, netHour, netMinute, netSecond, 0);
	        RTC.adjust(dt); 		//	Adjust date-time as defined above
	        sprintf(theDate, "%d/%d/%d %d:%d", netMonth, netDay, netYear, netHour, netMinute);
 	        Serial.println(theDate);
        }
        else
        {
                Serial.println(F("Didn't find reliable time. Will continue to use RTC's current time."));
                method = 'X';
        }

	delay(500);							//	Give FONA time to catch up
        return true;
}



void flushFona()
{
	// Read all available serial input to flush pending data.
	while(fona.available())
	{
		fona.read();
	}
}
