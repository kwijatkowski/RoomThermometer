#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <LiquidCrystal_I2C.h>
#include <virtuabotixRTC.h>
#include "Utils.h"
#include <ArduinoTrace.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <MovingAverage.h>

//HARDWARE SETUP

bool arduinoMega = false;

//NANO
#define HISTORY_ARRY_LENGTH 40
#define HISTORY_ARRY_PARAMS_COLUMNS 3
#define TIME_HISTORY_ARRY_COLUMNS 4

#define VAL_ROTARY_MIN 0
#define VAL_ROTARY_MAX 150

#define VOLTAGE_PIN 1
#define encoder0PinA 2
#define encoder0PinB 3
#define DISPLAY_POWER_PIN 9

#define LIGHT_SENSOR_PIN 0
#define LIGHT_SENSOR_MIN_VALUE 10
#define LIGHT_SENSOR_MAX_VALUE 900
#define BACKLIGHT_MIN_VALUE 1
#define BACKLIGHT_MAX_VALUE 255

#define LCD_CHARS 16
#define CLOCK_POWER_PIN 11
#define LOW_BATTERY_LED 13

#define BACKLIGHT_PIN 5

float R1 = 37.3;
float R2 = 17.337;

float LOW_VOLTAGE_TRESHOLD = 3.5; // >> measurement below this value is not reliable 
float CRITICAL_LOW_VOLTAGE_TRESHOLD = 3.4; // >> measurement below this value is not reliable 

//HARDWARE
Adafruit_BMP280 bmp; // I2C
LiquidCrystal_I2C lcd(0x27, LCD_CHARS, 2); // I2C address 0x27, 16 column and 2 rows
virtuabotixRTC myRTC(6, 7, 8);
MovingAverage<unsigned> lightMA(10, 0);

//HISTORY ARRYS ALLOCATION
float history_arry[HISTORY_ARRY_LENGTH][HISTORY_ARRY_PARAMS_COLUMNS];
int timeHistoryArry[HISTORY_ARRY_LENGTH][TIME_HISTORY_ARRY_COLUMNS];

//STATE VARIABLES
int counter = 0;
int totalEntries = 0;
int valRotary = 0;
int lastValRotary = 0;
int row1StringLength = 0;
int row2StringLength = 0;
int activityTimerTimes = 0;
int measureIterator = 0;
int lowVoltageCheckIterator = 0;
bool measure = false;
bool checkVoltage = false;
bool ledOn = true;
bool userActive = true;
bool isLcdOn = false;
bool batteryLow = false;
bool lowBatteryLedOn = false;
bool killMode = false;
int backlight = BACKLIGHT_MIN_VALUE;

int INT_MAX = 10000;

//SETTINGS
int MEASUREMENTS_ITERATOR_TRESHOLD = 1800;//600; // MEASUREMENTS_INTERVAL_SECONDS / MEASURE_TIMER_OVERFLOW_SECONDS;
int LOW_VOLTAGE_ITERATOR = 1;
int USER_INACTIVITY_ITERATOR = 10000; //1000 = 1s //USER_INACTIVITY_TRESHOLD_SECONDS / (USER_INACTIVITY_TIMER_OVERFLOW_MILISECONDS / 1000);

void setup() {
	//Serial.begin(19200);
	//Serial.println("Setup begin..");
	//Serial.println("Setup timers");
	cli();                       // disable all interrupts
//measure timer
	setupOneSecondTimer();
	//activity timer
	setupActivityTimer();
	sei();

	//Serial.println("Setup pins");
	//pinMode(LED_PIN, OUTPUT);
	pinMode(DISPLAY_POWER_PIN, OUTPUT);
	digitalWrite(DISPLAY_POWER_PIN, HIGH);

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
	pinMode(LOW_BATTERY_LED, OUTPUT);
	digitalWrite(LOW_BATTERY_LED, LOW);

	pinMode(BACKLIGHT_PIN, OUTPUT);

	//Serial.println("Setup sensors");
	setupEncoder();
	//bmp
	uint8_t bmpAddress = 0x76;
	initializeBMP(bmpAddress, arduinoMega);
	//lcd
	turnDisplayOn(true);

	//clock
	//Serial.println("Setup clock");
  // seconds, minutes, hours, day of the week, day of the month, month, year
	myRTC.setDS1302Time(0, 25, 9, 6, 14, 3, 2021); 
	setupClock();
	clockPowerEnable(true);

	//Serial.println("Measure and print first");
	myRTC.updateTime();
	printTimeToLcd(myRTC.year, myRTC.month, myRTC.dayofmonth, myRTC.hours, myRTC.minutes);	
	readTemperatureAndPressureAndStore(true, true, -1);
}

void setupClock() {
	pinMode(CLOCK_POWER_PIN, OUTPUT);
	digitalWrite(CLOCK_POWER_PIN, HIGH);
}


void setupEncoder() {
	//encoder
	pinMode(encoder0PinA, INPUT_PULLUP);
	pinMode(encoder0PinB, INPUT_PULLUP);
	attachInterrupt(0, doEncoder, CHANGE);
}


void setupOneSecondTimer() {
	TCCR1A = 0;
	TCCR1B = 0;
	TCNT1 = 0;

	// 1 Hz (16000000/((15624+1)*1024))
	OCR1A = 15624;
	// CTC
	TCCR1B |= (1 << WGM12);
	// Prescaler 1024
	TCCR1B |= (1 << CS12) | (1 << CS10);
	// Output Compare Match A Interrupt Enable
	TIMSK1 |= (1 << OCIE1A);
}

void setupActivityTimer() {
	TCCR2A = 0;
	TCCR2B = 0;
	TCNT2 = 0;

	// 1000 Hz (16000000/((124+1)*128))
	OCR2A = 124;
	// CTC
	TCCR2A |= (1 << WGM21);
	// Prescaler 128
	TCCR2B |= (1 << CS22) | (1 << CS20);
	// Output Compare Match A Interrupt Enable
	TIMSK2 |= (1 << OCIE2A);
}

//one second timer
ISR(TIMER1_COMPA_vect) 
{
	++measureIterator;
	if (measureIterator == MEASUREMENTS_ITERATOR_TRESHOLD) { 
		measureIterator = 0;
		measure = true;
	}

	++lowVoltageCheckIterator;
	if (lowVoltageCheckIterator == LOW_VOLTAGE_ITERATOR) {
		lowVoltageCheckIterator = 0;
		checkVoltage = true;
	}

	if (batteryLow) {
		blinkLowBatteryLed();
	}	
}

void blinkLowBatteryLed() {
	lowBatteryLedOn = !lowBatteryLedOn; //blink it on interrupt
	digitalWrite(LOW_BATTERY_LED, lowBatteryLedOn);
}

//user activity check
ISR(TIMER2_COMPA_vect)
{
	if (userActive) { ++activityTimerTimes; }	

	if (activityTimerTimes == USER_INACTIVITY_ITERATOR) {
		activityTimerTimes = 0;
		valRotary = 0;
		userActive = false;
	}
}

void resetActivityTimer() {
	TCNT2 = 0;
	activityTimerTimes = 0;
}

void loop() { 
	turnDisplayOn(userActive); //turn on if user active, otherwise turn off

	if (checkVoltage) {
		batteryLow = isLowBattery();
		killMode = isBatteryCriticallyLow();
		checkVoltage = false;
	}	

	//read sensors
	if (measure) {
		//Serial.print("M>>");
		readTemperatureAndPressureAndStore(true, false, -1);		
		//Serial.print("analog: ");
		//Serial.println(analogIn);		
		//ledOn = !ledOn;
		measure = false;	
		//Serial.println("M<<");
	}

	if (userActive) { //only show something if user active and battery not low		
    int back_into_history_rows = toHistory(valRotary);
    
		if (back_into_history_rows == 0) { //current screen >> live measurements
			myRTC.updateTime();
			printTimeToLcd(myRTC.year, myRTC.month, myRTC.dayofmonth, myRTC.hours, myRTC.minutes);
			//measure and print current
			readTemperatureAndPressureAndStore(false, true, back_into_history_rows);
		}
		else if (valRotary != lastValRotary) {
			Serial.print("history: ");
			Serial.println(back_into_history_rows);
			if (back_into_history_rows != INT_MAX) {
				int historyRowIndex = returnHistoryRowIndex(history_arry, counter - 1, back_into_history_rows - 1); // -1 as 1 into history is row 0 of history table

				myRTC.updateTime();
				printTimeToLcd(
					myRTC.year,
					timeHistoryArry[historyRowIndex][0],
					timeHistoryArry[historyRowIndex][1],
					timeHistoryArry[historyRowIndex][2],
					timeHistoryArry[historyRowIndex][3]);

				//crashing when turning left >> value not 0 yet
				printPTHistoryRow(history_arry[historyRowIndex], back_into_history_rows);
			}
			else { //status screen
				Serial.println("status Screen!");
				float batteryVoltage = readBatteryVoltage();
				printStatusScreen(millis(), batteryVoltage);
			}
			lastValRotary = valRotary;
		}
	}
	else {
		sleepNow();
	}
}

void printStatusScreen(int uptimeMillis, float voltage) {
	char lineBuff1[LCD_CHARS + 1] = "";

	int v1 = voltage;                  // Get the integer (678).
	float tmpFrac = voltage - v1;      // Get fraction (0.0123).
	int v2 = tmpFrac * 100;  // Turn into integer (123).

	int v3 = LOW_VOLTAGE_TRESHOLD;                  // Get the integer (678).
	float tmpFrac1 = LOW_VOLTAGE_TRESHOLD - v3;      // Get fraction (0.0123).
	int v4 = tmpFrac1 * 100;  // Turn into integer (123).

	int length1 = sprintf(lineBuff1, "%d.%02d/%d.%02dV", v1, v2, v3, v4);

	lcdClear(0);
	lcdClear(1);
	lcd.setCursor(0, 0);
	lcd.print("battery voltage:");
	lcd.setCursor(0, 1);
	lcd.print(lineBuff1);
}

void sleepNow(){
	//Serial.println("sleep");
	clockPowerEnable(false);
	digitalWrite(LED_BUILTIN, LOW);
	// Choose our preferred sleep mode:
	if (killMode) { 
		digitalWrite(LOW_BATTERY_LED, LOW);
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
		//Serial.println("kill!!!");
	}
	else { set_sleep_mode(SLEEP_MODE_IDLE); }

	//Serial.flush();
	power_timer0_disable();
	power_timer2_disable(); //activity timer
	power_adc_disable();
	power_spi_disable();
	power_twi_disable();
	// Set sleep enable (SE) bit:
	sleep_enable();
	// Put the device to sleep:
	sleep_mode();
	// Upon waking up, sketch continues from this point.
	sleep_disable();
/* Re-enable the peripherals. */
	//power_all_enable();
	power_twi_enable();
	power_timer0_enable();
	power_timer2_enable();
	power_adc_enable();
	power_spi_enable();
	clockPowerEnable(true);
	//digitalWrite(LED_BUILTIN, HIGH);
	//Serial.println("wakeUP");
}

bool isLowBattery() {
	float voltage = readBatteryVoltage();
	//Serial.print("voltage: ");
	//Serial.println(voltage);
	return voltage < LOW_VOLTAGE_TRESHOLD;
}

bool isBatteryCriticallyLow() {
	float voltage = readBatteryVoltage();
	bool critical = voltage < CRITICAL_LOW_VOLTAGE_TRESHOLD;
	if (critical) { //Serial.println("bat critical");
	}
	return critical;
}


float readBatteryVoltage() {
	int voltageIn = analogRead(VOLTAGE_PIN);
	Serial.print("voltageIn ");
	Serial.println(voltageIn);
	float voltage = voltageIn * (5.0 / 1023) * ((R1 + R2) / R2);
	return voltage;
}

void clockPowerEnable(bool enable) {
	if (enable) {
		digitalWrite(DISPLAY_POWER_PIN, HIGH);
	}
	else {
		digitalWrite(DISPLAY_POWER_PIN, LOW);
	}
}

int toHistory(int valRotary) {
	int toHistory = map(valRotary, VAL_ROTARY_MIN, VAL_ROTARY_MAX, -1, HISTORY_ARRY_LENGTH + 2 ); // +2 as I want to have status screen at the end
	if (toHistory < 0) {
		toHistory = 0;
	}

	if (toHistory > totalEntries) {
		toHistory = INT_MAX; //indicate status screen //HISTORY_ARRY_LENGTH
		return toHistory;
	}

	return toHistory;
}

int toHistory1(int valRotary) {

	int toHistory = map(valRotary, VAL_ROTARY_MIN, VAL_ROTARY_MAX, -1, HISTORY_ARRY_LENGTH + 2); // +2 as I want to have status screen at the end
	if (toHistory > HISTORY_ARRY_LENGTH) {
		toHistory = INT_MAX; //indicate status screen //HISTORY_ARRY_LENGTH
		return toHistory;
	}
	if (toHistory < 0) {
		toHistory = 0;
	}

	if (toHistory > totalEntries && toHistory != INT_MAX)
		toHistory = totalEntries;

	return toHistory;
}

void readTemperatureAndPressureAndStore(bool store, bool print, int backHistory) {
	float t = bmp.readTemperature();
	float p = bmp.readPressure() / 100; //to hPa
	if (store) {
		if (counter == HISTORY_ARRY_LENGTH) {
			counter = 0;
		}
		storeMeasurementsToHistoryTable(p, t, counter);
		storeTimeToHistoryArry(counter);
		counter++;
	}

	if (print) {
		printPTToLcd(t, p, backHistory);
	}
}

void storeTimeToHistoryArry(int counter) {
	myRTC.updateTime();
	timeHistoryArry[counter][0] = myRTC.month;
	timeHistoryArry[counter][1] = myRTC.dayofmonth;
	timeHistoryArry[counter][2] = myRTC.hours;
	timeHistoryArry[counter][3] = myRTC.minutes;
}

void storeMeasurementsToHistoryTable(float press, float temp, int counter) {
	history_arry[counter][0] = totalEntries;
	history_arry[counter][1] = temp;
	history_arry[counter][2] = press;

	if (totalEntries != HISTORY_ARRY_LENGTH) {
		totalEntries++;
	}
}

void printTimeToLcd(int year, int month, int day, int hours, int minutes) {
	char lineBuff[LCD_CHARS + 1] = ""; //+1 for string termination char

	char* monthStr = NULL;
	char* dayOfMonthStr = NULL;
	char* minutesStr = NULL;
	monthStr = intToString(month, 2, monthStr);
	dayOfMonthStr = intToString(day, 2, dayOfMonthStr);
	minutesStr = intToString(minutes, 2, minutesStr);
	int length = sprintf(lineBuff, "%d/%s/%s %d:%s", year, monthStr, dayOfMonthStr, hours, minutesStr);

	if (length != row1StringLength) { //reduce blinking 
		lcdClear(0);
		row1StringLength = length;
	}

	lcd.setCursor(0, 0);
	lcd.print(lineBuff);

	free(monthStr);
	free(dayOfMonthStr);
	free(minutesStr);
}

void lcdClear(int row) {
	lcd.setCursor(0, row);
	lcd.println("                ");
}

void printPTToLcd(float temp, float press, int backHistory) {
	char lineBuff[LCD_CHARS + 1] = "";
	int tempInt1 = temp;
	float tempFrac = temp - tempInt1;
	int tempInt2 = tempFrac * 10;
	int pressInt = press;
	int length;

	if (backHistory == 0) {
		length = sprintf(lineBuff, "> %d.%dC %dhPa", tempInt1, tempInt2, pressInt);
	}
	else {
		length = sprintf(lineBuff, "%d %d.%dC %dhPa", backHistory, tempInt1, tempInt2, pressInt);
	}

	if (length != row2StringLength) { //reduce blinking 
		lcdClear(1);
		row2StringLength = length;
	}

	lcd.setCursor(0, 1);
	lcd.print(lineBuff);
}

void doEncoder()
{
	userActive = true;
	resetActivityTimer();

	if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB))
	{
		if (valRotary < VAL_ROTARY_MAX){
			valRotary++;      
		}
	}
	else
	{
		if (valRotary > VAL_ROTARY_MIN) {
			valRotary--;
		}
	}

}

int calculateBacklight() {
	int lightValue = analogRead(LIGHT_SENSOR_PIN); //up if brighter
	lightMA.push(lightValue);
	int backlightTmp = 2.5 * map(lightMA.get(), LIGHT_SENSOR_MIN_VALUE, LIGHT_SENSOR_MAX_VALUE, BACKLIGHT_MIN_VALUE, BACKLIGHT_MAX_VALUE);

	if (backlightTmp < BACKLIGHT_MIN_VALUE) backlightTmp = BACKLIGHT_MIN_VALUE;
	if (backlightTmp > BACKLIGHT_MAX_VALUE) backlightTmp = BACKLIGHT_MAX_VALUE;

	return backlightTmp;  
}

void turnDisplayOn(bool enable) {
	if (enable) {
		if (!isLcdOn) { //do not initialize if already on
			digitalWrite(DISPLAY_POWER_PIN, HIGH);
			lcd.init(); // initialize the lcd
		}
		isLcdOn = true;
		backlight = calculateBacklight();
		analogWrite(BACKLIGHT_PIN, backlight);
	}
	else {
		digitalWrite(DISPLAY_POWER_PIN, LOW);
		isLcdOn = false;
		analogWrite(BACKLIGHT_PIN, 0);
	}
}

void printPTHistoryRow(float row[], int back) {
	float press = row[2];
	float temp = row[1];
	printPTToLcd(temp, press, back);
}

int returnHistoryRowIndex(float arry[][3], int lastHistoryRowIndex, int rowsToThePast) {
	if (lastHistoryRowIndex > HISTORY_ARRY_LENGTH) {
		return HISTORY_ARRY_LENGTH - 1;
	}

	if (rowsToThePast >= HISTORY_ARRY_LENGTH) { //to the oldest one
		rowsToThePast = HISTORY_ARRY_LENGTH;
	}

	int rowToReturn = lastHistoryRowIndex - rowsToThePast;

	if (rowToReturn < 0) {
		rowToReturn = HISTORY_ARRY_LENGTH + rowToReturn;
	}
	return rowToReturn;
}

void initializeBMP(uint8_t address, bool arduinoMega) {

	bool initialized = false;

	if (arduinoMega) {
		initialized = bmp.begin(address);
	}
	else { //settings for arduino NANO
		// Default settings from datasheet.
		bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, // Operating Mode. 
			Adafruit_BMP280::SAMPLING_X2, // Temp. oversampling
			Adafruit_BMP280::SAMPLING_X16, // Pressure oversampling
			Adafruit_BMP280::FILTER_X16, // Filtering. 
			Adafruit_BMP280::STANDBY_MS_500); // Standby time. 

		initialized = bmp.begin(address);
	}

	if (!initialized) {
		while (1) {
			Serial.begin(9600);
			Serial.print("Problem communicating with sensor, address: ");
			Serial.println(address);
			blinkLowBatteryLed();
			delay(1000);
		}
	}
}
