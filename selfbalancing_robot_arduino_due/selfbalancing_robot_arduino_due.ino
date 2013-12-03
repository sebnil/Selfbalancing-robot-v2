#include <Wire.h>
#include <L29x.h>
#include <PID_v1.h> //github.com/mwoodward/Arduino-PID-Library
#include <Button.h>        //github.com/JChristensen/Button
#include <SerialCommand.h> //github.com/kroimon/Arduino-SerialCommand
#include <MovingAvarageFilter.h> //github.com/sebnil/Moving-Avarage-Filter--Arduino-Library-
#include <FIR.h> // github.com/sebnil/FIR-filter-Arduino-Library
#include <FIR_v2.h> // github.com/sebnil/FIR-filter-Arduino-Library
#include <KalmanFilter.h> // github.com/nut-code-monkey/KalmanFilter-for-Arduino
#include <FreeRTOS_ARM.h>
#include <stdlib.h>
//#include <efc.h>
#include <DueFlashStorage.h>
//#include <flash_efc.h>
DueFlashStorage dueFlashStorage;


// Redefine AVR Flash string macro as nop for ARM
#undef F
#define F(str) str

#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

const uint8_t LED_PIN = 13;

// motor config
#include <L29x.h>
L29x motorLeft(9, 22, 23); // enable (PWM), motor pin 1, motor pin 2
L29x motorRight(8, 24, 29); // enable (PWM), motor pin 1, motor pin 2

// button declarations
Button startBtn(30, false, false, 20);
Button stopBtn(31, false, false, 20);
Button calibrateBtn(42, false, false, 20);

// ss
boolean started = true;

SerialCommand sCmd;     // The demo SerialCommand object

// PID variables
double anglePIDSetpoint, anglePIDInput, anglePIDOutput;
double speedPIDInput, speedPIDOutput, speedPIDSetpoint;
double anglePIDInputFiltered;

// filters
MovingAvarageFilter speedMovingAvarageFilter(14);
MovingAvarageFilter angleMovingAvarageFilter(4);
KalmanFilter speedKalmanFilter;
KalmanFilter angleKalmanFilter;
FIR_v2 speedMovingAvarageFilter2;

// The cascading PIDs. The tunings are updated from the code
PID anglePID(&anglePIDInput, &anglePIDOutput, &anglePIDSetpoint, 0, 0, 0, DIRECT);
PID speedPID(&speedPIDInput, &speedPIDOutput, &speedPIDSetpoint, 0, 0, 0, DIRECT);

// handle for blink task
//xTaskHandle blink;

//v_read_imu_task
// Euler angles
float yaw;
float pitch;
float roll;
float angleRaw;
float roll_filtered;
// DCM timing in the main loop
unsigned long timestamp;
unsigned long timestamp_old;
float G_Dt; // Integration time for DCM algorithm

long leftMotorPosition  = 0;
long rightMotorPosition  = 0;
long lastLeftMotorPosition  = 0;
long lastRightMotorPosition  = 0;
float leftMotorSpeed  = 0;
float rightMotorSpeed  = 0;
float speedKalmanFiltered = 0;
float speedFIRFiltered = 0;

struct Configuration {
	double speedPIDKp;
	double speedPIDKi;
	double speedPIDKd;
	double speedPIDOutputLowerLimit;
	double speedPIDOutputHigherLimit;
	double anglePIDAggKp;
	double anglePIDAggKi;
	double anglePIDAggKd;
	double anglePIDConKp;
	double anglePIDConKi;
	double anglePIDConKd;
	double anglePIDLowerLimit;
	double calibratedZeroAngle;
	uint8_t anglePIDSampling;
	double angleKalmanFilterR;
	uint8_t speedPIDSampling;
	uint8_t angleSensorSampling;
	uint8_t motorSpeedSensorSampling;
	double speedKalmanFilterR;
	uint8_t debugLevel;
	uint8_t debugSampleRate;
	uint8_t speedPIDOutputDebug;
	uint8_t speedPIDInputDebug;
	uint8_t speedKalmanFilterDebug;
	uint8_t speedRawDebug;
	uint8_t speedMovingAvarageFilter2Debug;
	uint8_t anglePIDSetpointDebug;
	uint8_t anglePIDInputDebug;
	uint8_t anglePIDOutputDebug;
	uint8_t angleRawDebug;
};
Configuration configuration;
byte b[sizeof(Configuration)];

void setConfiguration() {
	/* Flash is erased every time new code is uploaded. Write the default configuration to flash if first time */
	// running for the first time?
	uint8_t codeRunningForTheFirstTime = dueFlashStorage.read(0); // flash bytes will be 255 at first run
	if (codeRunningForTheFirstTime) {
		Serial.println("yes");
		/* OK first time running, set defaults */
		configuration.speedPIDKp = 0.50;
		configuration.speedPIDKi = 0.01;
		configuration.speedPIDKd = 0.01;
		configuration.speedPIDOutputLowerLimit = -3.00;
		configuration.speedPIDOutputHigherLimit = 3.00;
		configuration.anglePIDAggKp = 50.00;
		configuration.anglePIDAggKi = 0.00;
		configuration.anglePIDAggKd = 0.00;
		configuration.anglePIDConKp = 20.00;
		configuration.anglePIDConKi = 0.50;
		configuration.anglePIDConKd = 0.25;
		configuration.anglePIDLowerLimit = 5.00;
		configuration.calibratedZeroAngle = -11.35;
		configuration.anglePIDSampling = 10;
		configuration.speedPIDSampling = 5;
		configuration.angleKalmanFilterR = 1.00;
		configuration.angleSensorSampling = 15;
		configuration.motorSpeedSensorSampling = 10;
		configuration.speedKalmanFilterR = 3.00;
		
		
		configuration.debugLevel = 0;
		configuration.debugSampleRate = 50;
		//  configuration.speedPIDSetpointDebug = 1;
		configuration.speedPIDOutputDebug = 1;
		configuration.speedPIDInputDebug = 1;
		configuration.speedKalmanFilterDebug = 1;
		configuration.speedRawDebug = 1;
		configuration.speedMovingAvarageFilter2Debug = 0;
		configuration.anglePIDSetpointDebug = 1;
		configuration.anglePIDInputDebug = 1;
		configuration.anglePIDOutputDebug = 1;
		configuration.angleRawDebug = 1;

		// write configuration struct to flash at address 4
		//byte b2[sizeof(Configuration)]; // create byte array to store the struct
		memcpy(b, &configuration, sizeof(Configuration)); // copy the struct to the byte array
		dueFlashStorage.write(4, b, sizeof(Configuration)); // write byte array to flash
		
		// write 0 to address 0 to indicate that it is not the first time running anymore
		dueFlashStorage.write(0, 0);
	}
	else {
		Serial.println("no");
		/* read configuration struct from flash */
		byte* b2 = (dueFlashStorage.readAddress(4)); // byte array which is read from flash at address 4
		memcpy(&configuration, b2, sizeof(Configuration)); // copy byte array to temporary struct
	}
}

void setup() {
	Serial.begin(115200);
	//  Serial3.begin(9600);
	// wait for Leonardo
	while(!Serial) {
	}
	Wire.begin();

	pinMode(A0, INPUT);
	pinMode(A1, INPUT);
	pinMode(A2, INPUT);
	pinMode(A3, INPUT);
	pinMode(30, INPUT);
	pinMode(31, INPUT);
	pinMode(42, INPUT);

	attachInterrupt(A2, A2interruptFun, RISING);
	attachInterrupt(A3, A3interruptFun, RISING);


	// filters
	speedKalmanFilter.setState(0);
	float coefficients[] = {
	1,1,1,1,1,1,1,1,1          };
	speedMovingAvarageFilter2.setCoefficients(coefficients);
	speedMovingAvarageFilter2.setNumberOfTaps(9);
	speedMovingAvarageFilter2.setGain((float) (1/9));


	setConfiguration();

	// create read imu task
	//xTaskCreate(vReadIMUTask, NULL, configMINIMAL_STACK_SIZE + 50, NULL, 1, NULL);

	// create control task
	xTaskCreate(vControlTask, NULL, configMINIMAL_STACK_SIZE + 100, NULL, 1, NULL);

	// create button task
	xTaskCreate(vInterfaceAndDebugTask,NULL, configMINIMAL_STACK_SIZE+100, NULL,1,NULL);


	// start FreeRTOS
	vTaskStartScheduler();

	// should never return
	Serial.println(F("Die"));
	while(1);
}

// WARNING idle loop has a very small stack (configMINIMAL_STACK_SIZE)
// loop must never block
volatile uint32_t count = 0;
void loop() {
	while(1) {
		// must insure increment is atomic
		// in case of context switch for print
		noInterrupts();
		count++;
		interrupts();
	}
}



void newConfig() {
	// init speed PID
	speedPIDSetpoint = 0;
	speedPID.SetOutputLimits(configuration.speedPIDOutputLowerLimit, configuration.speedPIDOutputHigherLimit);
	//anglePID.SetMode(AUTOMATIC);
	speedPID.SetSampleTime(configuration.speedPIDSampling);
	speedPID.SetTunings(configuration.speedPIDKp, configuration.speedPIDKi, configuration.speedPIDKd);

	//init angle PID
	anglePIDSetpoint = 0;
	anglePID.SetOutputLimits(-100, 100);
	//anglePID.SetMode(AUTOMATIC);
	anglePID.SetSampleTime(configuration.anglePIDSampling);
	
	angleKalmanFilter.setR(configuration.angleKalmanFilterR);
	speedKalmanFilter.setR(configuration.speedKalmanFilterR);
	
	// write configuration struct to flash at address 4
	//byte b2[sizeof(Configuration)]; // create byte array to store the struct
	memcpy(b, &configuration, sizeof(Configuration)); // copy the struct to the byte array
	dueFlashStorage.write(4, b, sizeof(Configuration)); // write byte array to flash
}

/* interrupt functions for counting revolutions in the encoders */
void A2interruptFun() {
	if (digitalRead(A1))
	rightMotorPosition++;
	else
	rightMotorPosition--;
}
void A3interruptFun() {
	if (digitalRead(A0))
	leftMotorPosition++;
	else
	leftMotorPosition--;
}

























