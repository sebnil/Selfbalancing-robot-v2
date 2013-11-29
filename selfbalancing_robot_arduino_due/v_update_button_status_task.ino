static void vUpdateButtonStatusTask(void *pvParameters) {
	pinMode(LED_PIN, OUTPUT);

	sCmd.addCommand("p", processCommand);  // Converts two arguments to integers and echos them back
	sCmd.addCommand("print", printCommand);
	sCmd.addCommand("set", setCommand);
	for (;;) {
		sCmd.readSerial();     // We don't do much, just process serial commands

		if (true) {
			// check buttons
			startBtn.read();
			stopBtn.read();
			calibrateBtn.read();
			if (startBtn.wasReleased()) {
				Serial.println("startBtn.wasReleased");
				anglePID.SetMode(AUTOMATIC);
				speedPID.SetMode(AUTOMATIC);
				started = true;
			}
			if (stopBtn.wasReleased()) {
				Serial.println("stopBtn.wasReleased");
				motorLeft.setSpeed(0);
				motorRight.setSpeed(0);
				anglePID.SetMode(MANUAL);
				speedPID.SetMode(MANUAL);
				started = false;
			}
			if (calibrateBtn.wasReleased()) {
				if (roll < 0)
				configuration.calibratedZeroAngle = TO_DEG(roll) + 180;
				else
				configuration.calibratedZeroAngle = TO_DEG(roll) - 180;
			}
		}

		// Turn LED on.
		digitalWrite(LED_PIN, HIGH);

		// Sleep for 50 milliseconds.
		vTaskDelay(50);

		// Turn LED off.
		digitalWrite(LED_PIN, LOW);

		// Sleep for 150 milliseconds.
		vTaskDelay(5);

		if (configuration.debugLevel > 2) {

			/*    Serial.print("y: ");
			Serial.print(TO_DEG(yaw));
			Serial.print("\tp: ");
			Serial.print(TO_DEG(pitch));
			*/
			//Serial.print("\tr: ");
			//Serial.print(TO_DEG(roll));
			/*       Serial.print("\taPIDs: ");
			Serial.print(anglePIDSetpoint);
			Serial.print("\taPIDi: ");
			Serial.print(anglePIDInput);
			Serial.print("\taPIDo: ");
			Serial.print(anglePIDOutput);
			Serial.print("\tleftP: ");
			Serial.print(leftMotorPosition);
			Serial.print("\trightP: ");
			Serial.print(rightMotorPosition);
			Serial.print("\tleftS: ");
			Serial.print(leftMotorSpeed);*/
			/*    Serial.print("\trightS: ");
			Serial.print(rightMotorSpeed);
			Serial.println();*/
			//    debugPrint("yaw", TO_DEG(yaw), 2, 7);
			//    debugPrint("pitch", TO_DEG(pitch), 2, 7);
			//    debugPrint("roll", TO_DEG(roll), 2, 7);
			//    debugPrint("sPIDs", speedPIDSetpoint, 1, 3);
			/*debugPrint("sPIDi", speedPIDInput, 2, 6);
			//    debugPrint("sPIDo", speedPIDOutput, 2, 6);
			debugPrint("aPIDs", anglePIDSetpoint, 2, 7);
			debugPrint("aPIDi", anglePIDInput, 2, 7);
			debugPrint("aPIDiF", anglePIDInputFiltered, 2, 7);
			debugPrint("aPIDo", anglePIDOutput, 2, 7);*/
			/*    debugPrint("lSpeed", leftMotorSpeed, 2, 10);
			debugPrint("rSpeed", rightMotorSpeed, 2, 10);*/


			if (configuration.speedPIDOutputDebug == 1) {
				Serial.print(speedPIDOutput);
				Serial.print(" ");
			}
			if (configuration.speedPIDInputDebug == 1) {
				Serial.print(speedPIDInput);
				Serial.print(" ");
			}
			if (configuration.speedKalmanFilterDebug == 1) {
				Serial.print(speedKalmanFiltered);
				Serial.print(" ");
			}
			if (configuration.speedMovingAvarageFilter2Debug == 1) {
				Serial.print(speedFIRFiltered);
				Serial.print(" ");
			}
			if (configuration.anglePIDSetpointDebug == 1) {
				Serial.print(anglePIDSetpoint);
				Serial.print(" ");
			}
			if (configuration.anglePIDInputDebug == 1) {
				Serial.print(anglePIDInput);
				Serial.print(" ");
			}
			if (configuration.anglePIDOutputDebug == 1) {
				Serial.print(anglePIDOutput);
				Serial.print(" ");
			}
			if (configuration.speedRawDebug == 1) {
				Serial.print(anglePIDOutput);
				Serial.print(" ");
			}
			if (configuration.rollDebug == 1) {
				Serial.print(anglePIDOutput);
				Serial.print(" ");
			}
			
			/*      Serial.print(" ");
			Serial.print(speedPIDInput);*/
			Serial.print('\r');
		}

	}
}


String debugTempString;
int8_t spaces;
void debugPrint(String name, double value, uint8_t decimals, uint8_t width) {
	//String sValue = String.toString(value);
	//int length = 10;
	Serial.print(name);
	Serial.print(": ");
	debugTempString = floatToString(value, decimals);
	spaces = width - debugTempString.length();
	if (spaces < 0)
	spaces = 0;
	/*  if (value < 0)
	spaces--;
	if (decimals > 0)
	spaces--;*/
	for (int i = 0; i<spaces;i++) {
		Serial.print(".");
	}
	Serial.print(debugTempString);
	Serial.print("  ");

	/*char  buffer[32];
	sprintf(buffer, "%10s", s);
	Serial.println(buffer);*/
}

static String floatToString(double number, uint8_t digits)
{
	String resultString = "";
	// Handle negative numbers
	if (number < 0.0)
	{
		resultString += "-";
		number = -number;
	}

	// Round correctly so that print(1.999, 2) prints as "2.00"
	double rounding = 0.5;
	for (uint8_t i=0; i<digits; ++i)
	rounding /= 10.0;

	number += rounding;

	// Extract the integer part of the number and print it
	unsigned long int_part = (unsigned long)number;
	double remainder = number - (double)int_part;
	resultString += int_part;

	// Print the decimal point, but only if there are digits beyond
	if (digits > 0)
	resultString += ".";

	// Extract digits from the remainder one at a time
	while (digits-- > 0)
	{
		remainder *= 10.0;
		int toPrint = int(remainder);
		resultString += toPrint;
		remainder -= toPrint;
	}
	return resultString;
}
