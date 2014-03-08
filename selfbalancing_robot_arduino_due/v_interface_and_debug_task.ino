static void vInterfaceAndDebugTask(void *pvParameters) {
	pinMode(LED_PIN, OUTPUT);

	// Initialize serial commands. They are located in serial_commands.ino
	sCmd.addCommand("print", printCommand);
	sCmd.addCommand("set", setCommand);
	
	for (;;) {
		// Process serial commands
		sCmd.readSerial();

		// Check button states
		startBtn.read();
		stopBtn.read();
		calibrateBtn.read();
		
		// Start the robot if start button is released
		if (startBtn.wasReleased()) {
			Serial.println("startBtn.wasReleased");
			anglePID.SetMode(AUTOMATIC);
			speedPID.SetMode(AUTOMATIC);
			started = true;
		}
		/* turn off the outer loop and only use the inner loop. 
		   This makes it possible to by hand move the robot by hand */
		if (startBtn.isPressed()) {
			speedPID.SetMode(MANUAL);
			speedPIDOutput = 0;
		}
		// Stop the robot when stop button is released
		if (stopBtn.wasReleased()) {
			Serial.println("stopBtn.wasReleased");
			motorLeft.setSpeed(0);
			motorRight.setSpeed(0);
			anglePID.SetMode(MANUAL);
			speedPID.SetMode(MANUAL);
			started = false;
		}
		// Set a new calibrated zero angle when calibrate button is released
		if (calibrateBtn.wasReleased()) {
			if (roll < 0)
			configuration.calibratedZeroAngle = TO_DEG(roll) + 180;
			else
			configuration.calibratedZeroAngle = TO_DEG(roll) - 180;
		}

		// blink the led and delay debug output for x ms
		digitalWrite(LED_PIN, HIGH);
		vTaskDelay((uint8_t)configuration.debugSampleRate/2);
		digitalWrite(LED_PIN, LOW);
		vTaskDelay((uint8_t)configuration.debugSampleRate/2);

		if (configuration.debugLevel > 0) {
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
				Serial.print(rightMotorSpeed+leftMotorSpeed);
				Serial.print(" ");
			}
			if (configuration.angleRawDebug == 1) {
				Serial.print(angleRaw);
				Serial.print(" ");
			}
			if (configuration.activePIDTuningDebug == 1) {
				if (activePIDTuning)
					Serial.print("1");
				else
					Serial.print("0");
				Serial.print(" ");
			}
			
			Serial.print('\r');
		}

	}
}

