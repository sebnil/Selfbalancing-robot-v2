uint32_t controlTaskTimestamp = 0;
static void vControlTask(void *pvParameters) {
	newConfig();

	for (;;) {


		// Time again?
		if((millis() - controlTaskTimestamp) >= 10) {
			controlTaskTimestamp = millis();

			// speed pid. input is wheel speed. output is angleSetpoint
			speedPID.Compute();
			anglePIDSetpoint = -speedPIDOutput;


			//anglePIDInput = anglePIDInput - configuration.calibratedZeroAngle;
			//Serial.println(anglePIDInput);

			// update angle pid tuning
			if(abs(anglePIDInput) < configuration.anglePIDLowerLimit && configuration.anglePIDLowerLimit != 0) {
				//we're close to setpoint, use conservative tuning parameters
				anglePID.SetTunings(configuration.anglePIDConKp, configuration.anglePIDConKi, configuration.anglePIDConKd);
			}
			else if (abs(anglePIDInput) < 45) {
				//we're far from setpoint, use aggressive tuning parameters
				anglePID.SetTunings(configuration.anglePIDAggKp, configuration.anglePIDAggKi, configuration.anglePIDAggKd);
			}
			else {
				// fell down
				started = false;
			}
			anglePID.Compute();
			if (started) {
				motorLeft.setSpeedPercentage(anglePIDOutput);
				motorRight.setSpeedPercentage(anglePIDOutput);
			}
			else {
				motorLeft.setSpeed(0);
				motorRight.setSpeed(0);
			}
		}
	}
}

