
enum PIDTuning {CONSERVATIVE, AGGRESSIVE};
PIDTuning activePIDTuning = CONSERVATIVE;

//uint32_t controlTaskTimestamp = 0;
static void vControlTask(void *pvParameters) {
	newConfig();
	
	// Init sensors
	vTaskDelay(50);  // Give sensors enough time to start
	I2C_Init();
	Accel_Init();
	Magn_Init();
	Gyro_Init();
	// Read sensors, init DCM algorithm
	vTaskDelay(20);  // Give sensors enough time to collect data
	reset_sensor_fusion();
	Serial.println("imu init done");

	for (;;) {
		// Time to read the sensors again?
		if((millis() - timestamp) >= configuration.angleSensorSampling) {
			timestamp_old = timestamp;
			timestamp = millis();
			if (timestamp > timestamp_old)
			G_Dt = (float) (timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
			else G_Dt = 0;

			// Update sensor readings
			read_sensors();

			// Apply sensor calibration
			compensate_sensor_errors();

			// Run DCM algorithm
			Compass_Heading(); // Calculate magnetic heading
			Matrix_update();
			Normalize();
			Drift_correction();
			Euler_angles();

			// update speeed
			rightMotorSpeed = (float) (rightMotorPosition - lastRightMotorPosition);
			lastRightMotorPosition = rightMotorPosition;
			leftMotorSpeed = (float) (leftMotorPosition - lastLeftMotorPosition);
			lastLeftMotorPosition = leftMotorPosition;

			//speedPIDInput = (rightMotorSpeed+leftMotorSpeed);

			speedKalmanFilter.correct(rightMotorSpeed+leftMotorSpeed);
			speedKalmanFiltered = speedKalmanFilter.getState();
			speedPIDInput = speedKalmanFiltered;
			//speedFIRFiltered = speedMovingAvarageFilter2.process(rightMotorSpeed+leftMotorSpeed);
			
			
			// move angle to around equilibrium
			if (roll < 0) {
				angleRaw = (TO_DEG(roll) + 180)-configuration.calibratedZeroAngle;
			}
			else {
				angleRaw =(TO_DEG(roll) - 180)-configuration.calibratedZeroAngle;
			}
			if (true)  {
				angleKalmanFilter.correct(angleRaw);
				anglePIDInput = angleKalmanFilter.getState();
			}
			else {
				anglePIDInput = angleRaw;
			}
		}

		// Time again?
		/*if((millis() - controlTaskTimestamp) >= 10) {
		controlTaskTimestamp = millis();*/

		// speed pid. input is wheel speed. output is angleSetpoint
		speedPID.Compute();
		anglePIDSetpoint = -speedPIDOutput;

		// update angle pid tuning. only update if different from current tuning
		if(activePIDTuning == AGGRESSIVE && abs(anglePIDInput) < configuration.anglePIDLowerLimit && configuration.anglePIDLowerLimit != 0) {
			//we're close to setpoint, use conservative tuning parameters
			activePIDTuning = CONSERVATIVE;
			anglePID.SetTunings(configuration.anglePIDConKp, configuration.anglePIDConKi, configuration.anglePIDConKd);
		}
		else if (activePIDTuning == CONSERVATIVE && abs(anglePIDInput) < 45) {
			//we're far from setpoint, use aggressive tuning parameters
			activePIDTuning = AGGRESSIVE;
			anglePID.SetTunings(configuration.anglePIDAggKp, configuration.anglePIDAggKi, configuration.anglePIDAggKd);
		}
		else if (abs(anglePIDInput > 45)){
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
		//}
	}
}

