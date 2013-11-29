
static void vReadIMUTask(void *pvParameters) {
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
		//read_sensors();
		/*    Serial.println("hej");
		vTaskDelay(2000);*/

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
			if (false)  {
				if (roll < 0) {
					angleKalmanFilter.correct((TO_DEG(roll) + 180)-configuration.calibratedZeroAngle);
				}
				else {
					angleKalmanFilter.correct((TO_DEG(roll) - 180)-configuration.calibratedZeroAngle);
				}
				anglePIDInput = angleKalmanFilter.getState();
			}
			else {
				if (roll < 0) {
					anglePIDInput = (TO_DEG(roll) + 180)-configuration.calibratedZeroAngle;
				}
				else {
					anglePIDInput =(TO_DEG(roll) - 180)-configuration.calibratedZeroAngle;
				}
			}
		}
	}
}
