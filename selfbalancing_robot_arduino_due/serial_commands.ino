void processCommand() {
	int aNumber;
	char *arg;

	Serial.println("We're in processCommand");
	arg = sCmd.next();
	if (arg != NULL) {
		Serial.print("First argument was: ");
		Serial.println(arg);
	}
	else {
		Serial.println("No arguments");
	}

	arg = sCmd.next();
	if (arg != NULL) {
		Serial.print("Second argument was: ");
		Serial.println(arg);
	}
	else {
		Serial.println("No second argument");
	}
}
void printCommand() {
	char *arg = sCmd.next();
	if (String("config").equals(arg)) {
		Serial.println("Configuration");
		Serial.print("configuration.speedPIDKp = ");
		Serial.print(configuration.speedPIDKp);
		Serial.print(";\nconfiguration.speedPIDKi = ");
		Serial.print(configuration.speedPIDKi);
		Serial.print(";\nconfiguration.speedPIDKd = ");
		Serial.print(configuration.speedPIDKd);
		Serial.print(";\nconfiguration.speedPIDOutputLowerLimit = ");
		Serial.print(configuration.speedPIDOutputLowerLimit);
		Serial.print(";\nconfiguration.speedPIDOutputHigherLimit = ");
		Serial.print(configuration.speedPIDOutputHigherLimit);
		Serial.print(";\nconfiguration.anglePIDAggKp = ");
		Serial.print(configuration.anglePIDAggKp);
		Serial.print(";\nconfiguration.anglePIDAggKi = ");
		Serial.print(configuration.anglePIDAggKi);
		Serial.print(";\nconfiguration.anglePIDAggKd = ");
		Serial.print(configuration.anglePIDAggKd);
		Serial.print(";\nconfiguration.anglePIDConKp = ");
		Serial.print(configuration.anglePIDConKp);
		Serial.print(";\nconfiguration.anglePIDConKi = ");
		Serial.print(configuration.anglePIDConKi);
		Serial.print(";\nconfiguration.anglePIDConKd = ");
		Serial.print(configuration.anglePIDConKd);
		Serial.print(";\nconfiguration.anglePIDLowerLimit = ");
		Serial.print(configuration.anglePIDLowerLimit);
		Serial.print(";\nconfiguration.calibratedZeroAngle = ");
		Serial.print(configuration.calibratedZeroAngle);
		Serial.print(";\nconfiguration.anglePIDSampling = ");
		Serial.print(configuration.anglePIDSampling);
		Serial.print(";\nconfiguration.speedPIDSampling = ");
		Serial.print(configuration.speedPIDSampling);
		Serial.print(";\nconfiguration.angleKalmanFilterR = ");
		Serial.print(configuration.angleKalmanFilterR);
		Serial.print(";\nconfiguration.angleSensorSampling = ");
		Serial.print(configuration.angleSensorSampling);
		Serial.print(";\nconfiguration.motorSpeedSensorSampling = ");
		Serial.print(configuration.motorSpeedSensorSampling);
		Serial.print(";\nconfiguration.speedKalmanFilterR = ");
		Serial.print(configuration.speedKalmanFilterR);
		Serial.print(";\nconfiguration.motorLeftMinimumSpeed = ");
		Serial.print(configuration.motorLeftMinimumSpeed);
		Serial.print(";\nconfiguration.motorRightMinimumSpeed = ");
		Serial.print(configuration.motorRightMinimumSpeed);
		
		/*Serial.print(";\nconfiguration.debugLevel = ");
		Serial.print(configuration.debugLevel);
		Serial.print(";\nconfiguration.debugSampleRate = ");
		Serial.print(configuration.debugSampleRate);
		Serial.print(";");*/
	}
}
		
void setCommand() {
	char *arg = sCmd.next();
	char *value = sCmd.next();
	if (*value != NULL) {
		// parameters
		if (String("speedPIDKp").equals(arg))
		configuration.speedPIDKp = atof(value);
		else if (String("speedPIDKi").equals(arg))
		configuration.speedPIDKi = atof(value);
		else if (String("speedPIDKd").equals(arg))
		configuration.speedPIDKd = atof(value);
		else if (String("speedPIDOutputLowerLimit").equals(arg))
		configuration.speedPIDOutputLowerLimit = atof(value);
		else if (String("speedPIDOutputHigherLimit").equals(arg))
		configuration.speedPIDOutputHigherLimit = atof(value);
		else if (String("anglePIDAggKp").equals(arg))
		configuration.anglePIDAggKp = atof(value);
		else if (String("anglePIDAggKi").equals(arg))
		configuration.anglePIDAggKi = atof(value);
		else if (String("anglePIDAggKd").equals(arg))
		configuration.anglePIDAggKd = atof(value);
		else if (String("anglePIDConKp").equals(arg))
		configuration.anglePIDConKp = atof(value);
		else if (String("anglePIDConKi").equals(arg))
		configuration.anglePIDConKi = atof(value);
		else if (String("anglePIDConKp").equals(arg))
		configuration.anglePIDConKd = atof(value);
		else if (String("anglePIDLowerLimit").equals(arg))
		configuration.anglePIDLowerLimit = atof(value);
		else if (String("anglePIDSampling").equals(arg))
		configuration.anglePIDLowerLimit = atof(value);
		else if (String("speedPIDSampling").equals(arg))
		configuration.speedPIDSampling = atof(value);
		else if (String("angleKalmanFilterR").equals(arg))
		configuration.angleKalmanFilterR = atof(value);
		else if (String("angleSensorSampling").equals(arg))
		configuration.angleSensorSampling = atof(value);
		else if (String("motorSpeedSensorSampling").equals(arg))
		configuration.motorSpeedSensorSampling = atof(value);
		else if (String("speedKalmanFilterR").equals(arg))
		configuration.speedKalmanFilterR = atof(value);
		else if (String("motorLeftMinimumSpeed").equals(arg))
		configuration.motorLeftMinimumSpeed = atoi(value);
		else if (String("motorRightMinimumSpeed").equals(arg))
		configuration.motorRightMinimumSpeed = atoi(value);
		
		// steering
		else if (String("steeringControl").equals(arg))
		userControl.steering = atof(value);
		else if (String("directionControl").equals(arg))
		userControl.direction = atof(value);
				
		// debug
		else if (String("speedPIDOutputDebug").equals(arg))
		configuration.speedPIDOutputDebug = atoi(value);
		else if (String("speedPIDInputDebug").equals(arg))
		configuration.speedPIDInputDebug= atoi(value);
		else if (String("speedKalmanFilterDebug").equals(arg))
		configuration.speedKalmanFilterDebug= atoi(value);
		else if (String("speedRawDebug").equals(arg))
		configuration.speedRawDebug= atoi(value);
		else if (String("speedMovingAvarageFilter2Debug").equals(arg))
		configuration.speedMovingAvarageFilter2Debug= atoi(value);
		else if (String("anglePIDSetpointDebug").equals(arg))
		configuration.anglePIDSetpointDebug= atoi(value);
		else if (String("anglePIDInputDebug").equals(arg))
		configuration.anglePIDInputDebug = atoi(value);
		else if (String("anglePIDOutputDebug").equals(arg))
		configuration.anglePIDOutputDebug = atoi(value);
		else if (String("angleRawDebug").equals(arg))
		configuration.angleRawDebug = atoi(value);
		else if (String("debugLevel").equals(arg))
		configuration.debugLevel = atoi(value);
		else if (String("debugSampleRate").equals(arg))
		configuration.debugSampleRate = atoi(value);
		newConfig();
	}
}










