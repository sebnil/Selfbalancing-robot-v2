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
	String c = "config";
	char *arg = sCmd.next();
	if (c.equals(arg)) {
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
		Serial.print(";\nconfiguration.debugLevel = ");
		Serial.print(configuration.debugLevel);
		Serial.print(";");
	}
	else {
		Serial.println("done");
	}
}

String spp = "speedPIDKp"; // speedPIDKp
String spi = "speedPIDKi"; // speedPIDKi
String spd = "speedPIDKd"; // speedPIDKd
String spoll = "speedPIDOutputLowerLimit"; //speedPIDOutputLowerLimit
String spohl = "speedPIDOutputHigherLimit"; //speedPIDOutputHigherLimit
String apap = "anglePIDAggKp"; // anglePIDAggKp
String apai = "anglePIDAggKi"; // anglePIDAggKi
String apad = "anglePIDAggKd"; // anglePIDAggKd
String apcp = "anglePIDConKp"; // anglePIDConKp
String apci = "anglePIDConKi"; // anglePIDConKi
String apcd = "anglePIDConKd"; // anglePIDConKd
String apll = "anglePIDLowerLimit"; // anglePIDLowerLimit
String aps = "anglePIDSampling"; // anglePIDSampling
String sps = "speedPIDSampling"; // speedPIDSampling
String ass = "angleSensorSampling"; // angleSensorSampling
String msss = "motorSpeedSensorSampling"; // motorSpeedSensorSampling
String dl = "debugLevel"; // debugLevel
String speedFilterCmd = "speedFilter"; //
void setCommand() {
	char *arg = sCmd.next();
	char *value = sCmd.next();


	if (*value != NULL) {
		if (spp.equals(arg)) {
			configuration.speedPIDKp = atof(value);
			Serial.print("speedPIDKp: ");
			Serial.println(configuration.speedPIDKp);
		}
		else if (spi.equals(arg)) {
			configuration.speedPIDKi = atof(value);
			Serial.print("speedPIDKi: ");
			Serial.println(configuration.speedPIDKi);
		}
		else if (spd.equals(arg)) {
			configuration.speedPIDKd = atof(value);
			Serial.print("speedPIDKd: ");
			Serial.println(configuration.speedPIDKd);
		}

		else if (spoll.equals(arg)) {
			configuration.speedPIDOutputLowerLimit = atof(value);
			Serial.print("speedPIDOutputLowerLimit: ");
			Serial.println(configuration.speedPIDOutputLowerLimit);
		}
		else if (spohl.equals(arg)) {
			configuration.speedPIDOutputHigherLimit = atof(value);
			Serial.print("speedPIDOutputHigherLimit: ");
			Serial.println(configuration.speedPIDOutputHigherLimit);
		}


		else if (apap.equals(arg)) {
			configuration.anglePIDAggKp = atof(value);
			Serial.print("anglePIDAggKp: ");
			Serial.println(configuration.anglePIDAggKp);
		}
		else if (apai.equals(arg)) {
			configuration.anglePIDAggKi = atof(value);
			Serial.print("anglePIDAggKi: ");
			Serial.println(configuration.anglePIDAggKi);
		}
		else if (apad.equals(arg)) {
			configuration.anglePIDAggKd = atof(value);
			Serial.print("anglePIDAggKd: ");
			Serial.println(configuration.anglePIDAggKd);
		}

		else if (apcp.equals(arg)) {
			configuration.anglePIDConKp = atof(value);
			Serial.print("anglePIDConKp: ");
			Serial.println(configuration.anglePIDConKp);
		}
		else if (apci.equals(arg)) {
			configuration.anglePIDConKi = atof(value);
			Serial.print("anglePIDConKi: ");
			Serial.println(configuration.anglePIDConKi);
		}
		else if (apcd.equals(arg)) {
			configuration.anglePIDConKd = atof(value);
			Serial.print("anglePIDConKd: ");
			Serial.println(configuration.anglePIDConKd);
		}
		else if (apll.equals(arg)) {
			configuration.anglePIDLowerLimit = atof(value);
			Serial.print("anglePIDLowerLimit: ");
			Serial.println(configuration.anglePIDLowerLimit);
		}
		else if (aps.equals(arg)) {
			configuration.anglePIDLowerLimit = atof(value);
			Serial.print("anglePIDLowerLimit: ");
			Serial.println(configuration.anglePIDLowerLimit);
		}
		else if (sps.equals(arg)) {
			configuration.speedPIDSampling = atof(value);
			Serial.print("speedPIDSampling: ");
			Serial.println(configuration.speedPIDSampling);
		}
		else if (String("angleKalmanFilterR").equals(arg)) {
			configuration.angleKalmanFilterR = atof(value);	
		}	
		else if (ass.equals(arg)) {
			configuration.angleSensorSampling = atof(value);
			Serial.print("angleSensorSampling: ");
			Serial.println(configuration.angleSensorSampling);
		}
		else if (msss.equals(arg)) {
			configuration.motorSpeedSensorSampling = atof(value);
			Serial.print("motorSpeedSensorSampling: ");
			Serial.println(configuration.motorSpeedSensorSampling);
		}
		else if (String("speedKalmanFilterR").equals(arg)) {
			configuration.speedKalmanFilterR = atof(value);
			Serial.print("speedKalmanFilterR: ");
			Serial.println(configuration.speedKalmanFilterR);
		}
		else if (String("speedPIDOutputDebug").equals(arg)) {
			configuration.speedPIDOutputDebug = atoi(value);
		}
		else if (String("speedPIDInputDebug").equals(arg)) {
			configuration.speedPIDInputDebug= atoi(value);
		}
		else if (String("speedKalmanFilterDebug").equals(arg)) {
			configuration.speedKalmanFilterDebug= atoi(value);
		}
		else if (String("speedMovingAvarageFilter2Debug").equals(arg)) {
			configuration.speedMovingAvarageFilter2Debug= atoi(value);
		}
		else if (String("anglePIDSetpointDebug").equals(arg)) {
			configuration.anglePIDSetpointDebug= atoi(value);
		}
		else if (String("anglePIDInputDebug").equals(arg)) {
			configuration.anglePIDInputDebug = atoi(value);
		}
		else if (String("anglePIDOutputDebug").equals(arg)) {
			configuration.anglePIDOutputDebug = atoi(value);
		}
		else if (dl.equals(arg)) {
			configuration.debugLevel = atoi(value);
			Serial.print("debugLevel: ");
			Serial.println(configuration.debugLevel);
		}
		else if (speedFilterCmd.equals(arg)) {
			//float coeff[40];
			uint8_t valueInt = atoi(value);

			float coeff[40] = {
			1, 1, 1      };
			int filterTaps = 3;

			switch (valueInt) {
				case 1:
				coeff = {
				1        };
				filterTaps = 1;
				break;
				case 2:
				coeff = {
				1, 1        };
				filterTaps = 2;
				case 3:
				coeff = {
				1, 1, 1        };
				filterTaps = 3;
				break;
				case 10:
				coeff = {
				1, 1, 1,1,1,1,1,1,1,1        };
				filterTaps = 10;
				break;
				case 20:
				coeff = {
				1, 1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1        };
				filterTaps = 20;
				break;
			}

			speedMovingAvarageFilter2.setNumberOfTaps(filterTaps);
			speedMovingAvarageFilter2.setCoefficients(coeff);
			Serial.print("speedFilter: ");
			Serial.print(valueInt);
			Serial.print(" ");
			Serial.print(sizeof(coeff)/sizeof(float));
			Serial.println();
		}
		newConfig();
	}
}










