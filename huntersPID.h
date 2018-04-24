

//By: Enzo    3/3/2018
//Updated 4/2/2018

#define _DISTANCETRAY 1100
#define _Kp 1
#define _Ki 0.8
#define _resetKi 20 // #Cycles we want to reset Ki

float getError()
{
	float error = 0;
	error = getGyroDegreesFloat(gyroSensor);
	return (error);
}

void ForwardPID(int distanceInDegrees, int speedPID){
	int speed = speedPID;
	int speedLeft = speedPID;
	int speedRight = speedPID;

	float Kproportional = _Kp;
	float error = 0;

	setMotorEncoderUnits(encoderDegrees);
	resetMotorEncoder(rightMotor);
	while (getMotorEncoder(rightMotor) < distanceInDegrees){
		setMotorSpeed(leftMotor, speedLeft);
		setMotorSpeed(rightMotor, speedRight);
		error = getError();

		if(error>0)
		{
			speedLeft= speed +((abs(error)/2)*Kproportional);
			speedRight= speed -((abs(error)/2)*Kproportional);
		}
		else if(error<0)
		{
			speedLeft = speed -((abs(error)/2)*Kproportional);
			speedRight = speed +((abs(error)/2)*Kproportional);
		}
		else if(error == 0)
		{
			speedLeft=speed;
			speedRight=speed;
		}

		setMotorSpeed(leftMotor, speedLeft);
		setMotorSpeed(rightMotor,speedRight);
		displayCenteredTextLine(2,"SpeedLeft: %f",speedLeft);
		displayCenteredTextLine(3,"SpeedRight: %f",speedRight);
		displayCenteredTextLine(4,"Error: %f",error);
	}
}


void sideWaysPID(int distanceInDegrees, int speedPID, int direction){
	int speed = speedPID;
	int speedLeft = speedPID / 1;
	int speedRight = speedPID / 1;
//	int resetKi = _resetKi;
	float Kproportional = _Kp;
//	float Kintegral = _Ki;
	float error = 0;
//	float sumError =0;

	if(getColorMode(MColorSensor) != colorTypeGrayscale_Reflected)
	{
		setColorMode(MColorSensor, colorTypeGrayscale_Reflected);
	}

	setMotorEncoderUnits(encoderDegrees);
	resetMotorEncoder(leftMotor);
	resetMotorEncoder(sideWays);
	if (distanceInDegrees <= 0) {
		// when distance is this way we use the colorSensor to detect and stop on the next black line

		while (getColorGrayscale(MColorSensor) > 80) {
			setMotor(sideWays,(speed * direction));
			error = getError();

			if(error>0) // Robot is deviating to the left
				{
				// proportonal only
				resetMotorEncoder(rightMotor);
				moveMotor(rightMotor, -abs(error)* Kproportional*3, degrees, -speedRight);


				}

			else if(error<0) // Robot is deviating to the right
				{
				// proportonal only
				resetMotorEncoder(leftMotor);
				moveMotor(leftMotor, abs(error)* Kproportional*3, degrees,-speedLeft);

				}
			} // end while
		} // end if

	else {
	// Loop to move
	while (abs(getMotorEncoder(sideWays)) < distanceInDegrees){




		setMotorSpeed(sideWays, (speed * direction));

		error = getError();

		if(error>0) // Robot is deviating to the left
			{

			// proportonal only

				resetMotorEncoder(rightMotor);
				moveMotor(rightMotor, - abs(error) * Kproportional*3,degrees, -speedRight);


			}
		else if(error<0) // Robot is deviating to the right
			{
			// proportonal only
				resetMotorEncoder(leftMotor);
				moveMotor(leftMotor, abs(error)* Kproportional*3, degrees, -speedLeft);

		}

	 } // end of While

  } // end of else

		setMotorSpeed(leftMotor, 0);
		setMotorSpeed(rightMotor, 0);
		setMotorSpeed(sideWays, 0);
		stopMultipleMotors(leftMotor, rightMotor,sideWays);

}

// turn to the left with PID
void TurnRobotL(int Degree, int Speed){
	float totalTicks= Degree*5.38*0.375;
	int error;
	float LSpeed;

	resetGyro(gyroSensor);
	resetMotorEncoder(rightMotor);
	resetMotorEncoder(leftMotor);

	setMotorSpeed(leftMotor, 0);

	setMotorEncoderUnits(encoderDegrees);
	while(getMotorEncoder(rightMotor)<= totalTicks){

		error= totalTicks-getMotorEncoder(rightMotor);
		LSpeed=(error/totalTicks)*Speed;

		if (LSpeed < 10) LSpeed = 10;

		setMotorSpeed(rightMotor, LSpeed);

		wait(50, milliseconds);
	}
	stopMotor(rightMotor);
	stopMotor(leftMotor);
}

// moving sideways left with PID
void MoveSidewaysL(int inchesToTravel, int setspeed){
	int setDist =  46 * inchesToTravel;
	float kp=0.5;
	float speed= setspeed*kp;

	resetGyro(gyroSensor);
	setMotorEncoderUnits(encoderDegrees);
	resetMotorEncoder(sideWays);
	resetMotorEncoder(rightMotor);
	resetMotorEncoder(leftMotor);

	while(getMotorEncoder(sideWays)>= -setDist){

		setMotor(sideWays, -setspeed);

		//in case the robot starts to deviate a little from the alignment
		if(getGyroDegrees(gyroSensor)<0){
			setMotorSpeed(rightMotor, speed);
			setMotorSpeed(leftMotor, 0);
		}
		else{
			setMotorSpeed(leftMotor, speed);
			setMotorSpeed(rightMotor, 0);
		}

		//in case the robot starts to travel forward or backward
		if(getMotorEncoder(rightMotor)<0 || getMotorEncoder(leftMotor)<0){
			setMotorSpeed(rightMotor, speed);
			setMotorSpeed(leftMotor, speed);
		}
		else{
			setMotorSpeed(leftMotor, -speed);
			setMotorSpeed(rightMotor, -speed);
		}
	}
}


// move sideways right with PID
void MoveSidewaysR(float inchesToTravel, int setspeed){
	int setDist =  46 * inchesToTravel;
	float kp=0.5;
	float speed= setspeed*kp;

	//	datalogClear();
	//	datalogRate(50, milliseconds);
	//	datalogStart();

	resetGyro(gyroSensor);
	setMotorEncoderUnits(encoderDegrees);
	resetMotorEncoder(sideWays);
	resetMotorEncoder(rightMotor);
	resetMotorEncoder(leftMotor);

	while(getMotorEncoder(sideWays)<= setDist){

		setMotor(sideWays, setspeed);

		//in case the robot starts to deviate a little from the alignment
		if(getGyroDegrees(gyroSensor)<0){
			setMotorSpeed(rightMotor, speed);
			setMotorSpeed(leftMotor, 0);
		}
		else{
			setMotorSpeed(leftMotor, speed);
			setMotorSpeed(rightMotor, 0);
		}

		//in case the robot starts to travel forward or backward
		if(getMotorEncoder(rightMotor)<0 || getMotorEncoder(leftMotor)<0){
			setMotorSpeed(rightMotor, speed);
			setMotorSpeed(leftMotor, speed);
		}
		else{
			setMotorSpeed(leftMotor, -speed);
			setMotorSpeed(rightMotor, -speed);
		}
	}
}



// turn to the right with PID
void TurnRobotR(int Degree, int Speed){
	float totalTicks= Degree*5.38*0.375;
	int error;
	float RSpeed;

	resetGyro(gyroSensor);
	resetMotorEncoder(rightMotor);
	resetMotorEncoder(leftMotor);

	setMotorSpeed(rightMotor, 0);

	setMotorEncoderUnits(encoderDegrees);
	while(getMotorEncoder(leftMotor)<= totalTicks){

		error= totalTicks-getMotorEncoder(leftMotor);
		RSpeed=(error/totalTicks)*Speed;

		if (RSpeed < 10) RSpeed = 10;

		setMotorSpeed(leftMotor, RSpeed);

		wait(50, milliseconds);
	}
	stopMotor(rightMotor);
	stopMotor(leftMotor);

	setMotorEncoderUnits(encoderDegrees);
}

// moving backwards with PID
void MoveRobotB (float distToTravel, int speed){
	int setpoint= 0;
	float kp= 0.5;
	int error;
	float PValue;
	int setDist =  14.55 * distToTravel; // encoder value for distance to travel

	resetGyro(gyroSensor);
	resetMotorEncoder(sideWays);
	resetMotorEncoder(rightMotor);
	resetMotorEncoder(leftMotor);

	setMotorEncoderUnits(encoderDegrees);
	while(getMotorEncoder(rightMotor)>-setDist || getMotorEncoder(leftMotor)>-setDist){

		error= setpoint-(getGyroDegrees(gyroSensor));


		displayCenteredTextLine(2,"degrees: %d", getGyroDegrees(gyroSensor));

		PValue= error*kp;

		setMotorSpeed(rightMotor, -1*(speed-PValue));
		setMotorSpeed(leftMotor, -1*(speed+PValue));
		wait(10, milliseconds);
	}
	stopAllMotors();
}


// moving front with PID
void MoveRobotF (float distToTravel, int speed){
	int setpoint= 0;
	float kp= 0.5;
	int error;
	float PValue;
	int setDist =  14.55 * distToTravel; // encoder value for distance to travel

	resetGyro(gyroSensor);
	resetMotorEncoder(sideWays);
	resetMotorEncoder(rightMotor);
	resetMotorEncoder(leftMotor);

	setMotorEncoderUnits(encoderDegrees);
	while(getMotorEncoder(rightMotor)<setDist || getMotorEncoder(leftMotor)<setDist){

		error= setpoint-(getGyroDegrees(gyroSensor));

		displayCenteredTextLine(2,"degrees: %d", getGyroDegrees(gyroSensor));

		PValue= error*kp;

		setMotorSpeed(rightMotor, speed+PValue);
		setMotorSpeed(leftMotor, speed-PValue);
		wait(10, milliseconds);
	}
	stopAllMotors();
}
