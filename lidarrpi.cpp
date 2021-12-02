#include "lidarrpi.h"
#include <math.h>


void Xv11::stop() {
	running = false;
	if (nullptr != worker) {
		worker->join();
		delete worker;
		worker = nullptr;
	}
	if (doInit) {
		gpioTerminate();
	}
}

void Xv11::start(const char *serial_port, 
		 const unsigned rpm) {
	if (nullptr != worker) return;

	if (doInit) {
		int cfg = gpioCfgGetInternals();
		cfg |= PI_CFG_NOSIGHANDLER;
		gpioCfgSetInternals(cfg);
		if (gpioInitialise() < 0) {
			throw "gpioInitialise failed";
		}
	}

	desiredRPM = (float)rpm;

	// init PWM
	gpioSetMode(GPIO_PWM,PI_OUTPUT);
	gpioSetPWMfrequency(GPIO_PWM,pwm_frequency);
	int rr = gpioGetPWMrealRange(GPIO_PWM);
        if ( ( rr > 255) && (rr < 20000) ) gpioSetPWMrange(GPIO_PWM, rr);
	pwmRange = gpioGetPWMrange(GPIO_PWM);
	if ( (pwmRange == PI_BAD_USER_GPIO) || (pwmRange < 25) ) {
		stop();
		const char msg[] = "Fatal GPIO error: Could not get the PWM range.";
		throw msg;
	}
	maxPWM = pwmRange / 2;

	updateMotorPWM(maxPWM / (7.0/5.0));

	// create the driver instance
	drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
	if (!drv) {
		throw "insufficent memory";
	}
    
	rplidar_response_device_info_t devinfo;
	bool connectSuccess = false;
	// make connection...
        if (IS_OK(drv->connect(serial_port, 115200)))
        {
		u_result op_result = drv->getDeviceInfo(devinfo);
		
		if (IS_OK(op_result)) 
		{
			connectSuccess = true;
            }
		else
		{
			delete drv;
			drv = NULL;
		}
        }
	
	rplidar_response_device_health_t healthinfo;
	u_result op_result = drv->getHealth(healthinfo);
	if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
		printf("RPLidar health status : %d\n", healthinfo.status);
		if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
			throw "Error, rplidar internal error detected. Please reboot the device to retry.";
		}
	} else {
		throw "Error, cannot retrieve the lidar health code.";
	}

	drv->startMotor();
	// start scan...
	drv->startScan(0,1);

	worker = new std::thread(Xv11::run,this);
}

void Xv11::updateMotorPWM(int _motorDrive) {
	motorDrive = _motorDrive;
	if (motorDrive > maxPWM) motorDrive = maxPWM;
	gpioPWM(GPIO_PWM,motorDrive);
	//fprintf(stderr,"motorDrive = %d\n",motorDrive);
}

void Xv11::getData() {
	size_t count = 8192;
	rplidar_response_measurement_node_t nodes[count];
	u_result op_result = drv->grabScanData(nodes, count);
	if (IS_OK(op_result)) {
		drv->ascendScanData(nodes, count);
		for (int pos = 0; pos < (int)count ; ++pos) {
			float angle = (float)pos / (float)count * (float)M_PI * 2.0f - (float)M_PI;
			float dist = nodes[pos].distance_q2/4.0f;
			if (dist > 0) {
				//fprintf(stderr,"%d,phi=%f,r=%f\n",j,angle,dist);
				xv11data[currentBufIdx][pos].phi = angle;
				xv11data[currentBufIdx][pos].r = dist;
				xv11data[currentBufIdx][pos].x = cos(angle) * dist;
				xv11data[currentBufIdx][pos].y = sin(angle) * dist;
				xv11data[currentBufIdx][pos].signal_strength =
					nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
				xv11data[currentBufIdx][pos].too_close = 0;
				xv11data[currentBufIdx][pos].valid = true;
				dataAvailable = true;
			} else {
				xv11data[currentBufIdx][pos].valid = false;
			}
		}
		//		updateMotorPWM(
		//			motorDrive +
		//			(int)round((desiredRPM - currentRPM) * loopRPMgain * (float)pwmRange)
		//			);
		if ( (dataAvailable) && (nullptr != dataInterface) ) {
			dataInterface->newScanAvail(currentRPM, xv11data[currentBufIdx]);
		}
		readoutMtx.lock();
		currentBufIdx = !currentBufIdx;
		readoutMtx.unlock();
	}
}

void Xv11::run(Xv11* xv11) {
	while (xv11->running) {
		xv11->getData();
	}
	xv11->updateMotorPWM(0);
	gpioPWM(GPIO_PWM,0);
	gpioSetMode(GPIO_PWM,PI_INPUT);
}
