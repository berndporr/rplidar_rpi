#include "a1lidarrpi.h"
#include <math.h>


void A1Lidar::stop() {
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

void A1Lidar::start(const char *serial_port, 
		 const unsigned rpm) {
	if (nullptr != worker) return;

	if (doInit) {
		int cfg = gpioCfgGetInternals();
		cfg |= PI_CFG_NOSIGHANDLER;
		gpioCfgSetInternals(cfg);
		if (gpioInitialise() < 0) {
			throw "gpioInitialise() failed";
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
		throw "Fatal GPIO error: Could not get the PWM range.";

	}
	maxPWM = pwmRange / 2;

	updateMotorPWM(maxPWM / (7.0/4.0));

	// create the driver instance
	drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
	if (!drv) {
		throw "insufficent memory";
	}
    
	// make connection...
        if (IS_OK(drv->connect(serial_port, 115200))) {
		rplidar_response_device_info_t devinfo;
		u_result op_result = drv->getDeviceInfo(devinfo);
		if (!IS_OK(op_result)) {
			delete drv;
			drv = NULL;
			updateMotorPWM(0);
			throw "Devinfo is not OK";
		}
        }
	
	rplidar_response_device_health_t healthinfo;
	u_result op_result = drv->getHealth(healthinfo);
	if (IS_OK(op_result)) {
		if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
			throw "Error, rplidar internal error detected. Please reboot the device to retry.";
		}
	} else {
		throw "Error, cannot retrieve the rplidar health code.";
	}

	// start scan...
	drv->startScan(0,true,0,&scanMode);

	worker = new std::thread(A1Lidar::run,this);
}

void A1Lidar::updateMotorPWM(int _motorDrive) {
	motorDrive = _motorDrive;
	if (motorDrive > maxPWM) motorDrive = maxPWM;
	gpioPWM(GPIO_PWM,motorDrive);
}

void A1Lidar::getData() {
	size_t count = (size_t)nDistance;
	rplidar_response_measurement_node_hq_t nodes[count];
	u_result op_result = drv->grabScanDataHq(nodes, count);
	if (IS_OK(op_result)) {
		unsigned long timeNow = getTimeMS();
		if (previousTime > 0) {
			float t = (timeNow - previousTime) / 1000.0f;
			currentRPM = 1.0f/t * 60.0f;
		}
		previousTime = timeNow;
		drv->ascendScanData(nodes, count);
		for (int pos = 0; pos < (int)count ; ++pos) {
			float angle = M_PI - nodes[pos].angle_z_q14 * (90.f / 16384.f / (180.0f / M_PI));
			float dist = nodes[pos].dist_mm_q2/4000.0f;
			if (dist > 0) {
				//fprintf(stderr,"%d,phi=%f,r=%f\n",j,angle,dist);
				a1LidarData[currentBufIdx][pos].phi = angle;
				a1LidarData[currentBufIdx][pos].r = dist;
				a1LidarData[currentBufIdx][pos].x = cos(angle) * dist;
				a1LidarData[currentBufIdx][pos].y = sin(angle) * dist;
				a1LidarData[currentBufIdx][pos].signal_strength =
					nodes[pos].quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
				a1LidarData[currentBufIdx][pos].valid = true;
				dataAvailable = true;
			} else {
				a1LidarData[currentBufIdx][pos].valid = false;
			}
		}
		updateMotorPWM(
			       motorDrive +
			       (int)round((desiredRPM - currentRPM) * loopRPMgain * (float)pwmRange)
			       );
		if ( (dataAvailable) && (nullptr != dataInterface) ) {
			dataInterface->newScanAvail(currentRPM, a1LidarData[currentBufIdx]);
		}
		readoutMtx.lock();
		currentBufIdx = !currentBufIdx;
		readoutMtx.unlock();
	}
}

void A1Lidar::run(A1Lidar* a1Lidar) {
	while (a1Lidar->running) {
		a1Lidar->getData();
	}
	a1Lidar->updateMotorPWM(0);
	gpioPWM(GPIO_PWM,0);
	gpioSetMode(GPIO_PWM,PI_INPUT);
}
