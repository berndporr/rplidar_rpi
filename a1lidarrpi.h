/**
 * Copyright (C) 2021 by Bernd Porr
 **/

#ifndef A1LIDARRPI_H
#define A1LIDARRPI_H

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <pigpio.h>
#include <thread>
#include <mutex>

#include "rplidarsdk/rplidar.h"

using namespace rp::standalone::rplidar;

/**
 * One of the 8192 distance datapoints
 **/
class A1LidarData {
public:
	/**
	 * Distance in m
	 **/
	float r;
	
	/**
	 * Angle in rad with phi=0 in front of the robot
	 **/
	float phi;

	/**
	 * X position in m where positive values are in front
	 * of the robot and negative behind.
	 **/
	float x;

	/**
	* Y position in m where positive values are left in front of the
	* robot and negative right in front of the robot.
	**/
	float y;

	/**
	 * Signal strength as a value between 0 and 1.
	 **/
	float signal_strength = 0;

	/**
	 * Flag if the reading is valid
	 **/
	bool valid = false;
};


/**
 * Class to continously acquire data from the LIDAR
 **/
class A1Lidar {
public:
	/**
	 * Number of distance readings during one 360 degree
	 * rotation. So we get one reading per degree.
	 **/
	static const unsigned nDistance = 8192;

	/**
	 * Starts the data acquisition by spinning up the
	 * motor and then saving the data in the current
	 * buffer and providing the data via the callback.
	 * The default RPM of the motor is 250 but can be
	 * roughly betweeen 200 and 300. The serial port
	 * is the one which talks to the XV11.
	 **/
	void start(const char *serial_port = "/dev/serial0",
		   const unsigned rpm = 300);
		   
	/**
	 * Stops the data acquisition
	 **/
	void stop();

	A1Lidar(bool _doInit = true) {
		doInit = _doInit;
	}

	/**
	 * Destructor which stops the motor and the data acquisition thread.
	 **/
	~A1Lidar() {
		stop();
	}

	/**
	 * Callback interface which needs to be implemented by the user.
	 **/
	struct DataInterface {
		virtual void newScanAvail(float rpm, A1LidarData (&)[A1Lidar::nDistance]) = 0;
	};

	/**
	 * Register the callback interface here to receive data.
	 **/
	void registerInterface(DataInterface* di) {
		dataInterface = di;
	}

	/**
	 * Returns the current databuffer which is not being written to.
	 **/
	inline A1LidarData (&getCurrentData())[nDistance]  {
		readoutMtx.lock();
		return a1LidarData[!currentBufIdx];
		readoutMtx.unlock();
	}

	/**
	 * Returns the current RPM
	 **/
	float getRPM() { return (float)currentRPM; }

	/**
	 * Returns the actual PWM range
	 **/
	int getPWMrange() { return pwmRange; }

private:
	static unsigned long getTimeMS() {
		std::chrono::time_point<std::chrono::system_clock> now = 
			std::chrono::system_clock::now();
		auto duration = now.time_since_epoch();
		return (unsigned long)std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
	}

	unsigned long previousTime = 0;
	static const int GPIO_PWM = 18;
	int maxPWM = 1;
	static const int pwm_frequency = 50;
	float desiredRPM = 250;
	const float loopRPMgain = 0.00005f;
	static const int nPackets = 90;
	DataInterface* dataInterface = nullptr;
	float rpm(unsigned char *packet);
	void updateMotorPWM(int newMotorDrive);
	void getData();
	static void run(A1Lidar* a1Lidar);
	int tty_fd = 0;
	bool running = true;
        int motorDrive = 50;
	A1LidarData a1LidarData[2][nDistance];
	std::thread* worker = nullptr;
	float currentRPM = 0;
	std::mutex readoutMtx;
	int pwmRange = -1;
	bool doInit = true;
	bool dataAvailable = false;
	int currentBufIdx = 0;
	RPlidarDriver *drv;
	RplidarScanMode scanMode;
};

#endif
