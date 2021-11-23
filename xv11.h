/**
 * Copyright (C) 2021 by Bernd Porr
 * Copyright (C) 2020 by Dmitry V. Sokolov
 * Apache License 2.0
 **/

#ifndef XV11_H
#define XV11_H

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <pigpio.h>
#include <thread>
#include <mutex>



/**
 * One of the 360 distance datapoints
 **/
class XV11Data {
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
	 * Flag if the object is too close.
	 **/
	bool too_close = false;

	/**
	 * Flag if the reading is valid
	 **/
	bool valid = false;
};


/**
 * Class to continously acquire data from the LIDAR
 **/
class Xv11 {
public:
	/**
	 * Number of distance readings during one 360 degree
	 * rotation. So we get one reading per degree.
	 **/
	static const unsigned nDistance = 360;

	/**
	 * Starts the data acquisition by spinning up the
	 * motor and then saving the data in the current
	 * buffer and providing the data via the callback.
	 * The default RPM of the motor is 250 but can be
	 * roughly betweeen 200 and 300. The serial port
	 * is the one which talks to the XV11.
	 **/
	void start(const char *serial_port = "/dev/serial0",
		   const unsigned rpm = 250);
		   
	/**
	 * Stops the data acquisition
	 **/
	void stop();

	Xv11(bool _doInit = true) {
		doInit = _doInit;
	}

	/**
	 * Destructor which stops the motor and the data acquisition thread.
	 **/
	~Xv11() {
		stop();
		gpioTerminate();
	}

	/**
	 * Callback interface which needs to be implemented by the user.
	 **/
	struct DataInterface {
		virtual void newScanAvail(float rpm, XV11Data (&)[Xv11::nDistance]) = 0;
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
	inline XV11Data (&getCurrentData())[nDistance]  {
		readoutMtx.lock();
		return xv11data[!currentBufIdx];
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
	static const int GPIO_PWM = 18;
	int maxPWM = 1;
	static const int pwm_frequency = 50;
	float desiredRPM = 250;
	const float loopRPMgain = 0.00005f;
	static const int nPackets = 90;
	DataInterface* dataInterface = nullptr;
	float rpm(unsigned char *packet);
	bool verify_packet_checksum(unsigned char *packet);
	unsigned count_errors(unsigned char *buf);
	bool invalid_data_flag(unsigned char *data);
	bool strength_warning_flag(unsigned char *data);
	unsigned dist_mm(unsigned char *data);
	unsigned signal_strength(unsigned char *data);
	void raw2data(unsigned char *buf);
	void updateMotorPWM(int newMotorDrive);
	static void run(Xv11* xv11);
	int tty_fd = 0;
	bool running = true;
        int motorDrive = 50;
	XV11Data xv11data[2][nDistance];
	std::thread* worker = nullptr;
	float currentRPM = 0;
	unsigned currentBufIdx = 0;
	std::mutex readoutMtx;
	int pwmRange = -1;
	bool doInit = true;
};

#endif
