#include "a1lidarrpi.h"
#include "signal.h"

A1Lidar lidar;

bool running = true;

void sig_handler(int signo)
{
	if (signo == SIGINT) {
		running = false;
		fprintf(stderr,"\n");
	}
}


int main(int, char **) {
	signal(SIGINT, sig_handler);
	fprintf(stderr,"Press ctrl-C to stop it.\n");
	lidar.start();
	fprintf(stderr,"PWM range = %d\n",lidar.getPWMrange());
	while (running) {
		fprintf(stderr,">");
		printf("%f\n",lidar.getRPM());
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	fprintf(stderr,"Stopping the LIDAR.\n");
	lidar.stop();
	fprintf(stderr,"All shut down.\n");
}
