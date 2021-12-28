#include "lidarrpi.h"
#include "signal.h"

Xv11 xv11;

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
	xv11.start();
	fprintf(stderr,"PWM range = %d\n",xv11.getPWMrange());
	while (running) {
		fprintf(stderr,">");
		printf("%f\n",xv11.getRPM());
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	fprintf(stderr,"Stopping the LIDAR.\n");
	xv11.stop();
	fprintf(stderr,"All shut down.\n");
}
