#include "lidarrpi.h"

int main(int, char **) {
	Xv11 xv11;
	xv11.start();
	fprintf(stderr,"PWM range = %d\n",xv11.getPWMrange());
	for(int i = 0; i<200; i++) {
		printf("%f %f\n",xv11.getRPM(),xv11.getSamplingRate());
		fprintf(stderr,">");
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	xv11.stop();
	fprintf(stderr,"\n");
}
