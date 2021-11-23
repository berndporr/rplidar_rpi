#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

int main (int argc, char* argv[]) {

	const int GPIO = 18;

	int v = 0;
	
	if (argc > 1) {
		v = atoi(argv[1]);
	}
  
	int pi = gpioInitialise();
	if ( pi < 0 ) {
		fprintf(stderr,"Could not init pigpio.\n");
		exit(1);
	}
  
	gpioSetMode(GPIO,PI_OUTPUT);
	
	if (v > 100) v = 100;
  
	gpioPWM(GPIO,v);

	getchar();

	gpioPWM(GPIO,0);

	gpioTerminate();
	
	return 0 ;
}
