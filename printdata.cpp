#include "xv11.h"

class DataInterface : public Xv11::DataInterface {
public:
	void newScanAvail(float,XV11Data (&data)[Xv11::nDistance]) {
		for(XV11Data &data: data) {
			if (data.valid)
				printf("%f\t%f\t%f\t%f\t%f\t%d\n",
				       data.x,
				       data.y,
				       data.r,
				       data.phi,
				       data.signal_strength,
				       (int)data.too_close);
		}
		fprintf(stderr,".");
	}
};

int main(int, char **) {
	fprintf(stderr,"Data format:"
		" x tab y tab r tab phi tab strengh tab too_close\n");
	Xv11 xv11;
	DataInterface dataInterface;
	xv11.registerInterface(&dataInterface);
	xv11.start();
	do {
	} while (!getchar());
	fprintf(stderr,"\n");
	xv11.stop();
}
