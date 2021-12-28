#include "a1lidarrpi.h"

class DataInterface : public A1Lidar::DataInterface {
public:
	void newScanAvail(float,A1LidarData (&data)[A1Lidar::nDistance]) {
		for(A1LidarData &data: data) {
			if (data.valid)
				printf("%e\t%e\t%e\t%e\t%e\n",
				       data.x,
				       data.y,
				       data.r,
				       data.phi,
				       data.signal_strength);
		}
		fprintf(stderr,".");
	}
};

int main(int, char **) {
	fprintf(stderr,"Data format:"
		" x tab y tab r tab phi tab strengh tab too_close\n");
	A1Lidar lidar;
	DataInterface dataInterface;
	lidar.registerInterface(&dataInterface);
	lidar.start();
	do {
	} while (!getchar());
	fprintf(stderr,"\n");
	lidar.stop();
}
