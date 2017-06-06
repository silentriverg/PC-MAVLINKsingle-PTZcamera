#include <stdio.h>
#include <math.h>

#define pi 3.14159265358979323846
#define earthRadiusKm 6371.0

// degrees to radians

class CSensorDataGenerator
{
private:
	double deg2rad(double deg) {
		return (deg * pi / 180);
	}

	// radians to degree
	double rad2deg(double rad) {
		return (rad * 180 / pi);
	}

	// calculate two point distance (unit: km)
	double calDistance(double lat1d, double lon1d, double lat2d, double lon2d) {
		double lat1r, lon1r, lat2r, lon2r, u, v;
		lat1r = deg2rad(lat1d);
		lon1r = deg2rad(lon1d);
		lat2r = deg2rad(lat2d);
		lon2r = deg2rad(lon2d);
		u = sin((lat2r - lat1r) / 2);
		v = sin((lon2r - lon1r) / 2);
		return 2.0 * earthRadiusKm * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v));
	}

public:
	double genSensorData(double lat, double lon, double lat_src, double lon_src) {
		double strength_src, d, rssi;
		// decay constant
		float alpha = 0.05;
		// signal source strength
		strength_src = 1000;
		d = calDistance(lat, lon, lat_src, lon_src);
		rssi = strength_src * exp(-alpha * d);
		return rssi;
	}
};
