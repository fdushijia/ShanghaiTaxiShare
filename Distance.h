#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#define PI              3.1415926
#define EARTH_RADIUS    6378.137


using namespace std;

class Distance{

private:
	static double radian(double d) {
	    return d * PI / 180.0;
    }
    
public:
    static double getDist(double lat1, double lon1, double lat2, double lon2) {
        double radLat1 = radian(lat1);
        double radLat2 = radian(lat2);
        double a = radLat1 - radLat2;
        double b = radian(lon1) - radian(lon2);
    
		double dst = 2 * asin((sqrt(pow(sin(a / 2), 2) + cos(radLat1) * cos(radLat2) * pow(sin(b / 2), 2) )));
    
		dst = dst * EARTH_RADIUS;
    
		return dst;
    }
};


