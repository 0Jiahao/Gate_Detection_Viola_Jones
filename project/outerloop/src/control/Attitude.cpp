#include <sstream>
#include "Attitude.h"
namespace mav{

Attitude::Attitude(float roll, float pitch, float yaw, float altitude) {
	this->yaw = yaw;
	this->pitch = pitch;
	this->roll = roll;
	this->altitude = altitude;
}

std::string Attitude::toString() {
    std::ostringstream ss;
	ss << " | Yaw: " << yaw << " | Pitch: " << pitch << " | Roll: " << roll << " | Z: " << altitude << std::endl;
	return ss.str();
}

}

