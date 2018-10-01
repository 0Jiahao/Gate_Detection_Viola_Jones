#include <sstream>
#include <cmath>
#include "MotionRead.h"
namespace mav{
MotionRead::MotionRead(	float phi, float theta, float psi,float a_x, float a_y, float a_z, float p,
                           float q, float r, float z){
	this->phi = phi;
	this->theta= theta;
	this->psi= psi;
	this->a_x= a_x;
	this->a_y= a_y;
	this->a_z= a_z;
	this->p= p;
	this->q= q;
	this->r= r;
	this->z = z;
}

MotionRead::MotionRead(const MotionRead &that){
	this->phi = that.phi;
	this->theta= that.theta;
	this->psi= that.psi;
	this->a_x= that.a_x;
	this->a_y= that.a_y;
	this->a_z= that.a_z;
	this->p= that.p;
	this->q= that.q;
	this->r= that.r;
}


std::string MotionRead::toString() {
	std::ostringstream str;
	str << "| AccX: " << this->a_x << "| AccY: " << this->a_y << " | AccZ: " <<this->a_z << 
		  " | P: " << this->p << " | Q: " << this->q << " | R: " << this->r <<
		  " | Phi: " << this->phi << " | Theta: " << this->theta << " | Psi: " << this->psi <<
          " | Z:  " << z
		<< std::endl;
	std::string out(str.str());
	return out;
}

float MotionRead::rad2degree(float rad) {
	return static_cast<float>(rad * 180 / M_PI);
}

float MotionRead::getPhiDegree() {
	return MotionRead::rad2degree(phi);
}
float MotionRead::getThetaDegree() {
	return MotionRead::rad2degree(theta);

}
float MotionRead::getPsiDegree() {
	return MotionRead::rad2degree(psi);
}

bool MotionRead::hasNaN() {
    return (std::isnan(phi) || std::isnan(theta) || std::isnan(psi) ||
            std::isnan(a_x) || std::isnan(a_y) || std::isnan(a_z) ||
            std::isnan(p) || std::isnan(q) || std::isnan(r));
}

}