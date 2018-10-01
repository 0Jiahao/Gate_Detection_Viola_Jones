#ifndef KINEMATICS_H
#define KINEMATICS_H
namespace mav {
	typedef struct {
		float x;
		float y;
		float z;
		float phi;
		float theta;
		float psi;
		float p;
		float q;
		float r;
		float accX;
		float accY;
		float accZ;
		float vX;
		float vY;
		float vZ;
	}Kinematics;
}

#endif