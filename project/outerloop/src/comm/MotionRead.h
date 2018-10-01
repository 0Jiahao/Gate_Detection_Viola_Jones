#pragma once
# define M_PI           3.14159265358979323846
#include <string>

namespace mav{
class MotionRead
{
private:
    float phi=0;
    float theta=0;
    float psi=0;
    float a_x=0;
    float a_y=0;
    float a_z=0;
    float p=0;
    float q=0;
    float r=0;
    float z=0;
public:
	MotionRead() = default;
	MotionRead(	float phi, float theta, float psi,float a_x,
                   float a_y, float a_z, float p, float q, float r, float z);
	~MotionRead() = default;
	MotionRead(const MotionRead &that);
    float getPhi(){return this->phi;};
    float getPhiDegree();
    float getThetaDegree();
    float getPsiDegree();

    float getTheta(){return this->theta;};
    float getPsi(){return this->psi;};
    float getA_x(){return this->a_x;};
    float getA_y(){return this->a_y;};
    float getA_z(){return this->a_z;};
    float getP(){return this->p;};
    float getQ(){return this->q;};
    float getR(){return this->r;};
    float getZ(){return z;}
	std::string toString();
	static float rad2degree(float rad);
	bool hasNaN();
};
}