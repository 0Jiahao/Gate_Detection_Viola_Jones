#pragma once
#include <string>
#include "Frame.h"
#include "Pose.h"

#include <Eigen/Dense>


using namespace Eigen;
namespace mav{


class Pose
{
private:
    Vector3d pos;
public:
	~Pose() = default;
    Pose();
    Pose(Vector3d pos, Vector3d att);
    Pose(float x, float y, float z, float phi, float theta, float psi);
    Pose(Vector3d pos);
	double getX(){return this->pos(0);};
	double getY(){return this->pos(1);};
	double getZ(){return this->pos(2);};
	Vector3d getPos(){return pos;};
	std::string toString();

};

}