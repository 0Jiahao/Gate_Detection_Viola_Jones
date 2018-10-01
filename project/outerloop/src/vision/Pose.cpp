#include <iostream>
#include "Pose.h"
using namespace std;
namespace mav{

std::string Pose::toString()
{
	std::ostringstream pose_str;
	pose_str << "X:" << pos[0] << " Y:" << pos[1] << " Z:" << pos[2];
	return pose_str.str();
}




Pose::Pose()
{
	this->pos = {0,0,0};
}

Pose::Pose(Vector3d pos) {
	this->pos = pos;
}

Pose::Pose(float x, float y, float z, float phi, float theta, float psi) {
	this->pos[0] = x;
	this->pos[1] = y;
	this->pos[2] = z;

}


}