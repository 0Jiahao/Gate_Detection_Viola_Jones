#pragma once

#include <iostream>

namespace mav{
class Attitude
{
private:

public:
	float yaw=0;
	float pitch=0;
	float roll=0;
	float altitude=0;
	Attitude() = default;
	~Attitude() = default;
	//Attitude(	double yaw, double pitch, double roll,double altitude);
	Attitude(float roll, float pitch, float yaw, float altitude);
	std::string toString();
};
}