#pragma once
#include <string>
#include <Eigen/Dense>

using namespace Eigen;

namespace mav{
class State
{
private:
	float X;
	float Y;
	float Z;
	float v_z_B;
	float b_x;
	float b_y;
	float b_z;
public:
	State();
	~State() = default;
	State(State const &that);
	State(float x, float y, float z, float v_z_B, float b_x, float b_y,float b_z);
    void setStateVector(Matrix<double, 7, 1> & states);
    Matrix<double,7,1> getStateVector();
    float getX(){return X;};
    float getY(){return Y;};
    float getZ(){return Z;};
    float getV_z_B(){return v_z_B;};
    float getBx(){return b_x;};
    float getBy(){return b_y;};
    float getBz(){return b_z;};
	std::string toString();
};
}