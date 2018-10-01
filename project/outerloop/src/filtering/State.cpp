#include <sstream>
#include "State.h"
namespace mav {
	State::State() {
		this->X = 0.0;
		this->Y = 0.0;
		this->Z = 0.0;
		this->v_z_B = 0.0;
		this->b_x = 0.0;
		this->b_y = 0.0;
		this->b_z = 0.0;

	}

	State::State(float x, float y, float z, float v_z_B, float b_x, float b_y,float b_z) {
		this->X = x;
		this->Y = y;
		this->Z = z;
		this->v_z_B = v_z_B;
		this->b_x = b_x;
		this->b_y = b_y;
		this->b_z = b_z;
	}

    std::string State::toString() {
		std::ostringstream state_str;
		state_str << "x: " << this->X << "| y: " << this->Y << " | z: " <<this->Z << std::endl;
		return state_str.str();

	}

	void State:: setStateVector(Matrix<double, 7, 1> & states)
	{
		this->X = states(0);
		this->Y = states(1);
		this->Z = states(2);
		this->v_z_B = states(3);
		this->b_x = states(4);
		this->b_y = states(5);
		this->b_z = states(6);
	}


	Matrix<double,7,1> State::getStateVector()
	{
		Matrix<double,7,1> state;
		state(0) = this->X;
		state(1) = this->Y;
		state(2) = this->Z;
		state(3) = this->v_z_B;
		state(4) = this->b_x;
		state(5) = this->b_y;
		state(6) = this->b_z;
        return state;
	}

    State::State(State const &that) {

		X=that.X;
		Y=that.Y;
		Z=that.Z;
		v_z_B = that.v_z_B;
		b_x = that.b_x;
		b_y = that.b_y;
		b_z = that.b_z;

    };
}