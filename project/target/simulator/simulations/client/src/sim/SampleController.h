#ifndef SAMPLE_CONTROLLER_H
#define SAMPLE_CONTROLLER_H

#include <Attitude.h>
#include "DynamicModel.h"

enum controlMode{ VELOCITY_MODE,POSITION_MODE,ATTITUDE_MODE };

class SampleController {
private:

	float x_r;
	float y_r;
	float z_r;
	float psi_r;
	float v_x_r;
	float v_y_r;
	float v_z_r;
	float phi_r;
	float theta_r;
	float thrust;
	float previous_error_phi;
	float sum_error_phi;
	float previous_error_theta;
	float sum_error_theta;
	float sum_error_thrust;
	long long lastTimeStamp;
	static long long startupTime;
	long long timeStampStartMoving;
	bool flagTimeIsSet;
	enum controlMode controllerMode;
public:
	void setReference(float x, float y, float z, float heading);
	mav::Attitude controllerRun(mav::Kinematics kin);
	static long long getCurrentTimeMillis();
	void setTimeStampStartMoving() { timeStampStartMoving = getCurrentTimeMillis(); };
	SampleController();
	void setVelocity(float v_x, float v_y, float v_z, float psi);
	void setAttitude(float phi_r, float theta_r, float psi_r, float z_r);

};
#endif