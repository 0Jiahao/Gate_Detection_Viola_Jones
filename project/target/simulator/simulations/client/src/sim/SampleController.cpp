#include "SampleController.h"
#include <iostream>
#include "tools/DataLogger.h"

long long SampleController::startupTime = SampleController::getCurrentTimeMillis();


void SampleController::setReference(float x, float y, float z, float heading)
{
	controllerMode = POSITION_MODE;
	x_r = x; y_r = y; z_r = z; psi_r = heading;
	//std::cout << "[positon autopilot]" << psi_r << std::endl;
}

void SampleController::setVelocity(float v_x, float v_y, float v_z, float psi)
{
	controllerMode = VELOCITY_MODE;
	v_x_r = v_x; v_y_r = v_y; v_z_r = v_z; psi_r = psi;
	//std::cout << "[velocity autopilot]" << psi_r << std::endl;
}

void SampleController::setAttitude(float phi_r, float theta_r, float psi_r, float z_r)
{
	controllerMode = ATTITUDE_MODE;
	this->phi_r = phi_r;
	this->theta_r = theta_r;
	this->psi_r = psi_r;
	this->z_r = z_r;
}

mav::Attitude SampleController::controllerRun(mav::Kinematics kin)
{
    float deltaT = (SampleController::getCurrentTimeMillis() - lastTimeStamp) / 1000.0;
//	std::cout << "x_r = " << x_r << std::endl;
//	std::cout << "y_r = " << y_r << std::endl;
//	std::cout << "psi_r = " << psi_r << std::endl;
	//std::cout << "Position: " << x << y << z << std::endl;
	/*std::cout << "Orientation: " << orientation[0] << orientation[1]  << orientation[2] << std::endl;
	std::cout << "Velocity: " << velocity[0] << velocity[1] << velocity[2] << std::endl;*/

	
	Vector3f deltaX = Vector3f(x_r, y_r, z_r) - Vector3f(kin.x, kin.y, kin.z);

	if (controllerMode == POSITION_MODE)
	{
		v_x_r = deltaX(0)*K_P_VX;
		v_y_r = deltaX(1)*K_P_VY;
		v_z_r = deltaX(2)*K_P_VZ;
	}

	if (controllerMode == ATTITUDE_MODE)
	{
		v_z_r = deltaX(2)*K_P_VZ;
	}
	
//	std::cout << "Vzr: "<<v_z_r << std::endl;
	float currentHeading = kin.psi;
	
	Vector3f deltaV = Vector3f(v_x_r, v_y_r, v_z_r) - Vector3f(kin.vX, kin.vY, kin.vZ);
	Matrix<float, 3, 3> rotationMatrix;
	rotationMatrix << cos(currentHeading), sin(currentHeading), 0, -sin(currentHeading), cos(currentHeading),0, 0, 0, 1;
	Vector3f deltaVLocal = rotationMatrix * deltaV;
	//std::cout << "deltaV: " << deltaV << std::endl;
	//std::cout << "deltaVLocal: " << deltaVLocal << std::endl;

	float errorPhi = deltaVLocal(1);
	float errorTheta = deltaVLocal(0);
	float errorThrust = deltaVLocal(2);

//	std::cout << "errorTheta " << errorTheta << std::endl;

	sum_error_phi += errorPhi * deltaT;
	float phi_cmd = errorPhi * K_P_PHI + (errorPhi - previous_error_phi) / deltaT * K_D_PHI + sum_error_phi * K_I_PHI;
	previous_error_phi = errorPhi;

	sum_error_theta += errorTheta * deltaT;
	float theta_cmd = errorTheta * K_P_THETA + (errorTheta - previous_error_theta) / deltaT * K_D_THETA + sum_error_theta * K_I_THETA;
	//std::cout << "current x: " <<  kin.x << std::endl;
	//std::cout << "current vy: " << kin.vY << std::endl;
	//std::cout << "theta_cmd: " << theta_cmd << std::endl;
    float psi_cmd = 0.0;
	previous_error_theta = errorTheta;

	sum_error_thrust += sum_error_thrust * deltaT;
	float thrust_cmd = -GRAVITY_FACTOR + errorThrust * K_P_THRUST + K_I_THRUST * sum_error_thrust;
	/*std::cout << "control thrust errir: " << errorThrust << std::endl;
	std::cout << "control thrust P tern: " << errorThrust * K_P_THRUST << std::endl;
	std::cout << "control thrust I tern: " << K_I_THRUST * sum_error_thrust << std::endl;*/
	lastTimeStamp = getCurrentTimeMillis();
	//std::cout << "phi_cmd: " << phi_cmd << std::endl;

	if (controllerMode == ATTITUDE_MODE)
	{
		phi_cmd = phi_r;
		theta_cmd = theta_r;
		psi_cmd = psi_r;
	}

	if (thrust_cmd > 0)
		thrust_cmd = 0;
	else if (thrust_cmd < -20)
		thrust_cmd = -20;

	if (theta_cmd > 40.0 / 180 * 3.14)
		theta_cmd = 40.0 / 180 * 3.14;
	else if (theta_cmd < -40.0 / 180 * 3.14)
		theta_cmd = -40.0 / 180 * 3.14;

	//std::cout << "[autopilot run] psi cmd = " << psi_r << std::endl;
	//std::cout << "[SampleController] phi_cmd" << phi_cmd/3.14*180 << std::endl;
	std::stringstream line;
//	std::cout << lastTimeStamp << ":" << startupTime << ":" << (int)(lastTimeStamp - startupTime);
	line << (int)(lastTimeStamp - startupTime) << ',' << x_r << ',' << y_r << ',' << z_r << ',' << v_x_r << ',' << v_y_r << ',' << v_z_r << ',' << psi_r;
	static std::vector<std::string> buffer;
	buffer.push_back(line.str());
	if (buffer.size() > 500*2){
		mav::DataLogger::log("./", "log_controller.csv", buffer);
	}

	// todo: psi is 0
	mav::Attitude att_cmd((double)phi_cmd, (double)theta_cmd, (double)psi_cmd, (double)thrust_cmd);
	//std::cout << "[autopilot] attitude command psi " << att_cmd.yaw << std::endl;
	return att_cmd;
	 

}

SampleController::SampleController()
{
    this->phi_r = 0;
    this->theta_r = 0;
    this->psi_r = 0;
    this->z_r = 0;
	controllerMode = POSITION_MODE; // !!!!!!!!!!!!!!!!!!!!!!!!!  for debug
	previous_error_theta = 0;
	previous_error_phi = 0;
	sum_error_thrust = 0;
	sum_error_theta = 0;
	sum_error_phi = 0;
	lastTimeStamp = getCurrentTimeMillis();
	
	
}

long long SampleController::getCurrentTimeMillis()
{
	using namespace std::chrono;
	milliseconds ms = duration_cast< milliseconds >(
		system_clock::now().time_since_epoch()
		);
	return ms.count();
}

