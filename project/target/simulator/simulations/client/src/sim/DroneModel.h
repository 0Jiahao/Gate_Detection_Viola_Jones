#ifndef DRONE_MODEL_H
#define DRONE_MODEL_H

#include <Eigen/Dense>

#ifndef K_X
#define K_X -0.5
#endif

#ifndef K_Y
#define K_Y -0.5
#endif

#ifndef K_Z
#define K_Z -0.5
#endif


#ifndef GRAVITY_FACTOR
#define GRAVITY_FACTOR 9.8
#endif

#ifndef OMEGA
#define OMEGA 10
#endif

#ifndef XI
#define XI 0.9
#endif

#ifndef K_P_PHI
#define K_P_PHI 0.5
#endif

#ifndef K_P_THETA
#define K_P_THETA -0.5
#endif

#ifndef K_P_THRUST
#define K_P_THRUST 3.0
#endif


#ifndef K_P_VX
#define K_P_VX 2.0
#endif

#ifndef K_P_VY
#define K_P_VY 2.0
#endif

#ifndef K_P_VZ
#define K_P_VZ 1.0
#endif


#ifndef K_D_PHI
#define K_D_PHI 0.05
#endif

#ifndef K_I_PHI
#define K_I_PHI 0.2
#endif



#ifndef K_D_THETA
#define K_D_THETA -0.05
#endif

#ifndef K_I_THETA
#define K_I_THETA -0.2
#endif



#ifndef K_I_THRUST
#define K_I_THRUST 2.0
#endif


using namespace Eigen;

class DroneModel {
private:
	float x;
	float y;
	float z;
	float v_x;
	float v_y;
	float v_z;
	float phi;
	float theta;
	float psi;
	float dPhi;
	float dTheta;
	float dPsi;
	float phi_r;
	float psi_r;
	float theta_r;
	float thrust;
	float p;
	float q;
	float r;
	long long lastPropagateTime;
	float lastStepThrust;
	Vector3f specificForce;
	void setStateVector(Matrix<float,12,1> state);
	static long long startupTime;
public:
	DroneModel();
	~DroneModel();
    void setCommand(float phi_r,float theta_r,float psi_r,float thrust);
	float getX(){return x;};
	float getY(){return y;};
	float getZ(){return z;};
	float getVx(){return v_x;};
	float getVy(){return v_y;};
	float getVz(){return v_z;};
	float getPhi(){ return  phi;};
	float getPsi(){return psi;};
	float getTheta(){return theta;};
	float getP(){return p;};
	float getQ(){return q;};
	float getR(){return r;};
	float getAccX() { return specificForce(0); };
	float getAccY() { return specificForce(1); };
	float getAccZ() { return specificForce(2); };
	Vector3f getSpecificForce(){ return specificForce;};
	Matrix<float,12,1> getStateVector();
	void propagateDroneModel();
	static long long getCurrentTimeMillis();
};

class TempleController
{
private:
	float x_r;
	float y_r;
	float z_r;
	float psi_r;
	float v_x_r;
	float v_y_r;
	float v_z_r;
    float previous_error_phi;
	float sum_error_phi;
	float previous_error_theta;
	float sum_error_theta;
	float sum_error_thrust;
    long long lastTimeStamp;
	long long getCurrentTimeMillis();
public:
    TempleController();
	~TempleController();
	void controllerRun(DroneModel & drone);
    void setReference(float x,float y,float z,float heading);
};


#endif