#include "DynamicModel.h"
#include "DroneModel.h"

#ifndef SENSOR_MODEL_H
#define SENSOR_MODEL_H
#include "Kinematics.h"
#include <cstdlib>
#include <ctime>

#ifndef SIGMA_ACC
#define SIGMA_ACC 1.0
#endif

#ifndef BIAS_ACC_X
#define BIAS_ACC_X 0.0
#endif

#ifndef BIAS_ACC_Y
#define BIAS_ACC_Y 0.0
#endif

#ifndef BIAS_ACC_Z
#define BIAS_ACC_Z 0.0
#endif

#ifndef SIGMA_GYRO
#define SIGMA_GYRO 0.5/180*3.14
#endif

#ifndef BIAS_GYRO
#define BIAS_GYRO 0.0
#endif

#ifndef KP_AHRS
#define KP_AHRS 0.5
#endif

#ifndef KI_AHRS
#define KI_AHRS 0.01
#endif

#ifndef P_INITIAL
#define P_INITIAL 10
#endif 

#ifndef Q_
#define Q_ pow(0.5/180*3.14,2)
#endif

#ifndef R_
#define R_ pow(0.8,2)
#endif

#ifndef USE_GROUND_TRUTH_ATTITUDE
#define USE_GROUND_TRUTH_ATTITUDE 1
#endif

#ifndef SIGMA_ATTITUDE
#define SIGMA_ATTITUDE 1.0/180*3.14
#endif

class SensorModel {
private:
	float accX, accY, accZ;
	float p,q,r;
	float phi, theta, psi;
	float b_p, b_q, b_r;
	long long lastUpdateTime;
	long long getCurrentTimeMillis();
	float randomNumberNorm(float randNum1,float randNum2,float sigma);
	Matrix<float, 6, 6> linearizeSystem(Matrix<float, 6, 1> currentStates, Vector3f inputs,float deltaT);
	Vector3f sumErrorAngularVelocity;
	Matrix<float, 6, 6> P;
	Matrix<float, 6, 6> Q;
	Matrix3f R;
	Matrix<float,6,1> predictionModel(Matrix<float, 6, 1> currentStates, Vector3f currentInputs, float deltaT);
public:
	SensorModel();
	~SensorModel();
	void complementaryFilterAHRS_run();
	void kalmanFilterAHRS_run();
    void update(mav::Kinematics kin);
	float getAccX() { return accX; };
	float getAccY() { return accY; };
	float getAccZ() { return accZ; };
	float getP() { return p; };
	float getQ() { return q; };
	float getR() { return r; };
	float getPhiAHRS() { return phi; };
	float getThetaAHRS() { return theta; };
	float getPSiAHRS() { return psi; };


};

#endif