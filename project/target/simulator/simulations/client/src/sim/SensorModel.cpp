#include <chrono>
#include "SensorModel.h"
#include <iostream>
#include "tools/DataLogger.h"
#include <stdlib.h>
#include <time.h>
#include <math.h>

SensorModel::SensorModel()
{
	accX = 0; accY = 0; accZ = 0; p = 0; q = 0; r = 0;
	phi = 0; theta = 0; psi = 0; b_p = 0; b_q = 0; b_r = 0;
	sumErrorAngularVelocity << 0, 0, 0;
	P.setIdentity();
	P *= P_INITIAL;
	Q.setZero();
	Q(0, 0) = Q_;
	Q(1, 1) = Q_;
	Q(2, 2) = Q_;
	R.setZero();
	R(0, 0) = R_;
	R(1, 1) = R_;
	//R(2, 2) = R_;
	lastUpdateTime = getCurrentTimeMillis();
	std::cout << "sesor model initialized" << std::endl;
}

SensorModel::~SensorModel() {}

void SensorModel::update(mav::Kinematics kin) {
	MatrixXd randomVector = MatrixXd::Random(18, 1);
	for (int i = 0; i < 18; i++)
	{
		if(randomVector(i) < 0) randomVector(i) *= -1;
	}
	//std::cout << "random vector:" << randomVector << std::endl;
	accX = kin.accX + randomNumberNorm(randomVector(0), randomVector(1),SIGMA_ACC) + BIAS_ACC_X;
	accY = kin.accY + randomNumberNorm(randomVector(2), randomVector(3),SIGMA_ACC) + BIAS_ACC_Y;
	accZ = kin.accZ + randomNumberNorm(randomVector(4), randomVector(5),SIGMA_ACC) + BIAS_ACC_Z;
	p = kin.p + randomNumberNorm(randomVector(6), randomVector(7),SIGMA_GYRO) + BIAS_GYRO;
	q = kin.q + randomNumberNorm(randomVector(8), randomVector(9),SIGMA_GYRO) + BIAS_GYRO;
	r = kin.r + randomNumberNorm(randomVector(10), randomVector(11),SIGMA_GYRO) + BIAS_GYRO;
	if(USE_GROUND_TRUTH_ATTITUDE)
	{
		phi = kin.phi +  randomNumberNorm(randomVector(12), randomVector(13),SIGMA_ATTITUDE);
		theta = kin.theta +  randomNumberNorm(randomVector(14), randomVector(15),SIGMA_ATTITUDE);
		psi = kin.psi +  randomNumberNorm(randomVector(16), randomVector(17),SIGMA_ATTITUDE);
	}
	//std::cout << "sensor model X:" << accX << "sensor model Y:" << accY << "sensor model Z:" << accZ << std::endl;
	// std::cout << "acc noise is :" << randomVector(0) * SIGMA_ACC + BIAS_ACC_X << std::endl;
}

float SensorModel::randomNumberNorm(float randNum1, float randNum2,float sigma)
{
	float a = 2 * 3.14*randNum1;
	float r = sqrt(-2 * log(randNum2));
	return sigma * r*cos(a);
}
long long SensorModel::getCurrentTimeMillis()
{
	using namespace std::chrono;
	milliseconds ms = duration_cast< milliseconds >(
		system_clock::now().time_since_epoch()
		);
	return ms.count();
}

void SensorModel::complementaryFilterAHRS_run()
{
	float deltaT = (getCurrentTimeMillis() - lastUpdateTime) / 1000.0;
	Vector3f specificForce(accX, accY, accZ);
	Vector3f currentGravityUnit = - specificForce / specificForce.norm();
	Vector3f previousGravityUnit(-sin(theta),sin(phi)*cos(theta),cos(phi)*cos(theta));
	Vector3f angularVelocityError = currentGravityUnit.cross(previousGravityUnit);

	Vector3f gyro(p, q, r);
	sumErrorAngularVelocity += angularVelocityError * deltaT;

	Vector3f angularVelocity = KP_AHRS * angularVelocityError + KI_AHRS * sumErrorAngularVelocity + gyro;
	Vector3f attitude(phi,theta,psi);
	Matrix<float, 3, 3> R;
	R <<1.0, tan(theta)*sin(phi), tan(theta)*cos(phi), 0.0, cos(phi), -sin(phi), 0.0, sin(phi) / cos(theta), cos(phi) / cos(theta);
	attitude += R * angularVelocity*deltaT;
	if(USE_GROUND_TRUTH_ATTITUDE)
	{
		phi = attitude(0);
		theta = attitude(1);
		psi = attitude(2);
	}
	else
	{
		phi = attitude(0);
		theta = attitude(1);
		psi = attitude(2);
	}

	lastUpdateTime = getCurrentTimeMillis();
	//std::cout << "Angles:" << phi/3.14*180.0 << "|" << theta / 3.14*180.0 << "|" << psi / 3.14*180.0 << std::endl;
	//std::cout << "angular Velocity: " << p << "|" << q << "|" << r << "| Acc: " << accX << "|" << accY << "|" << accZ << std::endl;

}

void SensorModel::kalmanFilterAHRS_run()
{

	//std::cout << "[kalman filter] P is" << P << std::endl;

	// one step prediction
	float deltaT = (getCurrentTimeMillis() - lastUpdateTime) / 1000.0;
	Vector3f currentInputs(p, q, r);
	Matrix<float, 6, 1> currentStates;
	currentStates<< phi, theta, psi, b_p, b_q, b_r;
	Matrix<float, 6, 1> predictedStates = predictionModel(currentStates, currentInputs, deltaT);

	// linearize the system around last step states
	Matrix<float, 6, 6> PHI = linearizeSystem(currentStates, currentInputs, deltaT);

	// calculate P_{k|k-1}
	P = PHI * P*PHI.transpose() + Q;

	// revise prediction
	Vector3f Z_k_k_1(predictedStates(0), predictedStates(1), predictedStates(2));
	Vector3f Z_k(atan2(-accY, -accZ), atan2(accX, sqrt(accY*accY + accZ * accZ)), 0);

	Matrix<float, 3, 6> H;
	H.setZero();
	H(0, 0) = 1;
	H(1, 1) = 1;
	H(2, 2) = 1;
	Matrix3f tmp;
	tmp = H * P*H.transpose() + R;
	Matrix<float, 6, 3> K_k = P * H.transpose() * tmp.inverse();
	Matrix<float, 6, 1> deltaX = K_k * (Z_k - Z_k_k_1);
	Matrix<float, 6, 1> revisedStates = predictedStates + deltaX;

	Matrix<float, 6, 6> identityMatrix6;
	identityMatrix6.setIdentity();
	P = (identityMatrix6 - K_k * H)*P;
	//std::cout << "kalman filter result" << revisedStates/3.14*180 << std::endl;
			if (!USE_GROUND_TRUTH_ATTITUDE)
			{
				phi = revisedStates(0);
				theta = revisedStates(1);
				psi = revisedStates(2);
				b_p = revisedStates(3);
				b_q = revisedStates(4);
				b_r = revisedStates(5);
			}

	lastUpdateTime = getCurrentTimeMillis();
}

Matrix<float, 6, 1> SensorModel::predictionModel(Matrix<float, 6, 1> currentStates, Vector3f currentInputs, float deltaT)
{
	float phi = currentStates(0);
	float theta = currentStates(1);
	float psi = currentStates(2);
	float b_p = currentStates(3);
	float b_q = currentStates(4);
	float b_r = currentStates(5);
	float p = currentInputs(0);
	float q = currentInputs(1);
	float r = currentInputs(2);

	Matrix<float, 3, 3> R;
	R << 1, tan(theta)*sin(phi), tan(theta)*cos(phi), 0, cos(phi), -sin(phi), 0, sin(phi) / cos(theta), cos(phi) / cos(theta);
	Vector3f bias(b_p, b_q, b_r);
	Vector3f dTheta = R * (currentInputs-bias) ;
	Vector3f dBias(0, 0, 0);
	Matrix<float,6,1> dx;
	dx << dTheta, dBias;
	Matrix<float, 6, 1> predictedStates = currentStates + dx * deltaT;
	return predictedStates;
}

Matrix<float, 6, 6> SensorModel::linearizeSystem(Matrix<float, 6, 1> currentStates, Vector3f inputs,float deltaT)
{
	float phi = currentStates(0);
	float theta = currentStates(1);
	float psi = currentStates(2);
	float b_p = currentStates(3);
	float b_q = currentStates(4);
	float b_r = currentStates(5);
	float p_m = inputs(0);
	float q_m = inputs(1);
	float r_m = inputs(2);

	Matrix<float, 6, 6> jacobianMatrix;
	jacobianMatrix << sin(phi)*tan(theta)*(b_r - r_m) - cos(phi)*tan(theta)*(b_q - q_m), -cos(phi)*(b_r - r_m)*(tan(theta)*tan(theta) + 1) - sin(phi)*(b_q - q_m)*(tan(theta) *tan(theta) + 1), 0, -1, -sin(phi)*tan(theta), -cos(phi)*tan(theta),
		cos(phi)*(b_r - r_m) + sin(phi)*(b_q - q_m), 0, 0, 0, -cos(phi), sin(phi),
		(sin(phi)*(b_r - r_m)) / cos(theta) - (cos(phi)*(b_q - q_m)) / cos(theta), -(cos(phi)*sin(theta)*(b_r - r_m)) / (cos(theta)*cos(theta)) - (sin(phi)*sin(theta)*(b_q - q_m)) / (cos(theta) *cos(theta)), 0, 0, -sin(phi) / cos(theta), -cos(phi) / cos(theta),
		0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0;

	Matrix<float, 6, 6> identityMatrix6;
	identityMatrix6.setIdentity();
	Matrix<float, 6, 6> PHI = identityMatrix6 + jacobianMatrix * deltaT;
	return PHI;
}