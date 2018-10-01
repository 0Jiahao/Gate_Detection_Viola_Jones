//
// Created by shuo on 17-9-18.
//

#include "ransacLinearModel.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <Eigen/Dense>

/** Perform RANSAC to fit a linear model.
 *
 * @param[in] n_samples The number of samples to use for a single fit
 * @param[in] n_iterations The number of times a linear fit is performed
 * @param[in] error_threshold The threshold used to cap errors in the RANSAC process
 * @param[in] targets The target values
 * @param[in] samples The samples / feature vectors
 * @param[in] D The dimensionality of the samples
 * @param[in] count The number of samples
 * @param[out] parameters* Parameters of the linear fit, of size D + 1 (accounting for a constant 1 being added to the samples to represent a potential bias)
 * @param[out] fit_error* Total error of the fit
 *
 */
void RANSAC_linear_model(int n_samples, int n_iterations, float error_threshold, float *targets, int D,
                         float (*samples)[DD], int count, float *params, float *fit_error)
{
    int i,j,k,d;
    float err_sum;
    float prediction;
    float min_err;
    int min_ind;

    int D_1 = D + 1;
    float err;
    float errors[n_iterations];
    int indices_subset[n_samples];
    float subset_targets[n_samples];
    float subset_samples[n_samples][DD];
    float subset_params[n_iterations][D_1];
    bool use_bias = true;

    // ensure that n_samples is high enough to ensure a result for a single fit:
    n_samples = (n_samples < D_1) ? D_1 : n_samples;
    // n_samples should not be higher than count:
    n_samples = (n_samples < count) ? n_samples : count;

    // do the RANSAC iterations:
    for ( i = 0; i < n_iterations; i++) {

        // get a subset of indices
        get_indices_without_replacement(indices_subset, n_samples, count);

        // get the corresponding samples and targets:
        for ( j = 0; j < n_samples; j++) {
            subset_targets[j] = targets[indices_subset[j]];
            for ( k = 0; k < D; k++) {
                subset_samples[j][k] = samples[indices_subset[j]][k];
            }
        }

        // fit a linear model on the small system:
        fit_linear_model(subset_targets, D, subset_samples, n_samples, use_bias, subset_params[i], &err);

        // determine the error on the whole set:
        err_sum = 0.0f;

        for ( j = 0; j < count; j++) {
            // predict the sample's value and determine the absolute error:
            prediction = predict_value(samples[j], subset_params[i], D, use_bias);
            err = fabsf(prediction - targets[j]);
            // cap the error with the threshold:
            err = (err > error_threshold) ? error_threshold : err;
            err_sum += err;
        }
        errors[i] = err_sum;
    }

    // determine the minimal error:
    min_err = errors[0];
    min_ind = 0;
    for ( i = 1; i < n_iterations; i++) {
        if (errors[i] < min_err) {
            min_err = errors[i];
            min_ind = i;
        }
    }

    // copy the parameters:
    for ( d = 0; d < D_1; d++) {
        params[d] = subset_params[min_ind][d];
    }

}

/** Predict the value of a sample with linear weights.
 *
 * @param[in] sample The sample vector of size D
 * @param[in] weights The weight vector of size D+1
 * @param[in] D The dimension of the sample.
 * @return The predicted value
 */
float predict_value(float *sample, float *weights, int D, bool use_bias)
{
    int w;

    float sum = 0.0f;

    for ( w = 0; w < D; w++) {
        sum += weights[w] * sample[w];
    }
    if (use_bias) {
        sum += weights[D];
    }

    // printf("Prediction = %f\n", sum);

    return sum;
}

/** Get indices without replacement.
 *
 * @param[out] indices_subset This will be filled with the sampled indices
 * @param[in] n_samples The number of samples / indices.
 * @param[in] count The function will sample n_sample numbers from the range 1, 2, 3,..., count
 */

void get_indices_without_replacement(int *indices_subset, int n_samples, int count)
{
    int j, k;
    bool new_val;

    int index;

    for ( j = 0; j < n_samples; j++) {
        bool picked_number = false;
        while (!picked_number) {
            index = rand() % count;
            new_val = true;
            for ( k = 0; k < j; k++) {
                if (indices_subset[k] == index) {
                    new_val = false;
                    break;
                }
            }
            if (new_val) {
                indices_subset[j] = index;
                picked_number = true;
            }
        }
    }
}


void fit_linear_model(float *targets, int D, float (*samples)[DD], int count, bool use_bias, float *params,
                      float *fit_error)
{
    // We will solve systems of the form A x = b,
    // where A = [nx(D+1)] matrix with entries [s1, ..., sD, 1] for each sample (1 is the bias)
    // and b = [nx1] vector with the target values.
    // x in the system are the parameters for the linear regression function.

	Eigen::MatrixXf R(count, 2);
	Eigen::VectorXf q(count);
	Eigen::VectorXf params_vec(2, 1);

	for (int i = 0; i < count; i++) {
		q(i) = targets[i];
		R(i, 0) = samples[i][0];
		R(i, 1) = 1.0f;
	}

	//get_R_and_q_ForLeastSquare(R, q);
	params_vec = R.colPivHouseholderQr().solve(q); // \hat{t} = R^{-1}q
	params[0] = params_vec(0);
	params[1] = params_vec(1);

    // local vars for iterating, random numbers:
}