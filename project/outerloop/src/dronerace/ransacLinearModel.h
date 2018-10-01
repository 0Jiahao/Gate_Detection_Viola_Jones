//
// Created by shuo on 17-9-18.
//

#ifndef MAVPILOTBASE_RANSACLINEARMODEL_H
#define MAVPILOTBASE_RANSACLINEARMODEL_H


#ifndef RANSAC_H
#define RANSAC_H

#ifdef __cplusplus
extern "C" {
#endif

#define DD 1

//#include "std.h"

/** Perform RANSAC to fit a linear model.
 *
 * @param[in] n_samples The number of samples to use for a single fit
 * @param[in] n_iterations The number of times a linear fit is performed
 * @param[in] error_threshold The threshold used to cap errors in the RANSAC process
 * @param[in] targets The target values
 * @param[in] samples The samples / feature vectors
 * @param[in] D The dimensionality of the samples
 * @param[in] count The number of samples
 * @param[out] parameters* Parameters of the linear fit
 * @param[out] fit_error* Total error of the fit
 */
void RANSAC_linear_model(int n_samples, int n_iterations, float error_threshold, float *targets, int D,
                         float (*samples)[DD], int count, float *params, float *fit_error);

/** Get indices without replacement.
 *
 * @param[out] indices_subset This will be filled with the sampled indices
 * @param[in] n_samples The number of samples / indices.
 * @param[in] count The function will sample n_sample numbers from the range 1, 2, 3,..., count
 */
void get_indices_without_replacement(int *indices_subset, int n_samples, int count);

/** Predict the value of a sample with linear weights.
 *
 * @param[in] sample The sample vector of size D
 * @param[in] weights The weight vector of size D+1
 * @param[in] D The dimension of the sample.
 * @return The predicted value
 */
float predict_value(float *sample, float *weights, int D, bool use_bias);


void fit_linear_model(float *targets, int D, float (*samples)[DD], int count, bool use_bias, float *params,
                      float *fit_error);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* RANSAC_H */
#endif //MAVPILOTBASE_RANSACLINEARMODEL_H
