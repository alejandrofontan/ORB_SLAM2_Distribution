//
// Created by fontan on 20/06/23.
//

#ifndef ORB_SLAM2_DETERMINISTIC_DISTRIBUTIONFITTER_H
#define ORB_SLAM2_DETERMINISTIC_DISTRIBUTIONFITTER_H

#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>
#include <map>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_multimin.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_cdf.h>
#include <gsl/gsl_histogram.h>
#include <gsl/gsl_min.h>
#include <gsl/gsl_statistics.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_roots.h>
#include <boost/math/special_functions/gamma.hpp>

#include "Utils.h"

using namespace std;
namespace DIST_FITTER {

    enum DistributionType {
        GAMMA = 0,
        LOGNORMAL = 1,
        LOGLOGISTIC = 2,
        TSTUDENT = 3
    };

    class DistributionFitterParameters;

    class DistributionFitter {
    public:

        static DistributionFitterParameters params;
        static VerbosityLevel verbosity;

        void static FitTwoParameterDistribution(vector<double>& data, double& param1, double& param2, const int& distributionType);
        void static FitGamma(vector<double>& data, double& alpha, double& beta);
        void static FitLogNormal(vector<double>& data_, double& mu, double& sigma);
        void static FitLogLogistic(vector<double>& data_, double& mu, double& sigma);
        void static FitTStudent(vector<double>& data_, double& nu, double& sigma);

        double static Gamma_pdf(const double &x, const double &alpha, const double &beta);
        double static LogNormal_pdf(const double &x, const double &mu, const double &sigma);
        double static LogLogistic_pdf(const double &x, const double &mu, const double &sigma);

        double static Gamma_icdf(const double& probability, const double& alpha, const double& beta);
        double static LogNormal_icdf(const double& probability, const double& mu, const double& sigma);
        double static LogLogistic_icdf(const double& probability, const double& mu, const double& sigma);

        vector<bool> static GetInliers(const vector<double>& data, const double& threshold);
        void static UpdateInliers(vector<bool>& isInlier, const vector<double>& data, const double& threshold);

    private:

        double static gamma_loglikelihood(const gsl_vector* params, void* data);
        double static logNormal_loglikelihood(const gsl_vector* params, void* data);
        double static logLogistic_loglikelihood(const gsl_vector* params, void* data);

        double static twoParameterNonNegativeDistributionLogLikelihood(const gsl_vector *fittedParams, void *data,
                                                                       double (*pdfFunction)(const double&, const double&, const double&),
                                                                       double (*icdfFunction)(const double&, const double&, const double&));

        double static tstudent_pdf(double x, double nu, double sigma);
        double static tstudent_loglikelihood(const gsl_vector* params, void* data);

        double static calculateKS(const std::vector<double>& data, const double& mu, const double& sigma);

    };

    class DistributionFitterParameters {
    public:
        double pSubset{0.5};
        const double minResidual{1e-8};
        const double maxResidual{100.0};
    private:
        const int maxNumberIterations{100};
        const double stepSize{0.1};
        const double tolerance{1e-4};


        friend std::ostream& operator<<(std::ostream& outstream, const DistributionFitterParameters& parameters);
        friend class DistributionFitter;
    };
}

#endif //ORB_SLAM2_DETERMINISTIC_DISTRIBUTIONFITTER_H
