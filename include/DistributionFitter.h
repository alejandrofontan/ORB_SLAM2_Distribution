//
// Created by fontan on 20/06/23.
//

#ifndef ORB_SLAM2_DETERMINISTIC_DISTRIBUTIONFITTER_H
#define ORB_SLAM2_DETERMINISTIC_DISTRIBUTIONFITTER_H

#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_multimin.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_cdf.h>
#include <gsl/gsl_histogram.h>
#include <gsl/gsl_min.h>
#include <gsl/gsl_statistics.h>

using namespace std;
namespace DIST_FITTER {

    class DistributionFitterParameters;

    class DistributionFitter {
    public:
        enum VerbosityLevel{
            LOW = 0,
            MEDIUM = 1,
            HIGH = 2
        };
        static DistributionFitterParameters parameters;

        void static fitLogNormal(vector<double>& data, double& mu, double& sigma);
        vector<bool> static inliersLogNormal(const vector<double>& data, const double& mu, const double& sigma,
                                     const double& probability);

        double static calculateKS(const std::vector<double>& data, const double& mu, const double& sigma);

        static VerbosityLevel verbosity;

    private:
        double static lognormal_pdf(double x, double mu, double sigma);
        double static loglikelihood(const gsl_vector* params, void* data);

    };

    class DistributionFitterParameters {
    public:
        struct LogNormal{
            int maxNumberIterations{60};
            double stepSize{0.1};
            double tolerance{1e-4};
            LogNormal() = default;
            LogNormal(const int& maxNumberIterations, const double& stepSize, const double& tolerance):
                    maxNumberIterations(maxNumberIterations),stepSize(stepSize),tolerance(tolerance){};
        };

    private:
        friend std::ostream& operator<<(std::ostream& outstream, const DistributionFitterParameters& parameters);
        friend class DistributionFitter;

        LogNormal logNormal{};

    public:
        void setParameters(const LogNormal& logNormal_){
            logNormal = logNormal_;
        }
    };
}

#endif //ORB_SLAM2_DETERMINISTIC_DISTRIBUTIONFITTER_H
