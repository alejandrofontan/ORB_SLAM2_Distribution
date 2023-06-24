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
#include <gsl/gsl_math.h>
#include <gsl/gsl_roots.h>

using namespace std;
namespace DIST_FITTER {

    class DistributionFitterParameters;

    struct BurrParameters {
        double x;
        double k;
        double alpha;
        double beta;
    };
    double burr_icdf(double x, void* params);

    class DistributionFitter {
    public:
        enum VerbosityLevel{
            LOW = 0,
            MEDIUM = 1,
            HIGH = 2
        };

        enum DistributionType{
            LOGNORMAL = 0,
            BURR = 1,
        };

        static DistributionFitterParameters params;
        static DistributionType distributionType;
        static VerbosityLevel verbosity;

        void static FitLogNormal(vector<double>& data_, double& mu, double& sigma);
        void static FitBurr(vector<double>& data_, double& k, double& alpha, double& beta);

        double static Lognormal_icdf(const double& probability, const double& mu, const double& sigma);
        double static Burr_icdf(const double& probability, const double& k, const double& alpha, const double& beta, double icdf_0 = 1.0);

        vector<bool> static GetInliers(const vector<double>& data, const double& threshold);

    private:
        double static lognormal_pdf(double x, double mu, double sigma);
        double static logNormal_loglikelihood(const gsl_vector* params, void* data);

        double static burr_pdf(double x, double k, double alpha, double beta);
        double static burr_loglikelihood(const gsl_vector* params, void* data);

        double static calculateKS(const std::vector<double>& data, const double& mu, const double& sigma);

    };

    class DistributionFitterParameters {
    public:
        struct LogNormal{
            int maxNumberIterations{100};
            double stepSize{0.1};
            double tolerance{1e-4};
            LogNormal() = default;
            LogNormal(const int& maxNumberIterations, const double& stepSize, const double& tolerance):
                    maxNumberIterations(maxNumberIterations),stepSize(stepSize),tolerance(tolerance){};
        };

        struct Burr{
            int maxNumberIterations{100};
            double stepSize{1.0};
            double tolerance{1e-2};
            double epsAbs{0.0};
            double epsRel{0.001};
            double icdf_upper_rel{10.0};
            Burr() = default;
            Burr(const int& maxNumberIterations, const double& stepSize, const double& tolerance):
                    maxNumberIterations(maxNumberIterations),stepSize(stepSize),tolerance(tolerance){};
        };

    private:
        friend std::ostream& operator<<(std::ostream& outstream, const DistributionFitterParameters& parameters);
        friend class DistributionFitter;

        LogNormal logNormal{};
        Burr burr{};
        const double minResidual{1e-8};

    public:
        void SetParameters(const LogNormal& logNormal_, const Burr& burr_){
            logNormal = logNormal_;
            burr = burr_;
        }
    };
}

#endif //ORB_SLAM2_DETERMINISTIC_DISTRIBUTIONFITTER_H
