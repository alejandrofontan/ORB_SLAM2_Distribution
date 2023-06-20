//
// Created by fontan on 20/06/23.
//

#include "DistributionFitter.h"
namespace DIST_FITTER {

DistributionFitter::VerbosityLevel DistributionFitter::verbosity{LOW};
DistributionFitterParameters DistributionFitter::parameters{};

double DistributionFitter::lognormal_pdf(double x, double mu, double sigma) {
    double logx = log(x);
    double logpdf = -(logx - mu) * (logx - mu) / (2.0 * sigma * sigma) - log(x * sigma * sqrt(2.0 * M_PI));
    return exp(logpdf);
}

double DistributionFitter::loglikelihood(const gsl_vector *params, void *data) {
    double mu = gsl_vector_get(params, 0);
    double sigma = gsl_vector_get(params, 1);

    auto *dataset = static_cast<std::vector<double> *>(data);

    double loglikelihood = 0.0;
    for (const auto &value: *dataset) {
        loglikelihood += log(lognormal_pdf(value, mu, sigma));
    }

    return -loglikelihood;
}

vector<bool> DistributionFitter::inliersLogNormal(const vector<double>& data, const double& mu, const double& sigma,
                                 const double& probability){

    if(data.empty())
        return vector<bool>{};

    double logNormalThreshold = gsl_cdf_lognormal_Pinv(probability, mu, sigma);

    double normalThreshold = (log(logNormalThreshold) - mu)/sigma;
    vector<bool> isInlier(data.size(), false);
    for(size_t iData{}; iData < data.size(); iData++){
        double normalizedValue = (log(data[iData]) - mu)/sigma;
        isInlier[iData] = (normalizedValue < normalThreshold);
    }
    if(verbosity >= MEDIUM){
        int numInliers{};
        for(auto value: isInlier)
            if(value)
                numInliers++;
        cout << "[DistributionFitter] inliersLogNormal(): " << endl;
        cout << "    Inlier percentaje Goal: "<< 100.0*probability << " %"<< endl;
        cout << "    Inlier percentaje: "<< numInliers << "/" << isInlier.size() << " = " << 100.0 * double(numInliers)/double(isInlier.size()) << " %"<< endl;
    }
    return isInlier;
}

double DistributionFitter::calculateKS(const std::vector<double>& data, const double& mu, const double& sigma) {
    size_t n = data.size();
    std::vector<double> sortedData = data;
    std::sort(sortedData.begin(), sortedData.end());
    double d = 0.0;
    for (size_t i = 0; i < n; ++i) {
        double currD = std::max(double(i + 1) * 1.0 / double(n) - gsl_cdf_lognormal_P(sortedData[i], mu, sigma),
                                gsl_cdf_lognormal_P(sortedData[i], mu, sigma) - i * 1.0 / n);
        d = std::max(d, currD);
    }

    return d;
}

void DistributionFitter::fitLogNormal(vector<double> &data,
                                      double &mu, double &sigma) {
    gsl_vector *params = gsl_vector_alloc(2); // Vector for parameters (mu, sigma)

    // Initialize the parameters with initial guesses
    gsl_vector_set(params, 0, mu);    // Initial guess for mu
    gsl_vector_set(params, 1, sigma); // Initial guess for sigma

    // Initialize the parameters
    gsl_multimin_function loglike_func;
    loglike_func.n = 2;
    loglike_func.f = &loglikelihood;
    loglike_func.params = &data;

    const gsl_multimin_fminimizer_type *T = gsl_multimin_fminimizer_nmsimplex2;
    gsl_multimin_fminimizer *s = gsl_multimin_fminimizer_alloc(T, 2);

    gsl_vector *stepSize = gsl_vector_alloc(2);
    gsl_vector_set(stepSize, 0, parameters.logNormal.stepSize);
    gsl_vector_set(stepSize, 1, parameters.logNormal.stepSize);

    gsl_multimin_fminimizer_set(s, &loglike_func, params, stepSize);

    int iter{0};
    int status;
    do {
        iter++;
        status = gsl_multimin_fminimizer_iterate(s);
        if (status != GSL_SUCCESS)
            break;

        double size = gsl_multimin_fminimizer_size(s);
        status = gsl_multimin_test_size(size, parameters.logNormal.tolerance);
    } while (status == GSL_CONTINUE && iter < parameters.logNormal.maxNumberIterations);

    mu = gsl_vector_get(s->x, 0);
    sigma = gsl_vector_get(s->x, 1);

    double KS = calculateKS(data, mu, sigma);
    if (verbosity >= MEDIUM) {
        cout << "[DistributionFitter] fitLogNormal(): " << endl;
        std::cout << "    Fitted Lognormal Distribution:\n";
        std::cout << "    Mu: " << mu << "\n";
        std::cout << "    Sigma: " << sigma << "\n";
        std::cout << "    KS: " << KS << "\n";
    }

    gsl_multimin_fminimizer_free(s);
    gsl_vector_free(params);


    }

std::ostream &operator<<(std::ostream &outstream, const DistributionFitterParameters &parameters) {
    cout << "\nDistribution Fitter parameters : " << endl;

    outstream << "- Log Normal : " << endl;
    outstream << "    - maxNumberIterations: " << parameters.logNormal.maxNumberIterations << endl;
    outstream << "    - stepSize: " << parameters.logNormal.stepSize << endl;
    outstream << "    - tolerance : " << parameters.logNormal.tolerance << endl;


    return outstream;
}
}