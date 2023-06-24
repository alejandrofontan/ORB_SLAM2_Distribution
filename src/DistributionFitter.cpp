//
// Created by fontan on 20/06/23.
//

#include "DistributionFitter.h"
namespace DIST_FITTER {

DistributionFitter::VerbosityLevel DistributionFitter::verbosity{LOW};
DistributionFitterParameters DistributionFitter::parameters{};
DistributionFitter::DistributionType DistributionFitter::distributionType{BURR};

double DistributionFitter::lognormal_pdf(double x, double mu, double sigma) {
    double logx = log(x);
    double logpdf = -(logx - mu) * (logx - mu) / (2.0 * sigma * sigma) - log(x * sigma * sqrt(2.0 * M_PI));
    return exp(logpdf);
}

double DistributionFitter::logNormal_loglikelihood(const gsl_vector *params, void *data) {
    double mu = gsl_vector_get(params, 0);
    double sigma = gsl_vector_get(params, 1);

    auto *dataset = static_cast<std::vector<double> *>(data);

    double loglikelihood = 0.0;
    for (const auto &value: *dataset) {
        loglikelihood += log(lognormal_pdf(value, mu, sigma));
    }

    return -loglikelihood;
}

double DistributionFitter::burr_pdf(double x, double k, double alpha, double beta) {
    double term1 = k*beta/alpha;
    double term2 = std::pow(x  / alpha, beta - 1);
    double term3 = std::pow(1.0 + pow(x/alpha,beta), k + 1);

    return term1 * term2 / term3;
}

double DistributionFitter::burr_loglikelihood(const gsl_vector* x_, void* params) {
    double k = gsl_vector_get(x_, 0);
    double alpha = gsl_vector_get(x_, 1);
    double beta = gsl_vector_get(x_, 2);

    std::vector<double>* data = static_cast<std::vector<double>*>(params);
    size_t n = data->size();

    double sum = 0.0;
    for (size_t i = 0; i < n; ++i) {
        double x = (*data)[i];
        double p = burr_pdf(x, k, alpha, beta);
        sum += std::log(p);
    }

    return -sum;
}

double DistributionFitter::Lognormal_icdf(double x, double mu, double sigma) {

    double logNormalThreshold = gsl_cdf_lognormal_Pinv(x, mu, sigma);

    if(verbosity >= MEDIUM){
        cout << "[DistributionFitter] Lognormal_icdf(): " << endl;
        cout << "    logNormal Threshold: "<< logNormalThreshold << " at probability " << 100.0*x << " %"<< endl;

    }
    return logNormalThreshold;
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
        cout << "[DistributionFitter] inliersLogNormal() aaaa: " << endl;
        cout << "    Inlier percentaje Goal: "<< 100.0*probability << " %"<< endl;
        cout << "    Inlier percentaje: "<< numInliers << "/" << isInlier.size() << " = " << 100.0 * double(numInliers)/double(isInlier.size()) << " %"<< endl;
    }
    return isInlier;
}

    struct Params {
        double x;
        double k;
        double alpha;
        double beta;
    };

double burr_cdf(double x, double k, double alpha, double beta) {

    double term1 = std::pow(x / alpha, beta);
    double term2 = std::pow(1 + term1, -k);

    return 1.0 - term2;
}

double burr_icdf_(double x, void* params) {
    Params* p = static_cast<Params*>(params);
    return burr_cdf(x, p->k, p->alpha, p->beta) - p->x;
}

double DistributionFitter::Burr_icdf(const double& k, const double& alpha, const double& beta, const double& probability,
                                     const double icdf_0) {
    Params params{ probability, k, alpha, beta };

    gsl_function F;
    F.function = &burr_icdf_;
    F.params = &params;

    double icdf, icdf_lower = 0.0, icdf_upper = 10.0*icdf_0;
    gsl_root_fsolver* solver = gsl_root_fsolver_alloc(gsl_root_fsolver_brent);
    gsl_root_fsolver_set(solver, &F, icdf_lower, icdf_upper);  // Provide an initial bracket

    int iter = 0;

    int status;
    do {
        iter++;
        status = gsl_root_fsolver_iterate(solver);
        icdf = gsl_root_fsolver_root(solver);
        icdf_lower = gsl_root_fsolver_x_lower(solver);
        icdf_upper = gsl_root_fsolver_x_upper(solver);
        status = gsl_root_test_interval(icdf_lower, icdf_upper, 0, 0.001);

        if (status == GSL_SUCCESS){
            if(verbosity >= HIGH){
                std::cout << "ICDF of probability " << probability << " in the Burr distribution: " << icdf << std::endl;
            }
        }

    } while (status == GSL_CONTINUE && iter < 100);

    gsl_root_fsolver_free(solver);

    if(verbosity >= MEDIUM){
        cout << "[DistributionFitter] Burr_icdf(): " << endl;
        cout << "    burr Threshold: "<< icdf << " at probability " << 100.0*probability << " %"<< endl;
    }

    return icdf;
}

vector<bool> DistributionFitter::GetInliers(const vector<double>& data, const double& threshold){
    if(data.empty())
        return vector<bool>{};

    vector<bool> isInlier(data.size(), false);
    for(size_t iData{}; iData < data.size(); iData++)
        isInlier[iData] = ((data[iData] < threshold) && (data[iData] > parameters.minResidual));

    if(verbosity >= MEDIUM){
        int numInliers{};
        for(auto value: isInlier)
            if(value)
                numInliers++;
        cout << "[DistributionFitter] GetInliers(): " << endl;
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

void DistributionFitter::FitLogNormal(vector<double> &data_, double &mu, double &sigma) {

    vector<double> data{};
    for(const auto& value: data_){
        if(value > parameters.minResidual)
            data.push_back(value);
    }

    gsl_vector *params = gsl_vector_alloc(2); // Vector for parameters (mu, sigma)

    // Initialize the parameters with initial guesses
    gsl_vector_set(params, 0, mu);    // Initial guess for mu
    gsl_vector_set(params, 1, sigma); // Initial guess for sigma

    // Initialize the parameters
    gsl_multimin_function loglike_func;
    loglike_func.n = 2;
    loglike_func.f = &logNormal_loglikelihood;
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

void DistributionFitter::FitBurr(vector<double>& data_, double& k, double& alpha, double& beta){

    vector<double> data{};
    for(const auto& value: data_){
        if(value > parameters.minResidual)
            data.push_back(value);
    }

    gsl_vector *params = gsl_vector_alloc(3); // Vector for parameters (k, alpha, beta)

    // Initialize the parameters with initial guesses
    gsl_vector_set (params, 0, 1.0);
    gsl_vector_set (params, 1, 1.0);
    gsl_vector_set (params, 2, 1.0);

    // Initialize the parameters
    gsl_multimin_function minex_func;
    minex_func.n = 3;
    minex_func.f = burr_loglikelihood;
    minex_func.params = &data;

    const gsl_multimin_fminimizer_type *T = gsl_multimin_fminimizer_nmsimplex2;
    gsl_multimin_fminimizer *s = NULL;
    gsl_vector *ss;

    /* Set initial step sizes to 1 */
    ss = gsl_vector_alloc (3);
    gsl_vector_set_all (ss, 1.0);

    s = gsl_multimin_fminimizer_alloc (T, 3);
    gsl_multimin_fminimizer_set (s, &minex_func, params, ss);

    size_t iter = 0;
    int status;
    double size;

    do
    {
        iter++;
        status = gsl_multimin_fminimizer_iterate(s);

        if (status)
            break;

        size = gsl_multimin_fminimizer_size (s);
        status = gsl_multimin_test_size (size, 1e-2);

        if (status == GSL_SUCCESS)
        {
            if(verbosity >= HIGH)
                printf ("converged to minimum at\n");
        }
        if(verbosity >= HIGH){
            printf ("%5d %10.3e %10.3e %10.3e f() = %7.3f size = %.3f\n",
                    iter,
                    gsl_vector_get (s->x, 0),
                    gsl_vector_get (s->x, 1),
                    gsl_vector_get (s->x, 2),
                    s->fval, size);
        }
    }
    while (status == GSL_CONTINUE && iter < 100);

    k = gsl_vector_get(s->x, 0);
    alpha = gsl_vector_get(s->x, 1);
    beta = gsl_vector_get(s->x, 2);

    if (verbosity >= MEDIUM) {
        cout << "[DistributionFitter] fitBurr(): " << endl;
        std::cout << "    Fitted Lognormal Distribution:\n";
        std::cout << "    k: " << k << "\n";
        std::cout << "    alpha: " << alpha << "\n";
        std::cout << "    beta: " << beta << "\n";
    }
    gsl_vector_free(params);
    gsl_vector_free(ss);
    gsl_multimin_fminimizer_free (s);
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