//
// Created by fontan on 20/06/23.
//

#include "DistributionFitter.h"
namespace DIST_FITTER {

DistributionFitter::VerbosityLevel DistributionFitter::verbosity{LOW};
DistributionFitterParameters DistributionFitter::params{};
DistributionFitter::DistributionType DistributionFitter::distributionType{LOGNORMAL};

void DistributionFitter::FitLogNormal(vector<double> &data_, double &mu, double &sigma) {

    // Filter points with negligible residuals
    vector<double> data{};
    for(const auto& value: data_){
        if(value > params.minResidual)
            data.push_back(value);
    }

    // Vector for parameters (mu, sigma)
    gsl_vector *fittedParams = gsl_vector_alloc(2);

    // Initialize the parameters with initial guesses
    gsl_vector_set(fittedParams, 0, mu);
    gsl_vector_set(fittedParams, 1, sigma);

    // Initialize loglikelihood function
    gsl_multimin_function loglike_func;
    loglike_func.n = 2;
    loglike_func.f = &logNormal_loglikelihood;
    loglike_func.params = &data;

    const gsl_multimin_fminimizer_type *T = gsl_multimin_fminimizer_nmsimplex2;
    gsl_multimin_fminimizer *s = gsl_multimin_fminimizer_alloc(T, 2);

    // Set initial step sizes
    gsl_vector *stepSize = gsl_vector_alloc(2);
    gsl_vector_set(stepSize, 0, params.logNormal.stepSize);
    gsl_vector_set(stepSize, 1, params.logNormal.stepSize);

    gsl_multimin_fminimizer_set(s, &loglike_func, fittedParams, stepSize);

    // Optimize
    int iter{0};
    int status;
    do {
        iter++;
        status = gsl_multimin_fminimizer_iterate(s);
        if (status != GSL_SUCCESS)
            break;

        double size = gsl_multimin_fminimizer_size(s);
        status = gsl_multimin_test_size(size, params.logNormal.tolerance);
    } while (status == GSL_CONTINUE && iter < params.logNormal.maxNumberIterations);

    // Recover fitted parameters
    mu = gsl_vector_get(s->x, 0);
    sigma = gsl_vector_get(s->x, 1);

    gsl_multimin_fminimizer_free(s);
    gsl_vector_free(fittedParams);
    gsl_vector_free(stepSize);

    //double KS = calculateKS(data, mu, sigma);
    if (verbosity >= MEDIUM) {
        cout << "[DistributionFitter] fitLogNormal(): " << endl;
        std::cout << "    Fitted Lognormal Distribution:\n";
        std::cout << "    Mu: " << mu << "\n";
        std::cout << "    Sigma: " << sigma << "\n";
        //std::cout << "    KS: " << KS << "\n";
    }
}

void DistributionFitter::FitLogLogistic(vector<double> &data_, double &mu, double &sigma) {

    // Filter points with negligible residuals
    vector<double> data{};
    for(const auto& value: data_){
        if(value > params.minResidual)
            data.push_back(value);
    }

    // Vector for parameters (mu, sigma)
    gsl_vector *fittedParams = gsl_vector_alloc(2);

    // Initialize the parameters with initial guesses
    gsl_vector_set(fittedParams, 0, mu);
    gsl_vector_set(fittedParams, 1, sigma);

    // Initialize loglikelihood function
    gsl_multimin_function loglike_func;
    loglike_func.n = 2;
    loglike_func.f = &logLogistic_loglikelihood;
    loglike_func.params = &data;

    const gsl_multimin_fminimizer_type *T = gsl_multimin_fminimizer_nmsimplex2;
    gsl_multimin_fminimizer *s = gsl_multimin_fminimizer_alloc(T, 2);

    // Set initial step sizes
    gsl_vector *stepSize = gsl_vector_alloc(2);
    gsl_vector_set(stepSize, 0, params.logNormal.stepSize);
    gsl_vector_set(stepSize, 1, params.logNormal.stepSize);

    gsl_multimin_fminimizer_set(s, &loglike_func, fittedParams, stepSize);

    // Optimize
    int iter{0};
    int status;
    do {
        iter++;
        status = gsl_multimin_fminimizer_iterate(s);
        if (status != GSL_SUCCESS)
            break;

        double size = gsl_multimin_fminimizer_size(s);
        status = gsl_multimin_test_size(size, params.logNormal.tolerance);
    } while (status == GSL_CONTINUE && iter < params.logNormal.maxNumberIterations);

    // Recover fitted parameters
    mu = gsl_vector_get(s->x, 0);
    sigma = gsl_vector_get(s->x, 1);

    gsl_multimin_fminimizer_free(s);
    gsl_vector_free(fittedParams);
    gsl_vector_free(stepSize);

    //double KS = calculateKS(data, mu, sigma);
    if (verbosity >= MEDIUM) {
        cout << "[DistributionFitter] fitLogLogistic(): " << endl;
        std::cout << "    Fitted LogLogistic Distribution:\n";
        std::cout << "    Mu: " << mu << "\n";
        std::cout << "    Sigma: " << sigma << "\n";
        //std::cout << "    KS: " << KS << "\n";
    }
}

void DistributionFitter::FitTStudent(vector<double> &data_, double &nu, double &sigma) {

    // Filter points with negligible residuals
    vector<double> data{};
    for(const auto& value: data_){
        if(value > params.minResidual)
            data.push_back(value);
    }

    // Vector for parameters (nu, sigma)
    gsl_vector *fittedParams = gsl_vector_alloc(2);

    // Initialize the parameters with initial guesses
    gsl_vector_set(fittedParams, 0, nu);
    gsl_vector_set(fittedParams, 1, sigma);

    // Initialize loglikelihood function
    gsl_multimin_function loglike_func;
    loglike_func.n = 2;
    loglike_func.f = &tstudent_loglikelihood;
    loglike_func.params = &data;

    const gsl_multimin_fminimizer_type *T = gsl_multimin_fminimizer_nmsimplex2;
    gsl_multimin_fminimizer *s = gsl_multimin_fminimizer_alloc(T, 2);

    // Set initial step sizes
    gsl_vector *stepSize = gsl_vector_alloc(2);
    gsl_vector_set(stepSize, 0, params.tStudent.stepSize);
    gsl_vector_set(stepSize, 1, params.tStudent.stepSize);

    gsl_multimin_fminimizer_set(s, &loglike_func, fittedParams, stepSize);

    // Optimize
    int iter{0};
    int status;
    do {
        iter++;
        status = gsl_multimin_fminimizer_iterate(s);
        if (status != GSL_SUCCESS)
            break;

        double size = gsl_multimin_fminimizer_size(s);
        status = gsl_multimin_test_size(size, params.logNormal.tolerance);
    } while (status == GSL_CONTINUE && iter < params.logNormal.maxNumberIterations);

    // Recover fitted parameters
    nu = gsl_vector_get(s->x, 0);
    sigma = gsl_vector_get(s->x, 1);

    gsl_multimin_fminimizer_free(s);
    gsl_vector_free(fittedParams);
    gsl_vector_free(stepSize);

    if (verbosity >= MEDIUM) {
        cout << "[DistributionFitter] FitTStudent(): " << endl;
        std::cout << "    Fitted T Student Distribution:\n";
        std::cout << "    Nu: " << nu << "\n";
        std::cout << "    Sigma: " << sigma << "\n";
    }
}

void DistributionFitter::FitBurr(vector<double>& data_, double& k, double& alpha, double& beta){

    // Filter points with negligible residuals
    vector<double> data{};
    for(const auto& value: data_){
        if(value > params.minResidual)
            data.push_back(value);
    }
    // Vector for parameters (k, alpha, beta)
    gsl_vector *fittedParams = gsl_vector_alloc(3);

    // Initialize the parameters with initial guesses
    gsl_vector_set (fittedParams, 0, k);
    gsl_vector_set (fittedParams, 1, alpha);
    gsl_vector_set (fittedParams, 2, beta);

    // Initialize minex_func function
    gsl_multimin_function minex_func;
    minex_func.n = 3;
    minex_func.f = burr_loglikelihood;
    minex_func.params = &data;

    const gsl_multimin_fminimizer_type *T = gsl_multimin_fminimizer_nmsimplex2;
    gsl_multimin_fminimizer *s = nullptr;

    // Set initial step sizes
    gsl_vector *stepSize;
    stepSize = gsl_vector_alloc (3);
    gsl_vector_set_all (stepSize, params.burr.stepSize);

    s = gsl_multimin_fminimizer_alloc (T, 3);
    gsl_multimin_fminimizer_set (s, &minex_func, fittedParams, stepSize);

    // Optimize
    size_t iter = 0;
    int status;
    do
    {
        iter++;
        status = gsl_multimin_fminimizer_iterate(s);

        if (status)
            break;

        double size = gsl_multimin_fminimizer_size (s);
        status = gsl_multimin_test_size (size, params.burr.tolerance);
    }
    while (status == GSL_CONTINUE && iter < params.burr.maxNumberIterations);

    // Recover fitted parameters
    k = gsl_vector_get(s->x, 0);
    alpha = gsl_vector_get(s->x, 1);
    beta = gsl_vector_get(s->x, 2);

    gsl_vector_free(fittedParams);
    gsl_vector_free(stepSize);
    gsl_multimin_fminimizer_free (s);

    if (verbosity >= MEDIUM) {
        cout << "[DistributionFitter] fitBurr(): " << endl;
        std::cout << "    Fitted Burr Distribution:\n";
        std::cout << "    k: " << k << "\n";
        std::cout << "    alpha: " << alpha << "\n";
        std::cout << "    beta: " << beta << "\n";
    }
}

double DistributionFitter::Lognormal_icdf(const double& probability, const double& mu, const double& sigma) {

    double logNormalThreshold = gsl_cdf_lognormal_Pinv(probability, mu, sigma);

    if(verbosity >= MEDIUM){
        cout << "[DistributionFitter] Lognormal_icdf(): " << endl;
        cout << "    logNormal Threshold: "<< logNormalThreshold << " at probability " << 100.0 * probability << " %"<< endl;
    }
    return logNormalThreshold;
}

double DistributionFitter::LogLogistic_icdf(const double& probability, const double& mu, const double& sigma) {

    double logLogisticThreshold = exp((log(probability/(1.0-probability)))*sigma + mu);

    if(verbosity >= MEDIUM){
        cout << "[DistributionFitter] LogLogistic_icdf(): " << endl;
        cout << "    logLogistic Threshold: "<< logLogisticThreshold << " at probability " << 100.0 * probability << " %"<< endl;
    }
    return logLogisticThreshold;
}

double DistributionFitter::Burr_icdf(const double& probability, const double& k, const double& alpha, const double& beta, double icdf_0){

    BurrParameters burrParameters{ probability, k, alpha, beta };

    gsl_function F;
    F.function = &burr_icdf;
    F.params = &burrParameters;

    double icdf, icdf_lower = 0.0, icdf_upper = params.burr.icdf_upper_rel*icdf_0;
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
        status = gsl_root_test_interval(icdf_lower, icdf_upper, params.burr.epsAbs, params.burr.epsRel);

        if (status == GSL_SUCCESS){
            if(verbosity >= HIGH){
                std::cout << "ICDF of probability " << probability << " in the Burr distribution: " << icdf << std::endl;
            }
        }
    } while (status == GSL_CONTINUE && iter < params.burr.maxNumberIterations);

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
    int numValidData{0};
    for(size_t iData{}; iData < data.size(); iData++){
        numValidData += int(data[iData] > params.minResidual);
        isInlier[iData] = ((data[iData] < threshold) && (data[iData] > params.minResidual));
    }

    if(verbosity >= MEDIUM){
        int numInliers{};
        for(auto value: isInlier)
            if(value)
                numInliers++;
        cout << "[DistributionFitter] GetInliers(): " << endl;
        cout << "    Inlier percentaje: "<< numInliers << "/" << numValidData << " = " << 100.0 * double(numInliers)/double(numValidData) << " %"<< endl;
    }

    return isInlier;
}

double DistributionFitter::lognormal_pdf(double x, double mu, double sigma) {
    double logx = log(x);
    double logpdf = -(logx - mu) * (logx - mu) / (2.0 * sigma * sigma) - log(x * sigma * sqrt(2.0 * M_PI));
    return exp(logpdf);
}

double DistributionFitter::logNormal_loglikelihood(const gsl_vector *fittedParams, void *data) {
    double mu = gsl_vector_get(fittedParams, 0);
    double sigma = gsl_vector_get(fittedParams, 1);

    auto *dataset = static_cast<std::vector<double> *>(data);

    double loglikelihood = 0.0;
    for (const auto &value: *dataset) {
        loglikelihood += log(lognormal_pdf(value, mu, sigma));
    }

    return -loglikelihood;
}

double DistributionFitter::logLogistic_pdf(double x, double mu, double sigma) {
    double term_1 = 1.0/sigma;
    double term_2 = 1.0/x;
    double z = (log(x) - mu)/sigma;
    double num = exp(z);
    double den = pow(1 + num,2);
    double pdf = (term_1 * term_2 * num)/ den;
    return pdf;
}

double DistributionFitter::logLogistic_loglikelihood(const gsl_vector *fittedParams, void *data) {
    double mu = gsl_vector_get(fittedParams, 0);
    double sigma = gsl_vector_get(fittedParams, 1);

    auto *dataset = static_cast<std::vector<double> *>(data);

    double loglikelihood = 0.0;
    if(params.p_subset < 1.0){
        double maxPdf = LogLogistic_icdf(params.p_subset, mu, sigma);
        for (const auto &value: *dataset) {
            if(value > maxPdf){
                loglikelihood -= 32.2362;
                continue;
            }
            loglikelihood += log(logLogistic_pdf(value, mu, sigma));
        }
    }else{
        for (const auto &value: *dataset)
            loglikelihood += log(logLogistic_pdf(value, mu, sigma));
    }

    return -loglikelihood;
}

double DistributionFitter::tstudent_pdf(double x, double nu, double sigma) {
    double term0 = (nu + 1)/2.0;
    double term1 = boost::math::tgamma(term0);
    double term2 = sigma*sqrt(nu*M_PI)*boost::math::tgamma(nu/2.0);
    double term3 = (nu + (pow(x,2)/pow(sigma,2)))/nu;
    double pdf = term1/term2 * pow(term3, -term0);

    return pdf;
}

double DistributionFitter::tstudent_loglikelihood(const gsl_vector *fittedParams, void *data) {
    double nu = gsl_vector_get(fittedParams, 0);
    double sigma = gsl_vector_get(fittedParams, 1);

    auto *dataset = static_cast<std::vector<double> *>(data);

    double loglikelihood = 0.0;
    for (const auto &value: *dataset) {
        loglikelihood += log(tstudent_pdf(value, nu, sigma));
    }

    return -loglikelihood;
}

double DistributionFitter::burr_pdf(double x, double k, double alpha, double beta) {
    double term1 = k*beta/alpha;
    double term2 = std::pow(x  / alpha, beta - 1);
    double term3 = std::pow(1.0 + pow(x/alpha,beta), k + 1);

    return term1 * term2 / term3;
}

double DistributionFitter::burr_loglikelihood(const gsl_vector* x_, void* fittedParams) {
    double k = gsl_vector_get(x_, 0);
    double alpha = gsl_vector_get(x_, 1);
    double beta = gsl_vector_get(x_, 2);

    std::vector<double>* data = static_cast<std::vector<double>*>(fittedParams);
    size_t n = data->size();

    double sum = 0.0;
    for (size_t i = 0; i < n; ++i) {
        double x = (*data)[i];
        double p = burr_pdf(x, k, alpha, beta);
        sum += std::log(p);
    }

    return -sum;
}

double burr_cdf(double x, double k, double alpha, double beta) {

    double term1 = std::pow(x / alpha, beta);
    double term2 = std::pow(1 + term1, -k);

    return 1.0 - term2;
}

double burr_icdf(double x, void* fittedParams) {
    auto* p = static_cast<BurrParameters*>(fittedParams);
    return burr_cdf(x, p->k, p->alpha, p->beta) - p->x;
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

double DistributionFitter::GetCorrectionFactor(const double& p_exp,const double& p, const double& sigma){
    double ms_p3 = -5.895;
    double ms_p2 = 11.12;
    double ms_p1 = -9.693;
    double ms_p0 = 4.52;
    double ms = ms_p3*pow(p_exp,3) + ms_p2*pow(p_exp,2) + ms_p1*p_exp + ms_p0;

    double bs_p3 = 1.128;
    double bs_p2 = -1.472;
    double bs_p1 = 0.7867;
    double bs_p0 = -0.4272;
    double bs = bs_p3*pow(p_exp,3) + bs_p2*pow(p_exp,2) + bs_p1*p_exp + bs_p0;

    double k = ms * sigma + bs;
    double correction = exp(k*p);
    cout << "[GetCorrectionFactor] p_exp = "<< p_exp << " , p = "<< p << " , sigma = "<< sigma << " , correction = "<< correction << endl;
    return correction;
}

std::ostream &operator<<(std::ostream &outstream, const DistributionFitterParameters &parameters) {
    cout << "\nDistribution Fitter parameters : " << endl;

    outstream << "- Log Normal : " << endl;
    outstream << "    - maxNumberIterations: " << parameters.logNormal.maxNumberIterations << endl;
    outstream << "    - stepSize: " << parameters.logNormal.stepSize << endl;
    outstream << "    - tolerance : " << parameters.logNormal.tolerance << endl;

    outstream << "- Burr : " << endl;
    outstream << "    - maxNumberIterations: " << parameters.burr.maxNumberIterations << endl;
    outstream << "    - stepSize: " << parameters.burr.stepSize << endl;
    outstream << "    - tolerance : " << parameters.burr.tolerance << endl;

    return outstream;
}
}