//
// Created by fontan on 20/06/23.
//

#include "DistributionFitter.h"
namespace DIST_FITTER {

VerbosityLevel DistributionFitter::verbosity{MEDIUM};
DistributionFitterParameters DistributionFitter::params{};
void DistributionFitter::FitTwoParameterDistribution(vector<double>& data, double& param1, double& param2, const int& distributionType) {

    const int numParameters{2};

    // Vector for parameters
    gsl_vector *fittedParams = gsl_vector_alloc(numParameters);

    // Initialize the parameters with initial guesses
    gsl_vector_set(fittedParams, 0, param1);
    gsl_vector_set(fittedParams, 1, param2);

    // Initialize loglikelihood function
    gsl_multimin_function loglike_func;
    loglike_func.n = numParameters;
    loglike_func.params = &data;
    switch (distributionType) {
        case 0:
            loglike_func.f = &gamma_loglikelihood;
            break;
        case 1:
            loglike_func.f = &logNormal_loglikelihood;
            break;
        case 2:
            loglike_func.f = &logLogistic_loglikelihood;
            break;
        case 3:
            loglike_func.f = &tstudent_loglikelihood;
            break;
        default:
            cout << "Distribution " + to_string(distributionType) + " not implemented."<< endl;
            terminate();
    }

    const gsl_multimin_fminimizer_type *T = gsl_multimin_fminimizer_nmsimplex2;
    gsl_multimin_fminimizer *s = gsl_multimin_fminimizer_alloc(T, numParameters);

    // Set initial step sizes
    gsl_vector *stepSize = gsl_vector_alloc(numParameters);
    gsl_vector_set(stepSize, 0, params.stepSize);
    gsl_vector_set(stepSize, 1, params.stepSize);
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
        status = gsl_multimin_test_size(size, params.tolerance);
    } while (status == GSL_CONTINUE && iter < params.maxNumberIterations);

    // Recover fitted parameters
    param1 = gsl_vector_get(s->x, 0);
    param2 = gsl_vector_get(s->x, 1);

    gsl_multimin_fminimizer_free(s);
    gsl_vector_free(fittedParams);
    gsl_vector_free(stepSize);

    //double KS = calculateKS(data, mu, sigma);
    if (verbosity >= MEDIUM) {
        string distributionName{};
        switch (distributionType) {
            case 0:
                distributionName = "Gamma";
                break;
            case 1:
                distributionName = "LogNormal";
                break;
            case 2:
                 distributionName = "LogLogistic";
                 break;
            case 3:
                distributionName = "T-student";
                break;
            case 4:
                distributionName = "Burr";
                break;
        }

        cout << "[DistributionFitter] Fit " + distributionName + " (): " << endl;
        std::cout << "    Fitted " + distributionName + " Distribution:\n";
        std::cout << "        param1: " << param1 << "\n";
        std::cout << "        param2: " << param2 << "\n";
        //std::cout << "    KS: " << KS << "\n";
    }
}

double DistributionFitter::twoParameterNonNegativeDistributionLogLikelihood
                                                                (const gsl_vector *fittedParams, void *data,
                                                                 double (*pdfFunction)(const double&, const double&, const double&),
                                                                 double (*icdfFunction)(const double&, const double&, const double&)) {
    double param1 = gsl_vector_get(fittedParams, 0);
    double param2 = gsl_vector_get(fittedParams, 1);

    auto *dataset = static_cast<std::vector<double> *>(data);

    double loglikelihood = 0.0;
    if(params.pSubset < 1.0){
        double maxPdf = icdfFunction(params.pSubset, param1, param2);
        for (const auto &value: *dataset) {
            if(value > maxPdf){
                loglikelihood -= 32.2362;
                continue;
            }
            loglikelihood += log(pdfFunction(value, param1, param2));
        }
    }
    else{
        for (const auto &value: *dataset)
            loglikelihood += log(pdfFunction(value, param1, param2));
    }

    return -loglikelihood;
}

double DistributionFitter::Gamma_pdf(const double &x, const double &alpha, const double &beta) {
    double term_1 =  1.0/(pow(beta,alpha)*tgamma(alpha));
    double term_2 =  pow(x,(alpha - 1.0));
    double term_3 =  exp(-x/beta);
    double pdf = term_1 * term_2 * term_3;
    return pdf;
}

double DistributionFitter::LogNormal_pdf(const double &x, const double &mu, const double &sigma) {
    double logx = log(x);
    double logpdf = -(logx - mu) * (logx - mu) / (2.0 * sigma * sigma) - log(x * sigma * sqrt(2.0 * M_PI));
    return exp(logpdf);
}

double DistributionFitter::LogLogistic_pdf(const double &x, const double &mu, const double &sigma) {
    double term_1 = 1.0/sigma;
    double term_2 = 1.0/x;
    double z = (log(x) - mu)/sigma;
    double num = exp(z);
    double den = pow(1 + num,2);
    double pdf = (term_1 * term_2 * num)/ den;
    return pdf;
}

double DistributionFitter::Gamma_icdf(const double& probability, const double& alpha, const double& beta) {

    double gammaThreshold = gsl_cdf_gamma_Pinv(probability, alpha, beta);

    if(verbosity >= MEDIUM){
        cout << "[DistributionFitter] LogLogistic_icdf(): " << endl;
        cout << "    Gamma Threshold: "<< gammaThreshold << " at probability " << 100.0 * probability << " %"<< endl;
    }
    return gammaThreshold;
}

double DistributionFitter::LogNormal_icdf(const double& probability, const double& mu, const double& sigma) {

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

double DistributionFitter::gamma_loglikelihood(const gsl_vector *fittedParams, void *data) {
    return twoParameterNonNegativeDistributionLogLikelihood(fittedParams, data, Gamma_pdf, Gamma_icdf);
}

double DistributionFitter::logNormal_loglikelihood(const gsl_vector *fittedParams, void *data) {
    return twoParameterNonNegativeDistributionLogLikelihood(fittedParams, data, LogNormal_icdf, LogNormal_pdf);
}

double DistributionFitter::logLogistic_loglikelihood(const gsl_vector *fittedParams, void *data) {
    return twoParameterNonNegativeDistributionLogLikelihood(fittedParams, data, LogLogistic_icdf, LogLogistic_pdf);
}

void DistributionFitter::FitGamma(vector<double> &data, double &alpha, double &beta) {
    FitTwoParameterDistribution(data, alpha, beta, DIST_FITTER::DistributionType::GAMMA);
}

void DistributionFitter::FitLogNormal(vector<double> &data, double &mu, double &sigma) {
    FitTwoParameterDistribution(data, mu, sigma, DIST_FITTER::DistributionType::LOGNORMAL);
}

void DistributionFitter::FitLogLogistic(vector<double> &data, double &mu, double &sigma) {
    FitTwoParameterDistribution(data, mu, sigma, DIST_FITTER::DistributionType::LOGLOGISTIC);
}

void DistributionFitter::FitTStudent(vector<double> &data, double &nu, double &sigma) {
    FitTwoParameterDistribution(data, nu, sigma, DIST_FITTER::DistributionType::TSTUDENT);
}

vector<bool> DistributionFitter::GetInliers(const vector<double>& data, const double& threshold){
    if(data.empty())
        return vector<bool>{};

    vector<bool> isInlier(data.size(), false);
    int numValidData{0};
    for(size_t iData{}; iData < data.size(); iData++){
        numValidData += int((data[iData] > params.minResidual) && (data[iData] < params.maxResidual));
        isInlier[iData] = ((data[iData] < threshold) && (data[iData] > params.minResidual) && (data[iData] < params.maxResidual));
    }

    if(verbosity >= LOW){
        int numInliers{};
        for(auto value: isInlier)
            if(value)
                numInliers++;
        cout << "[DistributionFitter] GetInliers(): " << endl;
        cout << "    Inlier percentaje: "<< numInliers << "/" << numValidData << " = " << 100.0 * double(numInliers)/double(numValidData) << " %"<< endl;
    }

    return isInlier;
}

void DistributionFitter::UpdateInliers(vector<bool>& isInlier, const vector<double>& data, const double& threshold){
    if(data.empty())
        return;

    int numValidData{0};
    for(size_t iData{}; iData < data.size(); iData++){
        numValidData += int((data[iData] > params.minResidual) && (data[iData] < params.maxResidual) && (isInlier[iData]));
        isInlier[iData] = ((data[iData] < threshold) && (data[iData] > params.minResidual) && (data[iData] < params.maxResidual) && (isInlier[iData]));
    }

    if(verbosity >= LOW){
        int numInliers{};
        for(auto value: isInlier)
            if(value)
                numInliers++;
        cout << "[DistributionFitter] GetInliers(): " << endl;
        cout << "    Inlier percentaje: "<< numInliers << "/" << numValidData << " = " << 100.0 * double(numInliers)/double(numValidData) << " %"<< endl;
    }
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

std::ostream &operator<<(std::ostream &outstream, const DistributionFitterParameters &parameters) {
    cout << "\nDistribution Fitter parameters : " << endl;

    outstream << "    - pSubset: " << parameters.pSubset << endl;
    outstream << "    - maxNumberIterations: " << parameters.maxNumberIterations << endl;
    outstream << "    - stepSize : " << parameters.stepSize << endl;
    outstream << "    - tolerance: " << parameters.tolerance << endl;
    outstream << "    - minResidual: " << parameters.minResidual << endl;
    outstream << "    - maxResidual : " << parameters.maxResidual << endl;

    return outstream;
}
}