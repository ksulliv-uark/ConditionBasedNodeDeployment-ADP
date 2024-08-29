//
// Created by nboardma on 7/1/2020.
//

#ifndef CPP_ADP_ADPRELIABILITY_H
#define CPP_ADP_ADPRELIABILITY_H

#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
using namespace std;

// number of combinations selecting k items from a total of N
double NchooseK(int N, int k);

// calculates network reliability weibull dist
double NetworkReliabilityWeibull(double t, int numNodes, vector<int> numFailed, vector<double> probFailure, double myShape,
                                 double myscale);

// calculate variance
double NetworkReliabilityVarianceWeibull(double t, int numNodes, vector<int> numFailed, vector<double> probFailure,
                                         int sampleSize, double myShape, double myScale);

// calculates reliability when maintenance action of adding new sensors every delta time units is performed
double StableMaintenanceReliabilityWeibull(double delta, int numNodes, vector<int> numFailed,
                                           vector<double> probFailure, double mShape, double mScale);

// calculates the residual life distribution/probability of maintenance actions
// H(delta ; delta) or G(delta ; delta)
double StableLifeResidualProbWeibull(double delta, double mShape, double mScale);

// calculates the variance of the TBM policy
double StableMaintenanceVarianceWeibull(double delta, int numNodes, vector<int> numFailed, vector<double> probFailure,
                                        int sampleSize, double myShape, double myScale);

// read in data for number failed and probability
void ReadFailData(char* filename, vector<int> &numFailed, vector<double> &probFail);
void ReadFailData(char* filename, vector<int> &numFailed, vector<double> &probFail, int &numRep);

void TestDSpec(char *fileName, int numSen, double t, double mShape, double mScale);

void ManualEvaluation(int minSize, double maxDelta, vector<int> dSpec, vector<double> dSpecProb, double costFixed,
                      double costVar, double myShape, double myScale, ofstream &outputFile);

// void SignatureRelation(int numSensors, int newSize, double t, double mShape, double mScale);

double MaintenancePolicyCostWeibull(double delta, double cF, double cV, int numSensors, double mShape, double mScale);

double ApproxStableResidualLifeDist(double delta, vector<int> ageDist, double mShape, double mScale);

double ADPReliabilityEstimate(int numNodes, double prob, vector<int> numFailed, vector<double> probFailure);

/////////////////////////////////////////////////////////////////////////////////////////////////////////
class ExponentialDistribution{

public:
    ExponentialDistribution(double lambda);

    double cdf(double x);

private:
    double myLambda;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////

class BinomialDistribution{

public:
    BinomialDistribution(int numTrials, double prob);

    void SetNumTrials(int numTrials);
    void SetProbability(double prob);

    double pdf(int x);
    double cdf(int x);

    double pdf_LargeSample(int x);
    double cdf_LargeSample(int x);

private:
    int myNumTrials;
    double myProb;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////

class WeibullDistribution{

public:
    WeibullDistribution(double shape, double scale);

    double cdf(double x);
    void UpdateWeibullParam(double shape, double scale);

private:
    double myShape; // beta
    double myScale; // eta
};

#endif //CPP_ADP_ADPRELIABILITY_H
