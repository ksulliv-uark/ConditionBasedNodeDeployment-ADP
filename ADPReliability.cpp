//
// Created by nboardma on 7/1/2020.
//

#include "ADPReliability.h"

#include <vector>
#include <iostream>
#include <fstream>
#include <limits>
#include <numeric>

double NchooseK(int N, int k){
    double temp = 1;

    for (int i = 1; i <= k ; ++i) {
        temp = temp * ((N + 1 - i) / (double) i);
    }

    return temp;
}

// numFailed - the number of of sensors that fail to cause network coverage to drop below threshold
// probFailure - probability that the corresponding number of sensors in numFailed causes network failure
// t - calculates reliability at time t
// numNodes - total number of sensors in the network
double NetworkReliabilityWeibull(double t, int numNodes, vector<int> numFailed, vector<double> probFailure, double myShape,
                                 double myScale){

    double tao = 0;

    // TODO update this for a different sensor distribution
    // ExponentialDistribution mySensorDist(1.0 / 30.0);
    WeibullDistribution mySensorDist(myShape, myScale);

    BinomialDistribution myBinomDist(numNodes, 0.5);

    double prob = mySensorDist.cdf(t);
    myBinomDist.SetProbability(prob);

    for (int i = 0; i <numFailed.size(); ++i) {

        double f_i = probFailure.at(i);

        int n = numFailed.at(i);
        //TODO delete
        tao = tao + (f_i * myBinomDist.cdf_LargeSample(n - 1));
//        tao = tao + (f_i * myBinomDist.cdf(n-1));

    }

    return tao;

}

double NetworkReliabilityVarianceWeibull(double t, int numNodes, vector<int> numFailed, vector<double> probFailure,
                                         int sampleSize, double myShape, double myScale){

    double var = 0;

    // TODO update this for a different sensor distribution
    // ExponentialDistribution mySensorDist(1.0 / 30.0);
    WeibullDistribution mySensorDist(myShape, myScale);

    BinomialDistribution myBinomDist(numNodes, 0.5);

    double prob = mySensorDist.cdf(t);
    myBinomDist.SetProbability(prob);

    for (int i = 0; i <numFailed.size(); ++i) {

        double f_i = probFailure.at(i);
        int n = numFailed.at(i);
        var = var + (f_i * (1-f_i) * pow(myBinomDist.cdf(n-1),2));

    }

    double temp = 0;

    for (int j = 0; j < numFailed.size(); ++j) {

        double f_j = probFailure.at(j);
        int n_j = numFailed.at(j);
        double F_j = myBinomDist.cdf(n_j - 1);

        for (int k = (j+1); k <numFailed.size() ; ++k) {

            double f_k = probFailure.at(k);
            int n_k = numFailed.at(k);
            double F_k = myBinomDist.cdf(n_k - 1);

            temp = temp + (F_j * F_k * f_j * f_k);

        }
    }

    var = (var - 2 * temp) / sampleSize;
    return var;

}

double StableMaintenanceReliabilityWeibull(double delta, int numNodes, vector<int> numFailed,
                                           vector<double> probFailure, double mShape, double mScale){

    double tao = 0;

//    // TODO update this for a different sensor distribution
//    // ExponentialDistribution mySensorDist(1.0 / 30.0);
//     WeibullDistribution mySensorDist(1.5, 10);

    BinomialDistribution myBinomDist(numNodes, 0.5);

    double prob = StableLifeResidualProbWeibull(delta, mShape, mScale);

    myBinomDist.SetProbability(prob);

    for (int i = 0; i <numFailed.size(); ++i) {

        double f_i = probFailure.at(i);
        int n = numFailed.at(i);
        //TODO Delete
        tao = tao + (f_i * myBinomDist.cdf_LargeSample(n - 1));
//        tao = tao + (f_i * myBinomDist.cdf(n-1));

    }

    return tao;

}

double StableMaintenanceVarianceWeibull(double delta, int numNodes, vector<int> numFailed, vector<double> probFailure,
                                        int sampleSize, double myShape, double myScale){

    double var = 0;

    BinomialDistribution myBinomDist(numNodes, 0.5);

    double prob = StableLifeResidualProbWeibull(delta, myShape, myScale);

    myBinomDist.SetProbability(prob);

    for (int i = 0; i <numFailed.size(); ++i) {

        double f_i = probFailure.at(i);
        int n = numFailed.at(i);
        var = var + (f_i * (1-f_i) * pow(myBinomDist.cdf(n-1),2));

    }

    double temp = 0;

    for (int j = 0; j < numFailed.size(); ++j) {

        double f_j = probFailure.at(j);
        int n_j = numFailed.at(j);
        double F_j = myBinomDist.cdf(n_j - 1);

        for (int k = (j+1); k <numFailed.size() ; ++k) {

            double f_k = probFailure.at(k);
            int n_k = numFailed.at(k);
            double F_k = myBinomDist.cdf(n_k - 1);

            temp = temp + (F_j * F_k * f_j * f_k);

        }
    }

    var = (var - 2 * temp) / sampleSize;
    return var;

}

double MaintenancePolicyCostWeibull(double delta, double cF, double cV, int numSensors, double mShape, double mScale){

    double prob = StableLifeResidualProbWeibull(delta, mShape , mScale);

    double num =  cF * (1 - pow(1 - prob,numSensors)) + (numSensors * cV * prob);

    return (num / delta);
}

double StableLifeResidualProbWeibull(double delta, double mShape, double mScale){

    // TODO update this for a different sensor distribution
    // ExponentialDistribution mySensorDist(1.0 / 30.0);
    WeibullDistribution mySensorDist(mShape, mScale);

    double num = 0;
    double denom = 0;

    for (int k = 0; k <= 200 ; ++k) {
        num = num + mySensorDist.cdf((double) k * delta + delta) - mySensorDist.cdf((double) k * delta);
        denom = denom + (1-mySensorDist.cdf((double) k * delta));
    }

    double prob = num / denom;

    return prob;

}

double ApproxStableResidualLifeDist(double delta, vector<int> ageDist, double mShape, double mScale){

    WeibullDistribution mySensorDist(mShape, mScale);

    int curNetworkSize = accumulate(ageDist.begin(), ageDist.end(),0);

    double residProb = 0;

    for (int k = 0; k < ageDist.size(); ++k) {
        double tdist = ((double) ageDist.at(k)) / ((double) curNetworkSize);

        double num = mySensorDist.cdf((double) k * delta + delta) - mySensorDist.cdf((double) k * delta);
        double denom = (1-mySensorDist.cdf((double) k * delta));

        residProb = residProb + ((num / denom) * tdist);
    }

    return residProb;
}

double ADPReliabilityEstimate(int numNodes, double prob, vector<int> numFailed, vector<double> probFailure){

    double tao = 0;

    BinomialDistribution myBinomDist(numNodes, 0.5);

    myBinomDist.SetProbability(prob);

    for (int i = 0; i < numFailed.size(); ++i) {

        double f_i = probFailure.at(i);
        int n = numFailed.at(i);
        tao = tao + (f_i * myBinomDist.cdf_LargeSample(n - 1));
    }

    return tao;

}

// reads in the D - spectrum for a network
// first entry in file is the number of rows, remaining rows are:
// number of sensors failed    |    probability this causes network failure
void ReadFailData(char* filename, vector<int> &numFailed, vector<double> &probFail){

    ifstream file;
    int numValues;      //number of unique fail times
    int critNum;        //number of sensors failed
    double prob;        // corresponding probability of failure

    file.open(filename);    //open network file
    file >> numValues;      //read number of times

    //read fail time data
    for (int i = 0; i < numValues; i++){
        file >> critNum;
        numFailed.push_back(critNum);

        file >> prob;
        probFail.push_back(prob);
    }

    file.close();
}

void ReadFailData(char* filename, vector<int> &numFailed, vector<double> &probFail, int &numRep){

    ifstream file;
    int numValues;      //number of unique fail times
    int critNum;        //number of sensors failed
    int tempRep;
    double prob;        // corresponding probability of failure

    file.open(filename);    //open network file
    file >> numValues;      //read number of times

    file >> tempRep;
    numRep = tempRep;

    //read fail time data
    for (int i = 0; i < numValues; i++){
        file >> critNum;
        numFailed.push_back(critNum);

        file >> prob;
        probFail.push_back(prob);
    }

    file.close();
}

void TestDSpec(char *fileName, int numSen, double t, double myShape, double myScale) {

    vector<int> tempNumFailed;
    vector<double> tempProb;


    ReadFailData(fileName,tempNumFailed, tempProb);

    for (int i = 0; i <= t ; ++i) {
        cout << " Reliability : " << NetworkReliabilityWeibull(i, numSen, tempNumFailed, tempProb, myShape, myScale) << endl;
    }

    cout << " Stable Maintenance Reliability for changing delta " << endl;
    double i = 1.0;
    while (i < 10.0){
        cout << StableMaintenanceReliabilityWeibull(i, numSen, tempNumFailed, tempProb, myShape, myScale) << endl;
        i = i + 0.1;

    }
}

void ManualEvaluation(int minSize, double maxDelta, vector<int> dSpec, vector<double> dSpecProb, double costFixed,
                      double costVar, double myShape, double myScale, ofstream &outputFile) {

    int myNumSensors = 901;

    int currNum = myNumSensors - 1;
    while (currNum >= minSize) {
        // + 1 to include the sink node
        vector<int> newFailed;
        vector<double> newProb;

        int sizeDiff = myNumSensors - currNum;

        if (sizeDiff < 0) {
            cout << " The new network size is larger than initial network" << endl;
        }
        else {

            int firstFail = dSpec.at(0);

            if ((firstFail - sizeDiff) <= 0){
                newFailed.push_back(0);
                newProb.push_back(dSpecProb.at(0));
            }

            for (int i = 1; i < dSpec.size(); ++i) {
                if ((dSpec.at(i) - sizeDiff) <= 0){
                    newProb[0] = newProb[0] + dSpecProb.at(i);
                }
                else{
                    newFailed.push_back(dSpec.at(i) - sizeDiff);
                    newProb.push_back(dSpecProb.at(i));
                }
            }
        }

        double i = 1.0;
        while (i < maxDelta){
            outputFile << "MaintenanceRel " << currNum << " delta = " << i << " -- " << StableMaintenanceReliabilityWeibull(i, currNum - 1, newFailed, newProb, myShape, myScale) << " -- " << MaintenancePolicyCostWeibull(i, costFixed, costVar, currNum - 1, myShape, myScale) << endl;
            i = i + 0.1;
        }

        currNum = currNum - 1;
    }
}

//// --------------------------------------------------------------------------------------------------
////                         exponential distribution
//// --------------------------------------------------------------------------------------------------

// constructor for exponential distribution
ExponentialDistribution::ExponentialDistribution(double lambda) {
    myLambda = lambda;
}

// CDF for exponential distribution
double ExponentialDistribution::cdf(double x) {
    return 1 - exp(-myLambda * x);
}

//// --------------------------------------------------------------------------------------------------
////                         binomial distribution
//// --------------------------------------------------------------------------------------------------

// constructor for binomial distribution
BinomialDistribution::BinomialDistribution(int numTrials, double prob) {
    myNumTrials = numTrials;
    myProb = prob;
}

// update the number of trials
void BinomialDistribution::SetNumTrials(int numTrials) {
    myNumTrials = numTrials;
}

// update the probability of success
void BinomialDistribution::SetProbability(double prob) {
    myProb = prob;
}

// probability distribution function
double BinomialDistribution::pdf(int x){
//    cout << NchooseK(myNumTrials, x) << " - " << (pow(myProb, x)) << endl;
    return NchooseK(myNumTrials, x) * (pow(myProb, x)) * pow(1-myProb, myNumTrials - x);
}

// cumulative distribution function
double BinomialDistribution::cdf(int x) {
    double temp = 0;

    for (int i = 0; i <= x; ++i) {
        temp = temp + this->pdf(i);
    }

    return temp;
}

// probability distribution function
double BinomialDistribution::pdf_LargeSample(int x){

    double tempProb = (pow(myProb, x)) * pow(1-myProb, myNumTrials - x);

    double temp = tempProb;
    for (int i = 1; i <= x ; ++i) {
        temp = temp * ((myNumTrials + 1 - i) / (double) i);
    }

    return temp;

}

// cumulative distribution function
double BinomialDistribution::cdf_LargeSample(int x) {
    double temp = 0;

    for (int i = 0; i <= x; ++i) {
        temp = temp + this->pdf_LargeSample(i);
    }

    return temp;
}

//// --------------------------------------------------------------------------------------------------
////                         Weibull distribution
//// --------------------------------------------------------------------------------------------------

// constructor for weibull distribution
WeibullDistribution::WeibullDistribution(double shape, double scale) {
    myShape = shape;
    myScale = scale;
}

// Updates parameters of Weibull distribution
void WeibullDistribution::UpdateWeibullParam(double shape, double scale) {
    myShape = shape;
    myScale = scale;
}

// cumulative distribution function
double WeibullDistribution::cdf(double x) {
    return  (1 - exp(-(pow((x / myScale),myShape))));
}