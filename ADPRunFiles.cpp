//
// Created by nboardma on 11/12/2020.
//
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <random>
#include <cmath>
#include <numeric>
#include <algorithm>

#include "ADPRunFiles.h"
#include "ADPNetworkElements.h"
#include "ADPReliability.h"

using namespace std;

// Determines how sensors are initially deployed in the region
void InitialSensorDeployment(vector<double> &gradientVect, vector<int> &numInRegion, vector<double> subArea, vector<double> subWeights, int nSens){

    double mPi = 3.14159265;
    int numReg = subWeights.size();

    // Initial Sensor Deployment

    for (int j = 0; j < numReg; ++j) {
        int tempN = numInRegion.at(j);

        double temp1 = mPi * (pow(0.07500,2) / subArea.at(j));
        double temp2 = exp(- (tempN * temp1));
        double temp3 = 1 - temp2;

        double tempGradient = ((tempN * temp1 * temp2) / temp3) + log(temp3);
        tempGradient = tempGradient * subWeights.at(j);

        gradientVect[j] = tempGradient;

    }

    int totalPlaced = numReg*20;

    while (totalPlaced < nSens){

        int gradientLoc = max_element(gradientVect.begin(),gradientVect.end()) - gradientVect.begin();

        numInRegion[gradientLoc] = numInRegion[gradientLoc] + 1;

        int tempN = numInRegion.at(gradientLoc);

        double temp1 = mPi * (pow(0.07500,2) / subArea.at(gradientLoc));
        double temp2 = exp(- (tempN * temp1));
        double temp3 = 1 - temp2;

        double tempGradient = ((tempN * temp1 * temp2) / temp3) + log(temp3);
        tempGradient = tempGradient * subWeights.at(gradientLoc);

        gradientVect[gradientLoc] = tempGradient;

        totalPlaced = totalPlaced + 1;

    }

}

void SensorDeploymentAction(vector<int> &numInRegion, vector<double> subArea, vector<double> subWeights, int numDeploy, vector<int> &regionDeployment){

    int numRegions = subWeights.size();
    double mPi = 3.14159265;

    vector<double> gradientVect(numRegions,0);

    for (int j = 0; j < 16; ++j) {
        int tempN = numInRegion.at(j);

        double temp1 = mPi * (pow(0.07500,2) / subArea.at(j));
        double temp2 = exp(- (tempN * temp1));
        double temp3 = 1 - temp2;

        double tempGradient = ((tempN * temp1 * temp2) / temp3) + log(temp3);
        tempGradient = tempGradient * subWeights.at(j);

        if (tempGradient < 0) {
            gradientVect[j] = 0.05;
        }
        else {
            gradientVect[j] = tempGradient;
        }
    }

    int numReplaced = 0;
    while (numReplaced < numDeploy){

        int gradientLoc = max_element(gradientVect.begin(),gradientVect.end()) - gradientVect.begin();

        numInRegion[gradientLoc] = numInRegion[gradientLoc] + 1;
        regionDeployment.push_back(gradientLoc);

        int tempN = numInRegion.at(gradientLoc);

        double temp1 = mPi * (pow(0.07500,2) / subArea.at(gradientLoc));
        double temp2 = exp(- (tempN * temp1));
        double temp3 = 1 - temp2;

        double tempGradient = ((tempN * temp1 * temp2) / temp3) + log(temp3);
        tempGradient = tempGradient * subWeights.at(gradientLoc);

        if (tempGradient < 0) {
            gradientVect[gradientLoc] = 0.05;
        }
        else {
            gradientVect[gradientLoc] = tempGradient;
        }

        numReplaced = numReplaced + 1;

    }

}

double AgeAggregate(vector<int> ageVector){
    double avgAge = 0;
    double totalCount = 0;
    for (int i = 0; i < ageVector.size() ; ++i) {
        totalCount = totalCount + (double) ageVector.at(i);
        avgAge = avgAge + ((double) i) * (double) ageVector.at(i);
    }
    return (avgAge / totalCount);
}

// CBM ADP model using AVI look up tables. The objective is to minimize the total cost required to maintain a minimum reliability requirement
// for the number of input missions. Only the state that is visited during an iteration has the value function updated
void ADP_CBM_AVI_CostModel(int nSens, int numReps, int nMissions, double relRequirement, double delta, double Budget,
                           double cFixed, double cVar){

    const int maxAge = 10;
    const int numRegions = 16;
    const int nMin = 384;
    const int nMax = 950;
    const int minDeploy = 10;

    double curShape = 1.5;
    double curScale = 10.0;
    double budgetRemaining;
    double expectedNum;
    double avgAge;
    int netValueIndex;

    vector<double> nodeTime;
    vector<double> iterationObjective;
    vector<double> reliabilityRecord;

    // number of nodes includes sink node, number of sensors, and number of targets
    int numTargets = 441;
    int numNodes = nSens + numTargets + 1;

    SignatureCollection mySignatures(37, nMin, nMax, numRegions);       // 37 = (nMax - nMin) / numRegions. This is the number of network signatures that have been pre-estimated
    mySignatures.ReadSignatures("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADPSignatureCollection.txt");
    mySignatures.SetFailureDist(curShape, curScale);

    ValueFunctionCostCollection myValues(nMissions, 0.7);       // 0.7 = alpha for updating the value function

    NodeLinkedListMC myNetworkBuckets(numNodes, (numNodes - numTargets), numTargets);

    myNetworkBuckets.SetRanges(0.0750000, 0.0750000);
    myNetworkBuckets.UniformTargetLoc();
    myNetworkBuckets.SetBinLimits(4);

    vector<double> subArea;
    vector<double> subWeights;

    myNetworkBuckets.SubregionArea(subArea);
//    myNetworkBuckets.SubregionDistanceWeight(subWeights);
    myNetworkBuckets.CustomWeights(subWeights);

    auto startTime = chrono::high_resolution_clock::now();

    for (int k = 1; k <= numReps ; ++k) {

        cout << "Start of Iteration " << k << endl;

        vector<double> gradientVect(numRegions,0);
        vector<int> numInRegion(numRegions,20);
        double iterationCost = cFixed + (cVar * nSens);

        budgetRemaining = Budget;

        InitialSensorDeployment(gradientVect, numInRegion, subArea, subWeights, nSens);

        // Generate initial fail times
        myNetworkBuckets.GenerateNewFailTime(nodeTime);

        myNetworkBuckets.UpdateFailTimesADP(nodeTime);

        vector<int> ageState(maxAge, 0);
        ageState[0] = nSens;

        double initialRel = mySignatures.ReliabilityEstimate_Approximation(delta, nSens, ageState);
        reliabilityRecord.push_back(initialRel);

        double curTime = delta;
        int curMission = 1;

        ageState[0] = 0;
        myNetworkBuckets.UpdateState(curTime, delta, ageState, expectedNum);

//        vector<int> regionState(numRegions,0);
//        myNetworkBuckets.UpdateState(curTime, delta, regionState, ageState, expectedNum);

        double postDecisionAge = AgeAggregate(ageState);        // Average age of all sensors in the network
        int postDecisionSize = (int) accumulate(ageState.begin(), ageState.end(),0) - nMin;     // size of the network, scaled between 0 and (nMax - nMin)

        double bestDecisionAge;
        int bestDecisionSize;

        while (curMission < nMissions) {

            int curNetworkSize = accumulate(ageState.begin(), ageState.end(),0);

            int curAction = nMax - curNetworkSize;

            int bestAction = 0;
            double bestValue = 0;
            double cost = 0;
            double tempValue;
            double bestRel = 0;
            double bestCost = 0;
            bool minNotFeasible = true;

            // action: decide not to deploy any new sensors
            avgAge = AgeAggregate(ageState);
            netValueIndex = curNetworkSize - nMin;
            double rel;
            if (curNetworkSize < nMin){
                rel = 0;
                netValueIndex = 0;
            }
            else{
                rel = mySignatures.ReliabilityEstimate_Approximation(delta, curNetworkSize, ageState);
                bestRel = rel;

                if (rel > relRequirement){
                    minNotFeasible = false;
                    bestDecisionAge = avgAge;
                    bestDecisionSize = netValueIndex;

                    bestValue = myValues.GetStateValue(curMission, avgAge, netValueIndex);
                    bestAction = 0;
                }
            }

            // action: deploy sensors to reach nMax
            ageState[0] = curAction;
            if (curAction > 0) {
                cost = cFixed + cVar * curAction;
            }
            else {
                cost = 0;
            }

            int tempNetSize = curNetworkSize + curAction;

            avgAge = AgeAggregate(ageState);
            netValueIndex = tempNetSize - nMin;

            // estimate initial reliability
            if (tempNetSize < nMin){
                rel = 0;
                netValueIndex = 0;
            }
            else{
                rel = mySignatures.ReliabilityEstimate_Approximation(delta, tempNetSize, ageState);
                bestRel = rel;
            }

            tempValue = cost + myValues.GetStateValue(curMission, avgAge, netValueIndex);

            if((tempValue < bestValue) || minNotFeasible){
                bestValue = tempValue;
                bestAction = curAction;
                bestCost = cost;
                bestRel = rel;

                bestDecisionAge = avgAge;
                bestDecisionSize = netValueIndex;
            }

            // determine best action
            curAction = curAction - minDeploy;
            tempNetSize = tempNetSize - minDeploy;

            while (curAction > 0 && rel > relRequirement){

                ageState[0] = curAction;
                cost = cFixed + cVar * curAction;

                avgAge = AgeAggregate(ageState);
                netValueIndex = tempNetSize - nMin;

                if (tempNetSize < nMin){
                    rel = 0;
                    netValueIndex = 0;
                }
                else{
                    rel = mySignatures.ReliabilityEstimate_Approximation(delta, tempNetSize, ageState);
                }

                tempValue = cost + myValues.GetStateValue(curMission, avgAge, netValueIndex);

                if ((tempValue < bestValue) && (rel > relRequirement)){
                    bestValue = tempValue;
                    bestAction = curAction;
                    bestCost = cost;
                    bestRel = rel;

                    bestDecisionAge = avgAge;
                    bestDecisionSize = netValueIndex;
                }

                curAction = curAction - minDeploy;
                tempNetSize = tempNetSize - minDeploy;
            }

            double tAlpha = myValues.GetStepSize(curMission - 1, postDecisionAge, postDecisionSize);
            double tValue = myValues.GetStateValue(curMission - 1, postDecisionAge, postDecisionSize);

            double newValue = ((1.0 - tAlpha) * tValue) + (tAlpha * bestValue);
            myValues.UpdateTempStateValue(curMission - 1, postDecisionAge, postDecisionSize, newValue);

            postDecisionAge = bestDecisionAge;
            postDecisionSize = bestDecisionSize;

            // redeploy sensors, and generate a new fail time for those added
            myNetworkBuckets.GenerateNewFailTimeADP(nodeTime, (double) nMissions*delta, curTime, bestAction);

            iterationCost = iterationCost + bestCost;
            reliabilityRecord.push_back(bestRel);

            curTime = curTime + delta;
            curMission = curMission + 1;

            vector<int> ageVector(maxAge, 0);
            myNetworkBuckets.UpdateState(curTime, delta, ageVector, expectedNum);
            copy(ageVector.begin(), ageVector.end(), ageState.begin());

//            vector<int> regionCount(numRegions,0);
//            myNetworkBuckets.UpdateState(curTime, delta, regionCount, ageVector, expectedNum);
//            copy(regionCount.begin(), regionCount.end(), regionState.begin());

        }
        iterationObjective.push_back(iterationCost);
        myValues.UpdateValueFunctions();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print out value functions
    cout << " Print Value Functions " << endl;
    myValues.PrintValueFunctions();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myObj;
    myObj.open ("CostFunctionImprovement.txt");

    for (int i = 0; i < iterationObjective.size() ; ++i) {
        myObj << "Iteration " << i << " -- ObjectiveValue : " << iterationObjective.at(i) << endl;
    }

    myObj.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // print out reliability over time
    ofstream myReliabilityFile;
    myReliabilityFile.open("ReliabilityRecord.txt");

    for (int j = 0; j < reliabilityRecord.size(); ++j) {
        myReliabilityFile << "Record " << j << " -- Reliability : " << reliabilityRecord.at(j) << endl;
    }

    myReliabilityFile.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    auto finishTime = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsedNetwork = finishTime - startTime;

    cout << "Total Time Required : " << elapsedNetwork.count() << endl;

//    cout << " myopicCBMStableMaintenance - CostRate : " << (totalCost / costIncured.size()) / delta << " - delta : " << delta << " - rel : " <<  (double) numTotalFailed / (double) maintStatus.size() << " - avgDeploy : " << averageDeployed << " - time : "<< elapsedNetwork.count() << endl;

}

// CBM ADP model using AVI look up tables. The objective is to minimize the total cost required to maintain a minimum reliability requirement
// for the number of input missions. Uses a neighborhood of the current state to update the value functions for states `close' to the one that is
// actually visited
void ADP_CBM_AVI_CostModel_CAVE(int nSens, int numReps, int nMissions, double relRequirement, double delta, double Budget,
                 double cFixed, double cVar){

    const int maxAge = 10;
    const int numRegions = 16;
    const int nMin = 384;
    const int nMax = 950;
    const int deployInterval = 10;
    const int minDeploy = 30;

    double curShape = 1.5;
    double curScale = 10.0;
    double expectedNum;
    double avgAge;
    int netValueIndex;

    vector<double> nodeTime;
    vector<double> iterationObjective;
    vector<double> reliabilityRecord;

    // number of nodes includes sink node, number of sensors, and number of targets
    int numTargets = 441;
    int numNodes = nSens + numTargets + 1;

    SignatureCollection mySignatures(567, nMin, nMax, numRegions);
//    SignatureCollection mySignatures(37, nMin, nMax, numRegions);
    mySignatures.ReadSignatures("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADPEntireSignatureCollection.txt");
    mySignatures.SetFailureDist(curShape, curScale);

    ValueFunctionCostCollection myValues(nMissions, 0.7);

    NodeLinkedListMC myNetworkBuckets(numNodes, (numNodes - numTargets), numTargets);

    myNetworkBuckets.SetRanges(0.0750000, 0.0750000);
    myNetworkBuckets.UniformTargetLoc();
    myNetworkBuckets.SetBinLimits(4);

    vector<double> subArea;
    vector<double> subWeights;

    myNetworkBuckets.SubregionArea(subArea);
    myNetworkBuckets.SubregionDistanceWeight(subWeights);
//    myNetworkBuckets.CustomWeights(subWeights);

    auto startTime = chrono::high_resolution_clock::now();

    for (int k = 1; k <= numReps ; ++k) {

        cout << "Start of Iteration " << k << endl;

        vector<double> gradientVect(numRegions,0);
        vector<int> numInRegion(numRegions,20);
        double iterationCost = cFixed + (cVar * nSens);

        // initial deployment of sensors and
        InitialSensorDeployment(gradientVect, numInRegion, subArea, subWeights, nSens);

        // Generate initial fail times and set ages
        myNetworkBuckets.GenerateNewFailTime(nodeTime);
        myNetworkBuckets.UpdateFailTimesADP(nodeTime);

        // Estimate reliability of initial state
        vector<int> ageState(maxAge, 0);
        ageState[0] = nSens;
        double initialRel = mySignatures.ReliabilityEstimate(delta, nSens, ageState);
        reliabilityRecord.push_back(initialRel);

        double curTime = delta;
        int curMission = 1;

        // update state and determine post decision state variable
        ageState[0] = 0;
        myNetworkBuckets.UpdateState(curTime, delta, ageState, expectedNum);

        double postDecisionAge = AgeAggregate(ageState);
        int postDecisionSize = (int) accumulate(ageState.begin(), ageState.end(),0) - nMin;

        double bestDecisionAge;
        int bestDecisionSize;

        while (curMission < nMissions) {

            int curNetworkSize = accumulate(ageState.begin(), ageState.end(),0);
            int curAction = nMax - curNetworkSize;

            int bestAction = 0;
            double bestValue = 0;
            double cost = 0;
            double tempValue;
            double bestRel = 0;
            double bestCost = 0;
            bool minNotFeasible = true;

            // action: decide not to deploy any new sensors
            avgAge = AgeAggregate(ageState);
            netValueIndex = curNetworkSize - nMin;

            double rel;
            if (curNetworkSize < nMin){
                rel = 0;
                netValueIndex = 0;
            }
            else{
                rel = mySignatures.ReliabilityEstimate(delta, curNetworkSize, ageState);
                bestRel = rel;

                if (rel > relRequirement){
                    minNotFeasible = false;
                    bestDecisionAge = avgAge;
                    bestDecisionSize = netValueIndex;

                    bestValue = myValues.GetStateValue(curMission, avgAge, netValueIndex);
                    bestAction = 0;
                }
            }

            // action: deploy sensors to reach nMax
            ageState[0] = curAction;
            if (curAction > 0) {
                cost = cFixed + cVar * curAction;
            }
            else {
                cost = 0;
            }

            int tempNetSize = curNetworkSize + curAction;

            avgAge = AgeAggregate(ageState);
            netValueIndex = tempNetSize - nMin;

            // estimate initial reliability of max action
            if (tempNetSize < nMin){
                rel = 0;
                netValueIndex = 0;
            }
            else{
                rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
            }

            tempValue = cost + myValues.GetStateValue(curMission, avgAge, netValueIndex);

            // determine best option from deploying 0 sensors or max sensors
            if ((tempValue < bestValue) || minNotFeasible){
                bestValue = tempValue;
                bestAction = curAction;
                bestCost = cost;
                bestRel = rel;

                bestDecisionAge = avgAge;
                bestDecisionSize = netValueIndex;
            }

            // Search through remaining actions
            curAction = curAction - deployInterval;
            tempNetSize = tempNetSize - deployInterval;

            while (curAction > minDeploy && rel > relRequirement){

                ageState[0] = curAction;
                cost = cFixed + cVar * curAction;

//                avgAge = AgeAggregate(ageState);
                avgAge = avgAge * ((double) (tempNetSize + deployInterval) / (double) tempNetSize);
                netValueIndex = tempNetSize - nMin;

                if (tempNetSize < nMin){
                    rel = 0;
                    netValueIndex = 0;
                }
                else{
                    rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
                }

                tempValue = cost + myValues.GetStateValue(curMission, avgAge, netValueIndex);

                if ((tempValue < bestValue) && (rel > relRequirement)){
                    bestValue = tempValue;
                    bestAction = curAction;
                    bestCost = cost;
                    bestRel = rel;

                    bestDecisionAge = avgAge;
                    bestDecisionSize = netValueIndex;
                }

                curAction = curAction - deployInterval;
                tempNetSize = tempNetSize - deployInterval;
            }

            double tAlpha = myValues.GetStepSize(curMission - 1, postDecisionAge, postDecisionSize);
            double tValue = myValues.GetStateValue(curMission - 1, postDecisionAge, postDecisionSize);

            double newValue = ((1.0 - tAlpha) * tValue) + (tAlpha * bestValue);
            myValues.UpdateTempStateValue(curMission - 1, postDecisionAge, postDecisionSize, newValue);
//            myValues.UpdateAdjacentStates(curMission - 1, postDecisionAge, postDecisionSize, bestValue);
            myValues.ADP_MonotoneUpdate(curMission - 1, postDecisionAge, postDecisionSize, bestValue, cVar);

            postDecisionAge = bestDecisionAge;
            postDecisionSize = bestDecisionSize;

            // redeploy sensors, and generate a new fail time for those added
            myNetworkBuckets.GenerateNewFailTimeADP(nodeTime, (double) nMissions*delta, curTime, bestAction);

            iterationCost = iterationCost + bestCost;
            reliabilityRecord.push_back(bestRel);

            curTime = curTime + delta;
            curMission = curMission + 1;

            vector<int> ageVector(maxAge, 0);
            myNetworkBuckets.UpdateState(curTime, delta, ageVector, expectedNum);
            copy(ageVector.begin(), ageVector.end(), ageState.begin());
        }
        iterationObjective.push_back(iterationCost);
        myValues.UpdateValueFunctions();
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print out value functions
    cout << " Print Value Functions " << endl;
    myValues.PrintValueFunctions();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myObj;
    myObj.open ("CostFunctionImprovement.txt");

    for (int i = 0; i < iterationObjective.size() ; ++i) {
        myObj << "Iteration " << i << " -- ObjectiveValue : " << iterationObjective.at(i) << endl;
    }

    myObj.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // print out reliability over time
    ofstream myReliabilityFile;
    myReliabilityFile.open("ReliabilityRecord.txt");

    for (int j = 0; j < reliabilityRecord.size(); ++j) {
        myReliabilityFile << "Record " << j << " -- Reliability : " << reliabilityRecord.at(j) << endl;
    }

    myReliabilityFile.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    auto finishTime = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsedNetwork = finishTime - startTime;

    cout << "Total Time Required : " << elapsedNetwork.count() << endl;

}

// CBM ADP model using AVI look up tables. The objective is to minimize the total cost required to maintain a minimum reliability requirement
// for the number of input missions. Uses a neighborhood of the current state to update the value functions for states `close' to the one that is
// actually visited. When determining the optimal action each iteration, this function uses a binary search to determine the action (compared to previous
// functions that sequentially search the entire space)
void ADP_CBM_AVI_CostModel_BinarySearch_CAVE(int nSens, int numReps, int nMissions, double relRequirement, double delta, double Budget,
                                double cFixed, double cVar){

    const int maxAge = 10;
    const int numRegions = 16;
    const int nMin = 384;
    const int nMax = 950;

    double curShape = 1.5;
    double curScale = 10.0;
    double expectedNum;
    double avgAge;
    int netValueIndex;

    vector<double> nodeTime;
    vector<double> iterationObjective;
    vector<double> reliabilityRecord;

    // number of nodes includes sink node, number of sensors, and number of targets
    int numTargets = 441;
    int numNodes = nSens + numTargets + 1;

    SignatureCollection mySignatures(37, nMin, nMax, numRegions);
    mySignatures.ReadSignatures("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADPSignatureCollection.txt");
    mySignatures.SetFailureDist(curShape, curScale);

    ValueFunctionCostCollection myValues(nMissions, 0.7);

    NodeLinkedListMC myNetworkBuckets(numNodes, (numNodes - numTargets), numTargets);

    myNetworkBuckets.SetRanges(0.0750000, 0.0750000);
    myNetworkBuckets.UniformTargetLoc();
    myNetworkBuckets.SetBinLimits(4);

    vector<double> subArea;
    vector<double> subWeights;

    myNetworkBuckets.SubregionArea(subArea);
//    myNetworkBuckets.SubregionDistanceWeight(subWeights);
    myNetworkBuckets.CustomWeights(subWeights);

    auto startTime = chrono::high_resolution_clock::now();

    for (int k = 1; k <= numReps ; ++k) {

        cout << "Start of Iteration " << k << endl;

        vector<double> gradientVect(numRegions,0);
        vector<int> numInRegion(numRegions,20);
        double iterationCost = cFixed + (cVar * nSens);

        // initial deployment of sensors and
        InitialSensorDeployment(gradientVect, numInRegion, subArea, subWeights, nSens);

        // Generate initial fail times and set ages
        myNetworkBuckets.GenerateNewFailTime(nodeTime);
        myNetworkBuckets.UpdateFailTimesADP(nodeTime);

        // Estimate reliability of initial state
        vector<int> ageState(maxAge, 0);
        ageState[0] = nSens;
        double initialRel = mySignatures.ReliabilityEstimate_Approximation(delta, nSens, ageState);
        reliabilityRecord.push_back(initialRel);

        double curTime = delta;
        int curMission = 1;

        // update state and determine post decision state variable
        ageState[0] = 0;
        myNetworkBuckets.UpdateState(curTime, delta, ageState, expectedNum);

        double postDecisionAge = AgeAggregate(ageState);
        int postDecisionSize = (int) accumulate(ageState.begin(), ageState.end(),0) - nMin;

        double bestDecisionAge;
        int bestDecisionSize;

        while (curMission < nMissions) {

            int curNetworkSize = accumulate(ageState.begin(), ageState.end(),0);
            int curAction = nMax - curNetworkSize;

            int bestAction = 0;
            double bestValue = 0;
            double cost = 0;
            double tempValue;
            double bestRel = 0;
            double bestCost = 0;
            bool minNotFeasible = true;

            double lowerBound = 0;
            int lowerAction = 0;

            double upperBound;
            int upperAction;

            // action: decide not to deploy any new sensors
            avgAge = AgeAggregate(ageState);
            netValueIndex = curNetworkSize - nMin;

            double rel;
            if (curNetworkSize < nMin){
                rel = 0;
                netValueIndex = 0;
            }
            else{
                rel = mySignatures.ReliabilityEstimate_Approximation(delta, curNetworkSize, ageState);

                if (rel > relRequirement){
                    minNotFeasible = false;
                    lowerBound = myValues.GetStateValue(curMission, avgAge, netValueIndex);
                }
            }

            // action: deploy sensors to reach nMax
            ageState[0] = curAction;
            if (curAction > 0) {
                cost = cFixed + cVar * curAction;
            }
            else {
                cost = 0;
            }

            int tempNetSize = curNetworkSize + curAction;

            avgAge = AgeAggregate(ageState);
            netValueIndex = tempNetSize - nMin;

            upperBound = cost + myValues.GetStateValue(curMission, avgAge, netValueIndex);
            upperAction = curAction;

            while (minNotFeasible) {
//                curAction = (upperAction + lowerAction) / 2;
                curAction = lowerAction + 10;

                ageState[0] = curAction;
                if (curAction > 0) {
                    cost = cFixed + cVar * curAction;
                }
                else {
                    cost = 0;
                }

                tempNetSize = curNetworkSize + curAction;
                avgAge = AgeAggregate(ageState);
                netValueIndex = tempNetSize - nMin;

                if (tempNetSize < nMin){
                    lowerAction = curAction;
                }
                else{
                    rel = mySignatures.ReliabilityEstimate_Approximation(delta, tempNetSize, ageState);

                    if (rel > relRequirement){
                        minNotFeasible = false;
                        lowerBound = cost + myValues.GetStateValue(curMission, avgAge, netValueIndex);
                    }
                    lowerAction = curAction;
                }
            }

            bool actionNotFound = true;

            while (actionNotFound){

                if (abs(lowerBound - upperBound) < 0.0001){
                    // stop, i have found the best action
                    actionNotFound = false;

                    bestAction = (upperAction + lowerAction) / 2;
                    ageState[0] = bestAction;
                    if (bestAction > 0) {
                        cost = cFixed + cVar * bestAction;
                    }
                    else {
                        cost = 0;
                    }

                    tempNetSize = curNetworkSize + bestAction;
                    avgAge = AgeAggregate(ageState);
                    netValueIndex = tempNetSize - nMin;

                    bestRel = mySignatures.ReliabilityEstimate_Approximation(delta, tempNetSize, ageState);
                    bestCost = cost;

                    bestValue = cost + myValues.GetStateValue(curMission, avgAge, netValueIndex);
                    bestDecisionAge = avgAge;
                    bestDecisionSize = netValueIndex;
                }
                else if (upperAction - lowerAction <= 2) {
                    actionNotFound = false;

                    if (lowerBound < upperBound){
                        bestAction = lowerAction;

                        ageState[0] = bestAction;
                        if (bestAction > 0) {
                            cost = cFixed + cVar * bestAction;
                        }
                        else {
                            cost = 0;
                        }

                        tempNetSize = curNetworkSize + bestAction;
                        avgAge = AgeAggregate(ageState);
                        netValueIndex = tempNetSize - nMin;

                        bestRel = mySignatures.ReliabilityEstimate_Approximation(delta, tempNetSize, ageState);
                        bestCost = cost;

                        bestValue = cost + myValues.GetStateValue(curMission, avgAge, netValueIndex);
                        bestDecisionAge = avgAge;
                        bestDecisionSize = netValueIndex;

                    }
                    else {
                        bestAction = upperAction;

                        ageState[0] = bestAction;
                        if (bestAction > 0) {
                            cost = cFixed + cVar * bestAction;
                        }
                        else {
                            cost = 0;
                        }

                        tempNetSize = curNetworkSize + bestAction;
                        avgAge = AgeAggregate(ageState);
                        netValueIndex = tempNetSize - nMin;

                        bestRel = mySignatures.ReliabilityEstimate_Approximation(delta, tempNetSize, ageState);
                        bestCost = cost;

                        bestValue = cost + myValues.GetStateValue(curMission, avgAge, netValueIndex);
                        bestDecisionAge = avgAge;
                        bestDecisionSize = netValueIndex;
                    }
                }
                else {
                    curAction = (upperAction + lowerAction) / 2;
                    ageState[0] = curAction;
                    if (curAction > 0) {
                        cost = cFixed + cVar * curAction;
                    }
                    else {
                        cost = 0;
                    }

                    tempNetSize = curNetworkSize + curAction;
                    avgAge = AgeAggregate(ageState);
                    netValueIndex = tempNetSize - nMin;

                    tempValue = cost + myValues.GetStateValue(curMission, avgAge, netValueIndex);

                    if (lowerBound < upperBound){
                        // move upper bound down
                        upperBound = tempValue;
                        upperAction = curAction;
                    }
                    else {
                        // move lower bound up
                        lowerBound = tempValue;
                        lowerAction = curAction;
                    }
                }
            }

            double tAlpha = myValues.GetStepSize(curMission - 1, postDecisionAge, postDecisionSize);
            double tValue = myValues.GetStateValue(curMission - 1, postDecisionAge, postDecisionSize);

            double newValue = ((1.0 - tAlpha) * tValue) + (tAlpha * bestValue);
            myValues.UpdateTempStateValue(curMission - 1, postDecisionAge, postDecisionSize, newValue);
//            myValues.UpdateAdjacentStates(curMission - 1, postDecisionAge, postDecisionSize, bestValue);
            myValues.ADP_MonotoneUpdate(curMission - 1, postDecisionAge, postDecisionSize, bestValue, cVar);

            postDecisionAge = bestDecisionAge;
            postDecisionSize = bestDecisionSize;

            // redeploy sensors, and generate a new fail time for those added
            myNetworkBuckets.GenerateNewFailTimeADP(nodeTime, (double) nMissions*delta, curTime, bestAction);

            iterationCost = iterationCost + bestCost;
            reliabilityRecord.push_back(bestRel);

            curTime = curTime + delta;
            curMission = curMission + 1;

            vector<int> ageVector(maxAge, 0);
            myNetworkBuckets.UpdateState(curTime, delta, ageVector, expectedNum);
            copy(ageVector.begin(), ageVector.end(), ageState.begin());
        }
        iterationObjective.push_back(iterationCost);
        myValues.UpdateValueFunctions();
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print out value functions
    cout << " Print Value Functions " << endl;
    myValues.PrintValueFunctions();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myObj;
    myObj.open ("CostFunctionImprovement.txt");

    for (int i = 0; i < iterationObjective.size() ; ++i) {
        myObj << "Iteration " << i << " -- ObjectiveValue : " << iterationObjective.at(i) << endl;
    }

    myObj.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // print out reliability over time
    ofstream myReliabilityFile;
    myReliabilityFile.open("ReliabilityRecord.txt");

    for (int j = 0; j < reliabilityRecord.size(); ++j) {
        myReliabilityFile << "Record " << j << " -- Reliability : " << reliabilityRecord.at(j) << endl;
    }

    myReliabilityFile.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    auto finishTime = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsedNetwork = finishTime - startTime;

    cout << "Total Time Required : " << elapsedNetwork.count() << endl;

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                         Reliability Section
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CBM ADP model using AVI look up tables. The objective is to maximize the number of successful missions subject to a budget constraint.
void ADP_CBM_AVI_ReliabilityModel(int nSens, int numReps, int nMissions, double delta, double Budget,
                                  double cFixed, double cVar) {

    uniform_real_distribution<> myUnif(0,1);

    random_device myDev;
    random_device myRandomOrder;

    const int maxAge = 10;
    const int numRegions = 16;
    const int nMin = 384;
    const int nMax = 950;
    const int deployInterval = 10;
    const int minDeploy = 30;
    const double explorationProb = 0.05;

    double curShape = 1.5;
    double curScale = 10.0;
    double expectedNum;
    int netValueIndex;
    double budgetRemaining;

    vector<double> nodeTime;
    vector<double> costRecord;
    vector<double> reliabilityRecord;
    vector<double> cumulativeMissionSuccess;

    // number of nodes includes sink node, number of sensors, and number of targets
    int numTargets = 441;
    int numNodes = nMax + numTargets + 1;

    SignatureCollection mySignatures(nMax - nMin + 1, nMin, nMax, numRegions);
    mySignatures.ReadSignatures("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADPEntireSignautreCollection_d5.txt");
    mySignatures.SetFailureDist(curShape, curScale);

//    ValueFunctionReliabilityCollection myValues(nMissions, 0.7);
    ValueFunctionReliabilityCollection myValues(nMissions, 0.7, Budget, nMin, nMax);

    NodeLinkedListMC myNetworkBuckets(numNodes, (numNodes - numTargets), numTargets);

    myNetworkBuckets.SetRanges(0.0750000, 0.0750000);
    myNetworkBuckets.UniformTargetLoc();
    myNetworkBuckets.SetBinLimits(4);

    vector<double> subArea;
    vector<double> subWeights;

    myNetworkBuckets.SubregionArea(subArea);
    myNetworkBuckets.SubregionDistanceWeight(subWeights);
//    myNetworkBuckets.CustomWeights(subWeights);

    auto startTime = chrono::high_resolution_clock::now();

    for (int k = 1; k <= numReps ; ++k) {

        cout << "Start of Iteration " << k << endl;

        vector<double> gradientVect(numRegions,0);
        vector<int> numInRegion(numRegions,20);
        double iterationCost = cFixed + (cVar * nSens);
        double missionSuccess = 0;

        budgetRemaining = Budget - iterationCost;

        // initial deployment of sensors and
        InitialSensorDeployment(gradientVect, numInRegion, subArea, subWeights, nSens);

        // Generate initial fail times and set ages
        myNetworkBuckets.GenerateNewFailTime_ADPEval(nodeTime, nSens);
        myNetworkBuckets.UpdateFailTimesADP(nodeTime);
        myNetworkBuckets.ClearSensorAge();

        // Estimate reliability of initial state
        vector<int> ageState(maxAge, 0);
        ageState[0] = nSens;

        double initialRel = mySignatures.ReliabilityEstimate(delta, nSens, ageState);
        reliabilityRecord.push_back(initialRel);
        missionSuccess = missionSuccess + initialRel;

        double curTime = delta;
        int curMission = 1;

        // update state and determine post decision state variable
        ageState[0] = 0;
        myNetworkBuckets.UpdateState(curTime, delta, ageState, expectedNum);

        double postDecisionBudget = (double) ((budgetRemaining / 10) * 10);
        int postDecisionSize = (int) accumulate(ageState.begin(), ageState.end(),0) - nMin;

        double bestDecisionBudget;
        int bestDecisionSize;

        while (curMission < nMissions) {

            int curNetworkSize = accumulate(ageState.begin(), ageState.end(),0);
            int curAction = nMax - curNetworkSize;

            int maxAction = max(0, min(curAction, (int) floor((budgetRemaining - cFixed) / cVar)));

            int bestAction = 0;
            double bestValue = 0;
            double cost = 0;
            double tempValue;
            double bestRel = 0;
            double bestCost = 0;
            double tBudget;
            double rel;

            double myNum = myUnif(myDev);

            // Select action at random
            if (myNum < explorationProb){
                uniform_int_distribution<> myRandomDeploy(0, maxAction);

                bestAction = myRandomDeploy(myRandomOrder);

                ageState[0] = bestAction;

                if (bestAction > 0){
                    tBudget = budgetRemaining - cFixed - (cVar * bestAction);
                    cost = cFixed + cVar * bestAction;
                }
                else {
                    tBudget = budgetRemaining;
                    cost = 0;
                }

                int tempNetSize = curNetworkSize + bestAction;
                netValueIndex = tempNetSize - nMin;

                if (tempNetSize < nMin){
                    rel = 0;
                    netValueIndex = 0;
                }
                else{
                    rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
                }

                bestValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

                bestCost = cost;
                bestRel = rel;

                bestDecisionBudget = tBudget;
                bestDecisionSize = netValueIndex;

            }
            else {
            // Search for optimal action

                // action: decide not to deploy any new sensors
                netValueIndex = curNetworkSize - nMin;
                if (curNetworkSize < nMin){
                    rel = 0;
                    netValueIndex = 0;
                }
                else{
                    rel = mySignatures.ReliabilityEstimate(delta, curNetworkSize, ageState);
                    bestRel = rel;
                }

                bestDecisionBudget = budgetRemaining;
                bestDecisionSize = netValueIndex;

                bestValue = rel + myValues.GetStateValue(curMission, budgetRemaining, netValueIndex);
                bestAction = 0;

                curAction = maxAction;

                while (curAction > minDeploy){

                    ageState[0] = curAction;
                    tBudget = budgetRemaining - cFixed - (cVar * curAction);

                    cost = cFixed + cVar * curAction;

                    int tempNetSize = curNetworkSize + curAction;
                    netValueIndex = tempNetSize - nMin;

                    if (tempNetSize < nMin){
                        rel = 0;
                        netValueIndex = 0;
                    }
                    else{
                        rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
                    }

                    tempValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

                    if (tempValue > bestValue){
                        bestValue = tempValue;
                        bestAction = curAction;
                        bestCost = cost;
                        bestRel = rel;

                        bestDecisionBudget = tBudget;
                        bestDecisionSize = netValueIndex;
                    }
                    curAction = curAction - deployInterval;
                }
            }

            double tAlpha = myValues.GetStepSize(curMission - 1, postDecisionBudget, postDecisionSize);
            double tValue = myValues.GetStateValue(curMission - 1, postDecisionBudget, postDecisionSize);

            double newValue = ((1.0 - tAlpha) * tValue) + (tAlpha * bestValue);
            myValues.UpdateTempStateValue(curMission - 1, postDecisionBudget, postDecisionSize, newValue);
//            myValues.UpdateAdjacentStates(curMission - 1, postDecisionAge, postDecisionSize, bestValue);

            postDecisionBudget = bestDecisionBudget;
            postDecisionSize = bestDecisionSize;

            // redeploy sensors, and generate a new fail time for those added
            myNetworkBuckets.GenerateNewFailTimeADP_Update(nodeTime, (double) nMissions*delta, curTime, bestAction);

            iterationCost = iterationCost + bestCost;
            missionSuccess = missionSuccess + bestRel;
            reliabilityRecord.push_back(bestRel);
            budgetRemaining = budgetRemaining - bestCost;

            curTime = curTime + delta;
            curMission = curMission + 1;

            vector<int> ageVector(maxAge, 0);
            myNetworkBuckets.UpdateState(curTime, delta, ageVector, expectedNum);
            copy(ageVector.begin(), ageVector.end(), ageState.begin());
        }
        costRecord.push_back(iterationCost);
        cumulativeMissionSuccess.push_back(missionSuccess);

        myValues.UpdateValueFunctionsConstant();
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print out value functions
    cout << " Print Value Functions " << endl;
    myValues.PrintValueFunctions();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myObj;
    myObj.open ("CostFunctionImprovement.txt");

    for (int i = 0; i < costRecord.size() ; ++i) {
        myObj << "Iteration " << i << " -- ObjectiveValue : " << costRecord.at(i) << endl;
    }

    myObj.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // print out reliability over time
    ofstream myReliabilityFile;
    myReliabilityFile.open("ReliabilityRecord.txt");

    for (int j = 0; j < reliabilityRecord.size(); ++j) {
        myReliabilityFile << "Record " << j << " -- Reliability : " << reliabilityRecord.at(j) << endl;
    }

    myReliabilityFile.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream mySuccess;
    mySuccess.open ("ExpectedNumSuccessfulMissions.txt");

    for (int i = 0; i < cumulativeMissionSuccess.size() ; ++i) {
        mySuccess << "Iteration " << i << " -- ObjectiveValue : " << cumulativeMissionSuccess.at(i) << endl;
    }

    mySuccess.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    auto finishTime = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsedNetwork = finishTime - startTime;

    cout << "Total Time Required : " << elapsedNetwork.count() << endl;

}

// CBM ADP model using AVI look up tables. The objective is to maximize the number of successful missions subject to a budget constraint.
// Uses a neighborhood of the current state to update the value functions for states `close' to the one that is actually visited
void ADP_CBM_AVI_ReliabilityModel_CAVE(int nSens, int numReps, int nMissions, double delta, double Budget,
                                  double cFixed, double cVar) {

    uniform_real_distribution<> myUnif(0,1);

    random_device myDev;
    random_device myRandomOrder;

    const int maxAge = 10;
    const int numRegions = 16;
    const int nMin = 384;
    const int nMax = 950;
    const int deployInterval = 10;
    const int minDeploy = 30;
    const double explorationProb = 0.05;
    const double learningRate = 0.7;

    double curShape = 1.5;
    double curScale = 10.0;
    double expectedNum;
    int netValueIndex;
    double budgetRemaining;

    vector<double> nodeTime;
    vector<double> costRecord;
    vector<double> reliabilityRecord;
    vector<double> cumulativeMissionSuccess;
    vector<int> netSizeBeginning;
    vector<int> missionDeployAction;

    // number of nodes includes sink node, number of sensors, and number of targets
    int numTargets = 441;
    int numNodes = nMax + numTargets + 1;

    SignatureCollection mySignatures(nMax - nMin + 1, nMin, nMax, numRegions);
//    SignatureCollection mySignatures(37, nMin, nMax, numRegions);
    mySignatures.ReadSignatures("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADPEntireSignatureCollection.txt");
    mySignatures.SetFailureDist(curShape, curScale);

    ValueFunctionReliabilityCollection myValues(nMissions, learningRate, Budget, nMin, nMax);
    myValues.ConstructValueFunction(nMissions, Budget, nMin, nMax);
//    myValues.InitializeValueFunction("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADPInitialValues.txt");
    myValues.InitializeValueFunctionStC("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADPInitialValues.txt");

    NodeLinkedListMC myNetworkBuckets(numNodes, (numNodes - numTargets), numTargets);

    myNetworkBuckets.SetRanges(0.0750000, 0.0750000);
    myNetworkBuckets.UniformTargetLoc();
    myNetworkBuckets.SetBinLimits(4);

    vector<double> subArea;
    vector<double> subWeights;

    myNetworkBuckets.SubregionArea(subArea);
    myNetworkBuckets.SubregionDistanceWeight(subWeights);
//    myNetworkBuckets.CustomWeights(subWeights);

    auto startTime = chrono::high_resolution_clock::now();

    for (int k = 1; k <= numReps ; ++k) {

        cout << "Start of Iteration " << k << endl;

        vector<double> gradientVect(numRegions,0);
        vector<int> numInRegion(numRegions, 20);
        double iterationCost = cFixed + (cVar * nSens);
        double missionSuccess = 0;

        budgetRemaining = Budget - iterationCost;

        // initial deployment of sensors and
        InitialSensorDeployment(gradientVect, numInRegion, subArea, subWeights, nSens);

        netSizeBeginning.push_back(0);
        missionDeployAction.push_back(nSens);

        // Generate initial fail times and set ages
        myNetworkBuckets.GenerateNewFailTime_ADPEval(nodeTime, nSens);
        myNetworkBuckets.UpdateFailTimesADP(nodeTime);
        myNetworkBuckets.ClearSensorAge();

        // Estimate reliability of initial state
        vector<int> ageState(maxAge, 0);
        ageState[0] = nSens;

        double initialRel = mySignatures.ReliabilityEstimate(delta, nSens, ageState);
//        double initialRel = mySignatures.ReliabilityEstimate_Approximation(delta, nSens, ageState);
        reliabilityRecord.push_back(initialRel);
        missionSuccess = missionSuccess + initialRel;

//        double postDecisionBudget = (double) ((budgetRemaining / 10) * 10);
        double postDecisionBudget = budgetRemaining;
        int postDecisionSize = (int) accumulate(ageState.begin(), ageState.end(),0) - nMin;

        double curTime = delta;
        int curMission = 1;

        // update state and determine post decision state variable
        ageState[0] = 0;
        myNetworkBuckets.UpdateState(curTime, delta, ageState, expectedNum);

        double bestDecisionBudget;
        int bestDecisionSize;

        while (curMission < nMissions) {

            int curNetworkSize = accumulate(ageState.begin(), ageState.end(),0);
            int curAction = nMax - curNetworkSize;

            netSizeBeginning.push_back(curNetworkSize);

            int maxAction = max(0, min(curAction, (int) floor((budgetRemaining - cFixed) / cVar)));

            int bestAction = 0;
            double bestValue = 0;
            double cost = 0;
            double tempValue;
            double bestRel = 0;
            double bestCost = 0;
            double tBudget;
            double rel;

            double myNum = myUnif(myDev);

            // Select action at random
            if (myNum < explorationProb){
                uniform_int_distribution<> myRandomDeploy(0, maxAction);

                bestAction = myRandomDeploy(myRandomOrder);

                ageState[0] = bestAction;

                if (bestAction > 0){
                    tBudget = budgetRemaining - cFixed - (cVar * bestAction);
                    cost = cFixed + cVar * bestAction;
                }
                else {
                    tBudget = budgetRemaining;
                    cost = 0;
                }

                int tempNetSize = curNetworkSize + bestAction;
                netValueIndex = tempNetSize - nMin;

                if (tempNetSize < nMin){
                    rel = 0;
                    netValueIndex = 0;
                }
                else{
                    rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
//                    rel = mySignatures.ReliabilityEstimate_Approximation(delta, tempNetSize, ageState);
                }

                bestValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

                bestCost = cost;
                bestRel = rel;

                bestDecisionBudget = tBudget;
                bestDecisionSize = netValueIndex;

            }
            else {
                // Search for optimal action

                // action: decide not to deploy any new sensors
                netValueIndex = curNetworkSize - nMin;
                if (curNetworkSize < nMin){
                    rel = 0;
                    netValueIndex = 0;
                }
                else{
                    rel = mySignatures.ReliabilityEstimate(delta, curNetworkSize, ageState);
//                    rel = mySignatures.ReliabilityEstimate_Approximation(delta, curNetworkSize, ageState);
                    bestRel = rel;
                }

                bestDecisionBudget = budgetRemaining;
                bestDecisionSize = netValueIndex;

                bestValue = rel + myValues.GetStateValue(curMission, budgetRemaining, netValueIndex);
                bestAction = 0;

                curAction = maxAction;

                while (curAction > minDeploy){

                    ageState[0] = curAction;
                    tBudget = budgetRemaining - cFixed - (cVar * curAction);

                    cost = cFixed + cVar * curAction;

                    int tempNetSize = curNetworkSize + curAction;
                    netValueIndex = tempNetSize - nMin;

                    if (tempNetSize < nMin){
                        rel = 0;
                        netValueIndex = 0;
                    }
                    else{
                        rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
//                        rel = mySignatures.ReliabilityEstimate_Approximation(delta, tempNetSize, ageState);
                    }

                    tempValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

                    if (tempValue > bestValue){
                        bestValue = tempValue;
                        bestAction = curAction;
                        bestCost = cost;
                        bestRel = rel;

                        bestDecisionBudget = tBudget;
                        bestDecisionSize = netValueIndex;
                    }
                    curAction = curAction - deployInterval;
                }
            }

//            cout << "       curMission " << curMission << " - best : " << bestAction << " - rel: " << bestRel  << " - bestvalue : " << bestValue << endl;

            double tAlpha = myValues.GetStepSize(curMission - 1, postDecisionBudget, postDecisionSize);
            double tValue = myValues.GetStateValue(curMission - 1, postDecisionBudget, postDecisionSize);

            double newValue = ((1.0 - tAlpha) * tValue) + (tAlpha * bestValue);

            myValues.UpdateTempStateValue(curMission - 1, postDecisionBudget, postDecisionSize, newValue);
            myValues.ADP_MonotoneUpdate(curMission - 1, postDecisionBudget, postDecisionSize, bestValue);

            postDecisionBudget = bestDecisionBudget;
            postDecisionSize = bestDecisionSize;

            // redeploy sensors, and generate a new fail time for those added
            myNetworkBuckets.GenerateNewFailTimeADP_Update(nodeTime, (double) nMissions*delta, curTime, bestAction);

            iterationCost = iterationCost + bestCost;
            missionSuccess = missionSuccess + bestRel;
            reliabilityRecord.push_back(bestRel);
            missionDeployAction.push_back(bestAction);
            budgetRemaining = budgetRemaining - bestCost;

            curTime = curTime + delta;
            curMission = curMission + 1;

            vector<int> ageVector(maxAge, 0);
            myNetworkBuckets.UpdateState(curTime, delta, ageVector, expectedNum);
            copy(ageVector.begin(), ageVector.end(), ageState.begin());
        }
        costRecord.push_back(iterationCost);
        cumulativeMissionSuccess.push_back(missionSuccess);

//        myValues.UpdateValueFunctionsConstant();
        myValues.UpdateValueFunctionsStC(learningRate);
    }

    auto finishTime = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsedNetwork = finishTime - startTime;

    cout << "Total Time Required : " << elapsedNetwork.count() << endl;

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print out value functions
    cout << " Print Value Functions " << endl;
    myValues.PrintValueFunctions();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myObj;
    myObj.open ("CostFunctionImprovement.txt");

    for (int i = 0; i < costRecord.size() ; ++i) {
        myObj << "Iteration " << i << " -- ObjectiveValue : " << costRecord.at(i) << endl;
    }

    myObj.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // print out reliability over time
    ofstream myReliabilityFile;
    myReliabilityFile.open("ReliabilityRecord.txt");

    for (int j = 0; j < reliabilityRecord.size(); ++j) {
        myReliabilityFile << "Record " << j << " -- Reliability : " << reliabilityRecord.at(j) << endl;
    }

    myReliabilityFile.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream mySuccess;
    mySuccess.open ("ExpectedNumSuccessfulMissions.txt");

    for (int i = 0; i < cumulativeMissionSuccess.size() ; ++i) {
        mySuccess << "Iteration " << i << " -- ObjectiveValue : " << cumulativeMissionSuccess.at(i) << endl;
    }

    mySuccess.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myActions;
    myActions.open ("NetworkSizeAndAction.txt");

    for (int i = 0; i < missionDeployAction.size() ; ++i) {
        myActions << "Iteration " << i << " -- NetworkSize : " << netSizeBeginning.at(i) << " -- Action : " << missionDeployAction.at(i) << endl;
    }

    myActions.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    ofstream myNotes;
    myNotes.open("Notes.txt");

    myNotes << "Time Required : " << elapsedNetwork.count() << endl;
    myNotes << "Number of Replications : " << numReps << endl;
    myNotes << "Budget Available : " << Budget << endl;
    myNotes << "Number of Missions : " << nMissions << endl;
    myNotes << "Time Between Maintenance Inspections : " << delta << endl;
    myNotes << "Minimum number of sensors to deploy : " << minDeploy << endl;
    myNotes << "Sensor Deployment Interval : " << deployInterval << endl;
    myNotes << "Minimum Network Size : " << nMin << endl;
    myNotes << "Maximum Network Size : " << nMax << endl;

    myNotes.close();

}

// CBM ADP model using AVI look up tables. The objective is to maximize the number of successful missions subject to a budget constraint.
// Uses a neighborhood of the current state to update the value functions for states `close' to the one that is actually visited.
// Once the value functions are updated a monte carlo simulation is performed to evaluate the performance of the resulting maintenance policy
void ADP_CBM_AVI_ReliabilityModel_CAVE_EvaluateMC(int nSens, int numReps, int nMissions, double delta, double Budget,
                                       double cFixed, double cVar, double covRequirement) {

    uniform_real_distribution<> myUnif(0,1);

    random_device myDev;
    random_device myRandomOrder;

    const int maxAge = 15;
    const int numRegions = 16;
    const int nMin = 300;
    const int nMax = 950;
    const int deployInterval = 10;
    const int minDeploy = 30;
    const double explorationProb = 0.05;
    const double learningRate = 0.7;

    double curShape = 1.5;
    double curScale = 10.0;
    double expectedNum;
    int netValueIndex;
    double budgetRemaining;

    vector<double> nodeTime;
    vector<double> costRecord;
    vector<double> reliabilityRecord;
    vector<double> cumulativeMissionSuccess;
    vector<int> netSizeBeginning;
    vector<int> missionDeployAction;

    // number of nodes includes sink node, number of sensors, and number of targets
    int numTargets = 441;
    int numNodes = nMax + numTargets + 1;

    SignatureCollection mySignatures(nMax - nMin + 1, nMin, nMax, numRegions);
    mySignatures.ReadSignatures("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADPEntireSignatureCollection_d1_2.txt");
    mySignatures.SetFailureDist(curShape, curScale);

    ValueFunctionReliabilityCollection myValues(nMissions, learningRate, Budget, nMin, nMax);
    myValues.ConstructValueFunction(nMissions, Budget, nMin, nMax);
//    myValues.InitializeValueFunction("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADPInitialValues.txt");
    myValues.InitializeValueFunctionStC("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADPInitialValues_d4_m25_new.txt");

    NodeLinkedListMC myNetworkBuckets(numNodes, (numNodes - numTargets), numTargets);

    myNetworkBuckets.SetRanges(0.0750000, 0.0750000);
    myNetworkBuckets.UniformTargetLoc();
    myNetworkBuckets.SetBinLimits(4);

    vector<double> subArea;
    vector<double> subWeights;

    myNetworkBuckets.SubregionArea(subArea);
    myNetworkBuckets.SubregionDistanceWeight(subWeights);
//    myNetworkBuckets.CustomWeights(subWeights);

    auto startTime = chrono::high_resolution_clock::now();

    for (int k = 1; k <= numReps ; ++k) {

        cout << "Start of Iteration " << k << endl;

        vector<double> gradientVect(numRegions,0);
        vector<int> numInRegion(numRegions, 20);
        double iterationCost = cFixed + (cVar * nSens);
        double missionSuccess = 0;

        budgetRemaining = Budget - iterationCost;

        // initial deployment of sensors and
        InitialSensorDeployment(gradientVect, numInRegion, subArea, subWeights, nSens);

        netSizeBeginning.push_back(0);
        missionDeployAction.push_back(nSens);

        // Generate initial fail times and set ages
        myNetworkBuckets.GenerateNewFailTime_ADPEval(nodeTime, nSens);
        myNetworkBuckets.UpdateFailTimesADP(nodeTime);
        myNetworkBuckets.ClearSensorAge();

        // Estimate reliability of initial state
        vector<int> ageState(maxAge, 0);
        ageState[0] = nSens;

        double initialRel = mySignatures.ReliabilityEstimate(delta, nSens, ageState);
        reliabilityRecord.push_back(initialRel);
        missionSuccess = missionSuccess + initialRel;

//        double postDecisionBudget = (double) ((budgetRemaining / 10) * 10);
        double postDecisionBudget = budgetRemaining;
        int postDecisionSize = (int) accumulate(ageState.begin(), ageState.end(),0) - nMin;

        double curTime = delta;
        int curMission = 1;

        // update state and determine post decision state variable
        ageState[0] = 0;
        myNetworkBuckets.UpdateState(curTime, delta, ageState, expectedNum);

        double bestDecisionBudget;
        int bestDecisionSize;

        while (curMission < nMissions) {

            int curNetworkSize = accumulate(ageState.begin(), ageState.end(),0);
            int curAction = nMax - curNetworkSize;

            netSizeBeginning.push_back(curNetworkSize);

            int maxAction = max(0, min(curAction, (int) floor((budgetRemaining - cFixed) / cVar)));

            int bestAction = 0;
            double bestValue = 0;
            double cost = 0;
            double tempValue;
            double bestRel = 0;
            double bestCost = 0;
            double tBudget;
            double rel;

            double myNum = myUnif(myDev);

            // Select action at random
            if (myNum < explorationProb){
                uniform_int_distribution<> myRandomDeploy(0, maxAction);

                bestAction = myRandomDeploy(myRandomOrder);

                ageState[0] = bestAction;

                if (bestAction > 0){
                    tBudget = budgetRemaining - cFixed - (cVar * bestAction);
                    cost = cFixed + cVar * bestAction;
                }
                else {
                    tBudget = budgetRemaining;
                    cost = 0;
                }

                int tempNetSize = curNetworkSize + bestAction;
                netValueIndex = tempNetSize - nMin;

                if (tempNetSize < nMin){
                    rel = 0;
                    netValueIndex = 0;
                }
                else{
                    rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
                }

                bestValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

                bestCost = cost;
                bestRel = rel;

                bestDecisionBudget = tBudget;
                bestDecisionSize = netValueIndex;

            }
            else {
                // Search for optimal action

                // action: decide not to deploy any new sensors
                netValueIndex = curNetworkSize - nMin;
                if (curNetworkSize < nMin){
                    rel = 0;
                    netValueIndex = 0;
                }
                else{
                    rel = mySignatures.ReliabilityEstimate(delta, curNetworkSize, ageState);
                    bestRel = rel;
                }

                bestDecisionBudget = budgetRemaining;
                bestDecisionSize = netValueIndex;

                bestValue = rel + myValues.GetStateValue(curMission, budgetRemaining, netValueIndex);
                bestAction = 0;

                curAction = maxAction;

                while (curAction > minDeploy){

                    ageState[0] = curAction;
                    tBudget = budgetRemaining - cFixed - (cVar * curAction);

                    cost = cFixed + cVar * curAction;

                    int tempNetSize = curNetworkSize + curAction;
                    netValueIndex = tempNetSize - nMin;

                    if (tempNetSize < nMin){
                        rel = 0;
                        netValueIndex = 0;
                    }
                    else{
                        rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
                    }

                    tempValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

                    if (tempValue > bestValue){
                        bestValue = tempValue;
                        bestAction = curAction;
                        bestCost = cost;
                        bestRel = rel;

                        bestDecisionBudget = tBudget;
                        bestDecisionSize = netValueIndex;
                    }
                    curAction = curAction - deployInterval;
                }
            }

//            cout << "       curMission " << curMission << " - best : " << bestAction << " - rel: " << bestRel  << " - bestvalue : " << bestValue << endl;

            double tAlpha = myValues.GetStepSize(curMission - 1, postDecisionBudget, postDecisionSize);
            double tValue = myValues.GetStateValue(curMission - 1, postDecisionBudget, postDecisionSize);

            double newValue = ((1.0 - tAlpha) * tValue) + (tAlpha * bestValue);

            myValues.UpdateTempStateValue(curMission - 1, postDecisionBudget, postDecisionSize, newValue);
            myValues.ADP_MonotoneUpdate(curMission - 1, postDecisionBudget, postDecisionSize, bestValue);

            postDecisionBudget = bestDecisionBudget;
            postDecisionSize = bestDecisionSize;

            // redeploy sensors, and generate a new fail time for those added
            myNetworkBuckets.GenerateNewFailTimeADP_Update(nodeTime, (double) nMissions*delta, curTime, bestAction);

            iterationCost = iterationCost + bestCost;
            missionSuccess = missionSuccess + bestRel;
            reliabilityRecord.push_back(bestRel);
            missionDeployAction.push_back(bestAction);
            budgetRemaining = budgetRemaining - bestCost;

            curTime = curTime + delta;
            curMission = curMission + 1;

            vector<int> ageVector(maxAge, 0);
            myNetworkBuckets.UpdateState(curTime, delta, ageVector, expectedNum);
            copy(ageVector.begin(), ageVector.end(), ageState.begin());
        }
        costRecord.push_back(iterationCost);
        cumulativeMissionSuccess.push_back(missionSuccess);

//        myValues.UpdateValueFunctionsConstant();
        myValues.UpdateValueFunctionsStC(learningRate);
    }

    auto finishValueFunctionApprox = chrono::high_resolution_clock::now();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print out value functions
    cout << " Print Value Functions " << endl;
    myValues.PrintValueFunctions();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myObj;
    myObj.open ("CostFunctionImprovement.txt");

    for (int i = 0; i < costRecord.size() ; ++i) {
        myObj << "Iteration " << i << " -- ObjectiveValue : " << costRecord.at(i) << endl;
    }

    myObj.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // print out reliability over time
    ofstream myReliabilityFile;
    myReliabilityFile.open("ReliabilityRecord.txt");

    for (int j = 0; j < reliabilityRecord.size(); ++j) {
        myReliabilityFile << "Record " << j << " -- Reliability : " << reliabilityRecord.at(j) << endl;
    }

    myReliabilityFile.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream mySuccess;
    mySuccess.open ("ExpectedNumSuccessfulMissions.txt");

    for (int i = 0; i < cumulativeMissionSuccess.size() ; ++i) {
        mySuccess << "Iteration " << i << " -- ObjectiveValue : " << cumulativeMissionSuccess.at(i) << endl;
    }

    mySuccess.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myActions;
    myActions.open ("NetworkSizeAndAction.txt");

    for (int i = 0; i < missionDeployAction.size() ; ++i) {
        myActions << "Iteration " << i << " -- NetworkSize : " << netSizeBeginning.at(i) << " -- Action : " << missionDeployAction.at(i) << endl;
    }

    myActions.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // MC simulation to evaluate performance of value functions
    auto startEval = chrono::high_resolution_clock::now();

    numNodes = nMax + numTargets + 1;
    NodeLinkedListMC myNetworkBuckets_ADPEval(numNodes, (numNodes - numTargets), numTargets);

    myNetworkBuckets_ADPEval.SetRanges(0.0750000, 0.0750000);
    myNetworkBuckets_ADPEval.UniformTargetLoc();
    myNetworkBuckets_ADPEval.SetBinLimits(4);

    vector<double> subAreaMC;
    vector<double> subWeightsMC;

    myNetworkBuckets_ADPEval.SubregionArea(subAreaMC);
    myNetworkBuckets_ADPEval.SubregionDistanceWeight(subWeightsMC);

    costRecord.clear();
    reliabilityRecord.clear();
    cumulativeMissionSuccess.clear();
    netSizeBeginning.clear();
    missionDeployAction.clear();

    double timePlace;
    vector<int> netStatus(nMissions,0);
    vector<int> successRecord;

    for (int k = 1; k <= numReps ; ++k) {

        cout << "Start of MC Iteration " << k << endl;

        vector<double> gradientVect(numRegions,0);
        vector<int> numInRegion(numRegions, 20);
        double iterationCost = cFixed + (cVar * nSens);
        double missionSuccess = 0;
        int totalSuccess = 0;

        budgetRemaining = Budget - iterationCost;

        // initial deployment of sensors and
        InitialSensorDeployment(gradientVect, numInRegion, subAreaMC, subWeightsMC, nSens);

        netSizeBeginning.push_back(0);
        missionDeployAction.push_back(nSens);

        // Generate initial fail times and set ages
        myNetworkBuckets_ADPEval.GenerateNewFailTime_ADPEval(nodeTime, nSens, (double) nMissions*delta);
        myNetworkBuckets_ADPEval.UpdateFailTimes(nodeTime);

        // Estimate reliability of initial state
        vector<int> ageState(maxAge, 0);
        vector<int> regionState(numRegions, 0);

        myNetworkBuckets_ADPEval.GenerateNewSpecificRGG_Bucket_ADPEval(numInRegion);

        ageState[0] = nSens;
        double initialRel = mySignatures.ReliabilityEstimate(delta, nSens, ageState);
        reliabilityRecord.push_back(initialRel);
        missionSuccess = missionSuccess + initialRel;

        double tarCovered;
        double curTime = delta;
        int curMission = 1;

        ageState[0] = 0;
        // update state and determine post decision state variable
        myNetworkBuckets_ADPEval.UpdateState(curTime, delta, regionState, ageState, expectedNum);

        while (curMission < nMissions) {

            myNetworkBuckets_ADPEval.UpdateRGG_ADPEval(curTime);
            myNetworkBuckets_ADPEval.UpdateFailTimes(nodeTime);

            tarCovered = myNetworkBuckets_ADPEval.BreadthFirstSearchReturn_ADPEval(numNodes, numTargets, timePlace);

            if (tarCovered >= covRequirement){
                netStatus[curMission-1] = netStatus[curMission-1] + 1;
                totalSuccess = totalSuccess + 1;
            }

            int curNetworkSize = accumulate(ageState.begin(), ageState.end(),0);
            int curAction = nMax - curNetworkSize;

            netSizeBeginning.push_back(curNetworkSize);

            int maxAction = max(0, min(curAction, (int) floor((budgetRemaining - cFixed) / cVar)));

            int bestAction = 0;
            double bestValue = 0;
            double cost = 0;
            double tempValue;
            double bestRel = 0;
            double bestCost = 0;
            double tBudget;
            double rel;

            // Search for optimal action
            // action: decide not to deploy any new sensors
            netValueIndex = curNetworkSize - nMin;
            if (curNetworkSize < nMin){
                rel = 0;
                netValueIndex = 0;
            }
            else{
                rel = mySignatures.ReliabilityEstimate(delta, curNetworkSize, ageState);
                bestRel = rel;
            }

            bestValue = rel + myValues.GetStateValue(curMission, budgetRemaining, netValueIndex);
            bestAction = 0;

            curAction = maxAction;

            while (curAction > minDeploy){

                ageState[0] = curAction;
                tBudget = budgetRemaining - cFixed - (cVar * curAction);

                cost = cFixed + cVar * curAction;

                int tempNetSize = curNetworkSize + curAction;
                netValueIndex = tempNetSize - nMin;

                if (tempNetSize < nMin){
                    rel = 0;
                    netValueIndex = 0;
                }
                else{
                    rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
                }

                tempValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

                if (tempValue > bestValue){
                    bestValue = tempValue;
                    bestAction = curAction;
                    bestCost = cost;
                    bestRel = rel;
                }
                curAction = curAction - deployInterval;
            }

//            vector<int> regionDeployment(numRegions, 0);
            vector<int> regionDeployment;

            SensorDeploymentAction(regionState, subAreaMC, subWeightsMC, bestAction, regionDeployment);

            // redeploy sensors, and generate a new fail time for those added
            myNetworkBuckets_ADPEval.GenerateNewFailTimeADP_ADPEval(nodeTime, (double) nMissions*delta, curTime, regionDeployment);

            myNetworkBuckets_ADPEval.GenerateNewRGG_ADPMaintAction();

            iterationCost = iterationCost + bestCost;
            missionSuccess = missionSuccess + bestRel;
            reliabilityRecord.push_back(bestRel);
            missionDeployAction.push_back(bestAction);
            budgetRemaining = budgetRemaining - bestCost;

            curTime = curTime + delta;
            curMission = curMission + 1;

            vector<int> ageVector(maxAge, 0);
            vector<int> regionVector(numRegions, 0);

            myNetworkBuckets_ADPEval.UpdateState(curTime, delta, regionVector, ageVector, expectedNum);
            copy(ageVector.begin(), ageVector.end(), ageState.begin());
            copy(regionVector.begin(), regionVector.end(), regionState.begin());

        }

        myNetworkBuckets_ADPEval.UpdateRGG_ADPEval(curTime);
        myNetworkBuckets_ADPEval.UpdateFailTimes(nodeTime);
        tarCovered = myNetworkBuckets_ADPEval.BreadthFirstSearchReturn_ADPEval(numNodes, numTargets, timePlace);
        if (tarCovered >= covRequirement){
            netStatus[curMission-1] = netStatus[curMission-1] + 1;
            totalSuccess = totalSuccess + 1;
        }

        costRecord.push_back(iterationCost);
        cumulativeMissionSuccess.push_back(missionSuccess);
        successRecord.push_back(totalSuccess);
    }

    int tSuccess = 0;
    for (int m = 0; m < successRecord.size() ; ++m) {
        tSuccess = tSuccess + successRecord.at(m);
    }

    auto totalTime = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsedNetwork = totalTime - startTime;

    chrono::duration<double> approxTime = finishValueFunctionApprox - startTime;
    chrono::duration<double> evalTime = totalTime - startEval;

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myObjMC;
    myObjMC.open ("CostFunctionMC.txt");

    for (int i = 0; i < costRecord.size() ; ++i) {
        myObjMC << "Iteration " << i << " -- ObjectiveValue : " << costRecord.at(i) << endl;
    }

    myObjMC.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // print out reliability over time
    ofstream myReliabilityFileMC;
    myReliabilityFileMC.open("ReliabilityRecordMC.txt");

    for (int j = 0; j < reliabilityRecord.size(); ++j) {
        myReliabilityFileMC << "Record " << j << " -- Reliability : " << reliabilityRecord.at(j) << endl;
    }

    myReliabilityFileMC.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream mySuccessMC;
    mySuccessMC.open ("ExpectedNumSuccessfulMissionsMC.txt");

    for (int i = 0; i < cumulativeMissionSuccess.size() ; ++i) {
        mySuccessMC << "Iteration " << i << " -- ObjectiveValue : " << cumulativeMissionSuccess.at(i) << endl;
    }

    mySuccessMC.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    cout << "Observed Average Number Successful Mission : " << (double) tSuccess / (double) numReps << endl;

    ofstream myObservedSuccessMC;
    myObservedSuccessMC.open("ObservedNumSuccessfulMissionsMC.txt");
    for (int l = 0; l <netStatus.size() ; ++l) {
        myObservedSuccessMC << "Mission : " << l << " - Status : " << netStatus.at(l) << " of " << numReps << endl;
        cout << "Mission : " << l << " - Status : " << netStatus.at(l) << " of " << numReps << endl;
    }

    for (int l = 0; l < successRecord.size(); ++l) {
        myObservedSuccessMC << "record " << l << " status : " << successRecord.at(l) << endl;
    }

    myObservedSuccessMC.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myActionsMC;
    myActionsMC.open ("NetworkSizeAndActionMC.txt");

    for (int i = 0; i < missionDeployAction.size() ; ++i) {
        myActionsMC << "Iteration " << i << " -- NetworkSize : " << netSizeBeginning.at(i) << " -- Action : " << missionDeployAction.at(i) << endl;
    }

    myActionsMC.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    ofstream myNotes;
    myNotes.open("Notes.txt");

    myNotes << "Total Time Required : " << elapsedNetwork.count() << endl;
    myNotes << "  Value Function Approximation Time : " << approxTime.count() << endl;
    myNotes << "  Policy Evaluation Time : " << evalTime.count() << endl;
    myNotes << "Number of Replications : " << numReps << endl;
    myNotes << "Budget Available : " << Budget << endl;
    myNotes << "Number of Missions : " << nMissions << endl;
    myNotes << "Time Between Maintenance Inspections : " << delta << endl;
    myNotes << "Minimum number of sensors to deploy : " << minDeploy << endl;
    myNotes << "Sensor Deployment Interval : " << deployInterval << endl;
    myNotes << "Minimum Network Size : " << nMin << endl;
    myNotes << "Maximum Network Size : " << nMax << endl;
    myNotes << "Observed Average Number Successful Mission : " << (double) tSuccess / (double) numReps << endl;

    myNotes.close();

    cout << "Total Time Required : " << elapsedNetwork.count() << endl;
    cout << "  Value Function Approximation Time : " << approxTime.count() << endl;
    cout << "  Policy Evaluation Time : " << evalTime.count() << endl;

}


// The objective is to maximize the number of successful missions subject to a budget constraint. Reads in a set of value functions and then
// a monte carlo simulation is performed to evaluate the performance of the resulting maintenance policy.
void ADP_CBM_InputValueFunction_EvaluateMC(int nSens, int numReps, int nMissions, double delta, double Budget,
                                                  double cFixed, double cVar, double covRequirement) {


    const int maxAge = 10;
    const int numRegions = 16;
    const int nMin = 384;
    const int nMax = 950;
    const int deployInterval = 10;
    const int minDeploy = 30;
    const double learningRate = 0.7;

    double curShape = 1.5;
    double curScale = 10.0;
    double expectedNum;
    int netValueIndex;
    double budgetRemaining;

    vector<double> nodeTime;
    vector<double> costRecord;
    vector<double> reliabilityRecord;
    vector<double> cumulativeMissionSuccess;
    vector<int> netSizeBeginning;
    vector<int> missionDeployAction;
    vector<int> successRecord;

    // number of nodes includes sink node, number of sensors, and number of targets
    int numTargets = 441;

    SignatureCollection mySignatures(567, nMin, nMax, numRegions);
    mySignatures.ReadSignatures("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADPEntireSignatureCollection_d5.txt");
    mySignatures.SetFailureDist(curShape, curScale);

    ValueFunctionReliabilityCollection myValues(nMissions, learningRate, Budget, nMin, nMax);
    myValues.ConstructValueFunction(nMissions, Budget, nMin, nMax);
    myValues.ReadValueFunctions();

    auto startTime = chrono::high_resolution_clock::now();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // MC simulation to evaluate performance of value functions

    int numNodes = nMax + numTargets + 1;
    NodeLinkedListMC myNetworkBuckets_ADPEval(numNodes, (numNodes - numTargets), numTargets);

    myNetworkBuckets_ADPEval.SetRanges(0.0750000, 0.0750000);
    myNetworkBuckets_ADPEval.UniformTargetLoc();
    myNetworkBuckets_ADPEval.SetBinLimits(4);

    vector<double> subAreaMC;
    vector<double> subWeightsMC;

    myNetworkBuckets_ADPEval.SubregionArea(subAreaMC);
    myNetworkBuckets_ADPEval.SubregionDistanceWeight(subWeightsMC);

    costRecord.clear();
    reliabilityRecord.clear();
    cumulativeMissionSuccess.clear();
    netSizeBeginning.clear();
    missionDeployAction.clear();

    double timePlace;
    vector<int> netStatus(nMissions,0);

    for (int k = 1; k <= numReps ; ++k) {

        cout << "Start of MC Iteration " << k << endl;

        vector<double> gradientVect(numRegions,0);
        vector<int> numInRegion(numRegions, 20);
        double iterationCost = cFixed + (cVar * nSens);
        double missionSuccess = 0;
        int totalSuccess = 0;

        budgetRemaining = Budget - iterationCost;

        // initial deployment of sensors and
        InitialSensorDeployment(gradientVect, numInRegion, subAreaMC, subWeightsMC, nSens);

        netSizeBeginning.push_back(0);
        missionDeployAction.push_back(nSens);

        // Generate initial fail times and set ages
        myNetworkBuckets_ADPEval.GenerateNewFailTime_ADPEval(nodeTime, nSens, (double) nMissions*delta);
        myNetworkBuckets_ADPEval.UpdateFailTimes(nodeTime);

        // Estimate reliability of initial state
        vector<int> ageState(maxAge, 0);
        vector<int> regionState(numRegions, 0);

        myNetworkBuckets_ADPEval.GenerateNewSpecificRGG_Bucket_ADPEval(numInRegion);

        ageState[0] = nSens;
        double initialRel = mySignatures.ReliabilityEstimate(delta, nSens, ageState);
        reliabilityRecord.push_back(initialRel);
        missionSuccess = missionSuccess + initialRel;

        double tarCovered;
        double curTime = delta;
        int curMission = 1;

        ageState[0] = 0;
        // update state and determine post decision state variable
        myNetworkBuckets_ADPEval.UpdateState(curTime, delta, regionState, ageState, expectedNum);

        while (curMission < nMissions) {

            myNetworkBuckets_ADPEval.UpdateRGG_ADPEval(curTime);
            myNetworkBuckets_ADPEval.UpdateFailTimes(nodeTime);

            tarCovered = myNetworkBuckets_ADPEval.BreadthFirstSearchReturn_ADPEval(numNodes, numTargets, timePlace);

            if (tarCovered >= covRequirement){
                netStatus[curMission-1] = netStatus[curMission-1] + 1;
                totalSuccess = totalSuccess + 1;
            }

            int curNetworkSize = accumulate(ageState.begin(), ageState.end(),0);
            int curAction = nMax - curNetworkSize;

            netSizeBeginning.push_back(curNetworkSize);

            int maxAction = max(0, min(curAction, (int) floor((budgetRemaining - cFixed) / cVar)));

            int bestAction = 0;
            double bestValue = 0;
            double cost = 0;
            double tempValue;
            double bestRel = 0;
            double bestCost = 0;
            double tBudget;
            double rel;

            // Search for optimal action
            // action: decide not to deploy any new sensors
            netValueIndex = curNetworkSize - nMin;
            if (curNetworkSize < nMin){
                rel = 0;
                netValueIndex = 0;
            }
            else{
                rel = mySignatures.ReliabilityEstimate(delta, curNetworkSize, ageState);
                bestRel = rel;
            }

            bestValue = rel + myValues.GetStateValue(curMission, budgetRemaining, netValueIndex);
            bestAction = 0;

            curAction = maxAction;

            while (curAction > minDeploy){

                ageState[0] = curAction;
                tBudget = budgetRemaining - cFixed - (cVar * curAction);

                cost = cFixed + cVar * curAction;

                int tempNetSize = curNetworkSize + curAction;
                netValueIndex = tempNetSize - nMin;

                if (tempNetSize < nMin){
                    rel = 0;
                    netValueIndex = 0;
                }
                else{
                    rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
                }

                tempValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

                if (tempValue > bestValue){
                    bestValue = tempValue;
                    bestAction = curAction;
                    bestCost = cost;
                    bestRel = rel;
                }
                curAction = curAction - deployInterval;
            }

//            vector<int> regionDeployment(numRegions, 0);
            vector<int> regionDeployment;

            SensorDeploymentAction(regionState, subAreaMC, subWeightsMC, bestAction, regionDeployment);

            // redeploy sensors, and generate a new fail time for those added
            myNetworkBuckets_ADPEval.GenerateNewFailTimeADP_ADPEval(nodeTime, (double) nMissions*delta, curTime, regionDeployment);

            myNetworkBuckets_ADPEval.GenerateNewRGG_ADPMaintAction();

            iterationCost = iterationCost + bestCost;
            missionSuccess = missionSuccess + bestRel;
            reliabilityRecord.push_back(bestRel);
            missionDeployAction.push_back(bestAction);
            budgetRemaining = budgetRemaining - bestCost;

            curTime = curTime + delta;
            curMission = curMission + 1;

            vector<int> ageVector(maxAge, 0);
            vector<int> regionVector(numRegions, 0);

            myNetworkBuckets_ADPEval.UpdateState(curTime, delta, regionVector, ageVector, expectedNum);
            copy(ageVector.begin(), ageVector.end(), ageState.begin());
            copy(regionVector.begin(), regionVector.end(), regionState.begin());

        }

        myNetworkBuckets_ADPEval.UpdateRGG_ADPEval(curTime);
        myNetworkBuckets_ADPEval.UpdateFailTimes(nodeTime);
        tarCovered = myNetworkBuckets_ADPEval.BreadthFirstSearchReturn_ADPEval(numNodes, numTargets, timePlace);
        if (tarCovered >= covRequirement){
            netStatus[curMission-1] = netStatus[curMission-1] + 1;
            totalSuccess = totalSuccess + 1;
        }

        costRecord.push_back(iterationCost);
        cumulativeMissionSuccess.push_back(missionSuccess);
        successRecord.push_back(totalSuccess);
    }

    int tSuccess = 0;
    for (int m = 0; m < successRecord.size() ; ++m) {
        tSuccess = tSuccess + successRecord.at(m);
    }

    auto finishTime = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsedNetwork = finishTime - startTime;

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myObjMC;
    myObjMC.open ("CostFunctionMC.txt");

    for (int i = 0; i < costRecord.size() ; ++i) {
        myObjMC << "Iteration " << i << " -- CostValue : " << costRecord.at(i) << endl;
    }

    myObjMC.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // print out reliability over time
    ofstream myReliabilityFileMC;
    myReliabilityFileMC.open("ReliabilityRecordMC.txt");

    for (int j = 0; j < reliabilityRecord.size(); ++j) {
        myReliabilityFileMC << "Record " << j << " -- Reliability : " << reliabilityRecord.at(j) << endl;
    }

    myReliabilityFileMC.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream mySuccessMC;
    mySuccessMC.open ("ExpectedNumSuccessfulMissionsMC.txt");

    for (int i = 0; i < cumulativeMissionSuccess.size() ; ++i) {
        mySuccessMC << "Iteration " << i << " -- ObjectiveValue : " << cumulativeMissionSuccess.at(i) << endl;
    }

    mySuccessMC.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    cout << "Observed Average Number Successful Mission : " << (double) tSuccess / (double) numReps << endl;

    ofstream myObservedSuccessMC;
    myObservedSuccessMC.open("ObservedNumSuccessfulMissionsMC.txt");
    for (int l = 0; l < netStatus.size() ; ++l) {
        myObservedSuccessMC << "Mission : " << l << " - Status : " << netStatus.at(l) << " of " << numReps << endl;
        cout << "Mission : " << l << " - Status : " << netStatus.at(l) << " of " << numReps << endl;
    }

    for (int l = 0; l < successRecord.size(); ++l) {
        myObservedSuccessMC << "record " << l << " status : " << successRecord.at(l) << endl;
    }

    myObservedSuccessMC.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myActionsMC;
    myActionsMC.open ("NetworkSizeAndActionMC.txt");

    for (int i = 0; i < missionDeployAction.size() ; ++i) {
        myActionsMC << "Iteration " << i << " -- NetworkSize : " << netSizeBeginning.at(i) << " -- Action : " << missionDeployAction.at(i) << endl;
    }

    myActionsMC.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    ofstream myNotes;
    myNotes.open("Notes.txt");

    myNotes << "Time Required : " << elapsedNetwork.count() << endl;
    myNotes << "Number of Replications : " << numReps << endl;
    myNotes << "Budget Available : " << Budget << endl;
    myNotes << "Number of Missions : " << nMissions << endl;
    myNotes << "Time Between Maintenance Inspections : " << delta << endl;
    myNotes << "Minimum number of sensors to deploy : " << minDeploy << endl;
    myNotes << "Sensor Deployment Interval : " << deployInterval << endl;
    myNotes << "Minimum Network Size : " << nMin << endl;
    myNotes << "Maximum Network Size : " << nMax << endl;
    myNotes << "Observed Average Number Successful Mission : " << (double) tSuccess / (double) numReps << endl;

    myNotes.close();

    cout << "Total Time Required : " << elapsedNetwork.count() << endl;

}

// CBM ADP model using AVI look up tables. The objective is to maximize the number of successful missions subject to a budget constraint.
// Uses a neighborhood of the current state to update the value functions for states `close' to the one that is actually visited.
// Once the value functions are updated a monte carlo simulation is performed to evaluate the performance of the resulting maintenance policy
// This function also requires a minimum reliability requirement, where each mission must achieve a reliability above this input
void ADP_CBM_AVI_ReliabilityModel_CAVE_EvaluateMC_MinRequirement(int nSens, int numReps, int nMissions, double delta, double Budget,
                                                  double cFixed, double cVar, double covRequirement, double relRequirement) {

    uniform_real_distribution<> myUnif(0,1);

    random_device myDev;
    random_device myRandomOrder;

    const int numRegions = 16;
    const int maxAge = 24;
    const int nMin = 300;
    const int nMax = 950;
    const int deployInterval = 10;
    const int minDeploy = 190;
    const double learningRate = 0.7;

    double explorationProb = 0.05;
    double curShape = 1.5;
    double curScale = 10.0;
    double expectedNum;
    int netValueIndex;
    double budgetRemaining;

    vector<double> nodeTime;
    vector<double> costRecord;
    vector<double> reliabilityRecord;
    vector<double> cumulativeMissionSuccess;
    vector<double> valueFunctionPerIteration;
    vector<int> netSizeBeginning;
    vector<int> missionDeployAction;

    // number of nodes includes sink node, number of sensors, and number of targets
    int numTargets = 441;
    int numNodes = nMax + numTargets + 1;

    SignatureCollection mySignatures(nMax - nMin + 1, nMin, nMax, numRegions);
    mySignatures.ReadSignatures("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADPEntireSignatureCollection_d1_2.txt");
    mySignatures.SetFailureDist(curShape, curScale);

    ValueFunctionReliabilityCollection myValues(nMissions, learningRate, Budget, nMin, nMax);
    myValues.ConstructValueFunction(nMissions, Budget, nMin, nMax);
//    myValues.InitializeValueFunctionStC("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADPInitialValues_d3_m33.txt");
    myValues.InitializeValueFunctionGHarmonic("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADPInitialValues_d2_m50_new.txt");

    NodeLinkedListMC myNetworkBuckets(numNodes, (numNodes - numTargets), numTargets);

    myNetworkBuckets.SetRanges(0.0750000, 0.0750000);
    myNetworkBuckets.UniformTargetLoc();
    myNetworkBuckets.SetBinLimits(4);

    vector<double> subArea;
    vector<double> subWeights;

    myNetworkBuckets.SubregionArea(subArea);
    myNetworkBuckets.SubregionDistanceWeight(subWeights);

    auto startTime = chrono::high_resolution_clock::now();

    for (int k = 1; k <= numReps ; ++k) {

        if (k > ((double) numReps / 2.0)){
            explorationProb = 0.00;
        }

        cout << "Start of Iteration " << k << endl;

        vector<double> gradientVect(numRegions,0);
        vector<int> numInRegion(numRegions, 20);
        double iterationCost = cFixed + (cVar * nSens);
        double missionSuccess = 0;

        budgetRemaining = Budget - iterationCost;

        // initial deployment of sensors and
        InitialSensorDeployment(gradientVect, numInRegion, subArea, subWeights, nSens);

        netSizeBeginning.push_back(0);
        missionDeployAction.push_back(nSens);

        // Generate initial fail times and set ages
        myNetworkBuckets.GenerateNewFailTime_ADPEval(nodeTime, nSens);
        myNetworkBuckets.UpdateFailTimesADP(nodeTime);
        myNetworkBuckets.ClearSensorAge();

        // Estimate reliability of initial state
        vector<int> ageState(maxAge, 0);
        ageState[0] = nSens;

        double initialRel = mySignatures.ReliabilityEstimate(delta, nSens, ageState);
        reliabilityRecord.push_back(initialRel);
        missionSuccess = missionSuccess + initialRel;

        double postDecisionBudget = budgetRemaining;
        int postDecisionSize = (int) accumulate(ageState.begin(), ageState.end(),0) - nMin;

        double curTime = delta;
        int curMission = 1;

        // update state and determine post decision state variable
        ageState[0] = 0;
        myNetworkBuckets.UpdateState(curTime, delta, ageState, expectedNum);

        double bestDecisionBudget;
        int bestDecisionSize;

        while (curMission < nMissions) {

            int curNetworkSize = accumulate(ageState.begin(), ageState.end(),0);
            int curAction = nMax - curNetworkSize;

            netSizeBeginning.push_back(curNetworkSize);

            int maxAction = max(0, min(curAction, (int) floor((budgetRemaining - cFixed) / cVar)));

            int bestAction = 0;
            double bestValue = 0;
            double cost = 0;
            double tempValue;
            double bestRel = 0;
            double bestCost = 0;
            double tBudget;
            double rel;

            double myNum = myUnif(myDev);

            // Select action at random
            if (myNum < explorationProb){
                int minAction = 0;
                bool actionNotFound = true;

                while (actionNotFound){

                    uniform_int_distribution<> myRandomDeploy(minAction, maxAction);

                    bestAction = myRandomDeploy(myRandomOrder);
                    ageState[0] = bestAction;

                    if (bestAction > 0){
                        tBudget = budgetRemaining - cFixed - (cVar * bestAction);
                        cost = cFixed + cVar * bestAction;
                    }
                    else {
                        tBudget = budgetRemaining;
                        cost = 0;
                    }

                    int tempNetSize = curNetworkSize + bestAction;
                    netValueIndex = tempNetSize - nMin;

                    if (tempNetSize < nMin){
                        rel = 0;
                        netValueIndex = 0;
                    }
                    else{
                        rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
                    }

                    bestValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

/*                    // This part is new. If my exploration action causes me to fail in the future then I might want to select a different action
                    if (myValues.GetStateValue(curMission, tBudget, netValueIndex) / ((double) nMissions - (double) curMission) < relRequirement){
                        maxAction = bestAction;
                        rel = 0;
                    }*/

                    if (rel > relRequirement || (maxAction - minAction < 2)){
                        actionNotFound = false;
                    }
                    else {
                        minAction = bestAction;
                    }

                    bestCost = cost;
                    bestRel = rel;

                    bestDecisionBudget = tBudget;
                    bestDecisionSize = netValueIndex;
                }
            }
            else {
                // Search for optimal action

                bool minNotFeasible = true;
                // action: decide not to deploy any new sensors
                netValueIndex = curNetworkSize - nMin;
                if (curNetworkSize < nMin){
                    rel = 0;
                    netValueIndex = 0;
                }
                else{
                    rel = mySignatures.ReliabilityEstimate(delta, curNetworkSize, ageState);
                    bestRel = rel;
                }

                if (rel >= relRequirement){
                    bestDecisionBudget = budgetRemaining;
                    bestDecisionSize = netValueIndex;

                    bestValue = rel + myValues.GetStateValue(curMission, budgetRemaining, netValueIndex);
                    bestAction = 0;
                    minNotFeasible = false;
                }

                // Deploy maximum number of sensors allowed
                curAction = maxAction;
                ageState[0] = curAction;

                if (curAction > 0){
                    tBudget = budgetRemaining - cFixed - (cVar * curAction);
                    cost = cFixed + cVar * curAction;
                } else{
                    tBudget = budgetRemaining;
                    cost = 0;
                }

                int tempNetSize = curNetworkSize + curAction;
                netValueIndex = tempNetSize - nMin;

                if (tempNetSize < nMin){
                    rel = 0;
                    netValueIndex = 0;
                }
                else{
                    rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
                }

                tempValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

                if (tempValue >= bestValue || minNotFeasible){
                    bestValue = tempValue;
                    bestAction = curAction;
                    bestCost = cost;
                    bestRel = rel;

                    bestDecisionBudget = tBudget;
                    bestDecisionSize = netValueIndex;
                }

                curAction = curAction - deployInterval;

                while (rel > relRequirement && curAction > minDeploy){

                    ageState[0] = curAction;
                    tBudget = budgetRemaining - cFixed - (cVar * curAction);

                    cost = cFixed + cVar * curAction;

                    tempNetSize = curNetworkSize + curAction;
                    netValueIndex = tempNetSize - nMin;

                    if (tempNetSize < nMin){
                        rel = 0;
                        netValueIndex = 0;
                    }
                    else{
                        rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
                    }

                    tempValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

                    if (tempValue >= bestValue && rel > relRequirement){
                        bestValue = tempValue;
                        bestAction = curAction;
                        bestCost = cost;
                        bestRel = rel;

                        bestDecisionBudget = tBudget;
                        bestDecisionSize = netValueIndex;
                    }
                    curAction = curAction - deployInterval;
                }
            }

            double tAlpha = myValues.GetStepSize(curMission - 1, postDecisionBudget, postDecisionSize);
            double tValue = myValues.GetStateValue(curMission - 1, postDecisionBudget, postDecisionSize);

            double newValue = ((1.0 - tAlpha) * tValue) + (tAlpha * bestValue);

            myValues.UpdateTempStateValue(curMission - 1, postDecisionBudget, postDecisionSize, newValue);
            myValues.ADP_MonotoneUpdate(curMission - 1, postDecisionBudget, postDecisionSize, bestValue);

            postDecisionBudget = bestDecisionBudget;
            postDecisionSize = bestDecisionSize;

            // redeploy sensors, and generate a new fail time for those added
            myNetworkBuckets.GenerateNewFailTimeADP_Update(nodeTime, (double) nMissions*delta, curTime, bestAction);

            iterationCost = iterationCost + bestCost;
            missionSuccess = missionSuccess + bestRel;
            reliabilityRecord.push_back(bestRel);
            missionDeployAction.push_back(bestAction);
            budgetRemaining = budgetRemaining - bestCost;

            curTime = curTime + delta;
            curMission = curMission + 1;

            vector<int> ageVector(maxAge, 0);
            myNetworkBuckets.UpdateState(curTime, delta, ageVector, expectedNum);
            copy(ageVector.begin(), ageVector.end(), ageState.begin());
        }
        costRecord.push_back(iterationCost);
        cumulativeMissionSuccess.push_back(missionSuccess);

//        myValues.UpdateValueFunctionsStC(learningRate);
        myValues.UpdateValueFunctionsGHarmonic(learningRate);

        valueFunctionPerIteration.push_back(myValues.GetStateValue(0, Budget - 750, nSens - nMin));
    }

    auto finishValueFunctionApprox = chrono::high_resolution_clock::now();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print out value functions
    cout << " Print Value Functions " << endl;
    myValues.PrintValueFunctions();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myObj;
    myObj.open ("CostFunctionImprovement.txt");

    for (int i = 0; i < costRecord.size() ; ++i) {
        myObj << "Iteration " << i << " -- ObjectiveValue : " << costRecord.at(i) << endl;
    }

    myObj.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // print out reliability over time
    ofstream myReliabilityFile;
    myReliabilityFile.open("ReliabilityRecord.txt");

    for (int j = 0; j < reliabilityRecord.size(); ++j) {
        myReliabilityFile << "Record " << j << " -- Reliability : " << reliabilityRecord.at(j) << endl;
    }

    myReliabilityFile.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream mySuccess;
    mySuccess.open ("ExpectedNumSuccessfulMissions.txt");

    for (int i = 0; i < cumulativeMissionSuccess.size() ; ++i) {
        mySuccess << "Iteration " << i << " -- ObjectiveValue : " << cumulativeMissionSuccess.at(i) << " -- ValueFunctionApprox : " << valueFunctionPerIteration.at(i) << endl;
    }

    mySuccess.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myActions;
    myActions.open ("NetworkSizeAndAction.txt");

    for (int i = 0; i < missionDeployAction.size() ; ++i) {
        myActions << "Iteration " << i << " -- NetworkSize : " << netSizeBeginning.at(i) << " -- Action : " << missionDeployAction.at(i) << endl;
    }

    myActions.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // MC simulation to evaluate performance of value functions
    auto startEval = chrono::high_resolution_clock::now();

    numNodes = nMax + numTargets + 1;
    NodeLinkedListMC myNetworkBuckets_ADPEval(numNodes, (numNodes - numTargets), numTargets);

    myNetworkBuckets_ADPEval.SetRanges(0.0750000, 0.0750000);
    myNetworkBuckets_ADPEval.UniformTargetLoc();
    myNetworkBuckets_ADPEval.SetBinLimits(4);

    vector<double> subAreaMC;
    vector<double> subWeightsMC;

    myNetworkBuckets_ADPEval.SubregionArea(subAreaMC);
    myNetworkBuckets_ADPEval.SubregionDistanceWeight(subWeightsMC);

    costRecord.clear();
    reliabilityRecord.clear();
    cumulativeMissionSuccess.clear();
    netSizeBeginning.clear();
    missionDeployAction.clear();

    double timePlace;
    vector<int> netStatus(nMissions,0);
    vector<int> successRecord;

    for (int k = 1; k <= numReps ; ++k) {

        cout << "Start of MC Iteration " << k << endl;

        vector<double> gradientVect(numRegions,0);
        vector<int> numInRegion(numRegions, 20);
        double iterationCost = cFixed + (cVar * nSens);
        double missionSuccess = 0;
        int totalSuccess = 0;

        budgetRemaining = Budget - iterationCost;

        // initial deployment of sensors and
        InitialSensorDeployment(gradientVect, numInRegion, subAreaMC, subWeightsMC, nSens);

        netSizeBeginning.push_back(0);
        missionDeployAction.push_back(nSens);

        // Generate initial fail times and set ages
        myNetworkBuckets_ADPEval.GenerateNewFailTime_ADPEval(nodeTime, nSens, (double) nMissions*delta);
        myNetworkBuckets_ADPEval.UpdateFailTimes(nodeTime);

        // Estimate reliability of initial state
        vector<int> ageState(maxAge, 0);
        vector<int> regionState(numRegions, 0);

        myNetworkBuckets_ADPEval.GenerateNewSpecificRGG_Bucket_ADPEval(numInRegion);

        ageState[0] = nSens;
        double initialRel = mySignatures.ReliabilityEstimate(delta, nSens, ageState);
        reliabilityRecord.push_back(initialRel);
        missionSuccess = missionSuccess + initialRel;

        double tarCovered;
        double curTime = delta;
        int curMission = 1;

        ageState[0] = 0;
        // update state and determine post decision state variable
        myNetworkBuckets_ADPEval.UpdateState(curTime, delta, regionState, ageState, expectedNum);

        while (curMission < nMissions) {

            myNetworkBuckets_ADPEval.UpdateRGG_ADPEval(curTime);
            myNetworkBuckets_ADPEval.UpdateFailTimes(nodeTime);

            tarCovered = myNetworkBuckets_ADPEval.BreadthFirstSearchReturn_ADPEval(numNodes, numTargets, timePlace);

            if (tarCovered >= covRequirement){
                netStatus[curMission-1] = netStatus[curMission-1] + 1;
                totalSuccess = totalSuccess + 1;
            }

            int curNetworkSize = accumulate(ageState.begin(), ageState.end(),0);
            int curAction = nMax - curNetworkSize;

            netSizeBeginning.push_back(curNetworkSize);

            int maxAction = max(0, min(curAction, (int) floor((budgetRemaining - cFixed) / cVar)));

            int bestAction = 0;
            double bestValue = 0;
            double cost = 0;
            double tempValue;
            double bestRel = 0;
            double bestCost = 0;
            double tBudget;
            double rel;

            bool minNotFeasible = true;

            // Search for optimal action
            // action: decide not to deploy any new sensors
            netValueIndex = curNetworkSize - nMin;
            if (curNetworkSize < nMin){
                rel = 0;
                netValueIndex = 0;
            }
            else{
                rel = mySignatures.ReliabilityEstimate(delta, curNetworkSize, ageState);
                bestRel = rel;
                if (rel > relRequirement){
                    minNotFeasible = false;
                }
            }

            bestValue = rel + myValues.GetStateValue(curMission, budgetRemaining, netValueIndex);
            bestAction = 0;

            // Deploy maximum number of sensors available
            curAction = maxAction;
            ageState[0] = curAction;

            if (curAction > 0){
                tBudget = budgetRemaining - cFixed - (cVar * curAction);
                cost = cFixed + cVar * curAction;
            }
            else {
                tBudget = budgetRemaining;
                cost = 0;
            }

            int tempNetSize = curNetworkSize + curAction;
            netValueIndex = tempNetSize - nMin;

            if (tempNetSize < nMin){
                rel = 0;
                netValueIndex = 0;
            }
            else{
                rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
            }

            tempValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

            if (tempValue >= bestValue || minNotFeasible){
                bestValue = tempValue;
                bestAction = curAction;
                bestCost = cost;
                bestRel = rel;
            }

            curAction = curAction - deployInterval;

            while (rel > relRequirement && curAction > minDeploy){

                ageState[0] = curAction;
                tBudget = budgetRemaining - cFixed - (cVar * curAction);

                cost = cFixed + cVar * curAction;

                tempNetSize = curNetworkSize + curAction;
                netValueIndex = tempNetSize - nMin;

                if (tempNetSize < nMin){
                    rel = 0;
                    netValueIndex = 0;
                }
                else{
                    rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
                }

                tempValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

                if (tempValue >= bestValue && rel > relRequirement){
                    bestValue = tempValue;
                    bestAction = curAction;
                    bestCost = cost;
                    bestRel = rel;
                }
                curAction = curAction - deployInterval;
            }

//            vector<int> regionDeployment(numRegions, 0);
            vector<int> regionDeployment;

            SensorDeploymentAction(regionState, subAreaMC, subWeightsMC, bestAction, regionDeployment);

            // redeploy sensors, and generate a new fail time for those added
            myNetworkBuckets_ADPEval.GenerateNewFailTimeADP_ADPEval(nodeTime, (double) nMissions*delta, curTime, regionDeployment);

            myNetworkBuckets_ADPEval.GenerateNewRGG_ADPMaintAction();

            iterationCost = iterationCost + bestCost;
            missionSuccess = missionSuccess + bestRel;
            reliabilityRecord.push_back(bestRel);
            missionDeployAction.push_back(bestAction);
            budgetRemaining = budgetRemaining - bestCost;

            curTime = curTime + delta;
            curMission = curMission + 1;

            vector<int> ageVector(maxAge, 0);
            vector<int> regionVector(numRegions, 0);

            myNetworkBuckets_ADPEval.UpdateState(curTime, delta, regionVector, ageVector, expectedNum);
            copy(ageVector.begin(), ageVector.end(), ageState.begin());
            copy(regionVector.begin(), regionVector.end(), regionState.begin());

        }

        myNetworkBuckets_ADPEval.UpdateRGG_ADPEval(curTime);
        myNetworkBuckets_ADPEval.UpdateFailTimes(nodeTime);
        tarCovered = myNetworkBuckets_ADPEval.BreadthFirstSearchReturn_ADPEval(numNodes, numTargets, timePlace);
        if (tarCovered >= covRequirement){
            netStatus[curMission-1] = netStatus[curMission-1] + 1;
            totalSuccess = totalSuccess + 1;
        }

        costRecord.push_back(iterationCost);
        cumulativeMissionSuccess.push_back(missionSuccess);
        successRecord.push_back(totalSuccess);
    }

    int tSuccess = 0;
    for (int m = 0; m < successRecord.size() ; ++m) {
        tSuccess = tSuccess + successRecord.at(m);
    }

    auto totalTime = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsedNetwork = totalTime - startTime;

    chrono::duration<double> approxTime = finishValueFunctionApprox - startTime;
    chrono::duration<double> evalTime = totalTime - startEval;

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myObjMC;
    myObjMC.open ("CostFunctionMC.txt");

    for (int i = 0; i < costRecord.size() ; ++i) {
        myObjMC << "Iteration " << i << " -- ObjectiveValue : " << costRecord.at(i) << endl;
    }

    myObjMC.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // print out reliability over time
    ofstream myReliabilityFileMC;
    myReliabilityFileMC.open("ReliabilityRecordMC.txt");

    for (int j = 0; j < reliabilityRecord.size(); ++j) {
        myReliabilityFileMC << "Record " << j << " -- Reliability : " << reliabilityRecord.at(j) << endl;
    }

    myReliabilityFileMC.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream mySuccessMC;
    mySuccessMC.open ("ExpectedNumSuccessfulMissionsMC.txt");

    for (int i = 0; i < cumulativeMissionSuccess.size() ; ++i) {
        mySuccessMC << "Iteration " << i << " -- ObjectiveValue : " << cumulativeMissionSuccess.at(i) << endl;
    }

    mySuccessMC.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    cout << "Observed Average Number Successful Mission : " << (double) tSuccess / (double) numReps << endl;

    ofstream myObservedSuccessMC;
    myObservedSuccessMC.open("ObservedNumSuccessfulMissionsMC.txt");
    for (int l = 0; l <netStatus.size() ; ++l) {
        myObservedSuccessMC << "Mission : " << l << " - Status : " << netStatus.at(l) << " of " << numReps << endl;
        cout << "Mission : " << l << " - Status : " << netStatus.at(l) << " of " << numReps << endl;
    }

    for (int l = 0; l < successRecord.size(); ++l) {
        myObservedSuccessMC << "record " << l << " status : " << successRecord.at(l) << endl;
    }

    myObservedSuccessMC.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myActionsMC;
    myActionsMC.open ("NetworkSizeAndActionMC.txt");

    for (int i = 0; i < missionDeployAction.size() ; ++i) {
        myActionsMC << "Iteration " << i << " -- NetworkSize : " << netSizeBeginning.at(i) << " -- Action : " << missionDeployAction.at(i) << endl;
    }

    myActionsMC.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    ofstream myNotes;
    myNotes.open("Notes.txt");

    myNotes << "Function : ADP AVI Reliability Model with Requirement " << endl;
    myNotes << "Total Time Required : " << elapsedNetwork.count() << endl;
    myNotes << "  Value Function Approximation Time : " << approxTime.count() << endl;
    myNotes << "  Policy Evaluation Time : " << evalTime.count() << endl;
    myNotes << "Number of Replications : " << numReps << endl;
    myNotes << "Budget Available : " << Budget << endl;
    myNotes << "Number of Missions : " << nMissions << endl;
    myNotes << "Time Between Maintenance Inspections : " << delta << endl;
    myNotes << "Minimum number of sensors to deploy : " << minDeploy << endl;
    myNotes << "Sensor Deployment Interval : " << deployInterval << endl;
    myNotes << "Minimum Network Size : " << nMin << endl;
    myNotes << "Maximum Network Size : " << nMax << endl;
    myNotes << "Minimum Reliability Requirement : " << relRequirement << endl;
    myNotes << "Observed Average Number Successful Mission : " << (double) tSuccess / (double) numReps << endl;

    myNotes.close();

    cout << "Total Time Required : " << elapsedNetwork.count() << endl;
    cout << "  Value Function Approximation Time : " << approxTime.count() << endl;
    cout << "  Policy Evaluation Time : " << evalTime.count() << endl;

}

// CBM ADP model using AVI look up tables. The objective is to maximize the number of successful missions subject to a budget constraint.
// Uses a neighborhood of the current state to update the value functions for states `close' to the one that is actually visited.
// Once the value functions are updated a monte carlo simulation is performed to evaluate the performance of the resulting maintenance policy
// This function also requires a minimum reliability requirement, where each mission must achieve a reliability above this input
void ADP_CBM_AVI_ReliabilityModel_CAVE_MinRequirement(int nSens, int numReps, int nMissions, double delta, double Budget,
                                                                 double cFixed, double cVar, double covRequirement, double relRequirement) {

    uniform_real_distribution<> myUnif(0,1);

    random_device myDev;
    random_device myRandomOrder;

    const int numRegions = 16;
    const int maxAge = 24;
    const int nMin = 320;
    const int nMax = 950;
    const int deployInterval = 10;
    const int minDeploy = 30;
    const double learningRate = 0.7;

    double explorationProb = 0.05;
    double curShape = 1.5;
    double curScale = 10.0;
    double expectedNum;
    int netValueIndex;
    double budgetRemaining;

    vector<double> nodeTime;
    vector<double> costRecord;
    vector<double> reliabilityRecord;
    vector<double> cumulativeMissionSuccess;
    vector<double> valueFunctionPerIteration;
    vector<int> netSizeBeginning;
    vector<int> missionDeployAction;

    // number of nodes includes sink node, number of sensors, and number of targets
    int numTargets = 441;
    int numNodes = nMax + numTargets + 1;

    SignatureCollection mySignatures(nMax - nMin + 1, nMin, nMax, numRegions);
    mySignatures.ReadSignatures("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADPEntireSignatureCollection_d3.txt");
    mySignatures.SetFailureDist(curShape, curScale);

    ValueFunctionReliabilityCollection myValues(nMissions, learningRate, Budget, nMin, nMax);
    myValues.ConstructValueFunction(nMissions, Budget, nMin, nMax);
//    myValues.InitializeValueFunctionStC("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADPInitialValues_d3_m33.txt");
    myValues.InitializeValueFunctionGHarmonic("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADPInitialValues_d4_m25.txt");

    NodeLinkedListMC myNetworkBuckets(numNodes, (numNodes - numTargets), numTargets);

    myNetworkBuckets.SetRanges(0.0750000, 0.0750000);
    myNetworkBuckets.UniformTargetLoc();
    myNetworkBuckets.SetBinLimits(4);

    vector<double> subArea;
    vector<double> subWeights;

    myNetworkBuckets.SubregionArea(subArea);
    myNetworkBuckets.SubregionDistanceWeight(subWeights);

    auto startTime = chrono::high_resolution_clock::now();

    for (int k = 1; k <= numReps ; ++k) {

        if (k > ((double) numReps / 2.0)){
            explorationProb = 0.00;
        }

        cout << "Start of Iteration " << k << endl;

        vector<double> gradientVect(numRegions,0);
        vector<int> numInRegion(numRegions, 20);
        double iterationCost = cFixed + (cVar * nSens);
        double missionSuccess = 0;

        budgetRemaining = Budget - iterationCost;

        // initial deployment of sensors and
        InitialSensorDeployment(gradientVect, numInRegion, subArea, subWeights, nSens);

        netSizeBeginning.push_back(0);
        missionDeployAction.push_back(nSens);

        // Generate initial fail times and set ages
        myNetworkBuckets.GenerateNewFailTime_ADPEval(nodeTime, nSens);
        myNetworkBuckets.UpdateFailTimesADP(nodeTime);
        myNetworkBuckets.ClearSensorAge();

        // Estimate reliability of initial state
        vector<int> ageState(maxAge, 0);
        ageState[0] = nSens;

        double initialRel = mySignatures.ReliabilityEstimate(delta, nSens, ageState);
        reliabilityRecord.push_back(initialRel);
        missionSuccess = missionSuccess + initialRel;

        double postDecisionBudget = budgetRemaining;
        int postDecisionSize = (int) accumulate(ageState.begin(), ageState.end(),0) - nMin;

        double curTime = delta;
        int curMission = 1;

        // update state and determine post decision state variable
        ageState[0] = 0;
        myNetworkBuckets.UpdateState(curTime, delta, ageState, expectedNum);

        double bestDecisionBudget;
        int bestDecisionSize;

        while (curMission < nMissions) {

            int curNetworkSize = accumulate(ageState.begin(), ageState.end(),0);
            int curAction = nMax - curNetworkSize;

            netSizeBeginning.push_back(curNetworkSize);

            int maxAction = max(0, min(curAction, (int) floor((budgetRemaining - cFixed) / cVar)));

            int bestAction = 0;
            double bestValue = 0;
            double cost = 0;
            double tempValue;
            double bestRel = 0;
            double bestCost = 0;
            double tBudget;
            double rel;

            double myNum = myUnif(myDev);

            // Select action at random
            if (myNum < explorationProb){
                int minAction = 0;
                bool actionNotFound = true;

                while (actionNotFound){

                    uniform_int_distribution<> myRandomDeploy(minAction, maxAction);

                    bestAction = myRandomDeploy(myRandomOrder);
                    ageState[0] = bestAction;

                    if (bestAction > 0){
                        tBudget = budgetRemaining - cFixed - (cVar * bestAction);
                        cost = cFixed + cVar * bestAction;
                    }
                    else {
                        tBudget = budgetRemaining;
                        cost = 0;
                    }

                    int tempNetSize = curNetworkSize + bestAction;
                    netValueIndex = tempNetSize - nMin;

                    if (tempNetSize < nMin){
                        rel = 0;
                        netValueIndex = 0;
                    }
                    else{
                        rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
                    }

                    bestValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

/*                    // This part is new. If my exploration action causes me to fail in the future then I might want to select a different action
                    if (myValues.GetStateValue(curMission, tBudget, netValueIndex) / ((double) nMissions - (double) curMission) < relRequirement){
                        maxAction = bestAction;
                        rel = 0;
                    }*/

                    if (rel > relRequirement || (maxAction - minAction < 2)){
                        actionNotFound = false;
                    }
                    else {
                        minAction = bestAction;
                    }

                    bestCost = cost;
                    bestRel = rel;

                    bestDecisionBudget = tBudget;
                    bestDecisionSize = netValueIndex;
                }
            }
            else {
                // Search for optimal action

                bool minNotFeasible = true;
                // action: decide not to deploy any new sensors
                netValueIndex = curNetworkSize - nMin;
                if (curNetworkSize < nMin){
                    rel = 0;
                    netValueIndex = 0;
                }
                else{
                    rel = mySignatures.ReliabilityEstimate(delta, curNetworkSize, ageState);
                    bestRel = rel;
                }

                if (rel >= relRequirement){
                    bestDecisionBudget = budgetRemaining;
                    bestDecisionSize = netValueIndex;

                    bestValue = rel + myValues.GetStateValue(curMission, budgetRemaining, netValueIndex);
                    bestAction = 0;
                    minNotFeasible = false;
                }

                // Deploy maximum number of sensors allowed
                curAction = maxAction;
                ageState[0] = curAction;

                if (curAction > 0){
                    tBudget = budgetRemaining - cFixed - (cVar * curAction);
                    cost = cFixed + cVar * curAction;
                } else{
                    tBudget = budgetRemaining;
                    cost = 0;
                }

                int tempNetSize = curNetworkSize + curAction;
                netValueIndex = tempNetSize - nMin;

                if (tempNetSize < nMin){
                    rel = 0;
                    netValueIndex = 0;
                }
                else{
                    rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
                }

                tempValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

                if (tempValue >= bestValue || minNotFeasible){
                    bestValue = tempValue;
                    bestAction = curAction;
                    bestCost = cost;
                    bestRel = rel;

                    bestDecisionBudget = tBudget;
                    bestDecisionSize = netValueIndex;
                }

                curAction = curAction - deployInterval;

                while (rel > relRequirement && curAction > minDeploy){

                    ageState[0] = curAction;
                    tBudget = budgetRemaining - cFixed - (cVar * curAction);

                    cost = cFixed + cVar * curAction;

                    tempNetSize = curNetworkSize + curAction;
                    netValueIndex = tempNetSize - nMin;

                    if (tempNetSize < nMin){
                        rel = 0;
                        netValueIndex = 0;
                    }
                    else{
                        rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
                    }

                    tempValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

                    if (tempValue >= bestValue && rel > relRequirement){
                        bestValue = tempValue;
                        bestAction = curAction;
                        bestCost = cost;
                        bestRel = rel;

                        bestDecisionBudget = tBudget;
                        bestDecisionSize = netValueIndex;
                    }
                    curAction = curAction - deployInterval;
                }
            }

            double tAlpha = myValues.GetStepSize(curMission - 1, postDecisionBudget, postDecisionSize);
            double tValue = myValues.GetStateValue(curMission - 1, postDecisionBudget, postDecisionSize);

            double newValue = ((1.0 - tAlpha) * tValue) + (tAlpha * bestValue);

            myValues.UpdateTempStateValue(curMission - 1, postDecisionBudget, postDecisionSize, newValue);
            myValues.ADP_MonotoneUpdate(curMission - 1, postDecisionBudget, postDecisionSize, bestValue);

            postDecisionBudget = bestDecisionBudget;
            postDecisionSize = bestDecisionSize;

            // redeploy sensors, and generate a new fail time for those added
            myNetworkBuckets.GenerateNewFailTimeADP_Update(nodeTime, (double) nMissions*delta, curTime, bestAction);

            iterationCost = iterationCost + bestCost;
            missionSuccess = missionSuccess + bestRel;
            reliabilityRecord.push_back(bestRel);
            missionDeployAction.push_back(bestAction);
            budgetRemaining = budgetRemaining - bestCost;

            curTime = curTime + delta;
            curMission = curMission + 1;

            vector<int> ageVector(maxAge, 0);
            myNetworkBuckets.UpdateState(curTime, delta, ageVector, expectedNum);
            copy(ageVector.begin(), ageVector.end(), ageState.begin());
        }
        costRecord.push_back(iterationCost);
        cumulativeMissionSuccess.push_back(missionSuccess);

//        myValues.UpdateValueFunctionsStC(learningRate);
        myValues.UpdateValueFunctionsGHarmonic(learningRate);

        valueFunctionPerIteration.push_back(myValues.GetStateValue(0, Budget - 750, nSens - nMin));
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print out value functions
    cout << " Print Value Functions " << endl;
    myValues.PrintValueFunctions();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myObj;
    myObj.open ("CostFunctionImprovement.txt");

    for (int i = 0; i < costRecord.size() ; ++i) {
        myObj << "Iteration " << i << " -- ObjectiveValue : " << costRecord.at(i) << endl;
    }

    myObj.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // print out reliability over time
    ofstream myReliabilityFile;
    myReliabilityFile.open("ReliabilityRecord.txt");

    for (int j = 0; j < reliabilityRecord.size(); ++j) {
        myReliabilityFile << "Record " << j << " -- Reliability : " << reliabilityRecord.at(j) << endl;
    }

    myReliabilityFile.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream mySuccess;
    mySuccess.open ("ExpectedNumSuccessfulMissions.txt");

    for (int i = 0; i < cumulativeMissionSuccess.size() ; ++i) {
        mySuccess << "Iteration " << i << " -- ObjectiveValue : " << cumulativeMissionSuccess.at(i) << " -- ValueFunctionApprox : " << valueFunctionPerIteration.at(i) << endl;
    }

    mySuccess.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myActions;
    myActions.open ("NetworkSizeAndAction.txt");

    for (int i = 0; i < missionDeployAction.size() ; ++i) {
        myActions << "Iteration " << i << " -- NetworkSize : " << netSizeBeginning.at(i) << " -- Action : " << missionDeployAction.at(i) << endl;
    }

    myActions.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////


    auto totalTime = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsedNetwork = totalTime - startTime;



    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    ofstream myNotes;
    myNotes.open("Notes.txt");

    myNotes << "Function : ADP AVI Reliability Model with Requirement " << endl;
    myNotes << "Total Time Required : " << elapsedNetwork.count() << endl;
    myNotes << "Number of Replications : " << numReps << endl;
    myNotes << "Budget Available : " << Budget << endl;
    myNotes << "Number of Missions : " << nMissions << endl;
    myNotes << "Time Between Maintenance Inspections : " << delta << endl;
    myNotes << "Minimum number of sensors to deploy : " << minDeploy << endl;
    myNotes << "Sensor Deployment Interval : " << deployInterval << endl;
    myNotes << "Minimum Network Size : " << nMin << endl;
    myNotes << "Maximum Network Size : " << nMax << endl;
    myNotes << "Minimum Reliability Requirement : " << relRequirement << endl;

    myNotes.close();

}

// The objective is to maximize the number of successful missions subject to a budget constraint. Reads in a set of value functions and then
// a monte carlo simulation is performed to evaluate the performance of the resulting maintenance policy.
void ADP_CBM_InputValueFunction_EvaluateMC_Requirement(int nSens, int numReps, int nMissions, double delta, double Budget,
                                           double cFixed, double cVar, double covRequirement, double relRequirement) {


    const int maxAge = 10;
    const int numRegions = 16;
    const int nMin = 320;
    const int nMax = 950;
    const int deployInterval = 10;
    const int minDeploy = 30;
    const double learningRate = 0.7;

    double curShape = 1.5;
    double curScale = 10.0;
    double expectedNum;
    int netValueIndex;
    double budgetRemaining;

    vector<double> nodeTime;
    vector<double> costRecord;
    vector<double> reliabilityRecord;
    vector<double> cumulativeMissionSuccess;
    vector<int> netSizeBeginning;
    vector<int> missionDeployAction;
    vector<int> successRecord;

    // number of nodes includes sink node, number of sensors, and number of targets
    int numTargets = 441;

    SignatureCollection mySignatures(631, nMin, nMax, numRegions);
    mySignatures.ReadSignatures("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADPEntireSignatureCollection_d3.txt");
    mySignatures.SetFailureDist(curShape, curScale);

    ValueFunctionReliabilityCollection myValues(nMissions, learningRate, Budget, nMin, nMax);
    myValues.ConstructValueFunction(nMissions, Budget, nMin, nMax);
    myValues.ReadValueFunctions();

    auto startTime = chrono::high_resolution_clock::now();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // MC simulation to evaluate performance of value functions

    int numNodes = nMax + numTargets + 1;
    NodeLinkedListMC myNetworkBuckets_ADPEval(numNodes, (numNodes - numTargets), numTargets);

    myNetworkBuckets_ADPEval.SetRanges(0.0750000, 0.0750000);
    myNetworkBuckets_ADPEval.UniformTargetLoc();
    myNetworkBuckets_ADPEval.SetBinLimits(4);

    vector<double> subAreaMC;
    vector<double> subWeightsMC;

    myNetworkBuckets_ADPEval.SubregionArea(subAreaMC);
    myNetworkBuckets_ADPEval.SubregionDistanceWeight(subWeightsMC);

    costRecord.clear();
    reliabilityRecord.clear();
    cumulativeMissionSuccess.clear();
    netSizeBeginning.clear();
    missionDeployAction.clear();

    double timePlace;
    vector<int> netStatus(nMissions,0);

    for (int k = 1; k <= numReps ; ++k) {

        cout << "Start of MC Iteration " << k << endl;

        vector<double> gradientVect(numRegions,0);
        vector<int> numInRegion(numRegions, 20);
        double iterationCost = cFixed + (cVar * nSens);
        double missionSuccess = 0;
        int totalSuccess = 0;

        budgetRemaining = Budget - iterationCost;

        // initial deployment of sensors and
        InitialSensorDeployment(gradientVect, numInRegion, subAreaMC, subWeightsMC, nSens);

        netSizeBeginning.push_back(0);
        missionDeployAction.push_back(nSens);

        // Generate initial fail times and set ages
        myNetworkBuckets_ADPEval.GenerateNewFailTime_ADPEval(nodeTime, nSens, (double) nMissions*delta);
        myNetworkBuckets_ADPEval.UpdateFailTimes(nodeTime);

        // Estimate reliability of initial state
        vector<int> ageState(maxAge, 0);
        vector<int> regionState(numRegions, 0);

        myNetworkBuckets_ADPEval.GenerateNewSpecificRGG_Bucket_ADPEval(numInRegion);

        ageState[0] = nSens;
        double initialRel = mySignatures.ReliabilityEstimate(delta, nSens, ageState);
        reliabilityRecord.push_back(initialRel);
        missionSuccess = missionSuccess + initialRel;

        double tarCovered;
        double curTime = delta;
        int curMission = 1;

        ageState[0] = 0;
        // update state and determine post decision state variable
        myNetworkBuckets_ADPEval.UpdateState(curTime, delta, regionState, ageState, expectedNum);

        while (curMission < nMissions) {

            myNetworkBuckets_ADPEval.UpdateRGG_ADPEval(curTime);
            myNetworkBuckets_ADPEval.UpdateFailTimes(nodeTime);

            tarCovered = myNetworkBuckets_ADPEval.BreadthFirstSearchReturn_ADPEval(numNodes, numTargets, timePlace);

            if (tarCovered >= covRequirement){
                netStatus[curMission-1] = netStatus[curMission-1] + 1;
                totalSuccess = totalSuccess + 1;
            }

            int curNetworkSize = accumulate(ageState.begin(), ageState.end(),0);
            int curAction = nMax - curNetworkSize;

            netSizeBeginning.push_back(curNetworkSize);

            int maxAction = max(0, min(curAction, (int) floor((budgetRemaining - cFixed) / cVar)));

            int bestAction = 0;
            double bestValue = 0;
            double cost = 0;
            double tempValue;
            double bestRel = 0;
            double bestCost = 0;
            double tBudget;
            double rel;

            bool minNotFeasible = true;

            // Search for optimal action
            // action: decide not to deploy any new sensors
            netValueIndex = curNetworkSize - nMin;
            if (curNetworkSize < nMin){
                rel = 0;
                netValueIndex = 0;
            }
            else{
                rel = mySignatures.ReliabilityEstimate(delta, curNetworkSize, ageState);
                bestRel = rel;
                if (rel > relRequirement){
                    minNotFeasible = false;
                }
            }

            bestValue = rel + myValues.GetStateValue(curMission, budgetRemaining, netValueIndex);
            bestAction = 0;

            double valueZero = bestValue;
            double budgetZero = budgetRemaining;
            double indexZero = netValueIndex;

            // deploy maximum number of sensors allowed
            curAction = maxAction;
            ageState[0] = curAction;

            if (curAction > 0){
                tBudget = budgetRemaining - cFixed - (cVar * curAction);
                cost = cFixed + cVar * curAction;
            }
            else {
                tBudget = budgetRemaining;
                cost = 0;
            }

            int tempNetSize = curNetworkSize + curAction;
            netValueIndex = tempNetSize - nMin;

            if (tempNetSize < nMin){
                rel = 0;
                netValueIndex = 0;
            }
            else {
                rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
            }

            tempValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

            cout << k << " -Mission: " << curMission << " -Action: " << curAction << " Budget: " << tBudget << " -Size : " << netValueIndex << " -Value: " << tempValue << endl;

            if (tempValue > bestValue || minNotFeasible){
                bestValue = tempValue;
                bestAction = curAction;
                bestCost = cost;
                bestRel = rel;
            }

            curAction = curAction - deployInterval;

            while (rel > relRequirement && curAction > minDeploy){

                ageState[0] = curAction;
                tBudget = budgetRemaining - cFixed - (cVar * curAction);

                cost = cFixed + cVar * curAction;

                tempNetSize = curNetworkSize + curAction;
                netValueIndex = tempNetSize - nMin;

                if (tempNetSize < nMin){
                    rel = 0;
                    netValueIndex = 0;
                }
                else{
                    rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
                }

                tempValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

                cout << k << " -Mission: " << curMission << " -Action: " << curAction << " Budget: " << tBudget << " -Size : " << netValueIndex << " -Value: " << tempValue << endl;

                if (tempValue > bestValue && rel > relRequirement){
                    bestValue = tempValue;
                    bestAction = curAction;
                    bestCost = cost;
                    bestRel = rel;
                }
                curAction = curAction - deployInterval;
            }

            cout << k << " -Mission: " << curMission << " -Action: " << 0 << " Budget: " << budgetZero << " -Size : " << indexZero << " -Value: " << valueZero << endl;

//            vector<int> regionDeployment(numRegions, 0);
            vector<int> regionDeployment;

            SensorDeploymentAction(regionState, subAreaMC, subWeightsMC, bestAction, regionDeployment);

            // redeploy sensors, and generate a new fail time for those added
            myNetworkBuckets_ADPEval.GenerateNewFailTimeADP_ADPEval(nodeTime, (double) nMissions*delta, curTime, regionDeployment);

            myNetworkBuckets_ADPEval.GenerateNewRGG_ADPMaintAction();

            iterationCost = iterationCost + bestCost;
            missionSuccess = missionSuccess + bestRel;
            reliabilityRecord.push_back(bestRel);
            missionDeployAction.push_back(bestAction);
            budgetRemaining = budgetRemaining - bestCost;

            curTime = curTime + delta;
            curMission = curMission + 1;

            vector<int> ageVector(maxAge, 0);
            vector<int> regionVector(numRegions, 0);

            myNetworkBuckets_ADPEval.UpdateState(curTime, delta, regionVector, ageVector, expectedNum);
            copy(ageVector.begin(), ageVector.end(), ageState.begin());
            copy(regionVector.begin(), regionVector.end(), regionState.begin());

        }

        myNetworkBuckets_ADPEval.UpdateRGG_ADPEval(curTime);
        myNetworkBuckets_ADPEval.UpdateFailTimes(nodeTime);
        tarCovered = myNetworkBuckets_ADPEval.BreadthFirstSearchReturn_ADPEval(numNodes, numTargets, timePlace);
        if (tarCovered >= covRequirement){
            netStatus[curMission-1] = netStatus[curMission-1] + 1;
            totalSuccess = totalSuccess + 1;
        }

        costRecord.push_back(iterationCost);
        cumulativeMissionSuccess.push_back(missionSuccess);
        successRecord.push_back(totalSuccess);
    }

    int tSuccess = 0;
    for (int m = 0; m < successRecord.size() ; ++m) {
        tSuccess = tSuccess + successRecord.at(m);
    }

    auto finishTime = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsedNetwork = finishTime - startTime;

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myObjMC;
    myObjMC.open ("CostFunctionMC.txt");

    for (int i = 0; i < costRecord.size() ; ++i) {
        myObjMC << "Iteration " << i << " -- CostValue : " << costRecord.at(i) << endl;
    }

    myObjMC.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // print out reliability over time
    ofstream myReliabilityFileMC;
    myReliabilityFileMC.open("ReliabilityRecordMC.txt");

    for (int j = 0; j < reliabilityRecord.size(); ++j) {
        myReliabilityFileMC << "Record " << j << " -- Reliability : " << reliabilityRecord.at(j) << endl;
    }

    myReliabilityFileMC.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream mySuccessMC;
    mySuccessMC.open ("ExpectedNumSuccessfulMissionsMC.txt");

    for (int i = 0; i < cumulativeMissionSuccess.size() ; ++i) {
        mySuccessMC << "Iteration " << i << " -- ObjectiveValue : " << cumulativeMissionSuccess.at(i) << endl;
    }

    mySuccessMC.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    cout << "Observed Average Number Successful Mission : " << (double) tSuccess / (double) numReps << endl;

    ofstream myObservedSuccessMC;
    myObservedSuccessMC.open("ObservedNumSuccessfulMissionsMC.txt");
    for (int l = 0; l < netStatus.size() ; ++l) {
        myObservedSuccessMC << "Mission : " << l << " - Status : " << netStatus.at(l) << " of " << numReps << endl;
        cout << "Mission : " << l << " - Status : " << netStatus.at(l) << " of " << numReps << endl;
    }

    for (int l = 0; l < successRecord.size(); ++l) {
        myObservedSuccessMC << "record " << l << " status : " << successRecord.at(l) << endl;
    }

    myObservedSuccessMC.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myActionsMC;
    myActionsMC.open ("NetworkSizeAndActionMC.txt");

    for (int i = 0; i < missionDeployAction.size() ; ++i) {
        myActionsMC << "Iteration " << i << " -- NetworkSize : " << netSizeBeginning.at(i) << " -- Action : " << missionDeployAction.at(i) << endl;
    }

    myActionsMC.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    ofstream myNotes;
    myNotes.open("Notes.txt");

    myNotes << "Function : ADP AVI Evaluation with Input Value Functions" << endl;
    myNotes << "Time Required : " << elapsedNetwork.count() << endl;
    myNotes << "Number of Replications : " << numReps << endl;
    myNotes << "Budget Available : " << Budget << endl;
    myNotes << "Number of Missions : " << nMissions << endl;
    myNotes << "Time Between Maintenance Inspections : " << delta << endl;
    myNotes << "Minimum number of sensors to deploy : " << minDeploy << endl;
    myNotes << "Sensor Deployment Interval : " << deployInterval << endl;
    myNotes << "Minimum Network Size : " << nMin << endl;
    myNotes << "Maximum Network Size : " << nMax << endl;
    myNotes << "Minimum Reliability Requirement : " << relRequirement << endl;
    myNotes << "Observed Average Number Successful Mission : " << (double) tSuccess / (double) numReps << endl;

    myNotes.close();

    cout << "Total Time Required : " << elapsedNetwork.count() << endl;
}

// CBM ADP model using AVI look up tables. The objective is to maximize the number of successful missions subject to a budget constraint.
// Uses a neighborhood of the current state to update the value functions for states `close' to the one that is actually visited.
// Once the value functions are updated a monte carlo simulation is performed to evaluate the performance of the resulting maintenance policy
// This function also requires a minimum reliability requirement, where each mission must achieve a reliability above this input. If there is not enough
// budget left to satisfy this minimum requirement, than the over cost is recorded.
void ADP_CBM_AVI_ReliabilityModel_CAVE_EvaluateMC_MinRequirement_PenaltyCost(int nSens, int numReps, int nMissions, double delta, double Budget,
                                                                 double cFixed, double cVar, double covRequirement, double relRequirement) {

    uniform_real_distribution<> myUnif(0,1);

    random_device myDev;
    random_device myRandomOrder;

    const int numRegions = 16;
    const int maxAge = 15;
    const int nMin = 300;
    const int nMax = 950;
    const int deployInterval = 10;
    const int minDeploy = 30;
    const double explorationProb = 0.05;
    const double learningRate = 0.7;

    double curShape = 1.5;
    double curScale = 10.0;
    double expectedNum;
    int netValueIndex;
    double budgetRemaining;

    vector<double> nodeTime;
    vector<double> costRecord;
    vector<double> reliabilityRecord;
    vector<double> cumulativeMissionSuccess;
    vector<int> netSizeBeginning;
    vector<int> missionDeployAction;

    // number of nodes includes sink node, number of sensors, and number of targets
    int numTargets = 441;
    int numNodes = nMax + numTargets + 1;

    SignatureCollection mySignatures(nMax - nMin + 1, nMin, nMax, numRegions);
    mySignatures.ReadSignatures("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADPEntireSignatureCollection_d1_2.txt");
    mySignatures.SetFailureDist(curShape, curScale);

    ValueFunctionReliabilityCollection myValues(nMissions, learningRate, Budget, nMin, nMax);
    myValues.ConstructValueFunction(nMissions, Budget, nMin, nMax);
    myValues.InitializeValueFunctionStC("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADPInitialValues_d2_m50.txt");

    NodeLinkedListMC myNetworkBuckets(numNodes, (numNodes - numTargets), numTargets);

    myNetworkBuckets.SetRanges(0.0750000, 0.0750000);
    myNetworkBuckets.UniformTargetLoc();
    myNetworkBuckets.SetBinLimits(4);

    vector<double> subArea;
    vector<double> subWeights;

    myNetworkBuckets.SubregionArea(subArea);
    myNetworkBuckets.SubregionDistanceWeight(subWeights);

    auto startTime = chrono::high_resolution_clock::now();

    for (int k = 1; k <= numReps ; ++k) {

        cout << "Start of Iteration " << k << endl;

        vector<double> gradientVect(numRegions,0);
        vector<int> numInRegion(numRegions, 20);
        double iterationCost = cFixed + (cVar * nSens);
        double missionSuccess = 0;

        budgetRemaining = Budget - iterationCost;

        // initial deployment of sensors and
        InitialSensorDeployment(gradientVect, numInRegion, subArea, subWeights, nSens);

        netSizeBeginning.push_back(0);
        missionDeployAction.push_back(nSens);

        // Generate initial fail times and set ages
        myNetworkBuckets.GenerateNewFailTime_ADPEval(nodeTime, nSens);
        myNetworkBuckets.UpdateFailTimesADP(nodeTime);
        myNetworkBuckets.ClearSensorAge();

        // Estimate reliability of initial state
        vector<int> ageState(maxAge, 0);
        ageState[0] = nSens;

        double initialRel = mySignatures.ReliabilityEstimate(delta, nSens, ageState);
        reliabilityRecord.push_back(initialRel);
        missionSuccess = missionSuccess + initialRel;

        double postDecisionBudget = budgetRemaining;
        int postDecisionSize = (int) accumulate(ageState.begin(), ageState.end(),0) - nMin;

        double curTime = delta;
        int curMission = 1;

        // update state and determine post decision state variable
        ageState[0] = 0;
        myNetworkBuckets.UpdateState(curTime, delta, ageState, expectedNum);

        double bestDecisionBudget;
        int bestDecisionSize;

        while (curMission < nMissions) {

            int curNetworkSize = accumulate(ageState.begin(), ageState.end(),0);
            int curAction = nMax - curNetworkSize;

            netSizeBeginning.push_back(curNetworkSize);

            int maxAction = max(0, min(curAction, (int) floor((budgetRemaining - cFixed) / cVar)));

            int bestAction = 0;
            double bestValue = 0;
            double cost = 0;
            double tempValue;
            double bestRel = 0;
            double bestCost = 0;
            double tBudget;
            double rel;

            double myNum = myUnif(myDev);

            // Select action at random
            if (myNum < explorationProb){
                int minAction = 0;
                bool actionNotFound = true;

                while (actionNotFound){

                    uniform_int_distribution<> myRandomDeploy(minAction, maxAction);

                    bestAction = myRandomDeploy(myRandomOrder);
                    ageState[0] = bestAction;

                    if (bestAction > 0){
                        tBudget = budgetRemaining - cFixed - (cVar * bestAction);
                        cost = cFixed + cVar * bestAction;
                    }
                    else {
                        tBudget = budgetRemaining;
                        cost = 0;
                    }

                    int tempNetSize = curNetworkSize + bestAction;
                    netValueIndex = tempNetSize - nMin;

                    if (tempNetSize < nMin){
                        rel = 0;
                        netValueIndex = 0;
                    }
                    else{
                        rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
                    }

                    if (rel > relRequirement || (maxAction - minAction < 2)){
                        actionNotFound = false;
                    }
                    else {
                        minAction = bestAction;
                    }

                    bestValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

                    bestCost = cost;
                    bestRel = rel;

                    bestDecisionBudget = tBudget;
                    bestDecisionSize = netValueIndex;
                }
            }
            else {
                // Search for optimal action

                bool minNotFeasible = true;
                // action: decide not to deploy any new sensors
                netValueIndex = curNetworkSize - nMin;
                if (curNetworkSize < nMin){
                    rel = 0;
                    netValueIndex = 0;
                }
                else{
                    rel = mySignatures.ReliabilityEstimate(delta, curNetworkSize, ageState);
                    bestRel = rel;
                }

                if (rel >= relRequirement){
                    bestDecisionBudget = budgetRemaining;
                    bestDecisionSize = netValueIndex;

                    bestValue = rel + myValues.GetStateValue(curMission, budgetRemaining, netValueIndex);
                    bestAction = 0;
                    minNotFeasible = false;
                }

                // Deploy maximum number of sensors allowed
                curAction = maxAction;
                ageState[0] = curAction;

                if (curAction > 0){
                    tBudget = budgetRemaining - cFixed - (cVar * curAction);
                    cost = cFixed + cVar * curAction;
                } else{
                    tBudget = budgetRemaining;
                    cost = 0;
                }

                int tempNetSize = curNetworkSize + curAction;
                netValueIndex = tempNetSize - nMin;

                if (tempNetSize < nMin){
                    rel = 0;
                    netValueIndex = 0;
                }
                else{
                    rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
                }

                tempValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

                if (tempValue > bestValue || minNotFeasible){
                    bestValue = tempValue;
                    bestAction = curAction;
                    bestCost = cost;
                    bestRel = rel;

                    bestDecisionBudget = tBudget;
                    bestDecisionSize = netValueIndex;
                }

                curAction = curAction - deployInterval;

                while (rel > relRequirement && curAction > minDeploy){

                    ageState[0] = curAction;
                    tBudget = budgetRemaining - cFixed - (cVar * curAction);

                    cost = cFixed + cVar * curAction;

                    tempNetSize = curNetworkSize + curAction;
                    netValueIndex = tempNetSize - nMin;

                    if (tempNetSize < nMin){
                        rel = 0;
                        netValueIndex = 0;
                    }
                    else{
                        rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
                    }

                    tempValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

                    if (tempValue > bestValue && rel > relRequirement){
                        bestValue = tempValue;
                        bestAction = curAction;
                        bestCost = cost;
                        bestRel = rel;

                        bestDecisionBudget = tBudget;
                        bestDecisionSize = netValueIndex;
                    }
                    curAction = curAction - deployInterval;
                }
            }

            double tAlpha = myValues.GetStepSize(curMission - 1, postDecisionBudget, postDecisionSize);
            double tValue = myValues.GetStateValue(curMission - 1, postDecisionBudget, postDecisionSize);

            double newValue = ((1.0 - tAlpha) * tValue) + (tAlpha * bestValue);

            myValues.UpdateTempStateValue(curMission - 1, postDecisionBudget, postDecisionSize, newValue);
            myValues.ADP_MonotoneUpdate(curMission - 1, postDecisionBudget, postDecisionSize, bestValue);

            postDecisionBudget = bestDecisionBudget;
            postDecisionSize = bestDecisionSize;

            // redeploy sensors, and generate a new fail time for those added
            myNetworkBuckets.GenerateNewFailTimeADP_Update(nodeTime, (double) nMissions*delta, curTime, bestAction);

            iterationCost = iterationCost + bestCost;
            missionSuccess = missionSuccess + bestRel;
            reliabilityRecord.push_back(bestRel);
            missionDeployAction.push_back(bestAction);
            budgetRemaining = budgetRemaining - bestCost;

            curTime = curTime + delta;
            curMission = curMission + 1;

            vector<int> ageVector(maxAge, 0);
            myNetworkBuckets.UpdateState(curTime, delta, ageVector, expectedNum);
            copy(ageVector.begin(), ageVector.end(), ageState.begin());
        }
        costRecord.push_back(iterationCost);
        cumulativeMissionSuccess.push_back(missionSuccess);

        myValues.UpdateValueFunctionsStC(learningRate);
    }

    auto finishValueFunctionApprox = chrono::high_resolution_clock::now();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print out value functions
    cout << " Print Value Functions " << endl;
    myValues.PrintValueFunctions();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myObj;
    myObj.open ("CostFunctionImprovement.txt");

    for (int i = 0; i < costRecord.size() ; ++i) {
        myObj << "Iteration " << i << " -- ObjectiveValue : " << costRecord.at(i) << endl;
    }

    myObj.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // print out reliability over time
    ofstream myReliabilityFile;
    myReliabilityFile.open("ReliabilityRecord.txt");

    for (int j = 0; j < reliabilityRecord.size(); ++j) {
        myReliabilityFile << "Record " << j << " -- Reliability : " << reliabilityRecord.at(j) << endl;
    }

    myReliabilityFile.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream mySuccess;
    mySuccess.open ("ExpectedNumSuccessfulMissions.txt");

    for (int i = 0; i < cumulativeMissionSuccess.size() ; ++i) {
        mySuccess << "Iteration " << i << " -- ObjectiveValue : " << cumulativeMissionSuccess.at(i) << endl;
    }

    mySuccess.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myActions;
    myActions.open ("NetworkSizeAndAction.txt");

    for (int i = 0; i < missionDeployAction.size() ; ++i) {
        myActions << "Iteration " << i << " -- NetworkSize : " << netSizeBeginning.at(i) << " -- Action : " << missionDeployAction.at(i) << endl;
    }

    myActions.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // MC simulation to evaluate performance of value functions
    auto startEval = chrono::high_resolution_clock::now();

    numNodes = nMax + numTargets + 1;
    NodeLinkedListMC myNetworkBuckets_ADPEval(numNodes, (numNodes - numTargets), numTargets);

    myNetworkBuckets_ADPEval.SetRanges(0.0750000, 0.0750000);
    myNetworkBuckets_ADPEval.UniformTargetLoc();
    myNetworkBuckets_ADPEval.SetBinLimits(4);

    vector<double> subAreaMC;
    vector<double> subWeightsMC;

    myNetworkBuckets_ADPEval.SubregionArea(subAreaMC);
    myNetworkBuckets_ADPEval.SubregionDistanceWeight(subWeightsMC);

    costRecord.clear();
    reliabilityRecord.clear();
    cumulativeMissionSuccess.clear();
    netSizeBeginning.clear();
    missionDeployAction.clear();

    double timePlace;
    vector<int> netStatus(nMissions,0);
    vector<int> successRecord;
    vector<double> penaltyCostRecord;
    vector<double> actionPenalty;

    for (int k = 1; k <= numReps ; ++k) {

        cout << "Start of MC Iteration " << k << endl;

        vector<double> gradientVect(numRegions,0);
        vector<int> numInRegion(numRegions, 20);
        double iterationCost = cFixed + (cVar * nSens);
        double missionSuccess = 0;
        int totalSuccess = 0;
        double penaltyCost = 0;

        budgetRemaining = Budget - iterationCost;

        // initial deployment of sensors and
        InitialSensorDeployment(gradientVect, numInRegion, subAreaMC, subWeightsMC, nSens);

        netSizeBeginning.push_back(0);
        missionDeployAction.push_back(nSens);
        actionPenalty.push_back(0);

        // Generate initial fail times and set ages
        myNetworkBuckets_ADPEval.GenerateNewFailTime_ADPEval(nodeTime, nSens, (double) nMissions*delta);
        myNetworkBuckets_ADPEval.UpdateFailTimes(nodeTime);

        // Estimate reliability of initial state
        vector<int> ageState(maxAge, 0);
        vector<int> regionState(numRegions, 0);

        myNetworkBuckets_ADPEval.GenerateNewSpecificRGG_Bucket_ADPEval(numInRegion);

        ageState[0] = nSens;
        double initialRel = mySignatures.ReliabilityEstimate(delta, nSens, ageState);
        reliabilityRecord.push_back(initialRel);
        missionSuccess = missionSuccess + initialRel;

        double tarCovered;
        double curTime = delta;
        int curMission = 1;

        ageState[0] = 0;
        // update state and determine post decision state variable
        myNetworkBuckets_ADPEval.UpdateState(curTime, delta, regionState, ageState, expectedNum);

        while (curMission < nMissions) {

            myNetworkBuckets_ADPEval.UpdateRGG_ADPEval(curTime);
            myNetworkBuckets_ADPEval.UpdateFailTimes(nodeTime);

            tarCovered = myNetworkBuckets_ADPEval.BreadthFirstSearchReturn_ADPEval(numNodes, numTargets, timePlace);

            if (tarCovered >= covRequirement){
                netStatus[curMission-1] = netStatus[curMission-1] + 1;
                totalSuccess = totalSuccess + 1;
            }

            int curNetworkSize = accumulate(ageState.begin(), ageState.end(),0);
            int curAction = nMax - curNetworkSize;

            netSizeBeginning.push_back(curNetworkSize);

            int maxAction = max(0, min(curAction, (int) floor((budgetRemaining - cFixed) / cVar)));

            int bestAction = 0;
            double bestValue = 0;
            double cost = 0;
            double tempValue;
            double bestRel = 0;
            double bestCost = 0;
            double tBudget;
            double rel;

            bool minNotFeasible = true;

            // Search for optimal action
            // action: decide not to deploy any new sensors
            netValueIndex = curNetworkSize - nMin;
            if (curNetworkSize < nMin){
                rel = 0;
                netValueIndex = 0;
            }
            else{
                rel = mySignatures.ReliabilityEstimate(delta, curNetworkSize, ageState);
                bestRel = rel;
                if (rel > relRequirement){
                    minNotFeasible = false;
                }
            }

            bestValue = rel + myValues.GetStateValue(curMission, budgetRemaining, netValueIndex);
            bestAction = 0;

            // Deploy maximum number of sensors available
            curAction = maxAction;
            ageState[0] = curAction;

            if (curAction > 0){
                tBudget = budgetRemaining - cFixed - (cVar * curAction);
                cost = cFixed + cVar * curAction;
            }
            else {
                tBudget = budgetRemaining;
                cost = 0;
            }

            int tempNetSize = curNetworkSize + curAction;
            netValueIndex = tempNetSize - nMin;

            if (tempNetSize < nMin){
                rel = 0;
                netValueIndex = 0;
            }
            else{
                rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
            }

            tempValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

            if (tempValue > bestValue || minNotFeasible){
                bestValue = tempValue;
                bestAction = curAction;
                bestCost = cost;
                bestRel = rel;
            }

            if (rel < relRequirement){
                bool penalty = true;

                while(penalty){
                    curAction = curAction + deployInterval;

                    ageState[0] = curAction;

                    cost = cFixed + cVar * curAction;

                    tempNetSize = curNetworkSize + curAction;

                    if (tempNetSize < nMin){
                        rel = 0;
                    }
                    else{
                        rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);

                        if (rel > relRequirement){
                            penalty = false;
                            penaltyCost = penaltyCost + (cost - budgetRemaining);

                            bestCost = budgetRemaining;
                            budgetRemaining = 0;

                            bestAction = curAction;
                            bestRel = rel;

                        }
                    }
                }
            }
            else {

                curAction = curAction - deployInterval;

                while (rel > relRequirement && curAction > minDeploy){

                    ageState[0] = curAction;
                    tBudget = budgetRemaining - cFixed - (cVar * curAction);

                    cost = cFixed + cVar * curAction;

                    tempNetSize = curNetworkSize + curAction;
                    netValueIndex = tempNetSize - nMin;

                    if (tempNetSize < nMin){
                        rel = 0;
                        netValueIndex = 0;
                    }
                    else{
                        rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
                    }

                    tempValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

                    if (tempValue > bestValue && rel > relRequirement){
                        bestValue = tempValue;
                        bestAction = curAction;
                        bestCost = cost;
                        bestRel = rel;
                    }
                    curAction = curAction - deployInterval;
                }
            }

 //           vector<int> regionDeployment(numRegions, 0);
            vector<int> regionDeployment;

            SensorDeploymentAction(regionState, subAreaMC, subWeightsMC, bestAction, regionDeployment);

            // redeploy sensors, and generate a new fail time for those added
            myNetworkBuckets_ADPEval.GenerateNewFailTimeADP_ADPEval(nodeTime, (double) nMissions*delta, curTime, regionDeployment);

            myNetworkBuckets_ADPEval.GenerateNewRGG_ADPMaintAction();

            iterationCost = iterationCost + bestCost;
            missionSuccess = missionSuccess + bestRel;
            reliabilityRecord.push_back(bestRel);
            missionDeployAction.push_back(bestAction);
            actionPenalty.push_back(penaltyCost);
            budgetRemaining = budgetRemaining - bestCost;

            curTime = curTime + delta;
            curMission = curMission + 1;

            vector<int> ageVector(maxAge, 0);
            vector<int> regionVector(numRegions, 0);

            myNetworkBuckets_ADPEval.UpdateState(curTime, delta, regionVector, ageVector, expectedNum);
            copy(ageVector.begin(), ageVector.end(), ageState.begin());
            copy(regionVector.begin(), regionVector.end(), regionState.begin());

        }

        myNetworkBuckets_ADPEval.UpdateRGG_ADPEval(curTime);
        myNetworkBuckets_ADPEval.UpdateFailTimes(nodeTime);
        tarCovered = myNetworkBuckets_ADPEval.BreadthFirstSearchReturn_ADPEval(numNodes, numTargets, timePlace);
        if (tarCovered >= covRequirement){
            netStatus[curMission-1] = netStatus[curMission-1] + 1;
            totalSuccess = totalSuccess + 1;
        }

        costRecord.push_back(iterationCost);
        cumulativeMissionSuccess.push_back(missionSuccess);
        successRecord.push_back(totalSuccess);
        penaltyCostRecord.push_back(penaltyCost);
    }

    int tSuccess = 0;
    for (int m = 0; m < successRecord.size() ; ++m) {
        tSuccess = tSuccess + successRecord.at(m);
    }

    auto totalTime = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsedNetwork = totalTime - startTime;

    chrono::duration<double> approxTime = finishValueFunctionApprox - startTime;
    chrono::duration<double> evalTime = totalTime - startEval;

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myObjMC;
    myObjMC.open ("CostFunctionMC.txt");

    for (int i = 0; i < costRecord.size() ; ++i) {
        myObjMC << "Iteration " << i << " -- ObjectiveValue : " << costRecord.at(i) << endl;
    }

    myObjMC.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myPenaltyCosts;
    myPenaltyCosts.open ("PenaltyCostFunctionMC.txt");

    for (int i = 0; i < penaltyCostRecord.size() ; ++i) {
        myPenaltyCosts << "Iteration " << i << " -- ObjectiveValue : " << penaltyCostRecord.at(i) << endl;
    }
    myPenaltyCosts.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // print out reliability over time
    ofstream myReliabilityFileMC;
    myReliabilityFileMC.open("ReliabilityRecordMC.txt");

    for (int j = 0; j < reliabilityRecord.size(); ++j) {
        myReliabilityFileMC << "Record " << j << " -- Reliability : " << reliabilityRecord.at(j) << endl;
    }

    myReliabilityFileMC.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream mySuccessMC;
    mySuccessMC.open ("ExpectedNumSuccessfulMissionsMC.txt");

    for (int i = 0; i < cumulativeMissionSuccess.size() ; ++i) {
        mySuccessMC << "Iteration " << i << " -- ObjectiveValue : " << cumulativeMissionSuccess.at(i) << endl;
    }

    mySuccessMC.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    cout << "Observed Average Number Successful Mission : " << (double) tSuccess / (double) numReps << endl;

    ofstream myObservedSuccessMC;
    myObservedSuccessMC.open("ObservedNumSuccessfulMissionsMC.txt");
    for (int l = 0; l <netStatus.size() ; ++l) {
        myObservedSuccessMC << "Mission : " << l << " - Status : " << netStatus.at(l) << " of " << numReps << endl;
        cout << "Mission : " << l << " - Status : " << netStatus.at(l) << " of " << numReps << endl;
    }

    for (int l = 0; l < successRecord.size(); ++l) {
        myObservedSuccessMC << "record " << l << " status : " << successRecord.at(l) << endl;
    }

    myObservedSuccessMC.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myActionsMC;
    myActionsMC.open ("NetworkSizeAndActionMC.txt");

    for (int i = 0; i < missionDeployAction.size() ; ++i) {
        myActionsMC << "Iteration " << i << " -- NetworkSize : " << netSizeBeginning.at(i) << " -- Action : " << missionDeployAction.at(i) << " --Penalty Cost : " << actionPenalty.at(i) << endl;
    }

    myActionsMC.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    ofstream myNotes;
    myNotes.open("Notes.txt");

    myNotes << "Function : ADP AVI Reliability Model with Requirement " << endl;
    myNotes << "Total Time Required : " << elapsedNetwork.count() << endl;
    myNotes << "  Value Function Approximation Time : " << approxTime.count() << endl;
    myNotes << "  Policy Evaluation Time : " << evalTime.count() << endl;
    myNotes << "Number of Replications : " << numReps << endl;
    myNotes << "Budget Available : " << Budget << endl;
    myNotes << "Number of Missions : " << nMissions << endl;
    myNotes << "Time Between Maintenance Inspections : " << delta << endl;
    myNotes << "Minimum number of sensors to deploy : " << minDeploy << endl;
    myNotes << "Sensor Deployment Interval : " << deployInterval << endl;
    myNotes << "Minimum Network Size : " << nMin << endl;
    myNotes << "Maximum Network Size : " << nMax << endl;
    myNotes << "Minimum Reliability Requirement : " << relRequirement << endl;
    myNotes << "Observed Average Number Successful Mission : " << (double) tSuccess / (double) numReps << endl;

    myNotes.close();

    cout << "Total Time Required : " << elapsedNetwork.count() << endl;
    cout << "  Value Function Approximation Time : " << approxTime.count() << endl;
    cout << "  Policy Evaluation Time : " << evalTime.count() << endl;

}

// CBM ADP model using AVI look up tables. The objective is to maximize the number of successful missions subject to a budget constraint.
// Uses a neighborhood of the current state to update the value functions for states `close' to the one that is actually visited.
// Once the value functions are updated a monte carlo simulation is performed to evaluate the performance of the resulting maintenance policy
// This function also requires a minimum reliability requirement, where each mission must achieve a reliability above this input
// This function also only considers a single subregion for comparison purposes with a TBM policy
void ADP_CBM_AVI_ReliabilityModel_CAVE_EvaluateMC_MinRequirement_SingleRegion(int nSens, int numReps, int nMissions, double delta, double Budget,
                                                                 double cFixed, double cVar, double covRequirement, double relRequirement) {

    uniform_real_distribution<> myUnif(0,1);

    random_device myDev;
    random_device myRandomOrder;

    const int maxAge = 24;
    const int nMin = 300;
    const int nMax = 950;
    const int deployInterval = 10;
    const int minDeploy = 190;
    const double learningRate = 0.7;

    double explorationProb = 0.05;
    double curShape = 1.5;
    double curScale = 10.0;
    double expectedNum;
    int netValueIndex;
    double budgetRemaining;

    vector<double> nodeTime;
    vector<double> costRecord;
    vector<double> reliabilityRecord;
    vector<double> cumulativeMissionSuccess;
    vector<double> valueFunctionPerIteration;
    vector<int> netSizeBeginning;
    vector<int> missionDeployAction;

    // number of nodes includes sink node, number of sensors, and number of targets
    int numTargets = 441;
    int numNodes = nMax + numTargets + 1;

    SingleRegionSignature mySignature("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADPSingleSignature.txt");
    mySignature.SetFailureDist(curShape, curScale);

    ValueFunctionReliabilityCollection myValues(nMissions, learningRate, Budget, nMin, nMax);
    myValues.ConstructValueFunction(nMissions, Budget, nMin, nMax);
//    myValues.InitializeValueFunctionStC("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADPInitialValues_d3_m33.txt");
    myValues.InitializeValueFunctionGHarmonic("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADP_SingleRegion_d2_m50.txt");

    NodeLinkedListMC myNetworkBuckets(numNodes, (numNodes - numTargets), numTargets);

    myNetworkBuckets.SetRanges(0.0750000, 0.0750000);
    myNetworkBuckets.UniformTargetLoc();


    auto startTime = chrono::high_resolution_clock::now();

    for (int k = 1; k <= numReps ; ++k) {

        if (k > ((double) numReps / 2.0)){
            explorationProb = 0.00;
        }

        cout << "Start of Iteration " << k << endl;

        double iterationCost = cFixed + (cVar * nSens);
        double missionSuccess = 0;

        budgetRemaining = Budget - iterationCost;

        netSizeBeginning.push_back(0);
        missionDeployAction.push_back(nSens);

        // Generate initial fail times and set ages
        myNetworkBuckets.GenerateNewFailTime_ADPEval(nodeTime, nSens);
        myNetworkBuckets.UpdateFailTimesADP(nodeTime);
        myNetworkBuckets.ClearSensorAge();

        // Estimate reliability of initial state
        vector<int> ageState(maxAge, 0);
        ageState[0] = nSens;

        double initialRel = mySignature.ReliabilityEstimate(delta, nSens, ageState);
        reliabilityRecord.push_back(initialRel);
        missionSuccess = missionSuccess + initialRel;

        double postDecisionBudget = budgetRemaining;
        int postDecisionSize = (int) accumulate(ageState.begin(), ageState.end(),0) - nMin;

        double curTime = delta;
        int curMission = 1;

        // update state and determine post decision state variable
        ageState[0] = 0;
        myNetworkBuckets.UpdateState(curTime, delta, ageState, expectedNum);

        double bestDecisionBudget;
        int bestDecisionSize;

        while (curMission < nMissions) {

            int curNetworkSize = accumulate(ageState.begin(), ageState.end(),0);
            int curAction = nMax - curNetworkSize;

            netSizeBeginning.push_back(curNetworkSize);

            int maxAction = max(0, min(curAction, (int) floor((budgetRemaining - cFixed) / cVar)));

            int bestAction = 0;
            double bestValue = 0;
            double cost = 0;
            double tempValue;
            double bestRel = 0;
            double bestCost = 0;
            double tBudget;
            double rel;

            double myNum = myUnif(myDev);

            // Select action at random
            if (myNum < explorationProb){
                int minAction = 0;
                bool actionNotFound = true;

                while (actionNotFound){

                    uniform_int_distribution<> myRandomDeploy(minAction, maxAction);

                    bestAction = myRandomDeploy(myRandomOrder);
                    ageState[0] = bestAction;

                    if (bestAction > 0){
                        tBudget = budgetRemaining - cFixed - (cVar * bestAction);
                        cost = cFixed + cVar * bestAction;
                    }
                    else {
                        tBudget = budgetRemaining;
                        cost = 0;
                    }

                    int tempNetSize = curNetworkSize + bestAction;
                    netValueIndex = tempNetSize - nMin;

                    if (tempNetSize < nMin){
                        rel = 0;
                        netValueIndex = 0;
                    }
                    else{
                        rel = mySignature.ReliabilityEstimate(delta, tempNetSize, ageState);
                    }

                    bestValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

                    if (rel > relRequirement || (maxAction - minAction < 2)){
                        actionNotFound = false;
                    }
                    else {
                        minAction = bestAction;
                    }

                    bestCost = cost;
                    bestRel = rel;

                    bestDecisionBudget = tBudget;
                    bestDecisionSize = netValueIndex;
                }
            }
            else {
                // Search for optimal action

                bool minNotFeasible = true;
                // action: decide not to deploy any new sensors
                netValueIndex = curNetworkSize - nMin;
                if (curNetworkSize < nMin){
                    rel = 0;
                    netValueIndex = 0;
                }
                else{
                    rel = mySignature.ReliabilityEstimate(delta, curNetworkSize, ageState);
                    bestRel = rel;
                }

                if (rel >= relRequirement){
                    bestDecisionBudget = budgetRemaining;
                    bestDecisionSize = netValueIndex;

                    bestValue = rel + myValues.GetStateValue(curMission, budgetRemaining, netValueIndex);
                    bestAction = 0;
                    minNotFeasible = false;
                }

                // Deploy maximum number of sensors allowed
                curAction = maxAction;
                ageState[0] = curAction;

                if (curAction > 0){
                    tBudget = budgetRemaining - cFixed - (cVar * curAction);
                    cost = cFixed + cVar * curAction;
                } else{
                    tBudget = budgetRemaining;
                    cost = 0;
                }

                int tempNetSize = curNetworkSize + curAction;
                netValueIndex = tempNetSize - nMin;

                if (tempNetSize < nMin){
                    rel = 0;
                    netValueIndex = 0;
                }
                else{
                    rel = mySignature.ReliabilityEstimate(delta, tempNetSize, ageState);
                }

                tempValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

                if (tempValue >= bestValue || minNotFeasible){
                    bestValue = tempValue;
                    bestAction = curAction;
                    bestCost = cost;
                    bestRel = rel;

                    bestDecisionBudget = tBudget;
                    bestDecisionSize = netValueIndex;
                }

                curAction = curAction - deployInterval;

                while (rel > relRequirement && curAction > minDeploy){

                    ageState[0] = curAction;
                    tBudget = budgetRemaining - cFixed - (cVar * curAction);

                    cost = cFixed + cVar * curAction;

                    tempNetSize = curNetworkSize + curAction;
                    netValueIndex = tempNetSize - nMin;

                    if (tempNetSize < nMin){
                        rel = 0;
                        netValueIndex = 0;
                    }
                    else{
                        rel = mySignature.ReliabilityEstimate(delta, tempNetSize, ageState);
                    }

                    tempValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

                    if (tempValue >= bestValue && rel > relRequirement){
                        bestValue = tempValue;
                        bestAction = curAction;
                        bestCost = cost;
                        bestRel = rel;

                        bestDecisionBudget = tBudget;
                        bestDecisionSize = netValueIndex;
                    }
                    curAction = curAction - deployInterval;
                }
            }

            double tAlpha = myValues.GetStepSize(curMission - 1, postDecisionBudget, postDecisionSize);
            double tValue = myValues.GetStateValue(curMission - 1, postDecisionBudget, postDecisionSize);

            double newValue = ((1.0 - tAlpha) * tValue) + (tAlpha * bestValue);

            myValues.UpdateTempStateValue(curMission - 1, postDecisionBudget, postDecisionSize, newValue);
            myValues.ADP_MonotoneUpdate(curMission - 1, postDecisionBudget, postDecisionSize, bestValue);

            postDecisionBudget = bestDecisionBudget;
            postDecisionSize = bestDecisionSize;

            // redeploy sensors, and generate a new fail time for those added
            myNetworkBuckets.GenerateNewFailTimeADP_Update(nodeTime, (double) nMissions*delta, curTime, bestAction);

            iterationCost = iterationCost + bestCost;
            missionSuccess = missionSuccess + bestRel;
            reliabilityRecord.push_back(bestRel);
            missionDeployAction.push_back(bestAction);
            budgetRemaining = budgetRemaining - bestCost;

            curTime = curTime + delta;
            curMission = curMission + 1;

            vector<int> ageVector(maxAge, 0);
            myNetworkBuckets.UpdateState(curTime, delta, ageVector, expectedNum);
            copy(ageVector.begin(), ageVector.end(), ageState.begin());
        }
        costRecord.push_back(iterationCost);
        cumulativeMissionSuccess.push_back(missionSuccess);

//        myValues.UpdateValueFunctionsStC(learningRate);
        myValues.UpdateValueFunctionsGHarmonic(learningRate);

        valueFunctionPerIteration.push_back(myValues.GetStateValue(0, Budget - 750, nSens - nMin));
    }

    auto finishValueFunctionApprox = chrono::high_resolution_clock::now();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print out value functions
    cout << " Print Value Functions " << endl;
    myValues.PrintValueFunctions();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myObj;
    myObj.open ("CostFunctionImprovement.txt");

    for (int i = 0; i < costRecord.size() ; ++i) {
        myObj << "Iteration " << i << " -- ObjectiveValue : " << costRecord.at(i) << endl;
    }

    myObj.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // print out reliability over time
    ofstream myReliabilityFile;
    myReliabilityFile.open("ReliabilityRecord.txt");

    for (int j = 0; j < reliabilityRecord.size(); ++j) {
        myReliabilityFile << "Record " << j << " -- Reliability : " << reliabilityRecord.at(j) << endl;
    }

    myReliabilityFile.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream mySuccess;
    mySuccess.open ("ExpectedNumSuccessfulMissions.txt");

    for (int i = 0; i < cumulativeMissionSuccess.size() ; ++i) {
        mySuccess << "Iteration " << i << " -- ObjectiveValue : " << cumulativeMissionSuccess.at(i) << " -- ValueFunctionApprox : " << valueFunctionPerIteration.at(i) << endl;
    }

    mySuccess.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myActions;
    myActions.open ("NetworkSizeAndAction.txt");

    for (int i = 0; i < missionDeployAction.size() ; ++i) {
        myActions << "Iteration " << i << " -- NetworkSize : " << netSizeBeginning.at(i) << " -- Action : " << missionDeployAction.at(i) << endl;
    }

    myActions.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // MC simulation to evaluate performance of value functions
    auto startEval = chrono::high_resolution_clock::now();

    numNodes = nMax + numTargets + 1;
    NodeLinkedListMC myNetworkBuckets_ADPEval(numNodes, (numNodes - numTargets), numTargets);

    myNetworkBuckets_ADPEval.SetRanges(0.0750000, 0.0750000);
    myNetworkBuckets_ADPEval.UniformTargetLoc();
    myNetworkBuckets_ADPEval.SetBinLimits(4);


    costRecord.clear();
    reliabilityRecord.clear();
    cumulativeMissionSuccess.clear();
    netSizeBeginning.clear();
    missionDeployAction.clear();

    double timePlace;
    vector<int> netStatus(nMissions,0);
    vector<int> successRecord;

    for (int k = 1; k <= numReps ; ++k) {

        cout << "Start of MC Iteration " << k << endl;

        double iterationCost = cFixed + (cVar * nSens);
        double missionSuccess = 0;
        int totalSuccess = 0;

        budgetRemaining = Budget - iterationCost;

        netSizeBeginning.push_back(0);
        missionDeployAction.push_back(nSens);

        // Generate initial fail times and set ages
        myNetworkBuckets_ADPEval.GenerateNewFailTime_ADPEval(nodeTime, nSens, (double) nMissions*delta);
        myNetworkBuckets_ADPEval.UpdateFailTimes(nodeTime);

        // Estimate reliability of initial state
        vector<int> ageState(maxAge, 0);

        myNetworkBuckets_ADPEval.GenerateNewRGG_Bucket_ADPEval(nSens);

        ageState[0] = nSens;
        double initialRel = mySignature.ReliabilityEstimate(delta, nSens, ageState);
        reliabilityRecord.push_back(initialRel);
        missionSuccess = missionSuccess + initialRel;

        double tarCovered;
        double curTime = delta;
        int curMission = 1;

        ageState[0] = 0;
        // update state and determine post decision state variable
        myNetworkBuckets_ADPEval.UpdateState(curTime, delta, ageState, expectedNum);

        while (curMission < nMissions) {

            myNetworkBuckets_ADPEval.UpdateRGG_ADPEval(curTime);
            myNetworkBuckets_ADPEval.UpdateFailTimes(nodeTime);

            tarCovered = myNetworkBuckets_ADPEval.BreadthFirstSearchReturn_ADPEval(numNodes, numTargets, timePlace);

            if (tarCovered >= covRequirement){
                netStatus[curMission-1] = netStatus[curMission-1] + 1;
                totalSuccess = totalSuccess + 1;
            }

            int curNetworkSize = accumulate(ageState.begin(), ageState.end(),0);
            int curAction = nMax - curNetworkSize;

            netSizeBeginning.push_back(curNetworkSize);

            int maxAction = max(0, min(curAction, (int) floor((budgetRemaining - cFixed) / cVar)));

            int bestAction = 0;
            double bestValue = 0;
            double cost = 0;
            double tempValue;
            double bestRel = 0;
            double bestCost = 0;
            double tBudget;
            double rel;

            bool minNotFeasible = true;

            // Search for optimal action
            // action: decide not to deploy any new sensors
            netValueIndex = curNetworkSize - nMin;
            if (curNetworkSize < nMin){
                rel = 0;
                netValueIndex = 0;
            }
            else{
                rel = mySignature.ReliabilityEstimate(delta, curNetworkSize, ageState);
                bestRel = rel;
                if (rel > relRequirement){
                    minNotFeasible = false;
                }
            }

            bestValue = rel + myValues.GetStateValue(curMission, budgetRemaining, netValueIndex);
            bestAction = 0;

            // Deploy maximum number of sensors available
            curAction = maxAction;
            ageState[0] = curAction;

            if (curAction > 0){
                tBudget = budgetRemaining - cFixed - (cVar * curAction);
                cost = cFixed + cVar * curAction;
            }
            else {
                tBudget = budgetRemaining;
                cost = 0;
            }

            int tempNetSize = curNetworkSize + curAction;
            netValueIndex = tempNetSize - nMin;

            if (tempNetSize < nMin){
                rel = 0;
                netValueIndex = 0;
            }
            else{
                rel = mySignature.ReliabilityEstimate(delta, tempNetSize, ageState);
            }

            tempValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

            if (tempValue >= bestValue || minNotFeasible){
                bestValue = tempValue;
                bestAction = curAction;
                bestCost = cost;
                bestRel = rel;
            }

            curAction = curAction - deployInterval;

            while (rel > relRequirement && curAction > minDeploy){

                ageState[0] = curAction;
                tBudget = budgetRemaining - cFixed - (cVar * curAction);

                cost = cFixed + cVar * curAction;

                tempNetSize = curNetworkSize + curAction;
                netValueIndex = tempNetSize - nMin;

                if (tempNetSize < nMin){
                    rel = 0;
                    netValueIndex = 0;
                }
                else{
                    rel = mySignature.ReliabilityEstimate(delta, tempNetSize, ageState);
                }

                tempValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

                if (tempValue >= bestValue && rel > relRequirement){
                    bestValue = tempValue;
                    bestAction = curAction;
                    bestCost = cost;
                    bestRel = rel;
                }
                curAction = curAction - deployInterval;
            }

            vector<int> regionDeployment(bestAction,1);

            // redeploy sensors, and generate a new fail time for those added
            myNetworkBuckets_ADPEval.GenerateNewFailTimeADP_ADPEval(nodeTime, (double) nMissions*delta, curTime, regionDeployment);

            myNetworkBuckets_ADPEval.GenerateNewRGG_ADPMaintAction_SingleRegion();

            iterationCost = iterationCost + bestCost;
            missionSuccess = missionSuccess + bestRel;
            reliabilityRecord.push_back(bestRel);
            missionDeployAction.push_back(bestAction);
            budgetRemaining = budgetRemaining - bestCost;

            curTime = curTime + delta;
            curMission = curMission + 1;

            vector<int> ageVector(maxAge, 0);

            myNetworkBuckets_ADPEval.UpdateState(curTime, delta, ageVector, expectedNum);
            copy(ageVector.begin(), ageVector.end(), ageState.begin());

        }

        myNetworkBuckets_ADPEval.UpdateRGG_ADPEval(curTime);
        myNetworkBuckets_ADPEval.UpdateFailTimes(nodeTime);
        tarCovered = myNetworkBuckets_ADPEval.BreadthFirstSearchReturn_ADPEval(numNodes, numTargets, timePlace);
        if (tarCovered >= covRequirement){
            netStatus[curMission-1] = netStatus[curMission-1] + 1;
            totalSuccess = totalSuccess + 1;
        }

        costRecord.push_back(iterationCost);
        cumulativeMissionSuccess.push_back(missionSuccess);
        successRecord.push_back(totalSuccess);
    }

    int tSuccess = 0;
    for (int m = 0; m < successRecord.size() ; ++m) {
        tSuccess = tSuccess + successRecord.at(m);
    }

    auto totalTime = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsedNetwork = totalTime - startTime;

    chrono::duration<double> approxTime = finishValueFunctionApprox - startTime;
    chrono::duration<double> evalTime = totalTime - startEval;

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myObjMC;
    myObjMC.open ("CostFunctionMC.txt");

    for (int i = 0; i < costRecord.size() ; ++i) {
        myObjMC << "Iteration " << i << " -- ObjectiveValue : " << costRecord.at(i) << endl;
    }

    myObjMC.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // print out reliability over time
    ofstream myReliabilityFileMC;
    myReliabilityFileMC.open("ReliabilityRecordMC.txt");

    for (int j = 0; j < reliabilityRecord.size(); ++j) {
        myReliabilityFileMC << "Record " << j << " -- Reliability : " << reliabilityRecord.at(j) << endl;
    }

    myReliabilityFileMC.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream mySuccessMC;
    mySuccessMC.open ("ExpectedNumSuccessfulMissionsMC.txt");

    for (int i = 0; i < cumulativeMissionSuccess.size() ; ++i) {
        mySuccessMC << "Iteration " << i << " -- ObjectiveValue : " << cumulativeMissionSuccess.at(i) << endl;
    }

    mySuccessMC.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    cout << "Observed Average Number Successful Mission : " << (double) tSuccess / (double) numReps << endl;

    ofstream myObservedSuccessMC;
    myObservedSuccessMC.open("ObservedNumSuccessfulMissionsMC.txt");
    for (int l = 0; l <netStatus.size() ; ++l) {
        myObservedSuccessMC << "Mission : " << l << " - Status : " << netStatus.at(l) << " of " << numReps << endl;
        cout << "Mission : " << l << " - Status : " << netStatus.at(l) << " of " << numReps << endl;
    }

    for (int l = 0; l < successRecord.size(); ++l) {
        myObservedSuccessMC << "record " << l << " status : " << successRecord.at(l) << endl;
    }

    myObservedSuccessMC.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myActionsMC;
    myActionsMC.open ("NetworkSizeAndActionMC.txt");

    for (int i = 0; i < missionDeployAction.size() ; ++i) {
        myActionsMC << "Iteration " << i << " -- NetworkSize : " << netSizeBeginning.at(i) << " -- Action : " << missionDeployAction.at(i) << endl;
    }

    myActionsMC.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    ofstream myNotes;
    myNotes.open("Notes.txt");

    myNotes << "Function : Single Region ADP AVI Reliability Model with Requirement " << endl;
    myNotes << "Total Time Required : " << elapsedNetwork.count() << endl;
    myNotes << "  Value Function Approximation Time : " << approxTime.count() << endl;
    myNotes << "  Policy Evaluation Time : " << evalTime.count() << endl;
    myNotes << "Number of Replications : " << numReps << endl;
    myNotes << "Budget Available : " << Budget << endl;
    myNotes << "Number of Missions : " << nMissions << endl;
    myNotes << "Time Between Maintenance Inspections : " << delta << endl;
    myNotes << "Minimum number of sensors to deploy : " << minDeploy << endl;
    myNotes << "Sensor Deployment Interval : " << deployInterval << endl;
    myNotes << "Minimum Network Size : " << nMin << endl;
    myNotes << "Maximum Network Size : " << nMax << endl;
    myNotes << "Minimum Reliability Requirement : " << relRequirement << endl;
    myNotes << "Observed Average Number Successful Mission : " << (double) tSuccess / (double) numReps << endl;

    myNotes.close();

    cout << "Total Time Required : " << elapsedNetwork.count() << endl;
    cout << "  Value Function Approximation Time : " << approxTime.count() << endl;
    cout << "  Policy Evaluation Time : " << evalTime.count() << endl;

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CBM ADP model using AVI look up tables. The objective is to maximize the number of successful missions subject to a budget constraint.
// Uses a neighborhood of the current state to update the value functions for states `close' to the one that is actually visited.
// Once the value functions are updated a monte carlo simulation is performed to evaluate the performance of the resulting maintenance policy
// This function also requires a minimum reliability requirement, where each mission must achieve a reliability above this input
void ADP_CBM_AVI_ReliabilityModel_CAVE_EvaluateMC_MinRequirement_Skip(int nSens, int numReps, int nMissions, double delta, double Budget,
                                                                 double cFixed, double cVar, double covRequirement, double relRequirement) {

    uniform_real_distribution<> myUnif(0,1);

    random_device myDev;
    random_device myRandomOrder;

    const int numRegions = 16;
    const int maxAge = 24;
    const int nMin = 300;
    const int nMax = 950;
    const int deployInterval = 10;
    const int minDeploy = 30;
    const double learningRate = 0.7;

    double explorationProb = 0.05;
    double curShape = 1.5;
    double curScale = 10.0;
    double expectedNum;
    int netValueIndex;
    double budgetRemaining;

    vector<double> nodeTime;
    vector<double> costRecord;
    vector<double> reliabilityRecord;
    vector<double> cumulativeMissionSuccess;
    vector<double> valueFunctionPerIteration;
    vector<int> netSizeBeginning;
    vector<int> missionDeployAction;

    // number of nodes includes sink node, number of sensors, and number of targets
    int numTargets = 441;
    int numNodes = nMax + numTargets + 1;

    SignatureCollection mySignatures(nMax - nMin + 1, nMin, nMax, numRegions);
    mySignatures.ReadSignatures("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADPEntireSignatureCollection_d1_2.txt");
    mySignatures.SetFailureDist(curShape, curScale);

    ValueFunctionReliabilityCollection myValues(nMissions, learningRate, Budget, nMin, nMax);
    myValues.ConstructValueFunction(nMissions, Budget, nMin, nMax);
//    myValues.InitializeValueFunctionStC("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADPInitialValues_d3_m33.txt");
//    myValues.InitializeValueFunctionGHarmonic("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADPInitialValues_d2_m50_new.txt");
    myValues.InitializeValueFunctionGHarmonic("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\testInput2.txt");

    NodeLinkedListMC myNetworkBuckets(numNodes, (numNodes - numTargets), numTargets);

    myNetworkBuckets.SetRanges(0.0750000, 0.0750000);
    myNetworkBuckets.UniformTargetLoc();
    myNetworkBuckets.SetBinLimits(4);

    vector<double> subArea;
    vector<double> subWeights;

    myNetworkBuckets.SubregionArea(subArea);
    myNetworkBuckets.SubregionDistanceWeight(subWeights);

    auto startTime = chrono::high_resolution_clock::now();

    for (int k = 1; k <= numReps ; ++k) {

        if (k > ((double) numReps / 2.0)){
            explorationProb = 0.00;
        }

        cout << "Start of Iteration " << k << endl;

        vector<double> gradientVect(numRegions,0);
        vector<int> numInRegion(numRegions, 20);
        double iterationCost = cFixed + (cVar * nSens);
        double missionSuccess = 0;

        budgetRemaining = Budget - iterationCost;

        // initial deployment of sensors and
        InitialSensorDeployment(gradientVect, numInRegion, subArea, subWeights, nSens);

        netSizeBeginning.push_back(0);
        missionDeployAction.push_back(nSens);

        // Generate initial fail times and set ages
        myNetworkBuckets.GenerateNewFailTime_ADPEval(nodeTime, nSens);
        myNetworkBuckets.UpdateFailTimesADP(nodeTime);
        myNetworkBuckets.ClearSensorAge();

        // Estimate reliability of initial state
        vector<int> ageState(maxAge, 0);
        ageState[0] = nSens;

        double initialRel = mySignatures.ReliabilityEstimate(delta, nSens, ageState);
        reliabilityRecord.push_back(initialRel);
        missionSuccess = missionSuccess + initialRel;

        double postDecisionBudget = budgetRemaining;
        int postDecisionSize = (int) accumulate(ageState.begin(), ageState.end(),0) - nMin;

        double curTime = delta;
        int curMission = 1;

        // update state and determine post decision state variable
        ageState[0] = 0;
        myNetworkBuckets.UpdateState(curTime, delta, ageState, expectedNum);

        double bestDecisionBudget;
        int bestDecisionSize;

        while (curMission < nMissions) {

            int curNetworkSize = accumulate(ageState.begin(), ageState.end(),0);
            int curAction = nMax - curNetworkSize;

            netSizeBeginning.push_back(curNetworkSize);

            int maxAction = max(0, min(curAction, (int) floor((budgetRemaining - cFixed) / cVar)));

            int bestAction = 0;
            double bestValue = 0;
            double cost = 0;
            double tempValue;
            double bestRel = 0;
            double bestCost = 0;
            double tBudget;
            double rel;

            double myNum = myUnif(myDev);

            if (curNetworkSize < nMin){
                rel = 0;
            }
            else{
                rel = mySignatures.ReliabilityEstimate(delta, curNetworkSize, ageState);
            }

            if (rel < relRequirement){

                // Select action at random
                if (myNum < explorationProb){
                    int minAction = 0;
                    bool actionNotFound = true;

                    while (actionNotFound){

                        uniform_int_distribution<> myRandomDeploy(minAction, maxAction);

                        bestAction = myRandomDeploy(myRandomOrder);
                        ageState[0] = bestAction;

                        if (bestAction > 0){
                            tBudget = budgetRemaining - cFixed - (cVar * bestAction);
                            cost = cFixed + cVar * bestAction;
                        }
                        else {
                            tBudget = budgetRemaining;
                            cost = 0;
                        }

                        int tempNetSize = curNetworkSize + bestAction;
                        netValueIndex = tempNetSize - nMin;

                        if (tempNetSize < nMin){
                            rel = 0;
                            netValueIndex = 0;
                        }
                        else{
                            rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
                        }

                        bestValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

/*                    // This part is new. If my exploration action causes me to fail in the future then I might want to select a different action
                    if (myValues.GetStateValue(curMission, tBudget, netValueIndex) / ((double) nMissions - (double) curMission) < relRequirement){
                        maxAction = bestAction;
                        rel = 0;
                    }*/

                        if (rel > relRequirement || (maxAction - minAction < 2)){
                            actionNotFound = false;
                        }
                        else {
                            minAction = bestAction;
                        }

                        bestCost = cost;
                        bestRel = rel;

                        bestDecisionBudget = tBudget;
                        bestDecisionSize = netValueIndex;
                    }
                }
                else {
                    // Search for optimal action

                    bool minNotFeasible = true;
                    // action: decide not to deploy any new sensors
                    netValueIndex = curNetworkSize - nMin;
                    if (curNetworkSize < nMin){
                        rel = 0;
                        netValueIndex = 0;
                    }
                    else{
                        rel = mySignatures.ReliabilityEstimate(delta, curNetworkSize, ageState);
                        bestRel = rel;
                    }

                    if (rel >= relRequirement){
                        bestDecisionBudget = budgetRemaining;
                        bestDecisionSize = netValueIndex;

                        bestValue = rel + myValues.GetStateValue(curMission, budgetRemaining, netValueIndex);
                        bestAction = 0;
                        minNotFeasible = false;
                    }

                    // Deploy maximum number of sensors allowed
                    curAction = maxAction;
                    ageState[0] = curAction;

                    if (curAction > 0){
                        tBudget = budgetRemaining - cFixed - (cVar * curAction);
                        cost = cFixed + cVar * curAction;
                    } else{
                        tBudget = budgetRemaining;
                        cost = 0;
                    }

                    int tempNetSize = curNetworkSize + curAction;
                    netValueIndex = tempNetSize - nMin;

                    if (tempNetSize < nMin){
                        rel = 0;
                        netValueIndex = 0;
                    }
                    else{
                        rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
                    }

                    tempValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

                    if (tempValue >= bestValue || minNotFeasible){
                        bestValue = tempValue;
                        bestAction = curAction;
                        bestCost = cost;
                        bestRel = rel;

                        bestDecisionBudget = tBudget;
                        bestDecisionSize = netValueIndex;
                    }

                    curAction = curAction - deployInterval;

                    while (rel > relRequirement && curAction > minDeploy){

                        ageState[0] = curAction;
                        tBudget = budgetRemaining - cFixed - (cVar * curAction);

                        cost = cFixed + cVar * curAction;

                        tempNetSize = curNetworkSize + curAction;
                        netValueIndex = tempNetSize - nMin;

                        if (tempNetSize < nMin){
                            rel = 0;
                            netValueIndex = 0;
                        }
                        else{
                            rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
                        }

                        tempValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

                        if (tempValue >= bestValue && rel > relRequirement){
                            bestValue = tempValue;
                            bestAction = curAction;
                            bestCost = cost;
                            bestRel = rel;

                            bestDecisionBudget = tBudget;
                            bestDecisionSize = netValueIndex;
                        }
                        curAction = curAction - deployInterval;
                    }
                }
            }
            else {

                netValueIndex = curNetworkSize - nMin;
                bestValue = rel + myValues.GetStateValue(curMission, budgetRemaining, netValueIndex);
                bestAction = 0;
                bestCost = 0;
                bestRel = rel;

                bestDecisionBudget = budgetRemaining;
                bestDecisionSize = netValueIndex;
            }

            double tAlpha = myValues.GetStepSize(curMission - 1, postDecisionBudget, postDecisionSize);
            double tValue = myValues.GetStateValue(curMission - 1, postDecisionBudget, postDecisionSize);

            double newValue = ((1.0 - tAlpha) * tValue) + (tAlpha * bestValue);

            myValues.UpdateTempStateValue(curMission - 1, postDecisionBudget, postDecisionSize, newValue);
            myValues.ADP_MonotoneUpdate(curMission - 1, postDecisionBudget, postDecisionSize, bestValue);

            postDecisionBudget = bestDecisionBudget;
            postDecisionSize = bestDecisionSize;

            // redeploy sensors, and generate a new fail time for those added
            myNetworkBuckets.GenerateNewFailTimeADP_Update(nodeTime, (double) nMissions*delta, curTime, bestAction);

            iterationCost = iterationCost + bestCost;
            missionSuccess = missionSuccess + bestRel;
            reliabilityRecord.push_back(bestRel);
            missionDeployAction.push_back(bestAction);
            budgetRemaining = budgetRemaining - bestCost;

            curTime = curTime + delta;
            curMission = curMission + 1;

            vector<int> ageVector(maxAge, 0);
            myNetworkBuckets.UpdateState(curTime, delta, ageVector, expectedNum);
            copy(ageVector.begin(), ageVector.end(), ageState.begin());
        }
        costRecord.push_back(iterationCost);
        cumulativeMissionSuccess.push_back(missionSuccess);

//        myValues.UpdateValueFunctionsStC(learningRate);
        myValues.UpdateValueFunctionsGHarmonic(learningRate);

        valueFunctionPerIteration.push_back(myValues.GetStateValue(0, Budget - 750, nSens - nMin));
    }

    auto finishValueFunctionApprox = chrono::high_resolution_clock::now();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print out value functions
    cout << " Print Value Functions " << endl;
    myValues.PrintValueFunctions();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myObj;
    myObj.open ("CostFunctionImprovement.txt");

    for (int i = 0; i < costRecord.size() ; ++i) {
        myObj << "Iteration " << i << " -- ObjectiveValue : " << costRecord.at(i) << endl;
    }

    myObj.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // print out reliability over time
    ofstream myReliabilityFile;
    myReliabilityFile.open("ReliabilityRecord.txt");

    for (int j = 0; j < reliabilityRecord.size(); ++j) {
        myReliabilityFile << "Record " << j << " -- Reliability : " << reliabilityRecord.at(j) << endl;
    }

    myReliabilityFile.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream mySuccess;
    mySuccess.open ("ExpectedNumSuccessfulMissions.txt");

    for (int i = 0; i < cumulativeMissionSuccess.size() ; ++i) {
        mySuccess << "Iteration " << i << " -- ObjectiveValue : " << cumulativeMissionSuccess.at(i) << " -- ValueFunctionApprox : " << valueFunctionPerIteration.at(i) << endl;
    }

    mySuccess.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myActions;
    myActions.open ("NetworkSizeAndAction.txt");

    for (int i = 0; i < missionDeployAction.size() ; ++i) {
        myActions << "Iteration " << i << " -- NetworkSize : " << netSizeBeginning.at(i) << " -- Action : " << missionDeployAction.at(i) << endl;
    }

    myActions.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // MC simulation to evaluate performance of value functions
    auto startEval = chrono::high_resolution_clock::now();

    numNodes = nMax + numTargets + 1;
    NodeLinkedListMC myNetworkBuckets_ADPEval(numNodes, (numNodes - numTargets), numTargets);

    myNetworkBuckets_ADPEval.SetRanges(0.0750000, 0.0750000);
    myNetworkBuckets_ADPEval.UniformTargetLoc();
    myNetworkBuckets_ADPEval.SetBinLimits(4);

    vector<double> subAreaMC;
    vector<double> subWeightsMC;

    myNetworkBuckets_ADPEval.SubregionArea(subAreaMC);
    myNetworkBuckets_ADPEval.SubregionDistanceWeight(subWeightsMC);

    costRecord.clear();
    reliabilityRecord.clear();
    cumulativeMissionSuccess.clear();
    netSizeBeginning.clear();
    missionDeployAction.clear();

    double timePlace;
    vector<int> netStatus(nMissions,0);
    vector<int> successRecord;

    for (int k = 1; k <= numReps ; ++k) {

        cout << "Start of MC Iteration " << k << endl;

        vector<double> gradientVect(numRegions,0);
        vector<int> numInRegion(numRegions, 20);
        double iterationCost = cFixed + (cVar * nSens);
        double missionSuccess = 0;
        int totalSuccess = 0;

        budgetRemaining = Budget - iterationCost;

        // initial deployment of sensors and
        InitialSensorDeployment(gradientVect, numInRegion, subAreaMC, subWeightsMC, nSens);

        netSizeBeginning.push_back(0);
        missionDeployAction.push_back(nSens);

        // Generate initial fail times and set ages
        myNetworkBuckets_ADPEval.GenerateNewFailTime_ADPEval(nodeTime, nSens, (double) nMissions*delta);
        myNetworkBuckets_ADPEval.UpdateFailTimes(nodeTime);

        // Estimate reliability of initial state
        vector<int> ageState(maxAge, 0);
        vector<int> regionState(numRegions, 0);

        myNetworkBuckets_ADPEval.GenerateNewSpecificRGG_Bucket_ADPEval(numInRegion);

        ageState[0] = nSens;
        double initialRel = mySignatures.ReliabilityEstimate(delta, nSens, ageState);
        reliabilityRecord.push_back(initialRel);
        missionSuccess = missionSuccess + initialRel;

        double tarCovered;
        double curTime = delta;
        int curMission = 1;

        ageState[0] = 0;
        // update state and determine post decision state variable
        myNetworkBuckets_ADPEval.UpdateState(curTime, delta, regionState, ageState, expectedNum);

        while (curMission < nMissions) {

            myNetworkBuckets_ADPEval.UpdateRGG_ADPEval(curTime);
            myNetworkBuckets_ADPEval.UpdateFailTimes(nodeTime);

            tarCovered = myNetworkBuckets_ADPEval.BreadthFirstSearchReturn_ADPEval(numNodes, numTargets, timePlace);

            if (tarCovered >= covRequirement){
                netStatus[curMission-1] = netStatus[curMission-1] + 1;
                totalSuccess = totalSuccess + 1;
            }

            int curNetworkSize = accumulate(ageState.begin(), ageState.end(),0);
            int curAction = nMax - curNetworkSize;

            netSizeBeginning.push_back(curNetworkSize);

            int maxAction = max(0, min(curAction, (int) floor((budgetRemaining - cFixed) / cVar)));

            int bestAction = 0;
            double bestValue = 0;
            double cost = 0;
            double tempValue;
            double bestRel = 0;
            double bestCost = 0;
            double tBudget;
            double rel;

            bool minNotFeasible = true;

            // Search for optimal action
            // action: decide not to deploy any new sensors
            netValueIndex = curNetworkSize - nMin;
            if (curNetworkSize < nMin){
                rel = 0;
                netValueIndex = 0;
            }
            else{
                rel = mySignatures.ReliabilityEstimate(delta, curNetworkSize, ageState);
                bestRel = rel;
                if (rel > relRequirement){
                    minNotFeasible = false;
                }
            }

            bestValue = rel + myValues.GetStateValue(curMission, budgetRemaining, netValueIndex);
            bestAction = 0;

            // Deploy maximum number of sensors available
            curAction = maxAction;
            ageState[0] = curAction;

            if (curAction > 0){
                tBudget = budgetRemaining - cFixed - (cVar * curAction);
                cost = cFixed + cVar * curAction;
            }
            else {
                tBudget = budgetRemaining;
                cost = 0;
            }

            int tempNetSize = curNetworkSize + curAction;
            netValueIndex = tempNetSize - nMin;

            if (tempNetSize < nMin){
                rel = 0;
                netValueIndex = 0;
            }
            else{
                rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
            }

            tempValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

            if (tempValue >= bestValue || minNotFeasible){
                bestValue = tempValue;
                bestAction = curAction;
                bestCost = cost;
                bestRel = rel;
            }

            curAction = curAction - deployInterval;

            while (rel > relRequirement && curAction > minDeploy){

                ageState[0] = curAction;
                tBudget = budgetRemaining - cFixed - (cVar * curAction);

                cost = cFixed + cVar * curAction;

                tempNetSize = curNetworkSize + curAction;
                netValueIndex = tempNetSize - nMin;

                if (tempNetSize < nMin){
                    rel = 0;
                    netValueIndex = 0;
                }
                else{
                    rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);
                }

                tempValue = rel + myValues.GetStateValue(curMission, tBudget, netValueIndex);

                if (tempValue >= bestValue && rel > relRequirement){
                    bestValue = tempValue;
                    bestAction = curAction;
                    bestCost = cost;
                    bestRel = rel;
                }
                curAction = curAction - deployInterval;
            }

//            vector<int> regionDeployment(numRegions, 0);
            vector<int> regionDeployment;

            SensorDeploymentAction(regionState, subAreaMC, subWeightsMC, bestAction, regionDeployment);

            // redeploy sensors, and generate a new fail time for those added
            myNetworkBuckets_ADPEval.GenerateNewFailTimeADP_ADPEval(nodeTime, (double) nMissions*delta, curTime, regionDeployment);

            myNetworkBuckets_ADPEval.GenerateNewRGG_ADPMaintAction();

            iterationCost = iterationCost + bestCost;
            missionSuccess = missionSuccess + bestRel;
            reliabilityRecord.push_back(bestRel);
            missionDeployAction.push_back(bestAction);
            budgetRemaining = budgetRemaining - bestCost;

            curTime = curTime + delta;
            curMission = curMission + 1;

            vector<int> ageVector(maxAge, 0);
            vector<int> regionVector(numRegions, 0);

            myNetworkBuckets_ADPEval.UpdateState(curTime, delta, regionVector, ageVector, expectedNum);
            copy(ageVector.begin(), ageVector.end(), ageState.begin());
            copy(regionVector.begin(), regionVector.end(), regionState.begin());

        }

        myNetworkBuckets_ADPEval.UpdateRGG_ADPEval(curTime);
        myNetworkBuckets_ADPEval.UpdateFailTimes(nodeTime);
        tarCovered = myNetworkBuckets_ADPEval.BreadthFirstSearchReturn_ADPEval(numNodes, numTargets, timePlace);
        if (tarCovered >= covRequirement){
            netStatus[curMission-1] = netStatus[curMission-1] + 1;
            totalSuccess = totalSuccess + 1;
        }

        costRecord.push_back(iterationCost);
        cumulativeMissionSuccess.push_back(missionSuccess);
        successRecord.push_back(totalSuccess);
    }

    int tSuccess = 0;
    for (int m = 0; m < successRecord.size() ; ++m) {
        tSuccess = tSuccess + successRecord.at(m);
    }

    auto totalTime = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsedNetwork = totalTime - startTime;

    chrono::duration<double> approxTime = finishValueFunctionApprox - startTime;
    chrono::duration<double> evalTime = totalTime - startEval;

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myObjMC;
    myObjMC.open ("CostFunctionMC.txt");

    for (int i = 0; i < costRecord.size() ; ++i) {
        myObjMC << "Iteration " << i << " -- ObjectiveValue : " << costRecord.at(i) << endl;
    }

    myObjMC.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // print out reliability over time
    ofstream myReliabilityFileMC;
    myReliabilityFileMC.open("ReliabilityRecordMC.txt");

    for (int j = 0; j < reliabilityRecord.size(); ++j) {
        myReliabilityFileMC << "Record " << j << " -- Reliability : " << reliabilityRecord.at(j) << endl;
    }

    myReliabilityFileMC.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream mySuccessMC;
    mySuccessMC.open ("ExpectedNumSuccessfulMissionsMC.txt");

    for (int i = 0; i < cumulativeMissionSuccess.size() ; ++i) {
        mySuccessMC << "Iteration " << i << " -- ObjectiveValue : " << cumulativeMissionSuccess.at(i) << endl;
    }

    mySuccessMC.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    cout << "Observed Average Number Successful Mission : " << (double) tSuccess / (double) numReps << endl;

    ofstream myObservedSuccessMC;
    myObservedSuccessMC.open("ObservedNumSuccessfulMissionsMC.txt");
    for (int l = 0; l <netStatus.size() ; ++l) {
        myObservedSuccessMC << "Mission : " << l << " - Status : " << netStatus.at(l) << " of " << numReps << endl;
        cout << "Mission : " << l << " - Status : " << netStatus.at(l) << " of " << numReps << endl;
    }

    for (int l = 0; l < successRecord.size(); ++l) {
        myObservedSuccessMC << "record " << l << " status : " << successRecord.at(l) << endl;
    }

    myObservedSuccessMC.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myActionsMC;
    myActionsMC.open ("NetworkSizeAndActionMC.txt");

    for (int i = 0; i < missionDeployAction.size() ; ++i) {
        myActionsMC << "Iteration " << i << " -- NetworkSize : " << netSizeBeginning.at(i) << " -- Action : " << missionDeployAction.at(i) << endl;
    }

    myActionsMC.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    ofstream myNotes;
    myNotes.open("Notes.txt");

    myNotes << "Function : ADP AVI Reliability Model with Requirement " << endl;
    myNotes << "Total Time Required : " << elapsedNetwork.count() << endl;
    myNotes << "  Value Function Approximation Time : " << approxTime.count() << endl;
    myNotes << "  Policy Evaluation Time : " << evalTime.count() << endl;
    myNotes << "Number of Replications : " << numReps << endl;
    myNotes << "Budget Available : " << Budget << endl;
    myNotes << "Number of Missions : " << nMissions << endl;
    myNotes << "Time Between Maintenance Inspections : " << delta << endl;
    myNotes << "Minimum number of sensors to deploy : " << minDeploy << endl;
    myNotes << "Sensor Deployment Interval : " << deployInterval << endl;
    myNotes << "Minimum Network Size : " << nMin << endl;
    myNotes << "Maximum Network Size : " << nMax << endl;
    myNotes << "Minimum Reliability Requirement : " << relRequirement << endl;
    myNotes << "Observed Average Number Successful Mission : " << (double) tSuccess / (double) numReps << endl;

    myNotes.close();

    cout << "Total Time Required : " << elapsedNetwork.count() << endl;
    cout << "  Value Function Approximation Time : " << approxTime.count() << endl;
    cout << "  Policy Evaluation Time : " << evalTime.count() << endl;

}


// CBM ADP model using AVI look up tables. The objective is to maximize the number of successful missions subject to a budget constraint.
// Uses a neighborhood of the current state to update the value functions for states `close' to the one that is actually visited.
// Once the value functions are updated a monte carlo simulation is performed to evaluate the performance of the resulting maintenance policy
// This function also requires a minimum reliability requirement, where each mission must achieve a reliability above this input
void DebugAge(int nSens, int numReps, int nMissions, double delta, double Budget,
                                                                 double cFixed, double cVar, double covRequirement, double relRequirement) {

    uniform_real_distribution<> myUnif(0,1);

    random_device myDev;
    random_device myRandomOrder;

    const int numRegions = 16;
    const int maxAge = 24;
    const int nMin = 320;
    const int nMax = 950;
    const int deployInterval = 10;
    const int minDeploy = 30;
    const double explorationProb = 0.00;
    const double learningRate = 0.7;

    double curShape = 1.5;
    double curScale = 10.0;
    double expectedNum;
    int netValueIndex;
    double budgetRemaining;

    vector<double> nodeTime;
    vector<double> costRecord;
    vector<double> reliabilityRecord;
    vector<double> cumulativeMissionSuccess;
    vector<int> netSizeBeginning;
    vector<int> missionDeployAction;

    // number of nodes includes sink node, number of sensors, and number of targets
    int numTargets = 441;
    int numNodes = nMax + numTargets + 1;

    SignatureCollection mySignatures(nMax - nMin + 1, nMin, nMax, numRegions);
    mySignatures.ReadSignatures("C:\\Users\\nboardma\\Documents\\Personal\\CPPFiles\\ADPEntireSignatureCollection_d3.txt");
    mySignatures.SetFailureDist(curShape, curScale);

    NodeLinkedListMC myNetworkBuckets(nMax + numTargets + 1, (numNodes - numTargets), numTargets);

    myNetworkBuckets.SetRanges(0.0750000, 0.0750000);
    myNetworkBuckets.UniformTargetLoc();
    myNetworkBuckets.SetBinLimits(4);

    vector<double> subArea;
    vector<double> subWeights;

    myNetworkBuckets.SubregionArea(subArea);
    myNetworkBuckets.SubregionDistanceWeight(subWeights);

    auto startTime = chrono::high_resolution_clock::now();

    for (int k = 1; k <= numReps ; ++k) {

        cout << "Start of Iteration " << k << endl;

        vector<double> gradientVect(numRegions,0);
        vector<int> numInRegion(numRegions, 20);
        double iterationCost = cFixed + (cVar * nSens);
        double missionSuccess = 0;

        budgetRemaining = Budget - iterationCost;

        // initial deployment of sensors and
        InitialSensorDeployment(gradientVect, numInRegion, subArea, subWeights, nSens);

        netSizeBeginning.push_back(0);
        missionDeployAction.push_back(nSens);

        // Generate initial fail times and set ages
        myNetworkBuckets.GenerateNewFailTime_ADPEval(nodeTime, nSens);
        myNetworkBuckets.UpdateFailTimesADP(nodeTime);
        myNetworkBuckets.ClearSensorAge();

        // Estimate reliability of initial state
        vector<int> ageState(maxAge, 0);
        ageState[0] = nSens;

        cout << "Start of Mission 0 : " ;
        for (int i = 0; i < ageState.size() ; ++i) {
            cout << ageState.at(i) << " " ;
        }
        cout << endl;


        double initialRel = mySignatures.ReliabilityEstimate(delta, nSens, ageState);
        reliabilityRecord.push_back(initialRel);
        missionSuccess = missionSuccess + initialRel;

        double postDecisionBudget = budgetRemaining;
        int postDecisionSize = (int) accumulate(ageState.begin(), ageState.end(),0) - nMin;

        double curTime = delta;
        int curMission = 1;

        // update state and determine post decision state variable
        ageState[0] = 0;
        myNetworkBuckets.UpdateState(curTime, delta, ageState, expectedNum);

        cout << "End of Mission 0 : " ;
        for (int i = 0; i < ageState.size() ; ++i) {
            cout << ageState.at(i) << " " ;
        }
        cout << endl;


        while (curMission < nMissions) {

            int curNetworkSize = accumulate(ageState.begin(), ageState.end(),0);
            int curAction = nMax - curNetworkSize;

            cout << "PreStart of Mission : " << curMission << " " ;
            for (int i = 0; i < ageState.size() ; ++i) {
                cout << ageState.at(i) << " " ;
            }
            cout << endl;

            netSizeBeginning.push_back(curNetworkSize);

            int maxAction = max(0, min(curAction, (int) floor((budgetRemaining - cFixed) / cVar)));

            int bestAction = 0;
            double rel;


            bestAction = 200;
            ageState[0] = bestAction;

            cout << "PostStart of Mission : " << curMission << " " ;
            for (int i = 0; i < ageState.size() ; ++i) {
                cout << ageState.at(i) << " " ;
            }
            cout << endl;



            int tempNetSize = curNetworkSize + curAction;
            rel = mySignatures.ReliabilityEstimate(delta, tempNetSize, ageState);

            // redeploy sensors, and generate a new fail time for those added
            myNetworkBuckets.GenerateNewFailTimeADP_Update(nodeTime, (double) nMissions*delta, curTime, bestAction);

            missionSuccess = missionSuccess + rel;
            reliabilityRecord.push_back(rel);
            missionDeployAction.push_back(bestAction);

            curTime = curTime + delta;
            curMission = curMission + 1;

            vector<int> ageVector(maxAge, 0);
            myNetworkBuckets.UpdateState(curTime, delta, ageVector, expectedNum);
            copy(ageVector.begin(), ageVector.end(), ageState.begin());
        }

        cumulativeMissionSuccess.push_back(missionSuccess);

    }

    auto finishValueFunctionApprox = chrono::high_resolution_clock::now();
/*
//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print out value functions
    cout << " Print Value Functions " << endl;
    myValues.PrintValueFunctions();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myObj;
    myObj.open ("CostFunctionImprovement.txt");

    for (int i = 0; i < costRecord.size() ; ++i) {
        myObj << "Iteration " << i << " -- ObjectiveValue : " << costRecord.at(i) << endl;
    }

    myObj.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // print out reliability over time
    ofstream myReliabilityFile;
    myReliabilityFile.open("ReliabilityRecord.txt");

    for (int j = 0; j < reliabilityRecord.size(); ++j) {
        myReliabilityFile << "Record " << j << " -- Reliability : " << reliabilityRecord.at(j) << endl;
    }

    myReliabilityFile.close();

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream mySuccess;
    mySuccess.open ("ExpectedNumSuccessfulMissions.txt");

    for (int i = 0; i < cumulativeMissionSuccess.size() ; ++i) {
        mySuccess << "Iteration " << i << " -- ObjectiveValue : " << cumulativeMissionSuccess.at(i) << endl;
    }

    mySuccess.close();

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Print Objective Function Value over iterations
    ofstream myActions;
    myActions.open ("NetworkSizeAndAction.txt");

    for (int i = 0; i < missionDeployAction.size() ; ++i) {
        myActions << "Iteration " << i << " -- NetworkSize : " << netSizeBeginning.at(i) << " -- Action : " << missionDeployAction.at(i) << endl;
    }

    myActions.close();


    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    ofstream myNotes;
    myNotes.open("Notes.txt");

    myNotes << "Function : ADP AVI Reliability Model with Requirement " << endl;

    myNotes << "Number of Replications : " << numReps << endl;
    myNotes << "Budget Available : " << Budget << endl;
    myNotes << "Number of Missions : " << nMissions << endl;
    myNotes << "Time Between Maintenance Inspections : " << delta << endl;
    myNotes << "Minimum number of sensors to deploy : " << minDeploy << endl;
    myNotes << "Sensor Deployment Interval : " << deployInterval << endl;
    myNotes << "Minimum Network Size : " << nMin << endl;
    myNotes << "Maximum Network Size : " << nMax << endl;
    myNotes << "Minimum Reliability Requirement : " << relRequirement << endl;

    myNotes.close();
*/

}