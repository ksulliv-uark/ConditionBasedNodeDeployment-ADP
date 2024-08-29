#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <random>
#include <sstream>
#include <algorithm>
#include "ADPNetworkElements.h"
#include "ADPRunFiles.h"
#include "ADPReliability.h"

using namespace std;

// creates a random network to initialize, no input file
void DialsRandomNetwork(int nSens, int minNetSize, int numReps, double time, double coverageLevel, double cFixed, double cVar){

    vector<Edge*> networkEdges;
    vector<int> nodeTime;

    // start time record
    auto startN = chrono::high_resolution_clock::now();

    // number of nodes includes sink node, number of sensors, and number of targets
    int numTargets = 441;
    int numNodes = nSens + numTargets + 1;
    double timeDials;
    double totalTimeDials = 0;
    double networkGenerationTime = 0;
    double simTimeTotal = 0;

    NodeLinkedList myNetworkBuckets(numNodes, (numNodes - numTargets), numTargets);
    myNetworkBuckets.SetRanges(0.0750000, 0.0750000);
    myNetworkBuckets.UniformTargetLoc();

    auto startSimTime = chrono::high_resolution_clock::now();

    myNetworkBuckets.GenerateNewFailOrder(nodeTime);

    auto endSimeTime = chrono::high_resolution_clock::now();
    chrono::duration<double> simTime = endSimeTime - startSimTime;

    simTimeTotal = simTimeTotal + simTime.count();

    auto startNetwork = chrono::high_resolution_clock::now();

    myNetworkBuckets.GenerateNewRGG_Bucket();

    auto finishNetwork = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsedNetwork = finishNetwork - startNetwork;
    networkGenerationTime = networkGenerationTime + elapsedNetwork.count();

    myNetworkBuckets.UpdateBucketTimes(nodeTime);

    myNetworkBuckets.DialsAlgorithm(networkEdges, numNodes, numTargets, timeDials);
    totalTimeDials = totalTimeDials + timeDials;

    int critNum = myNetworkBuckets.CoverageNumber(coverageLevel, numTargets);
    myNetworkBuckets.UpdateDSpecFeq(myNetworkBuckets.GetTargetFailTime(critNum));

    // Start of updating node failure times
    for (int k = 1; k < numReps ; ++k) {

        startSimTime = chrono::high_resolution_clock::now();

        myNetworkBuckets.GenerateNewFailOrder(nodeTime);

        endSimeTime = chrono::high_resolution_clock::now();
        simTime = endSimeTime - startSimTime;
        simTimeTotal = simTimeTotal + simTime.count();

        auto startNetworkLoop = chrono::high_resolution_clock::now();

        myNetworkBuckets.GenerateNewRGG_Bucket();

        finishNetwork = chrono::high_resolution_clock::now();
        elapsedNetwork = finishNetwork - startNetworkLoop;
        networkGenerationTime = networkGenerationTime + elapsedNetwork.count();

        myNetworkBuckets.UpdateBucketTimes(nodeTime);

        myNetworkBuckets.DialsAlgorithm(networkEdges, numNodes, numTargets, timeDials);
        totalTimeDials = totalTimeDials + timeDials;
        myNetworkBuckets.UpdateDSpecFeq(myNetworkBuckets.GetTargetFailTime(critNum));
    }

    auto finishLoop = chrono::high_resolution_clock::now();

    myNetworkBuckets.CalculateDSpec(numReps);
    myNetworkBuckets.CalculateReliabilityWeibull(time, 1.5, 10.0);


    double i = 1.0;
    double curShape = 1.5;
    double curScale = 10.0;
    double maxDelta = 10.0;

    ///////////////////////////////////////////////

    auto startPolicy = chrono::high_resolution_clock::now();
    cout << " Stable Maintenance Reliability for changing delta " << endl;
    while (i < maxDelta){
        myNetworkBuckets.CalculateStableMaintenanceReliabilityWeibull(i, curShape, curScale);
        i = i + 0.1;
    }

    i = 1.0;
    cout << "Stable maintenance cost " << endl;
    while (i < maxDelta){
        myNetworkBuckets.CalculateMaintenancePolicyCostWeibull(i, cFixed, cVar, curShape, curScale);
        i = i + 0.1;
    }

    myNetworkBuckets.OutputDSpec();

    ////////////////////////////////////////////////

    auto endPolicy = chrono::high_resolution_clock::now();
    chrono::duration<double> timeEvaluatingMaintPolicies = endPolicy - startPolicy;

    auto endPolicyVar = chrono::high_resolution_clock::now();
    chrono::duration<double> timeEvaluatingMaintVar = endPolicyVar - endPolicy;

    // end time record
    auto finishN = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed = finishN - startN;

    chrono::duration<double> dTime = finishLoop - startNetwork;

    cout << " Time Spent Generating Network : " << networkGenerationTime << endl;
    cout << " Time Spent implementing Dials : " << totalTimeDials << endl;
    cout << " Time Generating Fail Time : " << simTimeTotal << endl;
    cout << " Total Time for destruction algorithm : " << dTime.count() << endl;
    cout << " Time evaluating TBM policies : " << timeEvaluatingMaintPolicies.count() << endl;
    cout << " Time estimating Variance : " << timeEvaluatingMaintVar.count() << endl;
    cout << numReps << " - " << " TotalTimeDials " << elapsed.count() << endl;

}


void VectorCopy(){

    vector<int> vector1;
    vector<int> vector2;

    vector1.push_back(5);
    vector1.push_back(3);
    vector1.push_back(7);
    vector1.push_back(9);

    vector2.push_back(15);
    vector2.push_back(17);
    vector2.push_back(6);
    vector2.push_back(1);

    cout << "Vector 1 : " ;
    for (int i = 0; i < vector1.size() ; ++i) {
        cout << vector1.at(i) << " " ;
    }

    cout << endl;

    cout << "Vector 2 : " ;
    for (int i = 0; i < vector2.size() ; ++i) {
        cout << vector2.at(i) << " " ;
    }

    cout << endl;
    cout << " ---------------- " << endl;

    copy(vector1.begin(), vector1.end(), vector2.begin());

    cout << "Vector 1 : " ;
    for (int i = 0; i < vector1.size() ; ++i) {
        cout << vector1.at(i) << " " ;
    }

    cout << endl;

    cout << "Vector 2 : " ;
    for (int i = 0; i < vector2.size() ; ++i) {
        cout << vector2.at(i) << " " ;
    }

    cout << endl;
    cout << " ---------------- " << endl;

    vector1[1] = 8;

    cout << "Vector 1 : " ;
    for (int i = 0; i < vector1.size() ; ++i) {
        cout << vector1.at(i) << " " ;
    }

    cout << endl;

    cout << "Vector 2 : " ;
    for (int i = 0; i < vector2.size() ; ++i) {
        cout << vector2.at(i) << " " ;
    }

    cout << endl;
    cout << " ---------------- " << endl;

    double x = 5.6;
    cout << "x = " << round(x * 10) << endl;
}

// Functions to update in Cost Model based on state size and aggregations
// Class : Value Function
//      myValues[91][567] = {0.0}          91 = maxAge of sensor       567 = nMax - nMin
//      myTempValues[91][567] = {0.0}      91 = maxAge of sensor       567 = nMax - nMin
//      myStepSize[91][567] = {1.0}        91 = maxAge of sensor       567 = nMax - nMin
//      myNumVisits[91][567] = {0.0}       91 = maxAge of sensor       567 = nMax - nMin

// In ADP_CBM_AVI_CostModel_CAVE
//      SignatureCollection mySignatures(37, nMin, nMax, numRegions)       37 = (nMax - nMin) / numRegions. This is the number of network signatures that have been pre-estimated

void TempValue(int x, int y){

    int **myValues = new int*[x];

    for (int i = 0; i < x; ++i) {
        myValues[i] = new int[y];
    }

    for (int j = 0; j < x; ++j) {
        for (int i = 0; i < y; ++i) {
            myValues[j][i] = 0;
        }
    }

    myValues[0][0] = 4;
    myValues[0][2] = 1;
    myValues[1][1] = 3;
    myValues[2][2] = 2;

    cout << myValues[0][0] << " - " << myValues[0][1] << " - " << myValues[0][2] << endl;
    cout << myValues[1][0] << " - " << myValues[1][1] << " - " << myValues[1][2] << endl;
    cout << myValues[2][0] << " - " << myValues[2][1] << " - " << myValues[2][2] << endl;

    cout << " --------------------------------- " << endl;
    cout << myValues[0][0] << " - " << myValues[0][1] << " - " << myValues[0][2] << endl;
    cout << myValues[1][0] << " - " << myValues[1][1] << " - " << myValues[1][2] << endl;
    cout << myValues[2][0] << " - " << myValues[2][1] << " - " << myValues[2][2] << endl;
}

void PrintTest(){

    char *filenameStart = "StateValues_t";

    char *filenameEnd = ".txt";

    int age = 5;

    stringstream ss;

    ss << filenameStart << age <<filenameEnd;

    cout << ss.str() << endl;

    ofstream myState0;
    myState0.open (ss.str());
    myState0 << " print in txt file 5!" << endl;
    myState0.close();




    ss.str("");
    ss << filenameStart << (age+1) << filenameEnd;
    cout << ss.str() << endl;

    myState0.open(ss.str());

    myState0 << " Print in txt file 6!" << endl;
    myState0.close();

}

void TestRandomGen(){

    double myNum;

    uniform_real_distribution<> myUnif(0,1);
    random_device myDev;

    auto startTime = chrono::high_resolution_clock::now();

    for (int j = 0; j < 50000 ; ++j) {
        for (int i = 0; i < 2700; ++i) {
            myNum = myUnif(myDev);
        }
    }


    auto finishTime = chrono::high_resolution_clock::now();

    chrono::duration<double> elapsedNetwork = finishTime - startTime;
    cout << " Total Time Required : " << elapsedNetwork.count() << endl;


}

int main(int argc, char* argv[]) {

    double coverage = 0.80;
    double relRequirement = 0.95;

    double cfixed = 100.0;
    double cVar = 1.0;

    int numReps = 300;
    double Budget = 8700;
    int numMissions = 50;
    double delta = 2.0;


//    DialsRandomNetwork(650, 500, numReps, 30.0, coverage, cfixed, cVar);

    int nSens = 650;

// THIS IS THE GOOD FUNCTION !!!!
//   ADP_CBM_AVI_ReliabilityModel_CAVE_EvaluateMC_MinRequirement(nSens, numReps, numMissions, delta, Budget, cfixed, cVar, coverage, relRequirement);

    ADP_CBM_AVI_ReliabilityModel_CAVE_EvaluateMC_MinRequirement_SingleRegion(nSens, numReps, numMissions, delta, Budget, cfixed, cVar, coverage, relRequirement);

//    ADP_CBM_AVI_ReliabilityModel_CAVE_MinRequirement(nSens, numReps, numMissions, delta, Budget, cfixed, cVar, coverage, relRequirement);

//    ADP_CBM_AVI_ReliabilityModel_CAVE_EvaluateMC_MinRequirement_delta2(nSens, numReps, numMissions, delta, Budget, cfixed, cVar, coverage, relRequirement);


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//    ADP_CBM_AVI_ReliabilityModel(nSens, numReps, numMissions, delta, Budget, cfixed,  cVar);

//    ADP_CBM_AVI_ReliabilityModel_CAVE(nSens, numReps, numMissions, delta, Budget, cfixed,  cVar);

//    ADP_CBM_AVI_ReliabilityModel_CAVE_EvaluateMC(nSens, numReps, numMissions, delta, Budget, cfixed, cVar, coverage);

//    ADP_CBM_InputValueFunction_EvaluateMC(nSens, numReps, numMissions, delta, Budget, cfixed, cVar, coverage);

//    ADP_CBM_AVI_ReliabilityModel_CAVE_Boltzmann(nSens, numReps, numMissions, delta, Budget, cfixed, cVar);

//    ADP_CBM_InputValueFunction_EvaluateMC_Requirement(nSens, numReps, numMissions, delta, Budget, cfixed, cVar, coverage, relRequirement);

//    ADP_CBM_AVI_ReliabilityModel_CAVE_EvaluateMC_MinRequirement_PenaltyCost(nSens, numReps,  numMissions, delta, Budget, cfixed, cVar, coverage, relRequirement);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//    ADP_CBM_AVI_ReliabilityModel_CAVE_EvaluateMC_MinRequirement_Skip(nSens, numReps, numMissions, delta, Budget, cfixed, cVar, coverage, relRequirement);

//  TestRandomGen();


//    UNIX_ReliabilityADP_CAVE_MCEval(argv[1], argv[2], nSens, numReps, numMissions, delta, Budget, cfixed, cVar, coverage);

//    UNIX_ReliabilityADP_CAVE_EvaluateMC_MinRequirement(argv[1], argv[2], nSens, numReps, numMissions, delta, Budget, cfixed, cVar, coverage, relRequirement);


    return 0;
}