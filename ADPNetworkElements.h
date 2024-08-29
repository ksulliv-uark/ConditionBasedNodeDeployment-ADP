//
// Created by nboardma on 7/1/2020.
//

#ifndef CPP_ADP_ADPNETWORKELEMENTS_H
#define CPP_ADP_ADPNETWORKELEMENTS_H
#include <vector>
#include <random>
using namespace std;

class Node {
// myID: refers to the node number, 1, 2, ...
// myValue: a value associated with the node. Ex: fail time, value, etc,

//TODO: maybe add in an node identifier (sink, sensor, target). If target add a weight value

public:
    Node();
    Node(int inputID);

    void SetID(int temp);
    int GetNodeID();

    int GetValue();
    void SetValue(int data);

    void UpdateCapacityLabel(int newCapacity);
    int GetMaxCapacityPath();

    Node *GetPrev();
    Node *GetNext();
    Node *GetFirstElement();

    void SetPrev(Node *newPrev);
    void SetNext(Node *newNext);
    void SetFirstItem(Node *firstElement);

    void UpdateReached();
    bool IsReached();
    void MarkPermanent();
    bool IsPermanent();

    void ResetNodeReached();
    void ResetNodePermanent();

    void AddReachableNode(int nodeR);
    int GetReachableNode(int i);
    int GetDegree();
    void ResetReachableNodes();

    int GetNumberInCell();
    void AddNodeToBucket(int nodeNew);
    int GetNodeInBucket(int i);
    void ResetNodesInBucket();

    void SetNewLoc(double xLoc, double yLoc);
    double GetXLoc();
    double GetYLoc();

    void SetSubregion(int region);
    int GetSubregion();

    bool IsCovered();
    void SetCovered();
    void ResetCovered();

    bool IsFailed();
    void SetFailed();
    void ResetFailed();

    void SetXLimits(double low, double high);
    void SetYLimits(double low, double high);

    double GetXLow();
    double GetXHigh();
    double GetYLow();
    double GetYHigh();

protected:
    vector<int> forwardStar;
    int nodeDegree = 0;
    bool isReached = false;
    bool isCovered = false;

    bool isFailed = false;

    int myValue;
    int myID;
    int myMaxCapacityPath = 0;

    double myXLoc;
    double myYLoc;
    int mySubregionLoc;

    vector<int> nodesInBucket;
    int myNodesInBucket = 0;
    bool isPermanent = false;

    double myXLower;
    double myXHigher;
    double myYLower;
    double myYHigher;

private:
    Node *prev;
    Node *next;
    Node *firstInBucket;
};
/////////////////////////////////////////////////////////////////////////////////////////////////////////

class Edge {
// TODO: add an identifier for a directed edge or an undirected edge
public:
    Edge(int start, int end): myStartNode(start), myEndNode(end), myWeight(0){};
    Edge(int start, int end, double weight): myStartNode(start), myEndNode(end), myWeight(weight){};

public:
    int myStartNode;
    int myEndNode;
    double myWeight;
};
/////////////////////////////////////////////////////////////////////////////////////////////////////////

class NodeLinkedList {
    // numItems - number of nodes in the network
    // vector<int> time - a vector indicating order of failure

public:
    NodeLinkedList();
    NodeLinkedList(int numItems);
    NodeLinkedList(int numItems, vector<int> order);
    NodeLinkedList(int numItems, int numSens, int numTarg);
    NodeLinkedList(int numItems, int numSens, int numTarg, vector<int> order);

    void InitializeBuckets(int numItems);
    void InitializeBuckets(int numItems, vector<int> failOrder);

    void AddToList(int listNum, Node *newElement);
    Node *RemoveFirst(int listNum);
    Node *RemoveTargetElement(int listNum, Node *removeElement);
    bool IsListEmpty(int listNum);

    void BuildForwardStar(vector<Edge*> edgeList);

    void DialsAlgorithm(vector<Edge*> edgeList, int numNodes);
    void DialsAlgorithm(vector<Edge*> edgeList, int numNodes, int numTar, double &timeDuration);
    void DijkstrasAlgorithm(vector<Edge*> edgeList, int numNodes);
    void DijkstrasAlgorithm(vector<Edge*> edgeList, int numNodes, int numTar, double &timeDuration);
    void DialsAlgorithmUpdateRule(vector<Edge *> edgeList, int numNodes, int numTar, double &timeDuration);

    void UpdateDSpecFeq(int numFailed);
    void CalculateDSpec(int numReps);
    void OutputDSpec();

    void RecordDSpec(ofstream &outputFile);

    void SignatureRelation(int newSize, double t, double maxDelta, double costFixed, double costVar, double myShape, double myScale);
    void SignatureRelationTBM_Output(int minSize, double t, double maxDelta, double costFixed, double costVar,
                                     double myShape, double myScale);

    void SignatureRelationTBM_Output(int minSize, double t, double maxDelta, double costFixed, double costVar,
                                     double myShape, double myScale, ofstream &outputFile);

    void SignatureRelationVariance(int minSize, int numReps, double t, double maxDelta, double costFixed, double costVar, double myShape, double myScale);

    void CalculateReliabilityWeibull(double t, double myShape, double myScale);
    void CalculateReliabilityVarianceWeibull(double t, int numReps, double myShape, double myScale);

    void CalculateStableMaintenanceReliabilityWeibull(double delta, double mShape, double mScale);
    void CalculateStableMaintenanceReliabilityWeibull(double delta, double mShape, double mScale, ofstream &outputFile);
    void CalculateStableMaintenanceVarianceWeibull(double delta, int numReps , double mShape, double mScale);
    void CalculateStableMaintenanceReliabilityWeibullWithWeight(double delta, double mShape, double mScale, vector<double> regWeight, ofstream &outputFile);
    double StableMaintenanceRelReturn(double delta, double mShape, double mScale);

    void CalculateMaintenancePolicyCostWeibull(double delta, double costFixed, double costVar, double mShape, double mScale);
    void CalculateMaintenancePolicyCostWeibull(int numSens, double delta, double costFixed, double costVar, double mShape, double mScale);

    void SignatureRelationCBM_Output(int minSize, double costFixed, double costVar, double myShape, double myScale);

    void GenerateNewFailOrder(vector<int> &nodeTime);

    void GenerateNewRGG_naive();
    void GenerateNewRGG_Bucket();
    void GenerateNewSpecificRGG_Bucket(vector<int> numPerRegion);
    void CompareAdjacentCells(int cell1, int cell2);

    void UniformTargetLoc();
    void SetBinLimits(int numRegionsPerRow);
    void SubregionArea(vector<double> &areaVect);
    void SubregionDistanceWeight(vector<double> &subWeights);
    void CustomBins();

    void OutputCriticalTimes();
    void addFailTime(int failTime);
    void addFailOrder(int nodeNum);
    int GetFailTime(int num);
    int GetFailOrder(int num);

    void OutputTargetCriticalTimes(int numTargets);
    void OutputTargetCriticalTimes(int numTargets, ofstream &outputFile);
    void OutputTargetFailOrder(int numTargets);
    void addTargetFailTime(int failTime);
    void addTargetFailOrder(int nodeNum);
    int GetSensorFailOrder(int senNum);
    int GetTargetFailTime(int failNum);
    int GetTargetFailOrder(int targNum);

    int GetFailureSubregion(int sensNum);

    void OutputSensorFailOrder(int numSensors);
    void addSensorFailTime(int failTime);
    void addSensorFailOrder(int nodeNum);

    void SetRanges(double comRange, double sensingRange);

    int CoverageNumber(double coverage, int numT);

    void UpdateBucketTimes(vector<int> newOrder);
    void ClearOrderTimes();

protected:
    Node *firstItem;
    Node *lastItem;
    Node *myBuckets;
    int myNumElements;  // total number of nodes (includes sink, sensors, targets)
    int myNumSensors;   // number of sensor nodes, also includes the sink node
    int myNumTargets;   // number of targets

    random_device myDevX;       // used to generate x locations for a new RGG
    random_device myDevY;       // used to generate y locations for a new RGG
    random_device myDevOrder;   // used to generate a new order of sensor failures

    double mySensorRange;
    double myCommunicationRange;

    int myNumSubregions;

    vector<int> orderedCriticalFailTime;
    vector<int> orderedFail;

    vector<int> orderedTargetCriticalFailTime;
    vector<int> orderedTargetFail;

    vector<int> orderedSensorCriticalFailTime;
    vector<int> orderedSensorFail;

    vector<int> DSpecFreq;
    vector<int> DSpecElements;

    vector<int> myDSpec;
    vector<double> myDSpecProb;

};

/////////////////////////////////////////////////////////////////////////////////////////////////////////


class SimulationNodeMC : public Node {

public:
    SimulationNodeMC();

    int GetNodeID();
    double GetFailTime();
    void SetFailTime(double t);

    void UpdateFailCapacity (double newTime);
    double GetFailCapacity();

    void SetNewLoc(double xLoc, double yLoc);
    void ResetNode();

    double GetXLoc();
    double GetYLoc();

    bool IsCovered();
    void SetCovered();
    void ResetCovered();

    bool IsReplaced();
    void SetReplaced();
    void ResetReplaced();

    void SetSubregion(int region);

    bool IsFailed();
    void SetFailed();
    void ResetFailed();

    bool IsActive();
    void SetActive();
    void SetInactive();

    void AddNodeToBucket(int nodeNew);
    void ResetReachableNodes();
    void ResetNodesInBucket();

    void UpdateForwardStar(int removeNum);

    int GetNodeInBucket(int i);
    int GetNumberInCell();

    void SetHeapPosition(int pos);
    int GetHeapPosition();

    void SetAge(double age);
    double GetAge();
    void IncreaseAge();

private:

    double myFailTime;
    double myCapacityFailTime = 0;

    bool isReplaced = false;
    bool isActive = false;
    double myAge;

};

class NodeLinkedListMC : public NodeLinkedList {

public:
    NodeLinkedListMC(int numItems, int numSens, int numTarg);
    NodeLinkedListMC(int numItems, int numSens, int numTarg, vector<double> failTimes);

    void InitializeBuckets(int numItems);
    void InitializeBuckets(int numItems, vector<double> failTimes);
    void DijkstrasAlgorithm(vector<Edge*> edgeList, int numNodes, int numTar);
    void DijkstrasAlgorithm(vector<Edge*> edgeList, int numNodes, int numTar, double &timeDuration);

    void addFailTime(double failTime);
    void addTargetFailTime(double failTime);
    void addSensorFailTime(double failTime);
    void addNetworkFailTime(double failTime);

    void UpdateBucketTimes(vector<int> newOrder);

    double GetTargetFailTime(int failNum);
    void UniformTargetLoc();
    void GenerateNewRGG_Bucket();
    void GenerateNewSpecificRGG_Bucket(int numPerRegion);
    void GenerateNewSpecificRGG_Bucket(vector<int> numPerRegion);
    void GenerateNewSpecificRGG_Bucket_ADPEval(vector<int> numPerRegion);
    void CompareAdjacentCells(int cell1, int cell2);

    void GenerateNewFailTime(vector<double> &nodeTime);
    void GenerateNewFailTime_ADPEval(vector<double> &nodeTime, int numDeployed);
    void GenerateNewFailTime_ADPEval(vector<double> &nodeTime, int numDeployed, double maxTime);
    void UpdateFailTimes(vector<double> newTimes);
//    void UpdateFailTimes(vector<double> newTimes, int epochNum, double delta);

    void CalculateReliability(double t, int numReps);

    double TargetCoveragePercentage();

    void BreadthFirstSearch(int numNodes, int numTar, double &timeDuration);
    double BreadthFirstSearchReturn(int numNodes, int numTar);
    double BreadthFirstSearchReturn(int numNodes, int numTar, double &timeDuration);
    double BreadthFirstSearchReturn_ADPEval(int numNodes, int numTar, double &timeDuration);

    void GenerateNewFailTimeMaintenance(vector<double> &nodeTime, double t, double curTime);
    void GenerateNewRGG_BucketMaint();
    void GenerateNewRGG_BucketMaint(double costFixed, double costVar, double &policyCost);
    void GenerateNewRGG_BucketMaint(double costFixed, double costVar, double &policyCost, int &numDisconnected, int &numDeployed);
    void GenerateNewRGG_BucketMaintNodeReplace(double costFixed, double costVar, double &policyCost);
    void UpdateRGG(double time);

    void UpdateRGG_ADPEval(double time);
    void GenerateNewRGG_ADPMaintAction();

    int GetNumSensorsFailed();

    void TargetedDeployment(double costFixed, double costVar, double &policyCost);

    void GenerateNewRGG_BucketCBMMaint(double costFixed, double costVar, double &policyCost, int &numDisconnected, int &numDeployed);
    void GenerateNewFailTimeMyopicCBM(vector<double> &nodeTime, double t, double curTime, int numReplaced);
    void GenerateNewFailTimeMyopicCBM(vector<double> &nodeTime, double t, double curTime, vector<int> newDeploymentCount);
    void UpdateRGGMyopicCBM(double time, vector<int> &numInRegion);
    void UpdateRGGMyopicCBM(double time, vector<int> &numInRegion, vector<int> &ageCount);
    void UpdateRGGMyopicCBM(double time, double delta, vector<int> &numInRegion, vector<int> &ageCount, vector<double> &expectedNum);
    void SetBinLimits(int numRegionsPerRow);
    void CustomBins();

    void SetFailedNodes(vector<int> failNodes, int numFailed);

    void ResetAllNodes();
    void SubregionArea(vector<double> &areaVect);
    void SubregionDistanceWeight(vector<double> &subWeights);
    void CustomWeights(vector<double> &subWeights);

    void GenerateNewFailTimeADP(vector<double> &nodeTime, double t, double curTime, int numReplaced);
    void GenerateNewFailTimeADP_Update(vector<double> &nodeTime, double t, double curTime, int numReplaced);
    void UpdateState(double time, double delta, vector<int> &ageCount, double &expectedNum);
    void UpdateState(double time, double delta, vector<int> &numInRegion, vector<int> &ageCount, double &expectedNum);
    void UpdateFailTimesADP(vector<double> newTimes);

    void GenerateNewFailTimeADP_ADPEval(vector<double> &nodeTime, double t, double curTime, vector<int> newDeploymentCount);

    void ClearSensorAge();

    void GenerateNewRGG_Bucket_ADPEval(int nSize);
    void GenerateNewRGG_ADPMaintAction_SingleRegion();


protected:
    SimulationNodeMC *myBuckets;

    vector<double> orderedCriticalFailTime;
    vector<double> orderedTargetCriticalFailTime;
    vector<double> orderedSensorCriticalFailTime;

    vector<double> networkFailTimes;

    random_device myDevTime;
    random_device myBinGen;

};

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
class NetworkSignature {

public:

    NetworkSignature();
    NetworkSignature(int numSensors);

    void UpdateDSpec(int numFailed, double failProb);
    void SetNumSensors(int numSensors);
    int GetNumSensors();

    void GetSignature(vector<int> &failCount, vector<double> &failProb);


protected:
    vector<int> DSpecElements;
    vector<double> DSpecProb;

    int myNumSensors;

};

class SignatureCollection {

public:

    NetworkSignature *mySignatures;

    SignatureCollection(int numSignatures, int minSize, int maxSize, int numReg);
    void ReadSignatures(char* filename);

    void ReturnSignature_Approximation(int netSize, vector<int> &failCount, vector<double> &failProb);
    void ReturnSignature(int netSize, vector<int> &failCount, vector<double> &failProb);

    double ReliabilityEstimate_Approximation(double delta, int nSize, vector<int> ageVector);
    double ReliabilityEstimate(double delta, int nSize, vector<int> ageVector);
    void SetFailureDist(double shape, double scale);

protected:
    int minNetSize;
    int maxNetSize;
    int numRegions;

    int myNumSignatures;

    double myShape;
    double myScale;

};

class SingleRegionSignature{

public:

    SingleRegionSignature(char* filename);
    void ReadSignature(char* filename);
    double ReliabilityEstimate(double delta, int nSize, vector<int> ageVect);
    void SignautreUpdate(int nSize, vector<int> &failN, vector<double> &failP);

    void SetFailureDist(double shape, double scale);

protected:

    int maxSize;
    vector<int> failNum;
    vector<double> failProb;

    double myShape;
    double myScale;
};

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

class CostValueFunction {
public:

    CostValueFunction();

    double GetStateValue(double avgAge, int netSize);
    double GetTempStateValue(double avgAge, int netSize);
    double GetStepSize(double avgAge, int netSize);

    void UpdateValueFunction(double learningRate);
    void UpdateStateValue(double avgAge, int netSize, double newValue);
    void UpdateTempStateValue(double avgAge, int netSize, double newValue);
    void UpdateStepSize(double avgAge, int netSize, double newStep);

    void PrintValueFunction(ofstream &outputFile);

protected:
    double myValues[91][567] = {0.0};
    double myTempValues[91][567] = {0.0};
    double myStepSize[91][567] = {1.0};
    double myNumVisits[91][567] = {0.0};

    vector<int> ageStatesUpdates;
    vector<int> netSizeUpdates;

};

class ValueFunctionCostCollection {

public:

    ValueFunctionCostCollection(int numMissions, double learningRate);

    double GetStateValue(int curMission, double avgAge, int netSize);
    double GetTempStateValue(int curMission, double avgAge, int netSize);
    double GetStepSize(int curMission, double avgAge, int netSize);

    void UpdateValueFunctions();
    void UpdateTempStateValue(int curMission, double avgAge, int netSize, double newValue);
    void UpdateStepSize(int curMission, double avgAge, int netSize, double newStep);
    void UpdateAdjacentStates(int curMission, double postDecisionAge, int postDecisionSize, double newValue);
    void ADP_MonotoneUpdate(int curMission, double postDecisionAge, int postDecisionSize, double newValue, double cVar);

    void PrintValueFunctions();

private:
    CostValueFunction *myValueFunctions;

    int myNumFunctions = 0;
    double myLearningRate;

};

///////////////////////////////////////////////////////////////////////////////////////////

class ReliabilityValueFunction {
public:

    ReliabilityValueFunction();

    void ConstructValueFunction(double Budget, int nMin, int nMax);

    double GetStateValue(double budget, int netSize);
    double GetTempStateValue(double budget, int netSize);
    double GetStepSize(double budget, int netSize);

    int GetNumStateVisists(double budget, int netSize);

    void UpdateValueFunctionConstant(double learningRate);
    void UpdateValueFunctionStC(double initialLearningRate);
    void UpdateValueFunctionsGHarmonic(double initialLearningRate);
    void UpdateStateValue(double budget, int netSize, double newValue);
    void UpdateTempStateValue(double budget, int netSize, double newValue);
    void UpdateStepSize(double budget, int netSize, double newStep);

    void SetInitialStateValue(int budgetLevel, double initialValue, double learningRate);
    void SetInitialStateValueStC(int budgetLevel, double initialValue, double initialLearningRate);
    void SetInitialStateValueGHarmonic(int budgetLevel, double initialValue, double initialLearningRate);

    void ReadValueFunction(int budgetLevel, int netSize, double initialValue);

    void PrintValueFunction(ofstream &outputFile);

protected:

    double **myValues;
    double **myTempValues;
    double **myStepSize;
    int **myNumVisits;

    int minNetSize = 384;
    int maxNetSize = 950;
    int myBudgetSize;

    vector<int> budgetStatesUpdates;
    vector<int> netSizeUpdates;

    // used for Search then converge step updates
    double a = 1.0;
    double b = 1000.0;

    double generalizedHarmonicParam = 20.0;

};


class ValueFunctionReliabilityCollection {

public:

    ValueFunctionReliabilityCollection(int numMissions, double learningRate);
    ValueFunctionReliabilityCollection(int numMissions, double learningRate, double Budget, int nMin, int nMax);

    void ConstructValueFunction(int numMissions, double Budget, int nMin, int nMax);

    double GetStateValue(int curMission, double budget, int netSize);
    double GetTempStateValue(int curMission, double budget, int netSize);
    double GetStepSize(int curMission, double budget, int netSize);

    int GetNumStateVisits(int curMission, double budget, int netSize);

    void UpdateValueFunctionsConstant();
    void UpdateValueFunctionsStC(double initialLearningRate);
    void UpdateValueFunctionsGHarmonic(double initialLearningRate);
    void UpdateTempStateValue(int curMission, double budget, int netSize, double newValue);
    void UpdateStepSize(int curMission, double budget, int netSize, double newStep);
    void ADP_MonotoneUpdate(int curMission, double postDecisionBudget, int postDecisionSize, double newValue);

    void PrintValueFunctions();
    void InitializeValueFunction(char* filename);
    void InitializeValueFunctionStC(char* filename);
    void InitializeValueFunctionGHarmonic(char* filename);

    void ReadValueFunctions();

    int BoltzmannExploration(int curMission, int startSize, vector<int> actions, vector<double> stateValues, vector<double> stateBudget, double bestValue);

    void TempOutput();

private:
    ReliabilityValueFunction *myValueFunctions;

    int myNumFunctions = 0;
    double myLearningRate;

    random_device myExplorationGen;

    int minNetSize = 384;
    int maxNetSize = 950;
    double myBudget;

};

#endif //CPP_ADP_ADPNETWORKELEMENTS_H