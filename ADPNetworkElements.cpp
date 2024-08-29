//
// Created by nboardma on 7/1/2020.
//

#include "ADPNetworkElements.h"
#include "ADPReliability.h"
#include <math.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <chrono>
#include <random>
#include <numeric>
#include <sstream>

using namespace std;

// constructor for NodeLinkedList
NodeLinkedList::NodeLinkedList(){
    firstItem = nullptr;
    lastItem = nullptr;
}
NodeLinkedList::NodeLinkedList(int numItems){
    firstItem = nullptr;
    lastItem = nullptr;
    myNumElements = numItems;
    InitializeBuckets(numItems);
}
NodeLinkedList::NodeLinkedList(int numItems, vector<int> order){
    firstItem = nullptr;
    lastItem = nullptr;
    myNumElements = numItems;
    InitializeBuckets(numItems, order);
}

// use when no input file is to be provided, RGG and fail order can be generated directly in c++
NodeLinkedList::NodeLinkedList(int numItems, int numSens, int numTarg){
    firstItem = nullptr;
    lastItem = nullptr;
    myNumElements = numItems;   // total number of nodes
    myNumSensors = numSens;     // number of sensors (including sink)
    myNumTargets = numTarg;     // number of targets
    InitializeBuckets(numItems);
}

NodeLinkedList::NodeLinkedList(int numItems, int numSens, int numTarg, vector<int> order){
    firstItem = nullptr;
    lastItem = nullptr;
    myNumElements = numItems;   // total number of nodes
    myNumSensors = numSens;     // number of sensors (including sink)
    myNumTargets = numTarg;     // number of targets
    InitializeBuckets(numItems, order);
}

// initializes buckets. To begin, all nodes are in bucket zero and point to each other in sequential order
void NodeLinkedList::InitializeBuckets(int numItems){

    DSpecFreq.assign(myNumSensors, 0);

    myBuckets = new Node[numItems];

    // update all values for the sink node
    Node *ptr = myBuckets;
    ptr->SetID(0);
    ptr->UpdateCapacityLabel(numItems);
    ptr->UpdateReached();
    ptr->MarkPermanent();
    ptr->SetFirstItem((ptr + 1));
    ptr = ptr + 1;

    // sets the pointers for a node to be the node immediately prior and immediately after the node
    for (int i = 1; i < numItems ; ++i) {
        myBuckets[i].SetID(i);
        myBuckets[i].SetNext((ptr + 1));
        myBuckets[i].SetPrev((ptr - 1));
        myBuckets[i].SetFirstItem(nullptr);

        ptr = ptr + 1;

    }
    myBuckets[numItems-1].SetNext(nullptr);

}

// initializes buckets. To begin, all nodes are in bucket zero and point to each other in sequential order
// also sets the value for each node according to the input failure order in vector<int> failTime
void NodeLinkedList::InitializeBuckets(int numItems, vector<int> failOrder){

    DSpecFreq.assign(myNumSensors, 0);

    myBuckets = new Node[numItems];

    Node *ptr = myBuckets;
    // update all values for the sink node
    ptr->SetID(0);
    ptr->UpdateCapacityLabel(numItems);
    ptr->UpdateReached();
    ptr->MarkPermanent();
    ptr->SetValue(failOrder[0]);
    ptr->SetFirstItem((ptr + 1));

    ptr = ptr + 1;

    // sets the pointers for a node to be the node immediately prior and immediately after the node
    // values for sensor nodes
    for (int i = 1; i < myNumSensors ; ++i) {
        myBuckets[i].SetID(i);
        myBuckets[i].SetNext((ptr + 1));
        myBuckets[i].SetPrev((ptr - 1));

        myBuckets[failOrder[i]].SetValue(i);

        myBuckets[i].SetFirstItem(nullptr);

        ptr = ptr + 1;

    }

    // values for target nodes
    for (int j = myNumSensors ; j < numItems ; ++j) {

        myBuckets[j].SetID(j);
        myBuckets[j].SetNext((ptr + 1));
        myBuckets[j].SetPrev((ptr - 1));

        myBuckets[j].SetValue(myNumElements);

        myBuckets[j].SetFirstItem(nullptr);

        ptr = ptr + 1;

    }

    myBuckets[numItems-1].SetNext(nullptr);

}

// removes the first element from the list input by listNum
Node *NodeLinkedList::RemoveFirst(int listNum){

    Node *tempFirst = myBuckets[listNum].GetFirstElement();
    Node *tempNext = tempFirst->GetNext();

    // update the pointer to the first element on the list
    myBuckets[listNum].SetFirstItem(tempNext);

    // update the pointer of the new first element in the list
    if (tempNext != nullptr){
        tempNext->SetPrev(nullptr);
    }

    // update the Next pointer for element removed
    tempFirst->SetNext(nullptr);

    return tempFirst;

}

// removes a specific element from an input list
Node *NodeLinkedList::RemoveTargetElement(int listNum, Node *removeElement){

    // temporarily store Prev and Next pointer for the element removed
    Node *tempPrev = removeElement->GetPrev();
    Node *tempNext = removeElement->GetNext();

    // checks to see if the element removed is the first element in the list
    if (tempPrev == nullptr){
        myBuckets[listNum].SetFirstItem(tempNext);
    }

    // update Previous and Next pointers for nodes on either side of the removed node
    if (tempPrev != nullptr){
        tempPrev->SetNext(tempNext);
    }

    if (tempNext != nullptr){
        tempNext->SetPrev(tempPrev);
    }

    // remove pointers from the element removed
    removeElement->SetPrev(nullptr);
    removeElement->SetNext(nullptr);

    return removeElement;

}

// Adds a new element newElement to the front of an input list
void NodeLinkedList::AddToList(int listNum, Node *newElement){

    // store the original first item in the list
    Node *temp = myBuckets[listNum].GetFirstElement();

    // if the current first item in the list is null, place item in the front of the list
    if (temp == nullptr) {
        newElement->SetNext(nullptr);
    }
        // otherwise update the pointer for the original first item to have pointer prev equal to added element
        // update pointer next for the newly added element to be the original element
    else {
        newElement->SetNext(temp);
        temp->SetPrev(newElement);
    }

    // update pointer to the first element element in the list
    myBuckets[listNum].SetFirstItem(newElement);

    newElement->SetPrev(nullptr);

}

// returns true if list is empty, or false if there are elements in the list
bool NodeLinkedList::IsListEmpty(int listNum){
    Node *tempFirst = myBuckets[listNum].GetFirstElement();
    return (tempFirst == nullptr);
}

// Build the adjacency list for each node
void NodeLinkedList::BuildForwardStar(vector<Edge *> edgeList) {
    for (int i = 0; i < edgeList.size() ; ++i) {
        myBuckets[edgeList[i]->myStartNode].AddReachableNode(edgeList[i]->myEndNode);
    }
}

// performs Dials algorithm on a network. The Sink node is Node 0, sensor nodes 1,2,..., n
void NodeLinkedList::DialsAlgorithm(vector<Edge*> edgeList, int numNodes) {

    double timeDuration = 0;
    this->DialsAlgorithm(edgeList, numNodes, 0, timeDuration);

}

// performs Dials algorithm on a network. The Sink node is Node 0, sensor nodes 1,2,..., n
// and target nodes n+1, n+2, ... , n + numTargets
void NodeLinkedList::DialsAlgorithm(vector<Edge*> edgeList, int numNodes, int numTar, double &timeDuration) {

    // numNodes - Includes every node in the network (sink, sensors, and targets)
    int numPermanent = 1;

    // start time record
    auto start = chrono::high_resolution_clock::now();

    // iterates through edges in the network that leave the sink node, and update the bucket and max capacity path label
    // of the newly reached node
    Node sinkNode = myBuckets[0];

    for (int j = 0; j < sinkNode.GetDegree(); ++j) {

        int updateNode = sinkNode.GetReachableNode(j);
        int updateLabel = min(sinkNode.GetValue(), myBuckets[updateNode].GetValue());

        myBuckets[updateNode].UpdateReached();
        myBuckets[updateNode].UpdateCapacityLabel(updateLabel);

        Node *ptr = myBuckets;
        ptr = ptr + updateNode;

        this->RemoveTargetElement(0, ptr);
        this->AddToList(updateLabel, ptr);

    }

    int currBucket = numNodes - numTar;

    while (numPermanent < numNodes && currBucket > 0){

        // if the current bucket is not empty
        while (!(this->IsListEmpty(currBucket))){
            // remove the next node from the bucket

            Node *permNode = myBuckets;
            permNode = permNode + (this->RemoveFirst(currBucket))->GetNodeID();
            permNode->MarkPermanent();
            // stores the failure order and time at which node is disconnected
            this->addFailOrder(permNode->GetNodeID());
            this->addFailTime(permNode->GetMaxCapacityPath());
            numPermanent = numPermanent + 1;

            // stores the failure order and time at which Targets are disconnected
            if ( permNode->GetNodeID() >= numNodes - numTar ) {
                this->addTargetFailOrder(permNode->GetNodeID());
                this->addTargetFailTime(permNode->GetMaxCapacityPath());
            }
            else {
                this->addSensorFailOrder(permNode->GetNodeID());
                this->addSensorFailTime(permNode->GetMaxCapacityPath());
            }

            // iterate through edges
            for (int i = 0; i < permNode->GetDegree(); ++i) {

                int updateNode1 = permNode->GetReachableNode(i);

                if (!myBuckets[updateNode1].IsPermanent()){
                    myBuckets[updateNode1].UpdateReached();

                    // determine the current label and the new label for the reachable node

                    int currLabel =  myBuckets[updateNode1].GetMaxCapacityPath();
                    int updateLabel = min(permNode->GetMaxCapacityPath(), myBuckets[updateNode1].GetValue());

                    //TODO : think about adding an IF statement to see if updateLabel == node current max capacity
                    // if it is then the AddToList does not need to be reaccomplished. Right now it places the node
                    // at the beginning of the list anyway

                    Node *ptr = myBuckets;
                    ptr = ptr + updateNode1;

                    this->RemoveTargetElement(currLabel, ptr);

                    myBuckets[updateNode1].UpdateCapacityLabel(updateLabel);

                    this->AddToList(updateLabel, ptr);
                }
            }
        }
        currBucket = currBucket - 1;
    }

    // If a target is not connected to the sink then it will never be marked permanent, so it will never be added to the
    // order of target failures. This for loop checks for such a case, and adds the targets to ths list with a fail time of zero
    if (orderedTargetFail.size() < numTar){
        Node *ptr = myBuckets;
        ptr = ptr + (numNodes - numTar);
        for (int i = 0; i < numTar ; ++i) {
            if (!ptr->IsPermanent()){
                this->addTargetFailOrder(ptr->GetNodeID());
                this->addTargetFailTime(ptr->GetMaxCapacityPath());
            }
            ptr = ptr + 1;
        }
    }

    // end time record
    auto finish = chrono::high_resolution_clock::now();
    // calculate execution time
    chrono::duration<double> elapsed = finish - start;
    timeDuration = elapsed.count();

}

// This function performs Dials Algorithm the same as above, but uses the max(min()) rule to update
// The results are exactly the same
void NodeLinkedList::DialsAlgorithmUpdateRule(vector<Edge *> edgeList, int numNodes, int numTar, double &timeDuration) {

    // numNodes - Includes every node in the network (sink, sensors, and targets)
    int numPermanent = 1;

    // start time record
    auto start = chrono::high_resolution_clock::now();

    // iterates through edges in the network that leave the sink node, and update the bucket and max capacity path label
    // of the newly reached node
    Node sinkNode = myBuckets[0];

    for (int j = 0; j < sinkNode.GetDegree(); ++j) {

        int updateNode = sinkNode.GetReachableNode(j);
        int updateLabel = max(myBuckets[updateNode].GetMaxCapacityPath(), min(sinkNode.GetMaxCapacityPath(), myBuckets[updateNode].GetValue()));

        myBuckets[updateNode].UpdateReached();
        myBuckets[updateNode].UpdateCapacityLabel(updateLabel);

        Node *ptr = myBuckets;
        ptr = ptr + updateNode;

        this->RemoveTargetElement(0, ptr);
        this->AddToList(updateLabel, ptr);

    }

    int currBucket = numNodes - numTar;

    while (numPermanent < numNodes && currBucket > 0){

        // if the current bucket is not empty
        while (!(this->IsListEmpty(currBucket))){
            // remove the next node from the bucket

            Node *permNode = myBuckets;
            permNode = permNode + (this->RemoveFirst(currBucket))->GetNodeID();

            permNode->MarkPermanent();
            // stores the failure order and time at which node is disconnected
            this->addFailOrder(permNode->GetNodeID());
            this->addFailTime(permNode->GetMaxCapacityPath());
            numPermanent = numPermanent + 1;

            // stores the failure order and time at which Targets are disconnected
            if ( permNode->GetNodeID() >= numNodes - numTar ) {
                this->addTargetFailOrder(permNode->GetNodeID());
                this->addTargetFailTime(permNode->GetMaxCapacityPath());
            }
            else {
                this->addSensorFailOrder(permNode->GetNodeID());
                this->addSensorFailTime(permNode->GetMaxCapacityPath());
            }

            // iterate through edges
            for (int i = 0; i < permNode->GetDegree(); ++i) {

                int updateNode1 = permNode->GetReachableNode(i);

                int currLabel =  myBuckets[updateNode1].GetMaxCapacityPath();
                int updateLabel = max(myBuckets[updateNode1].GetMaxCapacityPath(), min(permNode->GetMaxCapacityPath(), myBuckets[updateNode1].GetValue()));

                Node *ptr = myBuckets;
                ptr = ptr + updateNode1;

                if (currLabel == updateLabel){
                    // prevents the situation in which a node is placed back in the same bucket (particularly if the node has already been permanent)
                }
                else {
                    this->RemoveTargetElement(currLabel, ptr);
                    myBuckets[updateNode1].UpdateCapacityLabel(updateLabel);
                    this->AddToList(updateLabel, ptr);

                }
            }
        }
        currBucket = currBucket - 1;
    }

    // If a target is not connected to the sink then it will never be marked permanent, so it will never be added to the
    // order of target failures. This for loop checks for such a case, and adds the targets to ths list with a fail time of zero
    if (orderedTargetFail.size() < numTar){
        Node *ptr = myBuckets;
        ptr = ptr + (numNodes - numTar);
        for (int i = 0; i < numTar ; ++i) {
            if (!ptr->IsPermanent()){
                this->addTargetFailOrder(ptr->GetNodeID());
                this->addTargetFailTime(ptr->GetMaxCapacityPath());
            }
            ptr = ptr + 1;
        }
    }

    // end time record
    auto finish = chrono::high_resolution_clock::now();
    // calculate execution time
    chrono::duration<double> elapsed = finish - start;
    timeDuration = elapsed.count();

}


// performs Dijkstra's algorithm on a network. The Sink node is Node 0, sensor nodes 1,2,..., n
void NodeLinkedList::DijkstrasAlgorithm(vector<Edge *> edgeList, int numNodes) {

    double timeDuration = 0;
    this->DijkstrasAlgorithm(edgeList, numNodes, 0, timeDuration);

}

// performs Dials algorithm on a network. The Sink node is Node 0, sensor nodes 1,2,..., n
// and target nodes n+1, n+2, ... , n + numTargets
void NodeLinkedList::DijkstrasAlgorithm(vector<Edge *> edgeList, int numNodes, int numTar, double &timeDuration) {

    // sink node is already permanent
    int numPermanent = 1;

    // vector to store ID's of temporary nodes
    vector<int> tempNodes;
    for (int i = 1; i <= (numNodes-1) ; ++i) {
        tempNodes.push_back(i);
    }

    // start time record
    auto start = chrono::high_resolution_clock::now();

    // update all nodes connected to sink
    for (int i = 0; i < myBuckets[0].GetDegree(); ++i) {
        int updateNode1 = myBuckets[0].GetReachableNode(i);
        int updateLabel = min( myBuckets[0].GetMaxCapacityPath(), myBuckets[updateNode1].GetValue());
        myBuckets[updateNode1].UpdateCapacityLabel(updateLabel);
    }

    while (numPermanent < numNodes) {

        int tempNode;
        int tempIndex = -1;
        int currMax = 0;
        int permNode = 0;

        // find the node with the largest distance label that is not permanent
        for (int j = 0; j < tempNodes.size(); ++j) {
            tempNode = tempNodes.at(j);
            if (myBuckets[tempNode].GetMaxCapacityPath() > currMax) {
                // store node ID and max label

                currMax = myBuckets[tempNode].GetMaxCapacityPath();
                permNode = myBuckets[tempNode].GetNodeID();
                tempIndex = j;
            }
        }

        if (tempIndex == -1) {
            // node not connected - should be able to break here
        } else {

            // remove from temporary nodes vector
            tempNodes.erase(tempNodes.begin() + tempIndex);

            // mark node permanent
            myBuckets[permNode].MarkPermanent();

            this->addFailOrder(permNode);
            this->addFailTime(currMax);

            // stores the failure order and time at which Targets are disconnected
            if (myBuckets[permNode].GetNodeID() >= numNodes - numTar) {
                this->addTargetFailOrder(myBuckets[permNode].GetNodeID());
                this->addTargetFailTime(myBuckets[permNode].GetMaxCapacityPath());
            }
            else {
                this->addSensorFailOrder(myBuckets[permNode].GetNodeID());
                this->addSensorFailTime(myBuckets[permNode].GetMaxCapacityPath());
            }

            // iterate through edges
            for (int k = 0; k < myBuckets[permNode].GetDegree(); ++k) {

                int updateNode1 = myBuckets[permNode].GetReachableNode(k);
                if (!myBuckets[updateNode1].IsPermanent()) {
                    int updateLabel = min(myBuckets[permNode].GetMaxCapacityPath(), myBuckets[updateNode1].GetValue());
                    myBuckets[updateNode1].UpdateCapacityLabel(updateLabel);
                }
            }
        }
        numPermanent = numPermanent + 1;
    }

    // If a target is not connected to the sink then it will never be marked permanent, so it will never be added to the
    // order of target failures. This for loop checks for such a case, and adds the targets to ths list with a fail time of zero
    if (orderedTargetFail.size() < numTar){
        Node *ptr = myBuckets;
        ptr = ptr + (numNodes - numTar);
        for (int i = 0; i < numTar ; ++i) {
            if (!ptr->IsPermanent()){
                this->addTargetFailOrder(ptr->GetNodeID());
                this->addTargetFailTime(ptr->GetMaxCapacityPath());
            }
            ptr = ptr + 1;
        }
    }

    // end time record
    auto finish = chrono::high_resolution_clock::now();
    // calculate execution time
    chrono::duration<double> elapsed = finish - start;
    timeDuration = elapsed.count();
}

// within an iteration, add the number of sensors failed that cause network failure
// DSpecFreq - vector of size (numSensors x 1) with frequency for ith sensor failure causing network failure
// DSpecElements - vector containing unique elements of DSpecFreq with elements greater than zero
void NodeLinkedList::UpdateDSpecFeq(int numFailed) {

    if (DSpecFreq.at(numFailed) == 0) {
        DSpecFreq[numFailed] = 1;
        DSpecElements.push_back(numFailed);
    }
    else {
        DSpecFreq[numFailed] = DSpecFreq[numFailed] + 1;
    }
}

// Once all replications are complete, calculate the D Spectrum. The number of failures is also in order
// myDSpec - number of sensors failed that cause network failure
// myDSpecProb - probability that the corresponding entry in myDSpec causes network failure
void NodeLinkedList::CalculateDSpec(int numReps){
    for (int i = 0; i < DSpecFreq.size() ; ++i) {
        if (DSpecFreq.at(i) == 0){
            // skip
        }
        else {
            myDSpec.push_back(i);
            double myProb = (double) DSpecFreq.at(i) / (double) numReps;
            myDSpecProb.push_back(myProb);
        }
    }
}

void NodeLinkedList::OutputDSpec() {
    for (int i = 0; i < myDSpec.size() ; ++i) {
        cout << myDSpec.at(i) << "     ----    " << myDSpecProb.at(i) << endl;
    }
}

void NodeLinkedList::RecordDSpec(ofstream &outputFile) {
    for (int i = 0; i < myDSpec.size() ; ++i) {
        outputFile << myNumSensors - 1 << " - " << myDSpec.at(i) << "     ----    " << myDSpecProb.at(i) << endl;
    }
}

// outputs critical times for every node in the network
void NodeLinkedList::OutputCriticalTimes(){
    for (int i = 0; i < myNumElements; ++i) {
        cout << " Node Number " << myBuckets[i].GetNodeID() << " --- Critical Time " << myBuckets[i].GetMaxCapacityPath() << "\n";
    }
}

// outputs critical times for targets only, in order
void NodeLinkedList::OutputTargetCriticalTimes(int numTargets) {
    cout << "TargetFail ";
    for (int i = (numTargets - 1); i >= 0; --i) {
        cout << orderedTargetCriticalFailTime.at(i) << " ";
    }
    cout << endl;
}

// outputs critical times for targets only, in order
void NodeLinkedList::OutputTargetCriticalTimes(int numTargets, ofstream &outputFile) {
//    cout << "TargetFail ";
    for (int i = (numTargets - 1); i >= 0; --i) {
        outputFile << orderedTargetCriticalFailTime.at(i) << " ";
    }
    outputFile << endl;
}

// outputs the target ID in order for which targets are disconnected
void NodeLinkedList::OutputTargetFailOrder(int numTargets){
    cout << "TargetFailOrder ";
    for (int i = (numTargets - 1); i >= 0; --i) {
        cout << orderedTargetFail.at(i) << " ";
    }
    cout << endl;
}

// outputs the sensor ID in order for which sensors are disconnected
void NodeLinkedList::OutputSensorFailOrder(int numSensors){
    cout << "SensorFailOrder ";
    for (int i = (numSensors - 1); i >= 0; --i) {
        cout << orderedSensorFail.at(i) << " " ;
    }
    cout << endl;
}

// vector that stores the ordered failed times
void NodeLinkedList::addFailTime(int failTime){
    orderedCriticalFailTime.push_back(failTime);
}

// vector that stores the order in which nodes are disconnected from the sink (node ID) - for all nodes
void NodeLinkedList::addFailOrder(int nodeNum){
    orderedFail.push_back(nodeNum);
}

// returns the time from an input location from the ordered list of fail times
// NOTE: the list is in reverse order (largest to smallest), does not include sink
// myNumElements-1 because myNumElements includes the sink node
int NodeLinkedList::GetFailTime(int num){
    return orderedCriticalFailTime.at(myNumElements - 1 - num);
}

// returns a number of the ordered list of failed nodes (node ID)
// NOTE: the list is in reverse order (largest to smallest), does not include sink
// myNumElements-1 because myNumElements includes the sink node
int NodeLinkedList::GetFailOrder(int num){
    return orderedFail.at(myNumElements - 1 - num);
}

// vector that stores the ordered failed times for targets
void NodeLinkedList::addTargetFailTime(int failTime){
    orderedTargetCriticalFailTime.push_back(failTime);
}

// vector that stores the order in which targets are disconnected from the sink (target ID)
void NodeLinkedList::addTargetFailOrder(int nodeNum){
    orderedTargetFail.push_back(nodeNum);
}

// vector that stores the ordered fail time for sensors
void NodeLinkedList::addSensorFailTime(int failTime){
    orderedSensorCriticalFailTime.push_back(failTime);
}

// vector that stores the order in which sensors are disconnected from the sink (sensors ID)
void NodeLinkedList::addSensorFailOrder(int nodeNum){
    orderedSensorFail.push_back(nodeNum);
}

int NodeLinkedList::GetSensorFailOrder(int senNum){
    return orderedSensorFail.at(orderedSensorFail.size() - senNum);
}

// returns a number of the ordered list of disconnected targets times
// NOTE: the list is in reverse order (largest to smallest)
int NodeLinkedList::GetTargetFailTime(int failNum){
    return orderedTargetCriticalFailTime.at(orderedTargetCriticalFailTime.size() - failNum);
}

// returns an element from the ordered list of targets that are disconnected
// NOTE: the list is in reverse order (largest to smallest)
int NodeLinkedList::GetTargetFailOrder(int targNum){
    return orderedTargetFail.at(orderedTargetFail.size() - targNum);
}

int NodeLinkedList::GetFailureSubregion(int sensNum){
    return myBuckets[sensNum].GetSubregion();
}

// determines how many TARGETS need to be disconnected/failed before coverage drops below a desired threshold
// returns this number. Coverage is between 0 and 1
int NodeLinkedList::CoverageNumber(double coverage, int numT) {
    // myNumElements includes the sink node, so subtract 1 to get number of sensor nodes
    int coverNum;
    double exactNum = numT * (1-coverage);
    int wholeNum = (int) ceil(numT * (1-coverage));

    if (abs(wholeNum - exactNum) <= 0.0001){
        coverNum = wholeNum + 1;
    }
    else {
        coverNum = wholeNum;
    }
    return coverNum;
}

// used to reinitialize buckets for a new test instance
void NodeLinkedList::UpdateBucketTimes(vector<int> newOrder){

    this->ClearOrderTimes();

    Node *ptr = myBuckets;
    // update all values for the sink node
    ptr->SetID(0);
    ptr->UpdateCapacityLabel(myNumElements);
    ptr->UpdateReached();
    ptr->MarkPermanent();
    ptr->SetValue(newOrder[0]);
    ptr->SetFirstItem((ptr + 1));

    ptr = ptr + 1;

    // sets the pointers for a node to be the node immediately prior and immediately after the node
    for (int i = 1; i < myNumSensors ; ++i) {

        // reset initial status
        myBuckets[i].ResetNodeReached();
        myBuckets[i].ResetNodePermanent();
        myBuckets[i].UpdateCapacityLabel(0);

        // re-initialize
        myBuckets[i].SetNext((ptr + 1));
        myBuckets[i].SetPrev((ptr - 1));
        myBuckets[newOrder[i]].SetValue(i);

        myBuckets[i].SetFirstItem(nullptr);

        ptr = ptr + 1;

    }

    // Update value for target nodes
    for (int j = myNumSensors ; j < myNumElements ; ++j) {

        // reset initial status
        myBuckets[j].ResetNodeReached();
        myBuckets[j].ResetNodePermanent();
        myBuckets[j].UpdateCapacityLabel(0);

        myBuckets[j].SetID(j);
        myBuckets[j].SetNext((ptr + 1));
        myBuckets[j].SetPrev((ptr - 1));
        myBuckets[j].SetValue(myNumElements);

        myBuckets[j].SetFirstItem(nullptr);

        ptr = ptr + 1;

    }

    myBuckets[myNumElements-1].SetNext(nullptr);
}

// clears out all information from a previous test instance
void NodeLinkedList::ClearOrderTimes() {
    orderedTargetFail.clear();
    orderedFail.clear();
    orderedCriticalFailTime.clear();
    orderedTargetCriticalFailTime.clear();
    orderedSensorCriticalFailTime.clear();
    orderedSensorFail.clear();

}

// Input the overall time interval, and calculate reliability for each unit of time
// myDSpec - number of sensors failed that cause network failure
// myDSpecProb - probability that the corresponding entry in myDSpec causes network failure
void NodeLinkedList::CalculateReliabilityWeibull(double t, double myShape, double myScale) {
/*    for (int i = 0; i <= t ; ++i) {
        cout << " Reliability : " << NetworkReliabilityWeibull(i, myNumSensors - 1, myDSpec, myDSpecProb, myShape, myScale) << endl;
//        NetworkReliabilityWeibull(i, myNumSensors - 1, myDSpec, myDSpecProb, myShape, myScale);

    }*/

    double i = 0;
    while (i < t) {
        cout << "Reliability " << myNumSensors << " t = " << i << " -- "
             << NetworkReliabilityWeibull(i, myNumSensors - 1, myDSpec, myDSpecProb, myShape, myScale) << endl;
        i = i + 0.2;
    }
}


// Input the overall time interval, and calculate variance for each unit of time
// myDSpec - number of sensors failed that cause network failure
// myDSpecProb - probability that the corresponding entry in myDSpec causes network failure
void NodeLinkedList::CalculateReliabilityVarianceWeibull(double t, int numReps, double myShape, double myScale) {
    for (int i = 0; i <= t ; ++i) {
        cout << "Variance t =  " << i << " -- " << NetworkReliabilityVarianceWeibull(i, myNumSensors - 1, myDSpec, myDSpecProb, numReps, myShape, myScale) << endl;
    }
}

// Calculates the stable network reliability in the presence of a delta TBM policy
void NodeLinkedList::CalculateStableMaintenanceReliabilityWeibull(double delta, double mShape, double mScale) {

//    cout << "Stable Network Reliability delta = " << delta << " : " << StableMaintenanceReliabilityWeibull(delta, myNumSensors - 1, myDSpec, myDSpecProb) << endl;
    cout << "MaintenanceRel " << myNumSensors << " delta = " << delta << " -- " <<  StableMaintenanceReliabilityWeibull(delta, myNumSensors - 1, myDSpec, myDSpecProb, mShape, mScale) << endl;
//    StableMaintenanceReliabilityWeibull(delta, myNumSensors - 1, myDSpec, myDSpecProb, mShape, mScale);

}

void NodeLinkedList::CalculateStableMaintenanceReliabilityWeibull(double delta, double mShape, double mScale, ofstream &outputFile){
    outputFile << "MaintenanceRel " << myNumSensors << " delta = " << delta << " -- " <<  StableMaintenanceReliabilityWeibull(delta, myNumSensors - 1, myDSpec, myDSpecProb, mShape, mScale) << endl;
}

void NodeLinkedList::CalculateStableMaintenanceReliabilityWeibullWithWeight(double delta, double mShape, double mScale, vector<double> regWeight, ofstream &outputFile){
    outputFile << "Weights: " ;
    outputFile << regWeight.at(0) << " " ;
    outputFile << regWeight.at(1) << " " ;
    outputFile << regWeight.at(5) << " " ;
    outputFile << " n1= " << myNumSensors << " delta= " << delta << " -- " <<  StableMaintenanceReliabilityWeibull(delta, myNumSensors - 1, myDSpec, myDSpecProb, mShape, mScale) << endl;
}

// Calculates the stable network reliability in the presence of a delta TBM policy
void NodeLinkedList::CalculateStableMaintenanceVarianceWeibull(double delta, int numReps, double mShape, double mScale) {

    cout << "MaintenanceVar " << myNumSensors << " delta = " << delta << " -- " <<  StableMaintenanceVarianceWeibull(delta, myNumSensors - 1, myDSpec, myDSpecProb, numReps, mShape, mScale) << endl;

}

// Calculates the cost of stable network reliability in the presence of a delta TBM policy
void NodeLinkedList::CalculateMaintenancePolicyCostWeibull(double delta, double costFixed, double costVar, double mShape,
                                                           double mScale) {

//    cout << "Stable Network Reliability Cost with delta " << delta << " : " << MaintenancePolicyCostWeibull(delta, costFixed, costVar, myNumSensors - 1) << endl;
    cout << MaintenancePolicyCostWeibull(delta, costFixed, costVar, myNumSensors - 1, mShape, mScale) << endl;
//    MaintenancePolicyCostWeibull(delta, costFixed, costVar, myNumSensors - 1, mShape, mScale);

}

// Calculates the cost of stable network reliability in the presence of a delta TBM policy
void NodeLinkedList::CalculateMaintenancePolicyCostWeibull(int numSens, double delta, double costFixed, double costVar, double mShape,
                                                           double mScale) {

//    cout << "Stable Network Reliability Cost with delta " << delta << " : " << MaintenancePolicyCostWeibull(delta, costFixed, costVar, myNumSensors - 1) << endl;
    cout << MaintenancePolicyCostWeibull(delta, costFixed, costVar, numSens, mShape, mScale) << endl;
}

// Calculates the stable network reliability in the presence of a delta TBM policy
double NodeLinkedList::StableMaintenanceRelReturn(double delta, double mShape, double mScale) {
    return StableMaintenanceReliabilityWeibull(delta, myNumSensors - 1, myDSpec, myDSpecProb, mShape, mScale);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

// Generates a new RGG, with a new x,y location for every sensor
void NodeLinkedList::GenerateNewRGG_naive() {

    Node *ptr = myBuckets;
    //Sink Location
    ptr->SetNewLoc(0.5,0.5);
    ptr->ResetReachableNodes();

    ptr = ptr + 1;

//    random_device myDev;
    uniform_real_distribution<> myUnif(0,1);

    double myXNum;
    double myYNum;

    // generate new location for every sensor
    for (int i = 1; i < myNumSensors ; ++i) {
        myXNum = myUnif(myDevX);
        myYNum = myUnif(myDevY);

        ptr->ResetReachableNodes();
        ptr->SetNewLoc(myXNum, myYNum);
        ptr = ptr + 1;
    }

    // build forward star for sink node separately, as we do not have to examine target nodes
    int startNode = 0;
    for (int l = startNode + 1; l < myNumSensors ; ++l) {
        double dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[l].GetXLoc(), 2) + pow(myBuckets[startNode].GetYLoc() - myBuckets[l].GetYLoc(), 2));
        if (dist <= myCommunicationRange){
            myBuckets[startNode].AddReachableNode(myBuckets[l].GetNodeID());
        }
    }

    // rebuild forward star for every node
    for (int j = 1; j < myNumSensors ; ++j) {
        for (int i = (j+1); i <myNumSensors ; ++i) {
            double dist = sqrt(pow(myBuckets[j].GetXLoc() - myBuckets[i].GetXLoc(), 2) + pow(myBuckets[j].GetYLoc() - myBuckets[i].GetYLoc(), 2));
            if (dist <= myCommunicationRange){
                myBuckets[j].AddReachableNode(myBuckets[i].GetNodeID());
                myBuckets[i].AddReachableNode(myBuckets[j].GetNodeID());
            }
        }


        // Add Target nodes to forward star
        for (int k = myNumSensors; k < (myNumSensors + myNumTargets) ; ++k) {
            double dist = sqrt(pow(myBuckets[j].GetXLoc() - myBuckets[k].GetXLoc(), 2) + pow(myBuckets[j].GetYLoc() - myBuckets[k].GetYLoc(), 2));
            if (dist <= mySensorRange){
                myBuckets[j].AddReachableNode(myBuckets[k].GetNodeID());
            }
        }
    }
}

// TODO: NOTE - if the number of grid cells is larger than the total number of nodes in the network, this process will have issues!!!
void NodeLinkedList::GenerateNewRGG_Bucket(){

//    random_device myDev;
    uniform_real_distribution<> myUnif(0,1);

    double myXNum;
    double myYNum;
    int myTempBin;

    int myBucketsPerRow = (int) ceil(1 / myCommunicationRange);

    Node *ptr = myBuckets;
    //Sink Location
    ptr->SetNewLoc(0.5,0.5);
    // clear the forward start of the node, and nodes in bin zero
    ptr->ResetReachableNodes();
    ptr->ResetNodesInBucket();

    ptr = ptr + 1;

    // sets the pointers for a node to be the node immediately prior and immediately after the node
    for (int i = 1; i < myNumSensors ; ++i) {
        ptr->ResetReachableNodes();
        ptr->ResetNodesInBucket();
        ptr = ptr + 1;

    }

    // Update value for target nodes
    for (int j = myNumSensors ; j < myNumElements ; ++j) {
        ptr->ResetReachableNodes();
        ptr->ResetNodesInBucket();
        ptr = ptr + 1;
    }

    // start placing sensor in bins, starting with sink node
    myXNum = myBuckets[0].GetXLoc();
    myYNum = myBuckets[0].GetYLoc();
    // determine which grid number the new sensor location falls in
    myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow +1);

    myBuckets[myTempBin].AddNodeToBucket(0);

    ptr = myBuckets;
    ptr = ptr + 1;

    // generate new location for every sensor
    for (int i = 1; i < myNumSensors ; ++i) {
        myXNum = myUnif(myDevX);
        myYNum = myUnif(myDevY);

        ptr->SetNewLoc(myXNum, myYNum);

        // determine which grid number the new sensor location falls in
        myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow + 1);

        // move sensor into a bin for this new location
        myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());

        ptr = ptr + 1;
    }

    // place target nodes in new bin
    for (int k = myNumSensors; k < myNumElements ; ++k) {

        myXNum = ptr->GetXLoc() + 0.0001;
        myYNum = ptr->GetYLoc() + 0.0001;

        // determine which grid number the new sensor location falls in
        myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow +1);

        //cout << " Node " << ptr->GetNodeID() << " Added to bucket " << myTempBin << endl;

        // move sensor into a bin for this new location
        myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());

        ptr = ptr + 1;

    }

    int myXBucket;
    int myYBucket;
    double dist;
    vector<int> myNodesInCurBucket;

    int curBucket = myBucketsPerRow + 1;
    for (int l = 0; l < pow(myBucketsPerRow,2) ; ++l) {
        // determine which grid cell we are currently in
        myXBucket = curBucket % myBucketsPerRow;
        myYBucket = (curBucket / myBucketsPerRow) % myBucketsPerRow ;

        if (myXBucket == 0){
            myXBucket = myBucketsPerRow;
            if (curBucket / myBucketsPerRow == myBucketsPerRow){
                myYBucket = myBucketsPerRow - 1;
            }
            else {
                myYBucket = myYBucket - 1;
            }
        }

        if (myYBucket == 0){
            myYBucket = myBucketsPerRow;
        }

        // all within bin comparison
        for (int i = 0; i < (myBuckets[l].GetNumberInCell()) ; ++i) {

            int startNode = myBuckets[l].GetNodeInBucket(i);

            // if the starting node is the sink node, then we only need to worry about connecting to sensor nodes, which is based on communication range
            if (startNode == 0) {
                for (int j = (i+1); j < myBuckets[l].GetNumberInCell() ; ++j) {
                    int endNode = myBuckets[l].GetNodeInBucket(j);
                    // determine if end node is a sensor node or a target node
                    if (endNode < myNumSensors){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        if (dist <= myCommunicationRange) {
                            myBuckets[startNode].AddReachableNode(endNode);
                        }
                    }
                }
            }
                // if starting node is a sensor node
            else if (startNode > 0 && startNode < myNumSensors){

                for (int j = (i+1); j < myBuckets[l].GetNumberInCell() ; ++j) {
                    int endNode = myBuckets[l].GetNodeInBucket(j);

                    // start node is a sensor node, end node is sink node, check for reverse arc
                    if (endNode == 0){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        // sensor to sink comparison, we can add both arcs
                        if (dist <= myCommunicationRange) {
                            myBuckets[endNode].AddReachableNode(startNode);
                        }

                    }
                    else if (endNode < myNumSensors){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        // sensor to sensor comparison, we can add both arcs
                        if (dist <= myCommunicationRange) {
                            myBuckets[startNode].AddReachableNode(endNode);
                            myBuckets[endNode].AddReachableNode(startNode);
                        }
                    }
                    else {
                        // end node must be a target node
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        if (dist <= mySensorRange){
                            myBuckets[startNode].AddReachableNode(endNode);
                        }
                    }
                }
            }
        }

        int myAdjacentCell;

        // look to the cell immediately above current cell
        int myTopBucket = myYBucket + 1;
        myAdjacentCell = myXBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow +1);
        if (l < myAdjacentCell && myTopBucket <= myBucketsPerRow){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell immediately to the right
        int myRightBucket = myXBucket + 1;
        myAdjacentCell = myRightBucket + (myYBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myRightBucket <= myBucketsPerRow){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell immediately below
        int myBottomBucket = myYBucket - 1;
        myAdjacentCell = myXBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myBottomBucket < 0){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to left of current cell
        int myLeftBucket = myXBucket - 1;
        myAdjacentCell = myLeftBucket + (myYBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myLeftBucket > 0){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the top right
        myAdjacentCell = myRightBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myRightBucket <= myBucketsPerRow) && (myTopBucket <= myBucketsPerRow)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the bottom right
        myAdjacentCell = myRightBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myRightBucket <= myBucketsPerRow) && (myBottomBucket > 0)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the top left
        myAdjacentCell = myLeftBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l< myAdjacentCell) && (myLeftBucket > 0 ) && (myTopBucket <= myBucketsPerRow)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the bottom left
        myAdjacentCell = myLeftBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myLeftBucket > 0 ) && (myBottomBucket > 0)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        curBucket = curBucket + 1;
    }
}

// Used to generate a RGG with specific structure, where a certain number of sensors are located in each subregion of the network
void NodeLinkedList::GenerateNewSpecificRGG_Bucket(vector<int> numPerRegion){

//    random_device myDev;

    double myXNum;
    double myYNum;
    int myTempBin;

    int myBucketsPerRow = (int) ceil(1 / myCommunicationRange);

    int myRegionsPerRow = 4;

    Node *ptr = myBuckets;
    //Sink Location
    ptr->SetNewLoc(0.5,0.5);
    // clear the forward start of the node, and nodes in bin zero
    ptr->ResetReachableNodes();
    ptr->ResetNodesInBucket();

    ptr = ptr + 1;

    // sets the pointers for a node to be the node immediately prior and immediately after the node
    for (int i = 1; i < myNumSensors ; ++i) {
        ptr->ResetReachableNodes();
        ptr->ResetNodesInBucket();
        ptr->ResetFailed();
        ptr->ResetCovered();
        ptr = ptr + 1;

    }

    // Update value for target nodes
    for (int j = myNumSensors ; j < myNumElements ; ++j) {
        ptr->ResetReachableNodes();
        ptr->ResetNodesInBucket();
        ptr->ResetFailed();
        ptr->ResetCovered();
        ptr = ptr + 1;
    }

    // start placing sensor in bins, starting with sink node
    myXNum = myBuckets[0].GetXLoc();
    myYNum = myBuckets[0].GetYLoc();
    // determine which grid number the new sensor location falls in
    myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow +1);

    myBuckets[myTempBin].AddNodeToBucket(0);

    ptr = myBuckets;
    ptr = ptr + 1;

    for (int j = 0; j < myRegionsPerRow*myRegionsPerRow ; ++j) {

        double tempXLow = myBuckets[j].GetXLow();
        double tempXHigh = myBuckets[j].GetXHigh() + 0.001;
        double tempYLow = myBuckets[j].GetYLow();
        double tempYHigh = myBuckets[j].GetYHigh() + 0.001;


        for (int i = 0; i < numPerRegion.at(j); ++i) {
            uniform_real_distribution<> myUnifX(tempXLow, fmin(tempXHigh, 1.0));
            uniform_real_distribution<> myUnifY(tempYLow, fmin(tempYHigh, 1.0));

            myXNum = myUnifX(myDevX);
            myYNum = myUnifY(myDevY);

            ptr->SetNewLoc(myXNum, myYNum);
            ptr->SetSubregion(j);

            // determine which grid number the new sensor location falls in
            myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow + 1);

            // move sensor into a bin for this new location
            myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());

            ptr = ptr + 1;

        }
    }

    // place target nodes in new bin
    for (int k = myNumSensors; k < myNumElements ; ++k) {

        myXNum = ptr->GetXLoc() + 0.0001;
        myYNum = ptr->GetYLoc() + 0.0001;

        // determine which grid number the new sensor location falls in
        myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow +1);

        //cout << " Node " << ptr->GetNodeID() << " Added to bucket " << myTempBin << endl;

        // move sensor into a bin for this new location
        myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());

        ptr = ptr + 1;

    }

    int myXBucket;
    int myYBucket;
    double dist;
    vector<int> myNodesInCurBucket;

    int curBucket = myBucketsPerRow + 1;
    for (int l = 0; l < pow(myBucketsPerRow,2) ; ++l) {
        // determine which grid cell we are currently in
        myXBucket = curBucket % myBucketsPerRow;
        myYBucket = (curBucket / myBucketsPerRow) % myBucketsPerRow ;

        if (myXBucket == 0){
            myXBucket = myBucketsPerRow;
            if (curBucket / myBucketsPerRow == myBucketsPerRow){
                myYBucket = myBucketsPerRow - 1;
            }
            else {
                myYBucket = myYBucket - 1;
            }
        }

        if (myYBucket == 0){
            myYBucket = myBucketsPerRow;
        }

        // all within bin comparison
        for (int i = 0; i < (myBuckets[l].GetNumberInCell()) ; ++i) {

            int startNode = myBuckets[l].GetNodeInBucket(i);

            // if the starting node is the sink node, then we only need to worry about connecting to sensor nodes, which is based on communication range
            if (startNode == 0) {
                for (int j = (i+1); j < myBuckets[l].GetNumberInCell() ; ++j) {
                    int endNode = myBuckets[l].GetNodeInBucket(j);
                    // determine if end node is a sensor node or a target node
                    if (endNode < myNumSensors){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        if (dist <= myCommunicationRange) {
                            myBuckets[startNode].AddReachableNode(endNode);
                        }
                    }
                }
            }
                // if starting node is a sensor node
            else if (startNode > 0 && startNode < myNumSensors){

                for (int j = (i+1); j < myBuckets[l].GetNumberInCell() ; ++j) {
                    int endNode = myBuckets[l].GetNodeInBucket(j);

                    // start node is a sensor node, end node is sink node, check for reverse arc
                    if (endNode == 0){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        // sensor to sink comparison, we can add both arcs
                        if (dist <= myCommunicationRange) {
                            myBuckets[endNode].AddReachableNode(startNode);
                        }

                    }
                    else if (endNode < myNumSensors){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        // sensor to sensor comparison, we can add both arcs
                        if (dist <= myCommunicationRange) {
                            myBuckets[startNode].AddReachableNode(endNode);
                            myBuckets[endNode].AddReachableNode(startNode);
                        }
                    }
                    else {
                        // end node must be a target node
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        if (dist <= mySensorRange){
                            myBuckets[startNode].AddReachableNode(endNode);
                        }
                    }
                }
            }
        }

        int myAdjacentCell;

        // look to the cell immediately above current cell
        int myTopBucket = myYBucket + 1;
        myAdjacentCell = myXBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow +1);
        if (l < myAdjacentCell && myTopBucket <= myBucketsPerRow){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell immediately to the right
        int myRightBucket = myXBucket + 1;
        myAdjacentCell = myRightBucket + (myYBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myRightBucket <= myBucketsPerRow){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell immediately below
        int myBottomBucket = myYBucket - 1;
        myAdjacentCell = myXBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myBottomBucket < 0){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to left of current cell
        int myLeftBucket = myXBucket - 1;
        myAdjacentCell = myLeftBucket + (myYBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myLeftBucket > 0){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the top right
        myAdjacentCell = myRightBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myRightBucket <= myBucketsPerRow) && (myTopBucket <= myBucketsPerRow)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the bottom right
        myAdjacentCell = myRightBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myRightBucket <= myBucketsPerRow) && (myBottomBucket > 0)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the top left
        myAdjacentCell = myLeftBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l< myAdjacentCell) && (myLeftBucket > 0 ) && (myTopBucket <= myBucketsPerRow)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the bottom left
        myAdjacentCell = myLeftBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myLeftBucket > 0 ) && (myBottomBucket > 0)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        curBucket = curBucket + 1;
    }
}

void NodeLinkedList::CompareAdjacentCells(int cell1, int cell2) {
    double dist;
    int startNode;
    int endNode;

    for (int i = 0; i < (myBuckets[cell1].GetNumberInCell()) ; ++i) {

        startNode = myBuckets[cell1].GetNodeInBucket(i);

        // if the starting node is the sink node, then we only need to worry about connecting to sensor nodes, which is based on communication range
        if (startNode == 0) {
            for (int j = 0; j < myBuckets[cell2].GetNumberInCell() ; ++j) {
                endNode = myBuckets[cell2].GetNodeInBucket(j);
                // determine if end node is a sensor node or a target node
                if (endNode < myNumSensors){
                    dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                    if (dist <= myCommunicationRange) {
                        myBuckets[startNode].AddReachableNode(endNode);
                    }
                }
            }
        }
            // if starting node is a sensor node
        else if (startNode > 0 && startNode < myNumSensors){

            for (int j = 0; j < myBuckets[cell2].GetNumberInCell() ; ++j) {
                endNode = myBuckets[cell2].GetNodeInBucket(j);

                if (endNode == 0){
                    // start node is a sensor node, end node is sink node, we check to add arc since nodes are in different grid cell
                    dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                    // sensor to sink comparison, we can arc from sink to sensor node
                    if (dist <= myCommunicationRange) {
                        myBuckets[endNode].AddReachableNode(startNode);
                    }
                }
                else if (endNode < myNumSensors){
                    dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                    // sensor to sensor comparison, we can add both arcs
                    if (dist <= myCommunicationRange) {
                        myBuckets[startNode].AddReachableNode(endNode);
                        myBuckets[endNode].AddReachableNode(startNode);
                    }
                }
                else {
                    // end node must be a target node
                    dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                    if (dist <= mySensorRange){
                        myBuckets[startNode].AddReachableNode(endNode);
                    }
                }
            }
        }
            // starting node is a target node, but we still need to check for sensor to target arcs
        else {
            for (int j = 0; j < myBuckets[cell2].GetNumberInCell() ; ++j) {
                endNode = myBuckets[cell2].GetNodeInBucket(j);

                if ((endNode == 0) || (endNode >= myNumSensors)){
                    // start node is target, end node is either a target or sink node, we skip
                }
                else{
                    // end node must be a sensor node
                    dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                    if (dist <= mySensorRange){
                        myBuckets[endNode].AddReachableNode(startNode);
                    }
                }
            }
        }
    }
}

// used to generate new fail times
void NodeLinkedList::GenerateNewFailOrder(vector<int> &nodeTime){

    // TODO change default random engine to a different generator
    //    default_random_engine myGen;
    //    mersenne_twister_engine myMersenne();

    nodeTime.clear();
    nodeTime.reserve(myNumElements - 1);

    for (int j = 0; j < (myNumSensors - 1) ; ++j) {
        nodeTime.push_back(j + 1);
    }

    for (int j = 0; j < (myNumSensors - 1) ; ++j) {
        // generate random number
        uniform_int_distribution<int> myDist(0, myNumSensors - j - 2);
        int myNum = myDist(myDevOrder);

        // store the last number that is not marked
        int tempFirst = nodeTime.at(myNum);
        // element to swap
        int tempLast = nodeTime.at(myNumSensors - j - 2);

        // swap elements
        nodeTime[myNum] = tempLast;
        nodeTime[myNumSensors - j - 2] = tempFirst;

    }

    // set fail time for sink
    nodeTime.insert(nodeTime.begin(), myNumSensors);

    // initial time for all targets
    for (int j = 0; j < myNumTargets ; ++j) {
        nodeTime.push_back(myNumSensors);
    }
}

// NOTE : newSize includes the sink node!!!! This outputs to the command line
// Used to evaluate the stable network reliability for a single network of different size
void NodeLinkedList::SignatureRelation(int newSize, double t, double maxDelta, double costFixed, double costVar, double myShape, double myScale) {

    vector<int> newFailed;
    vector<double> newProb;

    int sizeDiff = myNumSensors - newSize;

    if (sizeDiff < 0) {
        cout << " The new network size is larger than initial network" << endl;
    }
    else {

        int firstFail = myDSpec.at(0);

        if ((firstFail - sizeDiff) <= 0){
            newFailed.push_back(0);
            newProb.push_back(myDSpecProb.at(0));
        }

        for (int i = 1; i < myDSpec.size(); ++i) {
            if ((myDSpec.at(i) - sizeDiff) <= 0){
                newProb[0] = newProb[0] + myDSpecProb.at(i);
            }
            else{
                newFailed.push_back(myDSpec.at(i) - sizeDiff);
                newProb.push_back(myDSpecProb.at(i));
            }
        }
    }

    double i = 1.0;
    while (i < maxDelta){
        cout << "MaintenanceRel " << newSize << " delta = " << i << " -- " << StableMaintenanceReliabilityWeibull(i, newSize - 1, newFailed, newProb, myShape, myScale) << " -- " << MaintenancePolicyCostWeibull(i, costFixed, costVar, newSize - 1, myShape, myScale) << endl;
        i = i + 0.1;
    }
}

// outputs the maintenance reliability and cost to a txt file
// Used to evaluate the stable network reliability for a range of different network sizes (from current size down to minSize)
void NodeLinkedList::SignatureRelationTBM_Output(int minSize, double t, double maxDelta, double costFixed,
                                                 double costVar, double myShape, double myScale){

    ofstream myFile;
    myFile.open ("MaintReliabilityTBM.txt");

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

            int firstFail = myDSpec.at(0);

            if ((firstFail - sizeDiff) <= 0){
                newFailed.push_back(0);
                newProb.push_back(myDSpecProb.at(0));
            }

            for (int i = 1; i < myDSpec.size(); ++i) {
                if ((myDSpec.at(i) - sizeDiff) <= 0){
                    newProb[0] = newProb[0] + myDSpecProb.at(i);
                }
                else{
                    newFailed.push_back(myDSpec.at(i) - sizeDiff);
                    newProb.push_back(myDSpecProb.at(i));
                }
            }
        }

        double i = 1.0;
        while (i < maxDelta){
            myFile << "MaintenanceRel " << currNum << " delta = " << i << " -- " << StableMaintenanceReliabilityWeibull(i, currNum - 1, newFailed, newProb, myShape, myScale) << " -- " << MaintenancePolicyCostWeibull(i, costFixed, costVar, currNum - 1, myShape, myScale) << endl;
            i = i + 0.1;
        }

        currNum = currNum - 1;
    }

    myFile.close();
}

// outputs the maintenance reliability and cost to a given txt file
// Used to evaluate the stable network reliability for a range of different network sizes (from current size down to minSize)
void NodeLinkedList::SignatureRelationTBM_Output(int minSize, double t, double maxDelta, double costFixed,
                                                 double costVar, double myShape, double myScale, ofstream &outputFile){


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

            int firstFail = myDSpec.at(0);

            if ((firstFail - sizeDiff) <= 0){
                newFailed.push_back(0);
                newProb.push_back(myDSpecProb.at(0));
            }

            for (int i = 1; i < myDSpec.size(); ++i) {
                if ((myDSpec.at(i) - sizeDiff) <= 0){
                    newProb[0] = newProb[0] + myDSpecProb.at(i);
                }
                else{
                    newFailed.push_back(myDSpec.at(i) - sizeDiff);
                    newProb.push_back(myDSpecProb.at(i));
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

// outputs the maintenance reliability and cost to a txt file
void NodeLinkedList::SignatureRelationVariance(int minSize, int numReps, double t, double maxDelta, double costFixed, double costVar,
                                               double myShape, double myScale){

    ofstream myFile;
    myFile.open ("MaintReliabilityVar.txt");

    int currNum = myNumSensors;

    while (currNum >= minSize){
        // + 1 to include the sink node
        vector<int> newFailed;
        vector<double> newProb;

        int sizeDiff = myNumSensors - currNum;

        if (sizeDiff < 0) {
            cout << " The new network size is larger than initial network" << endl;
        }
        else {

            int firstFail = myDSpec.at(0);

            if ((firstFail - sizeDiff) <= 0){
                newFailed.push_back(0);
                newProb.push_back(myDSpecProb.at(0));
            }

            for (int i = 1; i < myDSpec.size(); ++i) {
                if ((myDSpec.at(i) - sizeDiff) <= 0){
                    newProb[0] = newProb[0] + myDSpecProb.at(i);
                }
                else{
                    newFailed.push_back(myDSpec.at(i) - sizeDiff);
                    newProb.push_back(myDSpecProb.at(i));
                }
            }
        }

        double i = 1.0;
        while (i < maxDelta){
            myFile << "MaintenanceVar " << currNum << " delta = " << i << " -- " << StableMaintenanceVarianceWeibull(i, currNum - 1, newFailed, newProb, numReps, myShape, myScale) << endl;
            i = i + 0.1;
        }

        currNum = currNum - 100;
    }

    myFile.close();
}

// Uses the destruction spectrum estimate to evaluate a CBM policy based on the number of failed sensors in the network
void NodeLinkedList::SignatureRelationCBM_Output(int minSize, double costFixed, double costVar, double myShape, double myScale){

    ofstream myFile;
    myFile.open ("MaintReliabilityCBM.txt");

    int currNum = myNumSensors;
    while (currNum >= minSize) {
        // + 1 to include the sink node
        vector<int> newFailed;
        vector<double> newProb;

        int sizeDiff = myNumSensors - currNum;

        if (sizeDiff < 0) {
            cout << " The new network size is larger than initial network" << endl;
        }
        else {

            int firstFail = myDSpec.at(0);

            if ((firstFail - sizeDiff) <= 0){
                newFailed.push_back(0);
                newProb.push_back(myDSpecProb.at(0));
            }

            for (int i = 1; i < myDSpec.size(); ++i) {
                if ((myDSpec.at(i) - sizeDiff) <= 0){
                    newProb[0] = newProb[0] + myDSpecProb.at(i);
                }
                else{
                    newFailed.push_back(myDSpec.at(i) - sizeDiff);
                    newProb.push_back(myDSpecProb.at(i));
                }
            }
        }

        double minRel = 0.5;
        double curRel = 1.0;
        double cost;
        double cbmRel;
        int condition = 1;

        while ((curRel > minRel) && (condition < currNum)){

            double minGuess = 0.01;
            double maxGuess = 15.0;
            double currGess;

            bool dSpec = true;
            int i = 0;
            double cumulativeSignature = 0;

            while (dSpec && (i < newFailed.size())){
                if (newFailed.at(i) <= condition){
                    cumulativeSignature = cumulativeSignature + newProb.at(i);
                    i = i + 1;
                }
                else {
                    dSpec = false;
                }
            }

            cost = costFixed + (costVar * condition);
            cbmRel = 1 - cumulativeSignature;
            curRel = cbmRel;

            // estimate delta
            bool deltaNotFound = true;
            double estFailed;

            while (deltaNotFound){

                currGess = (minGuess + maxGuess) / 2.0;
                estFailed = StableLifeResidualProbWeibull(currGess, myShape, myScale) * (currNum - 1);

                if (abs((estFailed - (double) condition)) < 0.0001){
                    deltaNotFound = false;
                }
                else if(estFailed > condition){
                    maxGuess = currGess;
                }
                else if(estFailed < condition){
                    minGuess = currGess;
                }
            }

            myFile << "CBMMaintenanceRel " << currNum << " Condition = " << condition << " -- " << cbmRel << " -- " << cost << " ApproxDelta : " << currGess << endl;

            condition = condition + 1;

        }

        currNum = currNum - 1;
    }

    myFile.close();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

// Uniformly locate target nodes over network
void NodeLinkedList::UniformTargetLoc() {

    double n = sqrt(myNumTargets);

    double dist = 1 / (n - 1);

    Node *ptr = myBuckets;
    ptr = ptr + myNumSensors;

    for (int i = 0; i < n ; ++i) {
        for (int j = 0; j < n ; ++j) {
            ptr->SetNewLoc(dist * (double) i, dist * (double) j);
            ptr = ptr + 1;
        }
    }
}

void NodeLinkedList::SetRanges(double comRange, double sensingRange) {
    myCommunicationRange = comRange;
    mySensorRange = sensingRange;
}


// Used to partition the region into certain bins for the purpose of determining the density of functioning sensors in a subregion
void NodeLinkedList::SetBinLimits(int numRegionsPerRow) {
//    int myRegionsPerRow = (int) ceil(1 / myCommunicationRange);
//    double myRegionWidth = 1 / (double) myBucketsPerRow;

    int myRegionsPerRow = numRegionsPerRow;
    myNumSubregions = numRegionsPerRow * numRegionsPerRow;
    double myRegionWidth = 1 / (double) myRegionsPerRow;

    int bucketIndex;

    double tempXLow;
    double tempXHigh;
    double tempYLow;
    double tempYHigh;

    for (int i = 1; i <= myRegionsPerRow; ++i) {
        for (int j = 1; j <= myRegionsPerRow; ++j) {
            tempXLow = (double) (j-1) * myRegionWidth;
            tempXHigh = (double)j * myRegionWidth;

            tempYLow = (double)(i-1) * myRegionWidth;
            tempYHigh = (double)i * myRegionWidth;

            bucketIndex = j + (i * myRegionsPerRow) - (myRegionsPerRow + 1);

            myBuckets[bucketIndex].SetXLimits(tempXLow, tempXHigh);
            myBuckets[bucketIndex].SetYLimits(tempYLow, tempYHigh);

//            cout << " Bin : " << bucketIndex << " -- " << tempXLow << " , " << tempXHigh << " -- " << tempYLow << " , " << tempYHigh << endl;

        }
    }
}

// Used to partition the region into certain bins for the purpose of determining the density of functioning sensors in a subregion
void NodeLinkedList::CustomBins(){

    myNumSubregions = 16;

    myBuckets[0].SetXLimits(0,0.3);
    myBuckets[0].SetYLimits(0,0.3);

    myBuckets[1].SetXLimits(0.3,0.5);
    myBuckets[1].SetYLimits(0,0.3);

    myBuckets[2].SetXLimits(0.5,0.7);
    myBuckets[2].SetYLimits(0,0.3);

    myBuckets[3].SetXLimits(0.7,1.0);
    myBuckets[3].SetYLimits(0,0.3);

    myBuckets[4].SetXLimits(0,0.3);
    myBuckets[4].SetYLimits(0.3,0.5);

    myBuckets[5].SetXLimits(0.3,0.5);
    myBuckets[5].SetYLimits(0.3,0.5);

    myBuckets[6].SetXLimits(0.5,0.7);
    myBuckets[6].SetYLimits(0.3,0.5);

    myBuckets[7].SetXLimits(0.7,1.0);
    myBuckets[7].SetYLimits(0.3,0.5);

    myBuckets[8].SetXLimits(0,0.3);
    myBuckets[8].SetYLimits(0.5,0.7);

    myBuckets[9].SetXLimits(0.3,0.5);
    myBuckets[9].SetYLimits(0.5,0.7);

    myBuckets[10].SetXLimits(0.5,0.7);
    myBuckets[10].SetYLimits(0.5,0.7);

    myBuckets[11].SetXLimits(0.7,1.0);
    myBuckets[11].SetYLimits(0.5,0.7);

    myBuckets[12].SetXLimits(0,0.3);
    myBuckets[12].SetYLimits(0.7,1.0);

    myBuckets[13].SetXLimits(0.3,0.5);
    myBuckets[13].SetYLimits(0.7,1.0);

    myBuckets[14].SetXLimits(0.5,0.7);
    myBuckets[14].SetYLimits(0.7,1.0);

    myBuckets[15].SetXLimits(0.7,1.0);
    myBuckets[15].SetYLimits(0.7,1.0);

}

void NodeLinkedList::SubregionArea(vector<double> &areaVect) {

    for (int i = 0; i < myNumSubregions ; ++i) {
        double xLow = myBuckets[i].GetXLow();
        double xHigh = myBuckets[i].GetXHigh();

        double yLow = myBuckets[i].GetYLow();
        double yHigh = myBuckets[i].GetYHigh();

        double area = (xHigh - xLow) * (yHigh - yLow);
        areaVect.push_back(area);
    }
}

void NodeLinkedList::SubregionDistanceWeight(vector<double> &subWeights){

    for (int i = 0; i < myNumSubregions ; ++i) {
        double xLow = myBuckets[i].GetXLow();
        double xHigh = myBuckets[i].GetXHigh();

        double yLow = myBuckets[i].GetYLow();
        double yHigh = myBuckets[i].GetYHigh();

        double dist = pow(((xLow + xHigh) / 2.0) - 0.5, 2) + pow(((yLow + yHigh) / 2.0) - 0.5, 2);
        dist = pow(dist, 0.5);
        subWeights.push_back(1.0 / dist);

    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                         Node Initialization Section
/////////////////////////////////////////////////////////////////////////////////////////////////////////

Node::Node() {
    myValue = 0;
    myID = 0;
    myMaxCapacityPath = 0;
    prev = nullptr;
    next = nullptr;
}

Node::Node(int inputID){
    myValue = 0;
    myID = inputID;
    myMaxCapacityPath = 0;
    prev = nullptr;
    next = nullptr;
}

// Node adjacency list
void Node::AddReachableNode(int nodeR){
    nodeDegree = nodeDegree + 1;
    forwardStar.push_back(nodeR);
}

int Node::GetReachableNode(int i){
    return forwardStar[i];
}

// Identifier for node
void Node::SetID(int temp){
    myID = temp;
}

int Node::GetNodeID(){
    return  myID;
}

// value associated with node
int Node::GetValue() {
    return myValue;
}

void Node::SetValue(int data){
    myValue = data;
}

void Node::UpdateCapacityLabel(int newCapacity){
    myMaxCapacityPath = newCapacity;
}

int Node::GetMaxCapacityPath(){
    return myMaxCapacityPath;
}

void Node::SetPrev(Node *newPrev){
    prev = newPrev;
}

void Node::SetNext(Node *newNext){
    next = newNext;
}

void Node::SetFirstItem(Node *firstElement){
    firstInBucket = firstElement;
}

void Node::UpdateReached(){
    isReached = true;
}

bool Node::IsReached(){
    return isReached;
}

void Node::MarkPermanent(){
    isPermanent = true;
}

bool Node::IsPermanent(){
    return isPermanent;
}

void Node::ResetNodeReached(){
    isReached = false;
}
void Node::ResetNodePermanent(){
    isPermanent = false;
}

int Node::GetDegree(){
    return nodeDegree;
}

// reset the forward star of a node
void Node::ResetReachableNodes(){
    forwardStar.clear();
    nodeDegree = 0;
}

Node *Node::GetPrev(){
    return prev;
}

Node *Node::GetNext(){
    return next;
}

Node *Node::GetFirstElement(){
    return firstInBucket;
}

void Node::SetNewLoc(double xLoc, double yLoc) {
    myXLoc = xLoc;
    myYLoc = yLoc;
}

double Node::GetXLoc() {
    return myXLoc;
}

double Node::GetYLoc() {
    return myYLoc;
}

void Node::SetSubregion(int region) {
    mySubregionLoc = region;
}

int Node::GetSubregion() {
    return mySubregionLoc;
}

void Node::ResetNodesInBucket() {
    nodesInBucket.clear();
    myNodesInBucket = 0;
}

void Node::AddNodeToBucket(int nodeNew) {
    nodesInBucket.push_back(nodeNew);
    myNodesInBucket = myNodesInBucket + 1;
}

int Node::GetNodeInBucket(int i) {
    return nodesInBucket[i];
}

int Node::GetNumberInCell() {
    return myNodesInBucket;
}

bool Node::IsCovered() {
    return isCovered;
}

void Node::SetCovered() {
    isCovered = true;
}

void Node::ResetCovered() {
    isCovered = false;
}

bool Node::IsFailed() {
    return isFailed;
}

void Node::SetFailed() {
    isFailed = true;
}

void Node::ResetFailed(){
    isFailed = false;
}

void Node::SetXLimits(double low, double high) {
    myXLower = low;
    myXHigher = high;
}

void Node::SetYLimits(double low, double high) {
    myYLower = low;
    myYHigher = high;
}

double Node::GetXLow() {
    return myXLower;
}

double Node::GetXHigh(){
    return myXHigher;
}

double Node::GetYLow() {
    return myYLower;
}

double Node::GetYHigh(){
    return myYHigher;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                         MC section
/////////////////////////////////////////////////////////////////////////////////////////////////////////

SimulationNodeMC::SimulationNodeMC() {
    this->SetValue(0);
    this->SetID(0);
    this->UpdateFailCapacity(0);
}

NodeLinkedListMC::NodeLinkedListMC(int numItems, int numSens, int numTarg){
    firstItem = nullptr;
    lastItem = nullptr;
    myNumElements = numItems;   // total number of nodes
    myNumSensors = numSens;     // number of sensors (including sink)
    myNumTargets = numTarg;     // number of targets
    InitializeBuckets(numItems);
}


NodeLinkedListMC::NodeLinkedListMC(int numItems, int numSens, int numTarg, vector<double> failTimes){
    firstItem = nullptr;
    lastItem = nullptr;
    myNumElements = numItems;   // total number of nodes
    myNumSensors = numSens;     // number of sensors (including sink)
    myNumTargets = numTarg;     // number of targets
    InitializeBuckets(numItems, failTimes);
}

void NodeLinkedListMC::InitializeBuckets(int numItems){

    myBuckets = new SimulationNodeMC[numItems];

    SimulationNodeMC *ptr = myBuckets;
    // update all values for the sink node
    ptr->SetID(0);
    ptr->UpdateReached();
    ptr->MarkPermanent();
    ptr->SetFirstItem((ptr + 1));

    ptr = ptr + 1;

    // sets the pointers for a node to be the node immediately prior and immediately after the node
    // values for sensor nodes
    for (int i = 1; i < myNumSensors ; ++i) {
        myBuckets[i].SetID(i);
        myBuckets[i].SetNext((ptr + 1));
        myBuckets[i].SetPrev((ptr - 1));
        myBuckets[i].SetFirstItem(nullptr);

        ptr = ptr + 1;

    }

    // values for target nodes
    for (int j = myNumSensors ; j < numItems ; ++j) {

        myBuckets[j].SetID(j);
        myBuckets[j].SetNext((ptr + 1));
        myBuckets[j].SetFirstItem(nullptr);
        ptr = ptr + 1;

    }
    myBuckets[numItems-1].SetNext(nullptr);
}

void NodeLinkedListMC::InitializeBuckets(int numItems,  vector<double> failTimes){

    double maxFail = *max_element(failTimes.begin(), failTimes.end());

//    max_element(failTimes.begin(), failTimes.end());

    myBuckets = new SimulationNodeMC[numItems];

    SimulationNodeMC *ptr = myBuckets;
    // update all values for the sink node
    ptr->SetID(0);
    ptr->UpdateFailCapacity(maxFail);
    ptr->UpdateReached();
    ptr->MarkPermanent();
    ptr->SetFailTime(failTimes[0]);
    ptr->SetFirstItem((ptr + 1));

    ptr = ptr + 1;

    // sets the pointers for a node to be the node immediately prior and immediately after the node
    // values for sensor nodes
    for (int i = 1; i < myNumSensors ; ++i) {
        myBuckets[i].SetID(i);
        myBuckets[i].SetNext((ptr + 1));
        myBuckets[i].SetPrev((ptr - 1));
        myBuckets[i].SetFailTime(failTimes[i]);

        myBuckets[i].SetFirstItem(nullptr);

        ptr = ptr + 1;

    }

    // values for target nodes
    for (int j = myNumSensors ; j < numItems ; ++j) {

        myBuckets[j].SetID(j);
        myBuckets[j].SetNext((ptr + 1));
        myBuckets[j].SetPrev((ptr - 1));
        myBuckets[j].SetFailTime(maxFail);
        myBuckets[j].SetFirstItem(nullptr);

        ptr = ptr + 1;

    }
    myBuckets[numItems-1].SetNext(nullptr);
}

void SimulationNodeMC::ResetNode(){
    isFailed = false;
    isCovered = false;
}

int SimulationNodeMC::GetNodeID(){
    return  myID;
}

bool SimulationNodeMC::IsCovered() {
    return isCovered;
}

void SimulationNodeMC::SetCovered() {
    isCovered = true;
}

void SimulationNodeMC::ResetCovered() {
    isCovered = false;
}

bool SimulationNodeMC::IsFailed() {
    return isFailed;
}

void SimulationNodeMC::SetFailed() {
    isFailed = true;
}

void SimulationNodeMC::ResetFailed() {
    isFailed = false;
}

void SimulationNodeMC::AddNodeToBucket(int nodeNew) {
    nodesInBucket.push_back(nodeNew);
    myNodesInBucket = myNodesInBucket + 1;
}

int SimulationNodeMC::GetNodeInBucket(int i) {
    return nodesInBucket[i];
}

int SimulationNodeMC::GetNumberInCell() {
    return myNodesInBucket;
}

double SimulationNodeMC::GetXLoc() {
    return myXLoc;
}

double SimulationNodeMC::GetYLoc() {
    return myYLoc;
}

// reset the forward star of a node
void SimulationNodeMC::ResetReachableNodes(){
    forwardStar.clear();
    nodeDegree = 0;
}

void SimulationNodeMC::ResetNodesInBucket() {
    nodesInBucket.clear();
    myNodesInBucket = 0;
}

double SimulationNodeMC::GetFailTime() {
    return myFailTime;
}

void SimulationNodeMC::SetNewLoc(double xLoc, double yLoc) {
    myXLoc = xLoc;
    myYLoc = yLoc;
}

void SimulationNodeMC::SetFailTime(double t) {
    myFailTime = t;
}

void SimulationNodeMC::UpdateFailCapacity(double newTime) {
    myCapacityFailTime = newTime;
}

double SimulationNodeMC::GetFailCapacity() {
    return myCapacityFailTime;
}

void SimulationNodeMC::UpdateForwardStar(int removeNum) {
    forwardStar.erase(forwardStar.begin() + removeNum);
    nodeDegree = nodeDegree - 1;
}

bool SimulationNodeMC::IsReplaced(){
    return isReplaced;
}

void SimulationNodeMC::SetReplaced(){
    isReplaced = true;
}

void SimulationNodeMC::ResetReplaced(){
    isReplaced = false;
}

void SimulationNodeMC::SetSubregion(int region) {
    mySubregionLoc = region;
}

void SimulationNodeMC::SetAge(double age) {
    myAge = age;
}

double SimulationNodeMC::GetAge() {
    return myAge;
}

void SimulationNodeMC::IncreaseAge() {
    myAge = myAge + 1.0;
}

void SimulationNodeMC::SetActive() {
    isActive = true;
}

void SimulationNodeMC::SetInactive() {
    isActive = false;
}

bool SimulationNodeMC::IsActive() {
    return isActive;
}

/////////////////////////////////////////////////////////////////////////////////
// NodeLinkedList Operations
/////////////////////////////////////////////////////////////////////////////////

// used to reinitialize buckets for a new test instance
void NodeLinkedListMC::UpdateBucketTimes(vector<int> newOrder){

    this->ClearOrderTimes();

    SimulationNodeMC *ptr = myBuckets;
    // update all values for the sink node
    ptr->SetID(0);
    ptr->UpdateCapacityLabel(myNumElements);
    ptr->UpdateReached();
    ptr->MarkPermanent();
    ptr->SetValue(newOrder[0]);
    ptr->SetFirstItem((ptr + 1));

    ptr = ptr + 1;

    // sets the pointers for a node to be the node immediately prior and immediately after the node
    for (int i = 1; i < myNumSensors ; ++i) {

        // reset initial status
        myBuckets[i].ResetNodeReached();
        myBuckets[i].ResetNodePermanent();
        myBuckets[i].UpdateCapacityLabel(0);

        // re-initialize
        myBuckets[i].SetNext((ptr + 1));
        myBuckets[i].SetPrev((ptr - 1));
        myBuckets[newOrder[i]].SetValue(i);

        myBuckets[i].SetFirstItem(nullptr);

        ptr = ptr + 1;

    }

    // Update value for target nodes
    for (int j = myNumSensors ; j < myNumElements ; ++j) {

        // reset initial status
        myBuckets[j].ResetNodeReached();
        myBuckets[j].ResetNodePermanent();
        myBuckets[j].UpdateCapacityLabel(0);

        myBuckets[j].SetID(j);
        myBuckets[j].SetNext((ptr + 1));
        myBuckets[j].SetPrev((ptr - 1));
        myBuckets[j].SetValue(myNumElements);

        myBuckets[j].SetFirstItem(nullptr);

        ptr = ptr + 1;

    }

    myBuckets[myNumElements-1].SetNext(nullptr);
}

void NodeLinkedListMC::DijkstrasAlgorithm(vector<Edge*> edgeList, int numNodes, int numTar){
    double mTime = 0;
    this->DijkstrasAlgorithm(edgeList, numNodes, numTar, mTime);

}

// used for a monte carlo simulation of network reliability
void NodeLinkedListMC::DijkstrasAlgorithm(vector<Edge *> edgeList, int numNodes, int numTar, double &timeDuration) {

    // sink node is already permanent
    int numPermanent = 1;

    // vector to store ID's of temporary nodes
    vector<int> tempNodes;
    for (int i = 1; i <= (numNodes-1) ; ++i) {
        tempNodes.push_back(i);
    }

    // start time record
    auto start = chrono::high_resolution_clock::now();

    // update all nodes connected to sink
    for (int i = 0; i < myBuckets[0].GetDegree(); ++i) {
        int updateNode1 = myBuckets[0].GetReachableNode(i);
        double updateLabel = min(myBuckets[0].GetFailCapacity(), myBuckets[updateNode1].GetFailTime());
        myBuckets[updateNode1].UpdateFailCapacity(updateLabel);
    }

    while (numPermanent < numNodes) {

        int tempNode;
        int tempIndex = -1;
        double currMax = 0;
        int permNode = 0;

        // find the node with the largest distance label that is not permanent
        for (int j = 0; j < tempNodes.size(); ++j) {
            tempNode = tempNodes.at(j);
            if (myBuckets[tempNode].GetFailCapacity() > currMax) {
                // store node ID and max label

                currMax = myBuckets[tempNode].GetFailCapacity();
                permNode = myBuckets[tempNode].GetNodeID();
                tempIndex = j;
            }
        }

        if (tempIndex == -1) {
            // node not connected - should be able to break here
        } else {

            // remove from temporary nodes vector
            tempNodes.erase(tempNodes.begin() + tempIndex);

            // mark node permanent
            myBuckets[permNode].MarkPermanent();

            this->addFailOrder(permNode);
            this->addFailTime(currMax);

            // stores the failure order and time at which Targets are disconnected
            if (myBuckets[permNode].GetNodeID() >= numNodes - numTar) {
                this->addTargetFailOrder(myBuckets[permNode].GetNodeID());
                this->addTargetFailTime(myBuckets[permNode].GetFailCapacity());

                myBuckets[permNode].SetCovered();
            }
            else {
                this->addSensorFailOrder(myBuckets[permNode].GetNodeID());
                this->addSensorFailTime(myBuckets[permNode].GetFailCapacity());
            }

            // iterate through edges
            for (int k = 0; k < myBuckets[permNode].GetDegree(); ++k) {

                int updateNode1 = myBuckets[permNode].GetReachableNode(k);
                if (!myBuckets[updateNode1].IsPermanent()) {
                    double updateLabel = min(myBuckets[permNode].GetFailCapacity(), myBuckets[updateNode1].GetFailTime());
                    myBuckets[updateNode1].UpdateFailCapacity(updateLabel);
                }
            }
        }
        numPermanent = numPermanent + 1;
    }

    // If a target is not connected to the sink then it will never be marked permanent, so it will never be added to the
    // order of target failures. This for loop checks for such a case, and adds the targets to ths list with a fail time of zero
    if (orderedTargetFail.size() < numTar){
        SimulationNodeMC *ptr = myBuckets;
        ptr = ptr + (numNodes - numTar);
        for (int i = 0; i < numTar ; ++i) {
            if (!ptr->IsPermanent()){
                this->addTargetFailOrder(ptr->GetNodeID());
                this->addTargetFailTime(ptr->GetFailCapacity());
            }
            ptr = ptr + 1;
        }
    }

    // end time record
    auto finish = chrono::high_resolution_clock::now();
    // calculate execution time
    chrono::duration<double> elapsed = finish - start;
    timeDuration = elapsed.count();
}

// used to generate new fail times for transient reliability
void NodeLinkedListMC::GenerateNewFailTime(vector<double> &nodeTime){

    nodeTime.clear();
    nodeTime.reserve(myNumElements - 1);

    weibull_distribution<double> myWeibull(1.5,10.0);

    for (int i = 0; i < myNumSensors - 1 ; ++i) {
        double myTime = myWeibull(myDevTime);
        nodeTime.push_back(myTime);
    }

    // set fail time for sink
    double maxFail = *max_element(nodeTime.begin(), nodeTime.end());
    nodeTime.insert(nodeTime.begin(), maxFail);

    // initial time for all targets
    for (int j = 0; j < myNumTargets ; ++j) {
        nodeTime.push_back(maxFail);
    }
}

// used to generate new fail times for transient reliability
void NodeLinkedListMC::GenerateNewFailTime_ADPEval(vector<double> &nodeTime, int numDeployed){

    nodeTime.clear();
    nodeTime.reserve(myNumElements - 1);

    weibull_distribution<double> myWeibull(1.5,10.0);

    for (int i = 0; i < numDeployed; ++i){
        double myTime = myWeibull(myDevTime);
        nodeTime.push_back(myTime);
    }

    for (int i = numDeployed; i < myNumSensors - 1 ; ++i) {
        nodeTime.push_back(0);
    }

    // set fail time for sink
    double maxFail = *max_element(nodeTime.begin(), nodeTime.end());
    nodeTime.insert(nodeTime.begin(), maxFail);

    // initial time for all targets
    for (int j = 0; j < myNumTargets ; ++j) {
        nodeTime.push_back(maxFail + 100.0);
    }
}

// used to generate new fail times for transient reliability
void NodeLinkedListMC::GenerateNewFailTime_ADPEval(vector<double> &nodeTime, int numDeployed, double maxTime){

    nodeTime.clear();
    nodeTime.reserve(myNumElements - 1);

    weibull_distribution<double> myWeibull(1.5,10.0);

    for (int i = 0; i < numDeployed; ++i){
        double myTime = myWeibull(myDevTime);
        nodeTime.push_back(myTime);
    }

    for (int i = numDeployed; i < myNumSensors - 1 ; ++i) {
        nodeTime.push_back(0);
    }

    nodeTime.insert(nodeTime.begin(), maxTime);

    // initial time for all targets
    for (int j = 0; j < myNumTargets ; ++j) {
        nodeTime.push_back(maxTime);
    }
}

// used to generate new fail times for a maintenance policy, where new sensors are deployed at time curTime
void NodeLinkedListMC::GenerateNewFailTimeMaintenance(vector<double> &nodeTime, double t, double curTime){

    weibull_distribution<double> myWeibull(1.5,10.0);

    for (int i = 1; i < myNumSensors; ++i) {

        if (myBuckets[i].IsFailed()){
            double myTime = myWeibull(myDevTime);
            nodeTime[i] = myTime + curTime;
        }
    }

    nodeTime[0] = t;

    // initial time for all targets
    for (int j = myNumSensors; j < myNumElements; ++j) {
        nodeTime[j] = t;
    }
}

void NodeLinkedListMC::UpdateFailTimes(vector<double> newTimes){

    orderedCriticalFailTime.clear();
    orderedTargetCriticalFailTime.clear();
    orderedSensorCriticalFailTime.clear();

    this->ClearOrderTimes();

    double maxFail = *max_element(newTimes.begin(), newTimes.end());

    SimulationNodeMC *ptr = myBuckets;
    // update all values for the sink node
    ptr->SetID(0);
    ptr->UpdateFailCapacity(maxFail);
    ptr->UpdateReached();
    ptr->MarkPermanent();
    ptr->SetFailTime(maxFail);

    ptr->SetFirstItem((ptr + 1));

    ptr = ptr + 1;

    // sets the pointers for a node to be the node immediately prior and immediately after the node
    for (int i = 1; i < myNumSensors ; ++i) {

        // reset initial status
        myBuckets[i].ResetNodeReached();
        myBuckets[i].ResetNodePermanent();
        myBuckets[i].UpdateFailCapacity(0);

        myBuckets[i].ResetCovered();

        // re-initialize
        myBuckets[i].SetNext((ptr + 1));
        myBuckets[i].SetPrev((ptr - 1));
        myBuckets[i].SetFailTime(newTimes[i]);

        myBuckets[i].SetFirstItem(nullptr);

        ptr = ptr + 1;

    }

    // Update value for target nodes
    for (int j = myNumSensors ; j < myNumElements ; ++j) {

        // reset initial status
        myBuckets[j].ResetNodeReached();
        myBuckets[j].ResetNodePermanent();
        myBuckets[j].UpdateFailCapacity(0);

        myBuckets[j].ResetCovered();

        myBuckets[j].SetID(j);
        myBuckets[j].SetNext((ptr + 1));
        myBuckets[j].SetPrev((ptr - 1));
        myBuckets[j].SetFailTime(newTimes[j]);

        myBuckets[j].SetFirstItem(nullptr);

        ptr = ptr + 1;

    }

    myBuckets[myNumElements-1].SetNext(nullptr);
}

void NodeLinkedListMC::addFailTime(double failTime){
    orderedCriticalFailTime.push_back(failTime);
}

// vector that stores the ordered failed times for targets
void NodeLinkedListMC::addTargetFailTime(double failTime){
    orderedTargetCriticalFailTime.push_back(failTime);
}

// vector that stores the ordered fail time for sensors
void NodeLinkedListMC::addSensorFailTime(double failTime){
    orderedSensorCriticalFailTime.push_back(failTime);
}

// NOTE: the list is in reverse order (largest to smallest)
double NodeLinkedListMC::GetTargetFailTime(int failNum){
    return orderedTargetCriticalFailTime.at(orderedTargetCriticalFailTime.size() - failNum);
}

void NodeLinkedListMC::addNetworkFailTime(double failTime) {
    networkFailTimes.push_back(failTime);
}

void NodeLinkedListMC::CalculateReliability(double t, int numReps) {

    double i = 0;
    while (i < t) {

        int numFailed = 0;

        for (int j = 0; j < networkFailTimes.size(); ++j) {

            if (networkFailTimes.at(j) <= i){
                numFailed = numFailed + 1;
            }

        }

        double rel = 1.0 - ((double) numFailed / (double) numReps);

        cout << "Reliability " << myNumSensors << " t = " << i << " -- " << rel << endl;
        i = i + 0.2;
    }
}

// Uniformly locate target nodes over network
void NodeLinkedListMC::UniformTargetLoc() {

    double n = sqrt(myNumTargets);

    double dist = 1 / (n - 1);

    SimulationNodeMC *ptr = myBuckets;
    ptr = ptr + myNumSensors;

    for (int i = 0; i < n ; ++i) {
        for (int j = 0; j < n ; ++j) {
            ptr->SetNewLoc(dist * (double) i, dist * (double) j);
            ptr = ptr + 1;
        }
    }
}

void NodeLinkedListMC::GenerateNewRGG_Bucket(){

//    random_device myDev;
    uniform_real_distribution<> myUnif(0,1);

    double myXNum;
    double myYNum;
    int myTempBin;

    int myBucketsPerRow = (int) ceil(1 / myCommunicationRange);

    SimulationNodeMC *ptr = myBuckets;
    //Sink Location
    ptr->SetNewLoc(0.5,0.5);
    // clear the forward start of the node, and nodes in bin zero
    ptr->ResetReachableNodes();
    ptr->ResetNodesInBucket();

    ptr = ptr + 1;

    // sets the pointers for a node to be the node immediately prior and immediately after the node
    for (int i = 1; i < myNumSensors ; ++i) {
        ptr->ResetReachableNodes();
        ptr->ResetNodesInBucket();
        ptr->ResetFailed();
        ptr->ResetCovered();
        ptr = ptr + 1;

    }

    // Update value for target nodes
    for (int j = myNumSensors ; j < myNumElements ; ++j) {
        ptr->ResetReachableNodes();
        ptr->ResetNodesInBucket();
        ptr->ResetFailed();
        ptr->ResetCovered();
        ptr = ptr + 1;
    }

    // start placing sensor in bins, starting with sink node
    myXNum = myBuckets[0].GetXLoc();
    myYNum = myBuckets[0].GetYLoc();
    // determine which grid number the new sensor location falls in
    myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow +1);

    myBuckets[myTempBin].AddNodeToBucket(0);

    ptr = myBuckets;
    ptr = ptr + 1;

    // generate new location for every sensor
    for (int i = 1; i < myNumSensors ; ++i) {
        myXNum = myUnif(myDevX);
        myYNum = myUnif(myDevY);

        ptr->SetNewLoc(myXNum, myYNum);

        // determine which grid number the new sensor location falls in
        myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow + 1);

        // move sensor into a bin for this new location
        myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());

        ptr = ptr + 1;
    }

    // place target nodes in new bin
    for (int k = myNumSensors; k < myNumElements ; ++k) {

        myXNum = ptr->GetXLoc() + 0.0001;
        myYNum = ptr->GetYLoc() + 0.0001;

        // determine which grid number the new sensor location falls in
        myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow +1);

        //cout << " Node " << ptr->GetNodeID() << " Added to bucket " << myTempBin << endl;

        // move sensor into a bin for this new location
        myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());

        ptr = ptr + 1;

    }

    int myXBucket;
    int myYBucket;
    double dist;
    vector<int> myNodesInCurBucket;

    int curBucket = myBucketsPerRow + 1;
    for (int l = 0; l < pow(myBucketsPerRow,2) ; ++l) {
        // determine which grid cell we are currently in
        myXBucket = curBucket % myBucketsPerRow;
        myYBucket = (curBucket / myBucketsPerRow) % myBucketsPerRow ;

        if (myXBucket == 0){
            myXBucket = myBucketsPerRow;
            if (curBucket / myBucketsPerRow == myBucketsPerRow){
                myYBucket = myBucketsPerRow - 1;
            }
            else {
                myYBucket = myYBucket - 1;
            }
        }

        if (myYBucket == 0){
            myYBucket = myBucketsPerRow;
        }

        // all within bin comparison
        for (int i = 0; i < (myBuckets[l].GetNumberInCell()) ; ++i) {

            int startNode = myBuckets[l].GetNodeInBucket(i);

            // if the starting node is the sink node, then we only need to worry about connecting to sensor nodes, which is based on communication range
            if (startNode == 0) {
                for (int j = (i+1); j < myBuckets[l].GetNumberInCell() ; ++j) {
                    int endNode = myBuckets[l].GetNodeInBucket(j);
                    // determine if end node is a sensor node or a target node
                    if (endNode < myNumSensors){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        if (dist <= myCommunicationRange) {
                            myBuckets[startNode].AddReachableNode(endNode);
                        }
                    }
                }
            }
                // if starting node is a sensor node
            else if (startNode > 0 && startNode < myNumSensors){

                for (int j = (i+1); j < myBuckets[l].GetNumberInCell() ; ++j) {
                    int endNode = myBuckets[l].GetNodeInBucket(j);

                    // start node is a sensor node, end node is sink node, check for reverse arc
                    if (endNode == 0){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        // sensor to sink comparison, we can add both arcs
                        if (dist <= myCommunicationRange) {
                            myBuckets[endNode].AddReachableNode(startNode);
                        }

                    }
                    else if (endNode < myNumSensors){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        // sensor to sensor comparison, we can add both arcs
                        if (dist <= myCommunicationRange) {
                            myBuckets[startNode].AddReachableNode(endNode);
                            myBuckets[endNode].AddReachableNode(startNode);
                        }
                    }
                    else {
                        // end node must be a target node
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        if (dist <= mySensorRange){
                            myBuckets[startNode].AddReachableNode(endNode);
                        }
                    }
                }
            }
        }

        int myAdjacentCell;

        // look to the cell immediately above current cell
        int myTopBucket = myYBucket + 1;
        myAdjacentCell = myXBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow +1);
        if (l < myAdjacentCell && myTopBucket <= myBucketsPerRow){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell immediately to the right
        int myRightBucket = myXBucket + 1;
        myAdjacentCell = myRightBucket + (myYBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myRightBucket <= myBucketsPerRow){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell immediately below
        int myBottomBucket = myYBucket - 1;
        myAdjacentCell = myXBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myBottomBucket < 0){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to left of current cell
        int myLeftBucket = myXBucket - 1;
        myAdjacentCell = myLeftBucket + (myYBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myLeftBucket > 0){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the top right
        myAdjacentCell = myRightBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myRightBucket <= myBucketsPerRow) && (myTopBucket <= myBucketsPerRow)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the bottom right
        myAdjacentCell = myRightBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myRightBucket <= myBucketsPerRow) && (myBottomBucket > 0)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the top left
        myAdjacentCell = myLeftBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l< myAdjacentCell) && (myLeftBucket > 0 ) && (myTopBucket <= myBucketsPerRow)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the bottom left
        myAdjacentCell = myLeftBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myLeftBucket > 0 ) && (myBottomBucket > 0)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        curBucket = curBucket + 1;
    }
}

void NodeLinkedListMC::GenerateNewSpecificRGG_Bucket(int numPerRegion){

//    random_device myDev;

    double myXNum;
    double myYNum;
    int myTempBin;

    int myBucketsPerRow = (int) ceil(1 / myCommunicationRange);

    int myRegionsPerRow = 4;

    SimulationNodeMC *ptr = myBuckets;
    //Sink Location
    ptr->SetNewLoc(0.5,0.5);
    // clear the forward start of the node, and nodes in bin zero
    ptr->ResetReachableNodes();
    ptr->ResetNodesInBucket();

    ptr = ptr + 1;

    // sets the pointers for a node to be the node immediately prior and immediately after the node
    for (int i = 1; i < myNumSensors ; ++i) {
        ptr->ResetReachableNodes();
        ptr->ResetNodesInBucket();
        ptr->ResetFailed();
        ptr->ResetCovered();
        ptr = ptr + 1;

    }

    // Update value for target nodes
    for (int j = myNumSensors ; j < myNumElements ; ++j) {
        ptr->ResetReachableNodes();
        ptr->ResetNodesInBucket();
        ptr->ResetFailed();
        ptr->ResetCovered();
        ptr = ptr + 1;
    }

    // start placing sensor in bins, starting with sink node
    myXNum = myBuckets[0].GetXLoc();
    myYNum = myBuckets[0].GetYLoc();
    // determine which grid number the new sensor location falls in
    myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow +1);

    myBuckets[myTempBin].AddNodeToBucket(0);

    ptr = myBuckets;
    ptr = ptr + 1;

    for (int j = 0; j < myRegionsPerRow*myRegionsPerRow ; ++j) {

        double tempXLow = myBuckets[j].GetXLow();
        double tempXHigh = myBuckets[j].GetXHigh() + 0.001;
        double tempYLow = myBuckets[j].GetYLow();
        double tempYHigh = myBuckets[j].GetYHigh() + 0.001;


        for (int i = 0; i < numPerRegion; ++i) {
            uniform_real_distribution<> myUnifX(tempXLow, fmin(tempXHigh, 1.0));
            uniform_real_distribution<> myUnifY(tempYLow, fmin(tempYHigh, 1.0));

            myXNum = myUnifX(myDevX);
            myYNum = myUnifY(myDevY);

            ptr->SetNewLoc(myXNum, myYNum);

            // determine which grid number the new sensor location falls in
            myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow + 1);

            // move sensor into a bin for this new location
            myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());

            ptr = ptr + 1;

        }
    }

    // place target nodes in new bin
    for (int k = myNumSensors; k < myNumElements ; ++k) {

        myXNum = ptr->GetXLoc() + 0.0001;
        myYNum = ptr->GetYLoc() + 0.0001;

        // determine which grid number the new sensor location falls in
        myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow +1);

        //cout << " Node " << ptr->GetNodeID() << " Added to bucket " << myTempBin << endl;

        // move sensor into a bin for this new location
        myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());

        ptr = ptr + 1;

    }

    int myXBucket;
    int myYBucket;
    double dist;
    vector<int> myNodesInCurBucket;

    int curBucket = myBucketsPerRow + 1;
    for (int l = 0; l < pow(myBucketsPerRow,2) ; ++l) {
        // determine which grid cell we are currently in
        myXBucket = curBucket % myBucketsPerRow;
        myYBucket = (curBucket / myBucketsPerRow) % myBucketsPerRow ;

        if (myXBucket == 0){
            myXBucket = myBucketsPerRow;
            if (curBucket / myBucketsPerRow == myBucketsPerRow){
                myYBucket = myBucketsPerRow - 1;
            }
            else {
                myYBucket = myYBucket - 1;
            }
        }

        if (myYBucket == 0){
            myYBucket = myBucketsPerRow;
        }

        // all within bin comparison
        for (int i = 0; i < (myBuckets[l].GetNumberInCell()) ; ++i) {

            int startNode = myBuckets[l].GetNodeInBucket(i);

            // if the starting node is the sink node, then we only need to worry about connecting to sensor nodes, which is based on communication range
            if (startNode == 0) {
                for (int j = (i+1); j < myBuckets[l].GetNumberInCell() ; ++j) {
                    int endNode = myBuckets[l].GetNodeInBucket(j);
                    // determine if end node is a sensor node or a target node
                    if (endNode < myNumSensors){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        if (dist <= myCommunicationRange) {
                            myBuckets[startNode].AddReachableNode(endNode);
                        }
                    }
                }
            }
                // if starting node is a sensor node
            else if (startNode > 0 && startNode < myNumSensors){

                for (int j = (i+1); j < myBuckets[l].GetNumberInCell() ; ++j) {
                    int endNode = myBuckets[l].GetNodeInBucket(j);

                    // start node is a sensor node, end node is sink node, check for reverse arc
                    if (endNode == 0){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        // sensor to sink comparison, we can add both arcs
                        if (dist <= myCommunicationRange) {
                            myBuckets[endNode].AddReachableNode(startNode);
                        }

                    }
                    else if (endNode < myNumSensors){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        // sensor to sensor comparison, we can add both arcs
                        if (dist <= myCommunicationRange) {
                            myBuckets[startNode].AddReachableNode(endNode);
                            myBuckets[endNode].AddReachableNode(startNode);
                        }
                    }
                    else {
                        // end node must be a target node
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        if (dist <= mySensorRange){
                            myBuckets[startNode].AddReachableNode(endNode);
                        }
                    }
                }
            }
        }

        int myAdjacentCell;

        // look to the cell immediately above current cell
        int myTopBucket = myYBucket + 1;
        myAdjacentCell = myXBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow +1);
        if (l < myAdjacentCell && myTopBucket <= myBucketsPerRow){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell immediately to the right
        int myRightBucket = myXBucket + 1;
        myAdjacentCell = myRightBucket + (myYBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myRightBucket <= myBucketsPerRow){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell immediately below
        int myBottomBucket = myYBucket - 1;
        myAdjacentCell = myXBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myBottomBucket < 0){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to left of current cell
        int myLeftBucket = myXBucket - 1;
        myAdjacentCell = myLeftBucket + (myYBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myLeftBucket > 0){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the top right
        myAdjacentCell = myRightBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myRightBucket <= myBucketsPerRow) && (myTopBucket <= myBucketsPerRow)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the bottom right
        myAdjacentCell = myRightBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myRightBucket <= myBucketsPerRow) && (myBottomBucket > 0)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the top left
        myAdjacentCell = myLeftBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l< myAdjacentCell) && (myLeftBucket > 0 ) && (myTopBucket <= myBucketsPerRow)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the bottom left
        myAdjacentCell = myLeftBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myLeftBucket > 0 ) && (myBottomBucket > 0)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        curBucket = curBucket + 1;
    }
}

// Used to generate a RGG with specific structure, where a certain number of sensors are located in each subregion of the network
void NodeLinkedListMC::GenerateNewSpecificRGG_Bucket(vector<int> numPerRegion){

//    random_device myDev;

    double myXNum;
    double myYNum;
    int myTempBin;

    int myBucketsPerRow = (int) ceil(1 / myCommunicationRange);

    int myRegionsPerRow = 4;

    SimulationNodeMC *ptr = myBuckets;
    //Sink Location
    ptr->SetNewLoc(0.5,0.5);
    // clear the forward start of the node, and nodes in bin zero
    ptr->ResetReachableNodes();
    ptr->ResetNodesInBucket();

    ptr = ptr + 1;

    // sets the pointers for a node to be the node immediately prior and immediately after the node
    for (int i = 1; i < myNumSensors ; ++i) {

        ptr->SetAge(0);
        ptr->ResetReachableNodes();
        ptr->ResetNodesInBucket();
        ptr->ResetFailed();
        ptr->ResetCovered();
        ptr = ptr + 1;

    }

    // Update value for target nodes
    for (int j = myNumSensors ; j < myNumElements ; ++j) {
        ptr->ResetReachableNodes();
        ptr->ResetNodesInBucket();
        ptr->ResetFailed();
        ptr->ResetCovered();
        ptr = ptr + 1;
    }

    // start placing sensor in bins, starting with sink node
    myXNum = myBuckets[0].GetXLoc();
    myYNum = myBuckets[0].GetYLoc();
    // determine which grid number the new sensor location falls in
    myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow +1);

    myBuckets[myTempBin].AddNodeToBucket(0);

    ptr = myBuckets;
    ptr = ptr + 1;

    for (int j = 0; j < myRegionsPerRow*myRegionsPerRow ; ++j) {

        double tempXLow = myBuckets[j].GetXLow();
        double tempXHigh = myBuckets[j].GetXHigh() + 0.001;
        double tempYLow = myBuckets[j].GetYLow();
        double tempYHigh = myBuckets[j].GetYHigh() + 0.001;


        for (int i = 0; i < numPerRegion.at(j); ++i) {
            uniform_real_distribution<> myUnifX(tempXLow, fmin(tempXHigh, 1.0));
            uniform_real_distribution<> myUnifY(tempYLow, fmin(tempYHigh, 1.0));

            myXNum = myUnifX(myDevX);
            myYNum = myUnifY(myDevY);

            ptr->SetNewLoc(myXNum, myYNum);
            ptr->SetSubregion(j);

            // determine which grid number the new sensor location falls in
            myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow + 1);

            // move sensor into a bin for this new location
            myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());

            ptr = ptr + 1;

        }
    }

    // place target nodes in new bin
    for (int k = myNumSensors; k < myNumElements ; ++k) {

        myXNum = ptr->GetXLoc() + 0.0001;
        myYNum = ptr->GetYLoc() + 0.0001;

        // determine which grid number the new sensor location falls in
        myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow +1);

        //cout << " Node " << ptr->GetNodeID() << " Added to bucket " << myTempBin << endl;

        // move sensor into a bin for this new location
        myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());

        ptr = ptr + 1;

    }

    int myXBucket;
    int myYBucket;
    double dist;
    vector<int> myNodesInCurBucket;

    int curBucket = myBucketsPerRow + 1;
    for (int l = 0; l < pow(myBucketsPerRow,2) ; ++l) {
        // determine which grid cell we are currently in
        myXBucket = curBucket % myBucketsPerRow;
        myYBucket = (curBucket / myBucketsPerRow) % myBucketsPerRow ;

        if (myXBucket == 0){
            myXBucket = myBucketsPerRow;
            if (curBucket / myBucketsPerRow == myBucketsPerRow){
                myYBucket = myBucketsPerRow - 1;
            }
            else {
                myYBucket = myYBucket - 1;
            }
        }

        if (myYBucket == 0){
            myYBucket = myBucketsPerRow;
        }

        // all within bin comparison
        for (int i = 0; i < (myBuckets[l].GetNumberInCell()) ; ++i) {

            int startNode = myBuckets[l].GetNodeInBucket(i);

            // if the starting node is the sink node, then we only need to worry about connecting to sensor nodes, which is based on communication range
            if (startNode == 0) {
                for (int j = (i+1); j < myBuckets[l].GetNumberInCell() ; ++j) {
                    int endNode = myBuckets[l].GetNodeInBucket(j);
                    // determine if end node is a sensor node or a target node
                    if (endNode < myNumSensors){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        if (dist <= myCommunicationRange) {
                            myBuckets[startNode].AddReachableNode(endNode);
                        }
                    }
                }
            }
                // if starting node is a sensor node
            else if (startNode > 0 && startNode < myNumSensors){

                for (int j = (i+1); j < myBuckets[l].GetNumberInCell() ; ++j) {
                    int endNode = myBuckets[l].GetNodeInBucket(j);

                    // start node is a sensor node, end node is sink node, check for reverse arc
                    if (endNode == 0){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        // sensor to sink comparison, we can add both arcs
                        if (dist <= myCommunicationRange) {
                            myBuckets[endNode].AddReachableNode(startNode);
                        }

                    }
                    else if (endNode < myNumSensors){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        // sensor to sensor comparison, we can add both arcs
                        if (dist <= myCommunicationRange) {
                            myBuckets[startNode].AddReachableNode(endNode);
                            myBuckets[endNode].AddReachableNode(startNode);
                        }
                    }
                    else {
                        // end node must be a target node
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        if (dist <= mySensorRange){
                            myBuckets[startNode].AddReachableNode(endNode);
                        }
                    }
                }
            }
        }

        int myAdjacentCell;

        // look to the cell immediately above current cell
        int myTopBucket = myYBucket + 1;
        myAdjacentCell = myXBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow +1);
        if (l < myAdjacentCell && myTopBucket <= myBucketsPerRow){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell immediately to the right
        int myRightBucket = myXBucket + 1;
        myAdjacentCell = myRightBucket + (myYBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myRightBucket <= myBucketsPerRow){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell immediately below
        int myBottomBucket = myYBucket - 1;
        myAdjacentCell = myXBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myBottomBucket < 0){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to left of current cell
        int myLeftBucket = myXBucket - 1;
        myAdjacentCell = myLeftBucket + (myYBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myLeftBucket > 0){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the top right
        myAdjacentCell = myRightBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myRightBucket <= myBucketsPerRow) && (myTopBucket <= myBucketsPerRow)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the bottom right
        myAdjacentCell = myRightBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myRightBucket <= myBucketsPerRow) && (myBottomBucket > 0)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the top left
        myAdjacentCell = myLeftBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l< myAdjacentCell) && (myLeftBucket > 0 ) && (myTopBucket <= myBucketsPerRow)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the bottom left
        myAdjacentCell = myLeftBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myLeftBucket > 0 ) && (myBottomBucket > 0)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        curBucket = curBucket + 1;
    }
}

// Used to generate a RGG with specific structure, where a certain number of sensors are located in each subregion of the network
void NodeLinkedListMC::GenerateNewSpecificRGG_Bucket_ADPEval(vector<int> numPerRegion){

//    random_device myDev;

    double myXNum;
    double myYNum;
    int myTempBin;

    int myBucketsPerRow = (int) ceil(1 / myCommunicationRange);

    int myRegionsPerRow = 4;

    SimulationNodeMC *ptr = myBuckets;
    //Sink Location
    ptr->SetNewLoc(0.5,0.5);
    // clear the forward start of the node, and nodes in bin zero
    ptr->ResetReachableNodes();
    ptr->ResetNodesInBucket();
    ptr->SetActive();

    ptr = ptr + 1;

    // sets the pointers for a node to be the node immediately prior and immediately after the node
    for (int i = 1; i < myNumSensors ; ++i) {

        ptr->SetAge(0);
        ptr->ResetReachableNodes();
        ptr->ResetNodesInBucket();
        ptr->ResetFailed();
        ptr->ResetCovered();
        ptr->SetInactive();
        ptr = ptr + 1;

    }

    // Update value for target nodes
    for (int j = myNumSensors ; j < myNumElements ; ++j) {
        ptr->ResetReachableNodes();
        ptr->ResetNodesInBucket();
        ptr->ResetFailed();
        ptr->ResetCovered();
        ptr->SetActive();
        ptr = ptr + 1;
    }

    // start placing sensor in bins, starting with sink node
    myXNum = myBuckets[0].GetXLoc();
    myYNum = myBuckets[0].GetYLoc();
    // determine which grid number the new sensor location falls in
    myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow +1);

    myBuckets[myTempBin].AddNodeToBucket(0);

    ptr = myBuckets;
    ptr = ptr + 1;

    for (int j = 0; j < myRegionsPerRow*myRegionsPerRow ; ++j) {

        double tempXLow = myBuckets[j].GetXLow();
        double tempXHigh = myBuckets[j].GetXHigh() + 0.001;
        double tempYLow = myBuckets[j].GetYLow();
        double tempYHigh = myBuckets[j].GetYHigh() + 0.001;


        for (int i = 0; i < numPerRegion.at(j); ++i) {
            uniform_real_distribution<> myUnifX(tempXLow, fmin(tempXHigh, 1.0));
            uniform_real_distribution<> myUnifY(tempYLow, fmin(tempYHigh, 1.0));

            myXNum = myUnifX(myDevX);
            myYNum = myUnifY(myDevY);

            ptr->SetNewLoc(myXNum, myYNum);
            ptr->SetSubregion(j);
            ptr->SetActive();

            // determine which grid number the new sensor location falls in
            myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow + 1);

            // move sensor into a bin for this new location
            myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());

            ptr = ptr + 1;

        }
    }

    ptr = myBuckets;
    ptr = ptr + myNumSensors;

    // place target nodes in new bin
    for (int k = myNumSensors; k < myNumElements ; ++k) {

        myXNum = ptr->GetXLoc() + 0.0001;
        myYNum = ptr->GetYLoc() + 0.0001;

        // determine which grid number the new sensor location falls in
        myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow +1);

        //cout << " Node " << ptr->GetNodeID() << " Added to bucket " << myTempBin << endl;

        // move sensor into a bin for this new location
        myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());

        ptr = ptr + 1;

    }

    int myXBucket;
    int myYBucket;
    double dist;
    vector<int> myNodesInCurBucket;

    int curBucket = myBucketsPerRow + 1;
    for (int l = 0; l < pow(myBucketsPerRow,2) ; ++l) {
        // determine which grid cell we are currently in
        myXBucket = curBucket % myBucketsPerRow;
        myYBucket = (curBucket / myBucketsPerRow) % myBucketsPerRow ;

        if (myXBucket == 0){
            myXBucket = myBucketsPerRow;
            if (curBucket / myBucketsPerRow == myBucketsPerRow){
                myYBucket = myBucketsPerRow - 1;
            }
            else {
                myYBucket = myYBucket - 1;
            }
        }

        if (myYBucket == 0){
            myYBucket = myBucketsPerRow;
        }

        // all within bin comparison
        for (int i = 0; i < (myBuckets[l].GetNumberInCell()) ; ++i) {

            int startNode = myBuckets[l].GetNodeInBucket(i);

            // if the starting node is the sink node, then we only need to worry about connecting to sensor nodes, which is based on communication range
            if (startNode == 0) {
                for (int j = (i+1); j < myBuckets[l].GetNumberInCell() ; ++j) {
                    int endNode = myBuckets[l].GetNodeInBucket(j);
                    // determine if end node is a sensor node or a target node
                    if (endNode < myNumSensors){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        if (dist <= myCommunicationRange) {
                            myBuckets[startNode].AddReachableNode(endNode);
                        }
                    }
                }
            }
                // if starting node is a sensor node
            else if (startNode > 0 && startNode < myNumSensors){

                for (int j = (i+1); j < myBuckets[l].GetNumberInCell() ; ++j) {
                    int endNode = myBuckets[l].GetNodeInBucket(j);

                    // start node is a sensor node, end node is sink node, check for reverse arc
                    if (endNode == 0){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        // sensor to sink comparison, we can add both arcs
                        if (dist <= myCommunicationRange) {
                            myBuckets[endNode].AddReachableNode(startNode);
                        }

                    }
                    else if (endNode < myNumSensors){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        // sensor to sensor comparison, we can add both arcs
                        if (dist <= myCommunicationRange) {
                            myBuckets[startNode].AddReachableNode(endNode);
                            myBuckets[endNode].AddReachableNode(startNode);
                        }
                    }
                    else {
                        // end node must be a target node
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        if (dist <= mySensorRange){
                            myBuckets[startNode].AddReachableNode(endNode);
                        }
                    }
                }
            }
        }

        int myAdjacentCell;

        // look to the cell immediately above current cell
        int myTopBucket = myYBucket + 1;
        myAdjacentCell = myXBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow +1);
        if (l < myAdjacentCell && myTopBucket <= myBucketsPerRow){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell immediately to the right
        int myRightBucket = myXBucket + 1;
        myAdjacentCell = myRightBucket + (myYBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myRightBucket <= myBucketsPerRow){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell immediately below
        int myBottomBucket = myYBucket - 1;
        myAdjacentCell = myXBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myBottomBucket < 0){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to left of current cell
        int myLeftBucket = myXBucket - 1;
        myAdjacentCell = myLeftBucket + (myYBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myLeftBucket > 0){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the top right
        myAdjacentCell = myRightBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myRightBucket <= myBucketsPerRow) && (myTopBucket <= myBucketsPerRow)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the bottom right
        myAdjacentCell = myRightBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myRightBucket <= myBucketsPerRow) && (myBottomBucket > 0)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the top left
        myAdjacentCell = myLeftBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l< myAdjacentCell) && (myLeftBucket > 0 ) && (myTopBucket <= myBucketsPerRow)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the bottom left
        myAdjacentCell = myLeftBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myLeftBucket > 0 ) && (myBottomBucket > 0)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        curBucket = curBucket + 1;
    }
}

void NodeLinkedListMC::CompareAdjacentCells(int cell1, int cell2) {
    double dist;
    int startNode;
    int endNode;

    for (int i = 0; i < (myBuckets[cell1].GetNumberInCell()) ; ++i) {

        startNode = myBuckets[cell1].GetNodeInBucket(i);

        // if the starting node is the sink node, then we only need to worry about connecting to sensor nodes, which is based on communication range
        if (startNode == 0) {
            for (int j = 0; j < myBuckets[cell2].GetNumberInCell() ; ++j) {
                endNode = myBuckets[cell2].GetNodeInBucket(j);
                // determine if end node is a sensor node or a target node
                if (endNode < myNumSensors){
                    dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                    if (dist <= myCommunicationRange) {
                        myBuckets[startNode].AddReachableNode(endNode);
                    }
                }
            }
        }
            // if starting node is a sensor node
        else if (startNode > 0 && startNode < myNumSensors){

            for (int j = 0; j < myBuckets[cell2].GetNumberInCell() ; ++j) {
                endNode = myBuckets[cell2].GetNodeInBucket(j);

                if (endNode == 0){
                    // start node is a sensor node, end node is sink node, we check to add arc since nodes are in different grid cell
                    dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                    // sensor to sink comparison, we can arc from sink to sensor node
                    if (dist <= myCommunicationRange) {
                        myBuckets[endNode].AddReachableNode(startNode);
                    }
                }
                else if (endNode < myNumSensors){
                    dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                    // sensor to sensor comparison, we can add both arcs
                    if (dist <= myCommunicationRange) {
                        myBuckets[startNode].AddReachableNode(endNode);
                        myBuckets[endNode].AddReachableNode(startNode);
                    }
                }
                else {
                    // end node must be a target node
                    dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                    if (dist <= mySensorRange){
                        myBuckets[startNode].AddReachableNode(endNode);
                    }
                }
            }
        }
            // starting node is a target node, but we still need to check for sensor to target arcs
        else {
            for (int j = 0; j < myBuckets[cell2].GetNumberInCell() ; ++j) {
                endNode = myBuckets[cell2].GetNodeInBucket(j);

                if ((endNode == 0) || (endNode >= myNumSensors)){
                    // start node is target, end node is either a target or sink node, we skip
                }
                else{
                    // end node must be a sensor node
                    dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                    if (dist <= mySensorRange){
                        myBuckets[endNode].AddReachableNode(startNode);
                    }
                }
            }
        }
    }
}

// Used to generate a RGG with specific structure, where a certain number of sensors are located in each subregion of the network
void NodeLinkedListMC::GenerateNewRGG_Bucket_ADPEval(int nSize){

//    random_device myDev;
    uniform_real_distribution<> myUnif(0,1);

    double myXNum;
    double myYNum;
    int myTempBin;

    int myBucketsPerRow = (int) ceil(1 / myCommunicationRange);

    int myRegionsPerRow = 4;

    SimulationNodeMC *ptr = myBuckets;
    //Sink Location
    ptr->SetNewLoc(0.5,0.5);
    // clear the forward start of the node, and nodes in bin zero
    ptr->ResetReachableNodes();
    ptr->ResetNodesInBucket();
    ptr->SetActive();

    ptr = ptr + 1;

    // sets the pointers for a node to be the node immediately prior and immediately after the node
    for (int i = 1; i < myNumSensors ; ++i) {

        ptr->SetAge(0);
        ptr->ResetReachableNodes();
        ptr->ResetNodesInBucket();
        ptr->ResetFailed();
        ptr->ResetCovered();
        ptr->SetInactive();
        ptr = ptr + 1;

    }

    // Update value for target nodes
    for (int j = myNumSensors ; j < myNumElements ; ++j) {
        ptr->ResetReachableNodes();
        ptr->ResetNodesInBucket();
        ptr->ResetFailed();
        ptr->ResetCovered();
        ptr->SetActive();
        ptr = ptr + 1;
    }

    // start placing sensor in bins, starting with sink node
    myXNum = myBuckets[0].GetXLoc();
    myYNum = myBuckets[0].GetYLoc();
    // determine which grid number the new sensor location falls in
    myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow +1);

    myBuckets[myTempBin].AddNodeToBucket(0);

    ptr = myBuckets;
    ptr = ptr + 1;

    for (int j = 0; j < nSize ; ++j) {

        myXNum = myUnif(myDevX);
        myYNum = myUnif(myDevY);

        ptr->SetNewLoc(myXNum, myYNum);
        ptr->SetActive();

        // determine which grid number the new sensor location falls in
        myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow + 1);

        // move sensor into a bin for this new location
        myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());

        ptr = ptr + 1;
    }

    ptr = myBuckets;
    ptr = ptr + myNumSensors;

    // place target nodes in new bin
    for (int k = myNumSensors; k < myNumElements ; ++k) {

        myXNum = ptr->GetXLoc() + 0.0001;
        myYNum = ptr->GetYLoc() + 0.0001;

        // determine which grid number the new sensor location falls in
        myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow +1);

        //cout << " Node " << ptr->GetNodeID() << " Added to bucket " << myTempBin << endl;

        // move sensor into a bin for this new location
        myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());

        ptr = ptr + 1;

    }

    int myXBucket;
    int myYBucket;
    double dist;
    vector<int> myNodesInCurBucket;

    int curBucket = myBucketsPerRow + 1;
    for (int l = 0; l < pow(myBucketsPerRow,2) ; ++l) {
        // determine which grid cell we are currently in
        myXBucket = curBucket % myBucketsPerRow;
        myYBucket = (curBucket / myBucketsPerRow) % myBucketsPerRow ;

        if (myXBucket == 0){
            myXBucket = myBucketsPerRow;
            if (curBucket / myBucketsPerRow == myBucketsPerRow){
                myYBucket = myBucketsPerRow - 1;
            }
            else {
                myYBucket = myYBucket - 1;
            }
        }

        if (myYBucket == 0){
            myYBucket = myBucketsPerRow;
        }

        // all within bin comparison
        for (int i = 0; i < (myBuckets[l].GetNumberInCell()) ; ++i) {

            int startNode = myBuckets[l].GetNodeInBucket(i);

            // if the starting node is the sink node, then we only need to worry about connecting to sensor nodes, which is based on communication range
            if (startNode == 0) {
                for (int j = (i+1); j < myBuckets[l].GetNumberInCell() ; ++j) {
                    int endNode = myBuckets[l].GetNodeInBucket(j);
                    // determine if end node is a sensor node or a target node
                    if (endNode < myNumSensors){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        if (dist <= myCommunicationRange) {
                            myBuckets[startNode].AddReachableNode(endNode);
                        }
                    }
                }
            }
                // if starting node is a sensor node
            else if (startNode > 0 && startNode < myNumSensors){

                for (int j = (i+1); j < myBuckets[l].GetNumberInCell() ; ++j) {
                    int endNode = myBuckets[l].GetNodeInBucket(j);

                    // start node is a sensor node, end node is sink node, check for reverse arc
                    if (endNode == 0){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        // sensor to sink comparison, we can add both arcs
                        if (dist <= myCommunicationRange) {
                            myBuckets[endNode].AddReachableNode(startNode);
                        }

                    }
                    else if (endNode < myNumSensors){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        // sensor to sensor comparison, we can add both arcs
                        if (dist <= myCommunicationRange) {
                            myBuckets[startNode].AddReachableNode(endNode);
                            myBuckets[endNode].AddReachableNode(startNode);
                        }
                    }
                    else {
                        // end node must be a target node
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        if (dist <= mySensorRange){
                            myBuckets[startNode].AddReachableNode(endNode);
                        }
                    }
                }
            }
        }

        int myAdjacentCell;

        // look to the cell immediately above current cell
        int myTopBucket = myYBucket + 1;
        myAdjacentCell = myXBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow +1);
        if (l < myAdjacentCell && myTopBucket <= myBucketsPerRow){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell immediately to the right
        int myRightBucket = myXBucket + 1;
        myAdjacentCell = myRightBucket + (myYBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myRightBucket <= myBucketsPerRow){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell immediately below
        int myBottomBucket = myYBucket - 1;
        myAdjacentCell = myXBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myBottomBucket < 0){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to left of current cell
        int myLeftBucket = myXBucket - 1;
        myAdjacentCell = myLeftBucket + (myYBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myLeftBucket > 0){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the top right
        myAdjacentCell = myRightBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myRightBucket <= myBucketsPerRow) && (myTopBucket <= myBucketsPerRow)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the bottom right
        myAdjacentCell = myRightBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myRightBucket <= myBucketsPerRow) && (myBottomBucket > 0)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the top left
        myAdjacentCell = myLeftBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l< myAdjacentCell) && (myLeftBucket > 0 ) && (myTopBucket <= myBucketsPerRow)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the bottom left
        myAdjacentCell = myLeftBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myLeftBucket > 0 ) && (myBottomBucket > 0)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        curBucket = curBucket + 1;
    }
}

void NodeLinkedListMC::UpdateRGG(double time) {
    for (int i = 0; i < myNumSensors ; ++i) {

        if (myBuckets[i].GetFailTime() < time){
            myBuckets[i].SetFailed();
        }
        else{

            int k = 0;
            while (k < myBuckets[i].GetDegree()){

                int tempNode = myBuckets[i].GetReachableNode(k);
                if (myBuckets[tempNode].GetFailTime() < time){
                    myBuckets[i].UpdateForwardStar(k);
                    myBuckets[tempNode].SetFailed();
                }

                k = k + 1;
            }
        }
    }
}

void NodeLinkedListMC::UpdateRGG_ADPEval(double time){
    for (int i = 0; i < myNumSensors ; ++i) {

        if (myBuckets[i].GetFailTime() < time){
            myBuckets[i].SetFailed();
            myBuckets[i].SetInactive();
        }
        else{

            int k = 0;
            while (k < myBuckets[i].GetDegree()){

                int tempNode = myBuckets[i].GetReachableNode(k);
                if (myBuckets[tempNode].GetFailTime() < time || !myBuckets[tempNode].IsActive()){
                    myBuckets[i].UpdateForwardStar(k);
                    myBuckets[tempNode].SetFailed();
                    myBuckets[tempNode].SetInactive();
                }

                k = k + 1;
            }
        }
    }
}

int NodeLinkedListMC::GetNumSensorsFailed() {

    int numFailed = 0;

    for (int i = 1; i < myNumSensors; ++i) {
        if (myBuckets[i].IsFailed()){
            numFailed = numFailed + 1;
        }
    }
    return numFailed;

}

void NodeLinkedListMC::GenerateNewRGG_BucketMaint(){

    double costFixed = 100.0;
    double costVar = 1.0;
    double policyCost = 0;
    int numDisconnected = 0;
    int numDeployed = 0;

    this->GenerateNewRGG_BucketMaint(costFixed, costVar, policyCost, numDisconnected, numDeployed);

}

// used to generate a new RGG in a maintenance policy
void NodeLinkedListMC::GenerateNewRGG_BucketMaint(double costFixed, double costVar, double &policyCost){

    int numDisconnected = 0;
    int numDeployed = 0;

    this->GenerateNewRGG_BucketMaint(costFixed, costVar, policyCost, numDisconnected, numDeployed);

}

// used to generate a new RGG in a maintenance policy, but also records the number of the new sensors that are deployed, and the number of sensors that are
// disconnected from the sink node at the time of maintenance
void NodeLinkedListMC::GenerateNewRGG_BucketMaint(double costFixed, double costVar, double &policyCost, int &numDisconnected, int &numDeployed){

    uniform_real_distribution<> myUnif(0,1);

    double myXNum;
    double myYNum;
    int myTempBin;
    int numReplaced = 0;
    bool maintOccur = false;
    double maintCost = 0;

    int myBucketsPerRow = (int) ceil(1 / myCommunicationRange);

    SimulationNodeMC *ptr = myBuckets;
    //Sink Location
    ptr->SetNewLoc(0.5,0.5);
    // clear the forward start of the node, and nodes in bin zero
    ptr->ResetReachableNodes();
    ptr->ResetNodesInBucket();

    ptr = ptr + 1;
    numDisconnected = 0;

    // sets the pointers for a node to be the node immediately prior and immediately after the node
    for (int i = 1; i < myNumSensors ; ++i) {
        ptr->ResetReachableNodes();
        ptr->ResetNodesInBucket();

        if (!ptr->IsReached()){
            numDisconnected = numDisconnected + 1;
        }
//        ptr->ResetNode();
        ptr = ptr + 1;

    }

    // Update value for target nodes
    for (int j = myNumSensors ; j < myNumElements ; ++j) {
//        ptr->ResetNode();
        ptr->ResetReachableNodes();
        ptr->ResetNodesInBucket();
        ptr = ptr + 1;
    }

    // start placing sensor in bins, starting with sink node
    myXNum = myBuckets[0].GetXLoc();
    myYNum = myBuckets[0].GetYLoc();
    // determine which grid number the new sensor location falls in
    myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow +1);

    myBuckets[myTempBin].AddNodeToBucket(0);

    ptr = myBuckets;
    ptr = ptr + 1;

    // generate new location for every sensor
    for (int i = 1; i < myNumSensors ; ++i) {

        myXNum = myBuckets[i].GetXLoc();
        myYNum = myBuckets[i].GetYLoc();

        if (myBuckets[i].IsFailed()){
            //           cout << " New location " << endl;
            myXNum = myUnif(myDevX);
            myYNum = myUnif(myDevY);
//            cout << " Node " << myBuckets[i].GetNodeID() <<  " New x : " << myXNum << " --- new y : " << myYNum << endl;

            ptr->ResetFailed();
            ptr->SetNewLoc(myXNum, myYNum);

            numReplaced = numReplaced + 1;
            maintOccur = true;
        }

        // determine which grid number the new sensor location falls in
        myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow + 1);

        // move sensor into a bin for this new location
        myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());

        ptr = ptr + 1;
    }

    // place target nodes in new bin
    for (int k = myNumSensors; k < myNumElements ; ++k) {

        myXNum = ptr->GetXLoc() + 0.0001;
        myYNum = ptr->GetYLoc() + 0.0001;

        // determine which grid number the new sensor location falls in
        myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow +1);

//        cout << " Node " << ptr->GetNodeID() << " -- XLoc : " << myXNum << " -- YLoc : " << myYNum << endl;

        // move sensor into a bin for this new location
        myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());

        ptr = ptr + 1;

    }


    // Calculate cost of replacing sensors
    if (numReplaced > 0){
        maintCost = ((double) numReplaced) * (double) costVar + costFixed;
        policyCost = maintCost;

    }
    else{
        policyCost = 0.0;
    }
    numDeployed = numReplaced;

    // reset failed nodes and covered nodes
    ptr = myBuckets;
    ptr = ptr + 1;
    for (int m = 1; m < myNumElements; ++m) {
        ptr->ResetFailed();
        ptr->ResetCovered();
        ptr = ptr + 1;
    }

    int myXBucket;
    int myYBucket;
    double dist;
    vector<int> myNodesInCurBucket;

    int curBucket = myBucketsPerRow + 1;
    for (int l = 0; l < pow(myBucketsPerRow,2) ; ++l) {
        // determine which grid cell we are currently in
        myXBucket = curBucket % myBucketsPerRow;
        myYBucket = (curBucket / myBucketsPerRow) % myBucketsPerRow ;

        if (myXBucket == 0){
            myXBucket = myBucketsPerRow;
            if (curBucket / myBucketsPerRow == myBucketsPerRow){
                myYBucket = myBucketsPerRow - 1;
            }
            else {
                myYBucket = myYBucket - 1;
            }
        }

        if (myYBucket == 0){
            myYBucket = myBucketsPerRow;
        }

        // all within bin comparison
        for (int i = 0; i < (myBuckets[l].GetNumberInCell()) ; ++i) {

            int startNode = myBuckets[l].GetNodeInBucket(i);

            // if the starting node is the sink node, then we only need to worry about connecting to sensor nodes, which is based on communication range
            if (startNode == 0) {
                for (int j = (i+1); j < myBuckets[l].GetNumberInCell() ; ++j) {
                    int endNode = myBuckets[l].GetNodeInBucket(j);
                    // determine if end node is a sensor node or a target node
                    if (endNode < myNumSensors){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        if (dist <= myCommunicationRange) {
                            myBuckets[startNode].AddReachableNode(endNode);
                        }
                    }
                }
            }
                // if starting node is a sensor node
            else if (startNode > 0 && startNode < myNumSensors){

                for (int j = (i+1); j < myBuckets[l].GetNumberInCell() ; ++j) {
                    int endNode = myBuckets[l].GetNodeInBucket(j);

                    // start node is a sensor node, end node is sink node, check for reverse arc
                    if (endNode == 0){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        // sensor to sink comparison, we can add both arcs
                        if (dist <= myCommunicationRange) {
                            myBuckets[endNode].AddReachableNode(startNode);
                        }

                    }
                    else if (endNode < myNumSensors){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        // sensor to sensor comparison, we can add both arcs
                        if (dist <= myCommunicationRange) {
                            myBuckets[startNode].AddReachableNode(endNode);
                            myBuckets[endNode].AddReachableNode(startNode);
                        }
                    }
                    else {
                        // end node must be a target node
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        if (dist <= mySensorRange){
                            myBuckets[startNode].AddReachableNode(endNode);
                        }
                    }
                }
            }
        }

        int myAdjacentCell;

        // look to the cell immediately above current cell
        int myTopBucket = myYBucket + 1;
        myAdjacentCell = myXBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow +1);
        if (l < myAdjacentCell && myTopBucket <= myBucketsPerRow){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell immediately to the right
        int myRightBucket = myXBucket + 1;
        myAdjacentCell = myRightBucket + (myYBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myRightBucket <= myBucketsPerRow){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell immediately below
        int myBottomBucket = myYBucket - 1;
        myAdjacentCell = myXBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myBottomBucket < 0){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to left of current cell
        int myLeftBucket = myXBucket - 1;
        myAdjacentCell = myLeftBucket + (myYBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myLeftBucket > 0){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the top right
        myAdjacentCell = myRightBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myRightBucket <= myBucketsPerRow) && (myTopBucket <= myBucketsPerRow)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the bottom right
        myAdjacentCell = myRightBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myRightBucket <= myBucketsPerRow) && (myBottomBucket > 0)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the top left
        myAdjacentCell = myLeftBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l< myAdjacentCell) && (myLeftBucket > 0 ) && (myTopBucket <= myBucketsPerRow)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the bottom left
        myAdjacentCell = myLeftBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myLeftBucket > 0 ) && (myBottomBucket > 0)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        curBucket = curBucket + 1;
    }
}

double NodeLinkedListMC::TargetCoveragePercentage() {

    double numCovered = 0;

    for (int i = myNumSensors; i < myNumTargets ; ++i) {
        if (myBuckets[i].IsCovered()){
            numCovered = numCovered + 1;
        }
    }

    double percentCovered = (double) numCovered / (double) myNumTargets;

    cout << " numCovered " << numCovered << " -- myNumTargets " << myNumTargets << endl;

    return percentCovered;

}

double NodeLinkedListMC::BreadthFirstSearchReturn(int numNodes, int numTar) {
    double mTime = 0;

    double netCoverage = this->BreadthFirstSearchReturn(numNodes, numTar, mTime);
    return netCoverage;
}

// implements a BFS to determine the percentage of targets that are covered at a given time
double NodeLinkedListMC::BreadthFirstSearchReturn(int numNodes, int numTar, double &timeDuration) {

    // sink node is already permanent
    int numPermanent = 1;

    vector<int> myList;
    int numCovered = 0;

    // start time record
    auto start = chrono::high_resolution_clock::now();

    myBuckets[0].UpdateReached();

    // update all nodes connected to sink
    for (int i = 0; i < myBuckets[0].GetDegree(); ++i) {
        int updateNode1 = myBuckets[0].GetReachableNode(i);


        if (!myBuckets[updateNode1].IsFailed()){
            myList.push_back(updateNode1);

            if (myBuckets[updateNode1].IsFailed()){
                cout << " This Node Has Failed!!!! " << endl;
            }

        }

        double updateLabel = min(myBuckets[0].GetFailCapacity(), myBuckets[updateNode1].GetFailTime());
        myBuckets[updateNode1].UpdateFailCapacity(updateLabel);

        myBuckets[updateNode1].UpdateReached();
    }

    while (myList.size() > 0) {

        double currMax = 0;
        int permNode = 0;

        int tempIndex = myList.at(0);
        myList.erase(myList.begin());

        permNode = tempIndex;

        if (myBuckets[permNode].IsFailed()){
            cout << " This Node Has Failed!!!! " << endl;
        }

        if (tempIndex == -1) {
            // node not connected - should be able to break here
        } else {

            // mark node permanent
            myBuckets[permNode].MarkPermanent();
            myBuckets[permNode].UpdateReached();

            this->addFailOrder(permNode);
            this->addFailTime(currMax);

            // stores the failure order and time at which Targets are disconnected
            if (myBuckets[permNode].GetNodeID() >= numNodes - numTar) {
                this->addTargetFailOrder(myBuckets[permNode].GetNodeID());
                this->addTargetFailTime(myBuckets[permNode].GetFailCapacity());

                myBuckets[permNode].SetCovered();
                numCovered = numCovered + 1;

            }
            else {
                this->addSensorFailOrder(myBuckets[permNode].GetNodeID());
                this->addSensorFailTime(myBuckets[permNode].GetFailCapacity());
            }

            // iterate through edges
            for (int k = 0; k < myBuckets[permNode].GetDegree(); ++k) {

                int updateNode1 = myBuckets[permNode].GetReachableNode(k);

                if (!myBuckets[updateNode1].IsReached() && !myBuckets[updateNode1].IsFailed()){
                    myList.push_back(updateNode1);
                    myBuckets[updateNode1].UpdateReached();
                }

                if (!myBuckets[updateNode1].IsPermanent()) {
                    double updateLabel = min(myBuckets[permNode].GetFailCapacity(), myBuckets[updateNode1].GetFailTime());
                    myBuckets[updateNode1].UpdateFailCapacity(updateLabel);
                }
            }
        }
        numPermanent = numPermanent + 1;
    }

    // If a target is not connected to the sink then it will never be marked permanent, so it will never be added to the
    // order of target failures. This for loop checks for such a case, and adds the targets to ths list with a fail time of zero
    if (orderedTargetFail.size() < numTar){
        SimulationNodeMC *ptr = myBuckets;
        ptr = ptr + (numNodes - numTar);
        for (int i = 0; i < numTar ; ++i) {
            if (!ptr->IsPermanent()){
                this->addTargetFailOrder(ptr->GetNodeID());
                this->addTargetFailTime(ptr->GetFailCapacity());
            }
            ptr = ptr + 1;
        }
    }

    // end time record
    auto finish = chrono::high_resolution_clock::now();
    // calculate execution time
    chrono::duration<double> elapsed = finish - start;
    timeDuration = elapsed.count();

    double percentCovered = (double) numCovered / (double) myNumTargets;

    return percentCovered;

}

// implements a BFS to determine the percentage of targets that are covered at a given time
double NodeLinkedListMC::BreadthFirstSearchReturn_ADPEval(int numNodes, int numTar, double &timeDuration) {

/*    int startSens = 0;
    int startTarg = 0;

    for (int j = 1; j < myNumSensors ; ++j) {
        if (!myBuckets[j].IsFailed() && myBuckets[j].IsActive()){
            startSens = startSens + 1;
        }
    }

    for (int l = myNumSensors; l < myNumElements; ++l) {
        if (myBuckets[l].IsActive()){
            startTarg = startTarg + 1;
        }
    }

    cout << "    Starting Network " << startSens << " - " << startTarg << endl;*/


    // sink node is already permanent
    int numPermanent = 1;

    vector<int> myList;
    int numCovered = 0;

    // start time record
    auto start = chrono::high_resolution_clock::now();

    myBuckets[0].UpdateReached();

    // update all nodes connected to sink
    for (int i = 0; i < myBuckets[0].GetDegree(); ++i) {
        int updateNode1 = myBuckets[0].GetReachableNode(i);

        if (!myBuckets[updateNode1].IsFailed() && myBuckets[updateNode1].IsActive()){
            myList.push_back(updateNode1);

            if (myBuckets[updateNode1].IsFailed()){
                cout << " This Node Has Failed!!!! " << endl;
            }

        }

        double updateLabel = min(myBuckets[0].GetFailCapacity(), myBuckets[updateNode1].GetFailTime());
        myBuckets[updateNode1].UpdateFailCapacity(updateLabel);

        myBuckets[updateNode1].UpdateReached();
    }

    while (myList.size() > 0) {

        double currMax = 0;
        int permNode = 0;

        int tempIndex = myList.at(0);
        myList.erase(myList.begin());

        permNode = tempIndex;

        if (myBuckets[permNode].IsFailed()){
            cout << " This Node Has Failed!!!! " << endl;
        }

        if (tempIndex == -1) {
            // node not connected - should be able to break here
        } else {

            // mark node permanent
            myBuckets[permNode].MarkPermanent();
            myBuckets[permNode].UpdateReached();

            this->addFailOrder(permNode);
            this->addFailTime(currMax);

            // stores the failure order and time at which Targets are disconnected
            if (myBuckets[permNode].GetNodeID() >= numNodes - numTar) {
                this->addTargetFailOrder(myBuckets[permNode].GetNodeID());
                this->addTargetFailTime(myBuckets[permNode].GetFailCapacity());

                myBuckets[permNode].SetCovered();
                numCovered = numCovered + 1;

            }
            else {
                this->addSensorFailOrder(myBuckets[permNode].GetNodeID());
                this->addSensorFailTime(myBuckets[permNode].GetFailCapacity());
            }

            // iterate through edges
            for (int k = 0; k < myBuckets[permNode].GetDegree(); ++k) {

                int updateNode1 = myBuckets[permNode].GetReachableNode(k);

                if (!myBuckets[updateNode1].IsReached() && !myBuckets[updateNode1].IsFailed() && myBuckets[updateNode1].IsActive()){
                    myList.push_back(updateNode1);
                    myBuckets[updateNode1].UpdateReached();
                }

                if (!myBuckets[updateNode1].IsPermanent()) {
                    double updateLabel = min(myBuckets[permNode].GetFailCapacity(), myBuckets[updateNode1].GetFailTime());
                    myBuckets[updateNode1].UpdateFailCapacity(updateLabel);
                }
            }
        }
        numPermanent = numPermanent + 1;
    }

    // If a target is not connected to the sink then it will never be marked permanent, so it will never be added to the
    // order of target failures. This for loop checks for such a case, and adds the targets to ths list with a fail time of zero
    if (orderedTargetFail.size() < numTar){
        SimulationNodeMC *ptr = myBuckets;
        ptr = ptr + (numNodes - numTar);
        for (int i = 0; i < numTar ; ++i) {
            if (!ptr->IsPermanent()){
                this->addTargetFailOrder(ptr->GetNodeID());
                this->addTargetFailTime(ptr->GetFailCapacity());
            }
            ptr = ptr + 1;
        }
    }

    // end time record
    auto finish = chrono::high_resolution_clock::now();
    // calculate execution time
    chrono::duration<double> elapsed = finish - start;
    timeDuration = elapsed.count();

    double percentCovered = (double) numCovered / (double) myNumTargets;

    return percentCovered;
}

void NodeLinkedListMC::BreadthFirstSearch(int numNodes, int numTar, double &timeDuration) {

    // sink node is already permanent
    int numPermanent = 1;

    vector<int> myList;

    // start time record
    auto start = chrono::high_resolution_clock::now();

    myBuckets[0].UpdateReached();

    // update all nodes connected to sink
    for (int i = 0; i < myBuckets[0].GetDegree(); ++i) {
        int updateNode1 = myBuckets[0].GetReachableNode(i);

        myList.push_back(updateNode1);

        double updateLabel = min(myBuckets[0].GetFailCapacity(), myBuckets[updateNode1].GetFailTime());
        myBuckets[updateNode1].UpdateFailCapacity(updateLabel);

        myBuckets[updateNode1].UpdateReached();
    }

    while (myList.size() > 0) {

        double currMax = 0;
        int permNode = 0;

        int tempIndex = myList.at(0);
        myList.erase(myList.begin());

        permNode = tempIndex;

        if (tempIndex == -1) {
            // node not connected - should be able to break here
        } else {

            // mark node permanent
            myBuckets[permNode].MarkPermanent();
            myBuckets[permNode].UpdateReached();

            this->addFailOrder(permNode);
            this->addFailTime(currMax);

            // stores the failure order and time at which Targets are disconnected
            if (myBuckets[permNode].GetNodeID() >= numNodes - numTar) {
                this->addTargetFailOrder(myBuckets[permNode].GetNodeID());
                this->addTargetFailTime(myBuckets[permNode].GetFailCapacity());

                myBuckets[permNode].SetCovered();

            }
            else {
                this->addSensorFailOrder(myBuckets[permNode].GetNodeID());
                this->addSensorFailTime(myBuckets[permNode].GetFailCapacity());
            }

            // iterate through edges
            for (int k = 0; k < myBuckets[permNode].GetDegree(); ++k) {

                int updateNode1 = myBuckets[permNode].GetReachableNode(k);

                if (!myBuckets[updateNode1].IsReached()){
                    myList.push_back(updateNode1);
                    myBuckets[updateNode1].UpdateReached();
                }


                if (!myBuckets[updateNode1].IsPermanent()) {
                    double updateLabel = min(myBuckets[permNode].GetFailCapacity(), myBuckets[updateNode1].GetFailTime());
                    myBuckets[updateNode1].UpdateFailCapacity(updateLabel);
                }
            }
        }
        numPermanent = numPermanent + 1;
    }

    // If a target is not connected to the sink then it will never be marked permanent, so it will never be added to the
    // order of target failures. This for loop checks for such a case, and adds the targets to ths list with a fail time of zero
    if (orderedTargetFail.size() < numTar){
        SimulationNodeMC *ptr = myBuckets;
        ptr = ptr + (numNodes - numTar);
        for (int i = 0; i < numTar ; ++i) {
            if (!ptr->IsPermanent()){
                this->addTargetFailOrder(ptr->GetNodeID());
                this->addTargetFailTime(ptr->GetFailCapacity());
            }
            ptr = ptr + 1;
        }
    }

    // end time record
    auto finish = chrono::high_resolution_clock::now();
    // calculate execution time
    chrono::duration<double> elapsed = finish - start;
    timeDuration = elapsed.count();

}

// Used to deploy sensors to subregions with a smaller number of functioning sensors, rather than randomly over the entire region
void NodeLinkedListMC::TargetedDeployment(double costFixed, double costVar, double &policyCost) {

    double myXNum;
    double myYNum;
    int myTempBin;

    int numReplaced = 0;
    double maintCost = 0;

    vector<int> numberPerBucket;

    int myBucketsPerRow = (int) ceil(1 / myCommunicationRange);
    double myBucketWidth = 1 / (double) myBucketsPerRow;

    int myRegionsPerRow = 4;
    double myRegionWidth = 1 / (double) myRegionsPerRow;

    numberPerBucket.resize(myRegionsPerRow*myRegionsPerRow);

    SimulationNodeMC *ptr = myBuckets;
    //Sink Location
    ptr->SetNewLoc(0.5,0.5);
    // clear the forward start of the node, and nodes in bin zero
    ptr->ResetReachableNodes();
    ptr->ResetNodesInBucket();

    ptr = ptr + 1;

    // sets the pointers for a node to be the node immediately prior and immediately after the node
    for (int i = 1; i < myNumSensors ; ++i) {
        ptr->ResetReachableNodes();
        ptr->ResetNodesInBucket();

        if (!ptr->IsFailed()){
            myXNum = ptr->GetXLoc();
            myYNum = ptr->GetYLoc();

            // determine which grid number the new sensor location falls in
            myTempBin = ((int) ceil(myXNum / myRegionWidth) + (int) (ceil(myYNum / myRegionWidth))*myRegionsPerRow) - (myRegionsPerRow + 1);

            numberPerBucket[myTempBin] = numberPerBucket[myTempBin] + 1;
        }

        ptr = ptr + 1;

    }

    // Update value for target nodes
    for (int j = myNumSensors ; j < myNumElements ; ++j) {
        ptr->ResetReachableNodes();
        ptr->ResetNodesInBucket();
        ptr = ptr + 1;
    }

    int totalSum = 0;
    for (int k = 0; k < numberPerBucket.size() ; ++k) {
        totalSum = totalSum + numberPerBucket.at(k);
    }

    for (int k = 0; k < numberPerBucket.size() ; ++k) {
        numberPerBucket[k] = totalSum - numberPerBucket[k];
    }

    discrete_distribution<> binGenDist(numberPerBucket.begin(), numberPerBucket.end());

//    uniform_int_distribution<> binGenDist(0,195);

    // start placing sensor in bins, starting with sink node
    myXNum = myBuckets[0].GetXLoc();
    myYNum = myBuckets[0].GetYLoc();
    // determine which grid number the new sensor location falls in
    myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow +1);

    myBuckets[myTempBin].AddNodeToBucket(0);

    ptr = myBuckets;
    ptr = ptr + 1;

    // generate new location for every sensor
    for (int i = 1; i < myNumSensors ; ++i) {

        myXNum = myBuckets[i].GetXLoc();
        myYNum = myBuckets[i].GetYLoc();

        if (myBuckets[i].IsFailed()){
            // get bin based on number of sensors already in bin
            int newBin = binGenDist(myBinGen);

            double tempXLow = myBuckets[newBin].GetXLow();
            double tempXHigh = myBuckets[newBin].GetXHigh() + 0.001;
            double tempYLow = myBuckets[newBin].GetYLow();
            double tempYHigh = myBuckets[newBin].GetYHigh() + 0.001;

            uniform_real_distribution<> myUnifX(tempXLow, fmin(tempXHigh, 1.0));
            uniform_real_distribution<> myUnifY(tempYLow, fmin(tempYHigh, 1.0));

            myXNum = myUnifX(myDevX);
            myYNum = myUnifY(myDevY);

            ptr->ResetFailed();
            ptr->SetNewLoc(myXNum, myYNum);

            numReplaced = numReplaced + 1;

        }

        // determine which grid number the new sensor location falls in
        myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow + 1);

        // move sensor into a bin for this new location
        myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());

        ptr = ptr + 1;
    }


    // place target nodes in new bin
    for (int k = myNumSensors; k < myNumElements ; ++k) {

        myXNum = ptr->GetXLoc() + 0.0001;
        myYNum = ptr->GetYLoc() + 0.0001;

        // determine which grid number the new sensor location falls in
        myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow +1);

//        cout << " Node " << ptr->GetNodeID() << " -- XLoc : " << myXNum << " -- YLoc : " << myYNum << endl;

        // move sensor into a bin for this new location
        myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());

        ptr = ptr + 1;

    }

    // Calculate cost of replacing sensors
    if (numReplaced > 0){
        maintCost = ((double) numReplaced) * (double) costVar + costFixed;
        policyCost = maintCost;

    }
    else{
        policyCost = 0.0;
    }

    // reset failed nodes and covered nodes
    ptr = myBuckets;
    ptr = ptr + 1;

    for (int m = 1; m < myNumElements; ++m) {
        ptr->ResetFailed();
        ptr->ResetCovered();
        ptr = ptr + 1;
    }

    int myXBucket;
    int myYBucket;
    double dist;
    vector<int> myNodesInCurBucket;

    int curBucket = myBucketsPerRow + 1;
    for (int l = 0; l < pow(myBucketsPerRow,2) ; ++l) {
        // determine which grid cell we are currently in
        myXBucket = curBucket % myBucketsPerRow;
        myYBucket = (curBucket / myBucketsPerRow) % myBucketsPerRow ;

        if (myXBucket == 0){
            myXBucket = myBucketsPerRow;
            if (curBucket / myBucketsPerRow == myBucketsPerRow){
                myYBucket = myBucketsPerRow - 1;
            }
            else {
                myYBucket = myYBucket - 1;
            }
        }

        if (myYBucket == 0){
            myYBucket = myBucketsPerRow;
        }

        // all within bin comparison
        for (int i = 0; i < (myBuckets[l].GetNumberInCell()) ; ++i) {

            int startNode = myBuckets[l].GetNodeInBucket(i);

            // if the starting node is the sink node, then we only need to worry about connecting to sensor nodes, which is based on communication range
            if (startNode == 0) {
                for (int j = (i+1); j < myBuckets[l].GetNumberInCell() ; ++j) {
                    int endNode = myBuckets[l].GetNodeInBucket(j);
                    // determine if end node is a sensor node or a target node
                    if (endNode < myNumSensors){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        if (dist <= myCommunicationRange) {
                            myBuckets[startNode].AddReachableNode(endNode);
                        }
                    }
                }
            }
                // if starting node is a sensor node
            else if (startNode > 0 && startNode < myNumSensors){

                for (int j = (i+1); j < myBuckets[l].GetNumberInCell() ; ++j) {
                    int endNode = myBuckets[l].GetNodeInBucket(j);

                    // start node is a sensor node, end node is sink node, check for reverse arc
                    if (endNode == 0){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        // sensor to sink comparison, we can add both arcs
                        if (dist <= myCommunicationRange) {
                            myBuckets[endNode].AddReachableNode(startNode);
                        }

                    }
                    else if (endNode < myNumSensors){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        // sensor to sensor comparison, we can add both arcs
                        if (dist <= myCommunicationRange) {
                            myBuckets[startNode].AddReachableNode(endNode);
                            myBuckets[endNode].AddReachableNode(startNode);
                        }
                    }
                    else {
                        // end node must be a target node
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        if (dist <= mySensorRange){
                            myBuckets[startNode].AddReachableNode(endNode);
                        }
                    }
                }
            }
        }

        int myAdjacentCell;

        // look to the cell immediately above current cell
        int myTopBucket = myYBucket + 1;
        myAdjacentCell = myXBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow +1);
        if (l < myAdjacentCell && myTopBucket <= myBucketsPerRow){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell immediately to the right
        int myRightBucket = myXBucket + 1;
        myAdjacentCell = myRightBucket + (myYBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myRightBucket <= myBucketsPerRow){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell immediately below
        int myBottomBucket = myYBucket - 1;
        myAdjacentCell = myXBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myBottomBucket < 0){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to left of current cell
        int myLeftBucket = myXBucket - 1;
        myAdjacentCell = myLeftBucket + (myYBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myLeftBucket > 0){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the top right
        myAdjacentCell = myRightBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myRightBucket <= myBucketsPerRow) && (myTopBucket <= myBucketsPerRow)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the bottom right
        myAdjacentCell = myRightBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myRightBucket <= myBucketsPerRow) && (myBottomBucket > 0)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the top left
        myAdjacentCell = myLeftBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l< myAdjacentCell) && (myLeftBucket > 0 ) && (myTopBucket <= myBucketsPerRow)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the bottom left
        myAdjacentCell = myLeftBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myLeftBucket > 0 ) && (myBottomBucket > 0)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        curBucket = curBucket + 1;
    }

}

void NodeLinkedListMC::SetFailedNodes(vector<int> failNodes, int numFailed){

    int tempFail;

    for (int i = 0; i < numFailed ; ++i) {
        tempFail = failNodes.at(i);

        myBuckets[tempFail].SetFailed();

    }
}

// used to generate update RGG in a maintenance policy. In this function, failed nodes are replaced with a new node at the same location
void NodeLinkedListMC::GenerateNewRGG_BucketMaintNodeReplace(double costFixed, double costVar, double &policyCost){

    double myXNum;
    double myYNum;
    int myTempBin;
    int numReplaced = 0;
    bool maintOccur = false;
    double maintCost = 0;

    int myBucketsPerRow = (int) ceil(1 / myCommunicationRange);

    SimulationNodeMC *ptr = myBuckets;
    //Sink Location
    ptr->SetNewLoc(0.5,0.5);
    // clear the forward start of the node, and nodes in bin zero
    ptr->ResetReachableNodes();
    ptr->ResetNodesInBucket();

    ptr = ptr + 1;

    // sets the pointers for a node to be the node immediately prior and immediately after the node
    for (int i = 1; i < myNumSensors ; ++i) {
        ptr->ResetReachableNodes();
        ptr->ResetNodesInBucket();
//        ptr->ResetNode();
        ptr = ptr + 1;

    }

    // Update value for target nodes
    for (int j = myNumSensors ; j < myNumElements ; ++j) {
//        ptr->ResetNode();
        ptr->ResetReachableNodes();
        ptr->ResetNodesInBucket();
        ptr = ptr + 1;
    }

    // start placing sensor in bins, starting with sink node
    myXNum = myBuckets[0].GetXLoc();
    myYNum = myBuckets[0].GetYLoc();
    // determine which grid number the new sensor location falls in
    myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow +1);

    myBuckets[myTempBin].AddNodeToBucket(0);

    ptr = myBuckets;
    ptr = ptr + 1;

    // generate new location for every sensor
    for (int i = 1; i < myNumSensors ; ++i) {

        myXNum = myBuckets[i].GetXLoc();
        myYNum = myBuckets[i].GetYLoc();

        if (myBuckets[i].IsFailed()){
            ptr->ResetFailed();
//            ptr->SetNewLoc(myXNum, myYNum);

            numReplaced = numReplaced + 1;
            maintOccur = true;
        }

        // determine which grid number the new sensor location falls in
        myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow + 1);

        // move sensor into a bin for this new location
        myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());

        ptr = ptr + 1;
    }

    // place target nodes in new bin
    for (int k = myNumSensors; k < myNumElements ; ++k) {

        myXNum = ptr->GetXLoc() + 0.0001;
        myYNum = ptr->GetYLoc() + 0.0001;

        // determine which grid number the new sensor location falls in
        myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow +1);

        // move sensor into a bin for this new location
        myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());

        ptr = ptr + 1;

    }

    // Calculate cost of replacing sensors
    if (numReplaced > 0){
        maintCost = ((double) numReplaced) * (double) costVar + costFixed;
        policyCost = maintCost;

    }
    else{
        policyCost = 0.0;
    }

    // reset failed nodes and covered nodes
    ptr = myBuckets;
    ptr = ptr + 1;
    for (int m = 1; m < myNumElements; ++m) {
        ptr->ResetFailed();
        ptr->ResetCovered();
        ptr = ptr + 1;
    }

    int myXBucket;
    int myYBucket;
    double dist;
    vector<int> myNodesInCurBucket;

    int curBucket = myBucketsPerRow + 1;
    for (int l = 0; l < pow(myBucketsPerRow,2) ; ++l) {
        // determine which grid cell we are currently in
        myXBucket = curBucket % myBucketsPerRow;
        myYBucket = (curBucket / myBucketsPerRow) % myBucketsPerRow ;

        if (myXBucket == 0){
            myXBucket = myBucketsPerRow;
            if (curBucket / myBucketsPerRow == myBucketsPerRow){
                myYBucket = myBucketsPerRow - 1;
            }
            else {
                myYBucket = myYBucket - 1;
            }
        }

        if (myYBucket == 0){
            myYBucket = myBucketsPerRow;
        }

        // all within bin comparison
        for (int i = 0; i < (myBuckets[l].GetNumberInCell()) ; ++i) {

            int startNode = myBuckets[l].GetNodeInBucket(i);

            // if the starting node is the sink node, then we only need to worry about connecting to sensor nodes, which is based on communication range
            if (startNode == 0) {
                for (int j = (i+1); j < myBuckets[l].GetNumberInCell() ; ++j) {
                    int endNode = myBuckets[l].GetNodeInBucket(j);
                    // determine if end node is a sensor node or a target node
                    if (endNode < myNumSensors){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        if (dist <= myCommunicationRange) {
                            myBuckets[startNode].AddReachableNode(endNode);
                        }
                    }
                }
            }
                // if starting node is a sensor node
            else if (startNode > 0 && startNode < myNumSensors){

                for (int j = (i+1); j < myBuckets[l].GetNumberInCell() ; ++j) {
                    int endNode = myBuckets[l].GetNodeInBucket(j);

                    // start node is a sensor node, end node is sink node, check for reverse arc
                    if (endNode == 0){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        // sensor to sink comparison, we can add both arcs
                        if (dist <= myCommunicationRange) {
                            myBuckets[endNode].AddReachableNode(startNode);
                        }

                    }
                    else if (endNode < myNumSensors){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        // sensor to sensor comparison, we can add both arcs
                        if (dist <= myCommunicationRange) {
                            myBuckets[startNode].AddReachableNode(endNode);
                            myBuckets[endNode].AddReachableNode(startNode);
                        }
                    }
                    else {
                        // end node must be a target node
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        if (dist <= mySensorRange){
                            myBuckets[startNode].AddReachableNode(endNode);
                        }
                    }
                }
            }
        }

        int myAdjacentCell;

        // look to the cell immediately above current cell
        int myTopBucket = myYBucket + 1;
        myAdjacentCell = myXBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow +1);
        if (l < myAdjacentCell && myTopBucket <= myBucketsPerRow){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell immediately to the right
        int myRightBucket = myXBucket + 1;
        myAdjacentCell = myRightBucket + (myYBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myRightBucket <= myBucketsPerRow){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell immediately below
        int myBottomBucket = myYBucket - 1;
        myAdjacentCell = myXBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myBottomBucket < 0){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to left of current cell
        int myLeftBucket = myXBucket - 1;
        myAdjacentCell = myLeftBucket + (myYBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myLeftBucket > 0){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the top right
        myAdjacentCell = myRightBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myRightBucket <= myBucketsPerRow) && (myTopBucket <= myBucketsPerRow)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the bottom right
        myAdjacentCell = myRightBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myRightBucket <= myBucketsPerRow) && (myBottomBucket > 0)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the top left
        myAdjacentCell = myLeftBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l< myAdjacentCell) && (myLeftBucket > 0 ) && (myTopBucket <= myBucketsPerRow)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the bottom left
        myAdjacentCell = myLeftBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myLeftBucket > 0 ) && (myBottomBucket > 0)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        curBucket = curBucket + 1;
    }
}

void NodeLinkedListMC::ResetAllNodes(){
    for (int i = 0; i < myNumElements ; ++i) {
        myBuckets[i].ResetFailed();
        myBuckets[i].ResetCovered();
        myBuckets[i].ResetNodeReached();
    }
}

void NodeLinkedListMC::ClearSensorAge() {
    for (int i = 0; i < myNumElements ; ++i) {
        myBuckets[i].SetAge(0);
    }
}

void NodeLinkedListMC::SubregionArea(vector<double> &areaVect) {

    for (int i = 0; i < myNumSubregions ; ++i) {
        double xLow = myBuckets[i].GetXLow();
        double xHigh = myBuckets[i].GetXHigh();

        double yLow = myBuckets[i].GetYLow();
        double yHigh = myBuckets[i].GetYHigh();

        double area = (xHigh - xLow) * (yHigh - yLow);
        areaVect.push_back(area);
    }
}

void NodeLinkedListMC::SubregionDistanceWeight(vector<double> &subWeights){

    vector<double> distVect;
    double totalDist = 0;

    for (int i = 0; i < myNumSubregions ; ++i) {
        double xLow = myBuckets[i].GetXLow();
        double xHigh = myBuckets[i].GetXHigh();

        double yLow = myBuckets[i].GetYLow();
        double yHigh = myBuckets[i].GetYHigh();

        double dist = pow(((xLow + xHigh) / 2.0) - 0.5, 2) + pow(((yLow + yHigh) / 2.0) - 0.5, 2);
        dist = pow(dist, 0.5);
        distVect.push_back(1.0 / dist);
        totalDist = totalDist + (1.0 / dist);

    }

    for (int j = 0; j < myNumSubregions ; ++j) {
        subWeights.push_back(distVect.at(j) / totalDist);
    }
}

void NodeLinkedListMC::CustomWeights(vector<double> &subWeights){

    subWeights.push_back(1.0);
    subWeights.push_back(1.0);
    subWeights.push_back(1.0);
    subWeights.push_back(1.0);

    subWeights.push_back(1.0);
    subWeights.push_back(1.0);
    subWeights.push_back(1.0);
    subWeights.push_back(1.0);

    subWeights.push_back(1.0);
    subWeights.push_back(1.0);
    subWeights.push_back(1.0);
    subWeights.push_back(1.0);

    subWeights.push_back(1.0);
    subWeights.push_back(1.0);
    subWeights.push_back(1.0);
    subWeights.push_back(1.0);

}

// used to generate new fail times for a maintenance policy, where new sensors are deployed at time curTime
void NodeLinkedListMC::GenerateNewFailTimeMyopicCBM(vector<double> &nodeTime, double t, double curTime, int numReplaced){

    weibull_distribution<double> myWeibull(1.5,10.0);

    int curIndex = 0;

    int curSensor = 1;
    while (curIndex < numReplaced && curSensor < myNumSensors){

        if (myBuckets[curSensor].IsFailed()){
            double myTime = myWeibull(myDevTime);
            nodeTime[curSensor] = myTime + curTime;

            myBuckets[curSensor].SetReplaced();
            myBuckets[curSensor].SetAge(0);
            myBuckets[curSensor].SetSubregion(1);
            curIndex = curIndex + 1;

        }
        curSensor = curSensor + 1;
    }

    nodeTime[0] = t;

    // initial time for all targets
    for (int j = myNumSensors; j < myNumElements; ++j) {
        nodeTime[j] = t;
    }
}

// used to generate new fail times for a maintenance policy, where new sensors are deployed at time curTime
void NodeLinkedListMC::GenerateNewFailTimeADP(vector<double> &nodeTime, double t, double curTime, int numReplaced){

    weibull_distribution<double> myWeibull(1.5,10.0);

    int curIndex = 0;

    int curSensor = 1;
    while (curIndex < numReplaced && curSensor < myNumSensors){

        if (myBuckets[curSensor].IsFailed()){
            double myTime = myWeibull(myDevTime);
            nodeTime[curSensor] = myTime + curTime;

            myBuckets[curSensor].SetReplaced();
            myBuckets[curSensor].SetAge(0);
            myBuckets[curSensor].SetSubregion(1);
            myBuckets[curSensor].SetFailTime(myTime + curTime);

            myBuckets[curSensor].ResetReplaced();

            curIndex = curIndex + 1;

        }
        curSensor = curSensor + 1;
    }

    nodeTime[0] = t;

}

// used to generate new fail times for a maintenance policy, where new sensors are deployed at time curTime
void NodeLinkedListMC::GenerateNewFailTimeADP_Update(vector<double> &nodeTime, double t, double curTime,
                                                     int numReplaced){

    weibull_distribution<double> myWeibull(1.5,10.0);

    int curIndex = 0;

    int curSensor = 1;
    while (curIndex < numReplaced && curSensor < myNumSensors){

        if (myBuckets[curSensor].IsFailed()){
            double myTime = myWeibull(myDevTime);
            nodeTime[curSensor] = myTime + curTime;

            myBuckets[curSensor].SetReplaced();
            myBuckets[curSensor].SetAge(0);
            myBuckets[curSensor].SetSubregion(1);
            myBuckets[curSensor].SetFailTime(myTime + curTime);
            myBuckets[curSensor].ResetFailed();

            myBuckets[curSensor].ResetReplaced();

            curIndex = curIndex + 1;

        }
        curSensor = curSensor + 1;
    }

    nodeTime[0] = t;

}

// used to generate new fail times for a maintenance policy, where new sensors are deployed at time curTime
void NodeLinkedListMC::GenerateNewFailTimeADP_ADPEval(vector<double> &nodeTime, double t, double curTime,
                                                      vector<int> newDeploymentCount){

    weibull_distribution<double> myWeibull(1.5,10.0);

    int totalReplace = newDeploymentCount.size();

    int curIndex = 0;

    int curSensor = 1;
    while (curIndex < totalReplace && curSensor < myNumSensors){

        if (myBuckets[curSensor].IsFailed()){
            double myTime = myWeibull(myDevTime);
            nodeTime[curSensor] = myTime + curTime;

            myBuckets[curSensor].SetReplaced();
            myBuckets[curSensor].SetAge(0);
            myBuckets[curSensor].SetSubregion(newDeploymentCount.at(curIndex));
            myBuckets[curSensor].SetFailTime(myTime + curTime);

            curIndex = curIndex + 1;

        }
        curSensor = curSensor + 1;
    }

    nodeTime[0] = t;

}

void NodeLinkedListMC::UpdateFailTimesADP(vector<double> newTimes){

    orderedCriticalFailTime.clear();
    orderedTargetCriticalFailTime.clear();
    orderedSensorCriticalFailTime.clear();

    this->ClearOrderTimes();

    double maxFail = *max_element(newTimes.begin(), newTimes.end());

    SimulationNodeMC *ptr = myBuckets;
    // update all values for the sink node
    ptr->SetID(0);
    ptr->UpdateFailCapacity(maxFail);
    ptr->UpdateReached();
    ptr->MarkPermanent();
    ptr->SetFailTime(maxFail);

    ptr->SetFirstItem((ptr + 1));

    ptr = ptr + 1;

    // sets the pointers for a node to be the node immediately prior and immediately after the node
    for (int i = 1; i < myNumSensors ; ++i) {

        // reset initial status
        ptr->SetAge(0);
        ptr->SetFailTime(newTimes[i]);
        ptr->ResetFailed();
        ptr->ResetReplaced();

        ptr = ptr + 1;
    }

}


// used to generate new fail times for a maintenance policy, where new sensors are deployed at time curTime
void NodeLinkedListMC::GenerateNewFailTimeMyopicCBM(vector<double> &nodeTime, double t, double curTime, vector<int> newDeploymentCount){

    weibull_distribution<double> myWeibull(1.5,10.0);

    int totalReplace = newDeploymentCount.size();

    int curIndex = 0;

    int curSensor = 1;
    while (curIndex < totalReplace && curSensor < myNumSensors){

        if (myBuckets[curSensor].IsFailed()){
            double myTime = myWeibull(myDevTime);
            nodeTime[curSensor] = myTime + curTime;

            myBuckets[curSensor].SetReplaced();
            myBuckets[curSensor].SetAge(0);
            myBuckets[curSensor].SetSubregion(newDeploymentCount.at(curIndex));
            curIndex = curIndex + 1;

        }
        curSensor = curSensor + 1;
    }

    nodeTime[0] = t;

    // initial time for all targets
    for (int j = myNumSensors; j < myNumElements; ++j) {
        nodeTime[j] = t;
    }

//    int countT = 0;
////    cout << "Current Time " << curTime << endl;
//    for (int i = 1; i < myNumSensors ; ++i) {
//
//   //     cout << " Sensor " << i << " FailTime : " << myBuckets[i].GetFailTime() << " IsFailed? " << myBuckets[i].IsFailed() << " IsReplaced? " << myBuckets[i].IsReplaced() << " newTime " << nodeTime[i] << endl;
//
//        if (!myBuckets[i].IsFailed() || myBuckets[i].IsReplaced()){
//            countT = countT + 1;
//        }
//    }
//
//    cout << "Network Size "<< countT<< endl;

}

void NodeLinkedListMC::UpdateRGGMyopicCBM(double time, vector<int> &numInRegion) {

    vector<double> tempExp(myNumSubregions, 0);
    vector<int> ageCount(10,0);
    double delta = 1.0;
    this->UpdateRGGMyopicCBM(time, delta, numInRegion, ageCount, tempExp);

}

void NodeLinkedListMC::UpdateRGGMyopicCBM(double time, vector<int> &numInRegion, vector<int> &ageCount) {

    vector<double> tempExp(myNumSubregions, 0);
    double delta = 1.0;
    this->UpdateRGGMyopicCBM(time, delta, numInRegion, ageCount, tempExp);

}

void NodeLinkedListMC::UpdateRGGMyopicCBM(double time, double delta, vector<int> &numInRegion, vector<int> &ageCount, vector<double> &expectedNum){

    WeibullDistribution mySensorDist(1.5, 10.0);

    int maxAge = ageCount.size();

    //    cout << "Current Time " << time << endl;
    for (int i = 0; i < myNumSensors ; ++i) {

        if (myBuckets[i].GetFailTime() < time){
            myBuckets[i].SetFailed();
        }
        else{

            if (i > 0){
                int myReg = myBuckets[i].GetSubregion();
                numInRegion[myReg] = numInRegion[myReg] + 1;

                int tempAge = (int) myBuckets[i].GetAge();

                double survivalProb = (1.0 - mySensorDist.cdf(((double) (1 + tempAge)) * delta)) / (1.0 - mySensorDist.cdf(((double) tempAge) * delta));
                expectedNum[myReg] = expectedNum[myReg] + survivalProb;

                if (tempAge + 1 < maxAge){
                    ageCount[tempAge + 1] = ageCount[tempAge + 1];
                }

                myBuckets[i].IncreaseAge();
            }

            int k = 0;
            while (k < myBuckets[i].GetDegree()){

                int tempNode = myBuckets[i].GetReachableNode(k);
                if (myBuckets[tempNode].GetFailTime() < time){
                    myBuckets[i].UpdateForwardStar(k);
                    myBuckets[tempNode].SetFailed();
                }

                k = k + 1;
            }
        }
    }
}

void NodeLinkedListMC::UpdateState(double time, double delta, vector<int> &ageCount, double &expectedNum){

    WeibullDistribution mySensorDist(1.5, 10.0);

    int maxAge = ageCount.size();
    expectedNum = 0;

    for (int i = 1; i < myNumSensors ; ++i) {

        if (myBuckets[i].GetFailTime() < time){
            myBuckets[i].SetFailed();
            myBuckets[i].SetInactive();
        }
        else{

            int tempAge = (int) myBuckets[i].GetAge();

            double survivalProb = (1.0 - mySensorDist.cdf(((double) (1 + tempAge)) * delta)) / (1.0 - mySensorDist.cdf(((double) tempAge) * delta));
            expectedNum = expectedNum + survivalProb;

            if (tempAge + 1 < maxAge){
                ageCount[tempAge + 1] = ageCount[tempAge + 1] + 1;
            }
            myBuckets[i].IncreaseAge();
        }
    }
}

void NodeLinkedListMC::UpdateState(double time, double delta, vector<int> &numInRegion, vector<int> &ageCount, double &expectedNum){

    WeibullDistribution mySensorDist(1.5, 10.0);

    int maxAge = ageCount.size();
    expectedNum = 0;

    for (int i = 1; i < myNumSensors ; ++i) {

        if (myBuckets[i].GetFailTime() < time){
            myBuckets[i].SetFailed();
            myBuckets[i].SetInactive();
        }
        else{

            int myReg = myBuckets[i].GetSubregion();
            numInRegion[myReg] = numInRegion[myReg] + 1;

            int tempAge = (int) myBuckets[i].GetAge();

            double survivalProb = (1.0 - mySensorDist.cdf(((double) (1 + tempAge)) * delta)) / (1.0 - mySensorDist.cdf(((double) tempAge) * delta));
            expectedNum = expectedNum + survivalProb;

            if (tempAge + 1 < maxAge){
                ageCount[tempAge + 1] = ageCount[tempAge + 1] + 1;
            }
            myBuckets[i].IncreaseAge();
        }
    }
}

// used to generate a new RGG in a maintenance policy, but also records the number of the new sensors that are deployed, and the number of sensors that are
// disconnected from the sink node at the time of maintenance
void NodeLinkedListMC::GenerateNewRGG_BucketCBMMaint(double costFixed, double costVar, double &policyCost, int &numDisconnected, int &numDeployed){

    uniform_real_distribution<> myUnif(0,1);

    double myXNum;
    double myYNum;
    int myTempBin;
    int numReplaced = 0;
    bool maintOccur = false;
    double maintCost = 0;

    int myBucketsPerRow = (int) ceil(1 / myCommunicationRange);

    SimulationNodeMC *ptr = myBuckets;
    //Sink Location
    ptr->SetNewLoc(0.5,0.5);
    // clear the forward start of the node, and nodes in bin zero
    ptr->ResetReachableNodes();
    ptr->ResetNodesInBucket();

    ptr = ptr + 1;
    numDisconnected = 0;

    // sets the pointers for a node to be the node immediately prior and immediately after the node
    for (int i = 1; i < myNumSensors ; ++i) {
        ptr->ResetReachableNodes();
        ptr->ResetNodesInBucket();

        if (!ptr->IsReached()){
            numDisconnected = numDisconnected + 1;
        }
//        ptr->ResetNode();
        ptr = ptr + 1;

    }

    // Update value for target nodes
    for (int j = myNumSensors ; j < myNumElements ; ++j) {
//        ptr->ResetNode();
        ptr->ResetReachableNodes();
        ptr->ResetNodesInBucket();
        ptr = ptr + 1;
    }

    // start placing sensor in bins, starting with sink node
    myXNum = myBuckets[0].GetXLoc();
    myYNum = myBuckets[0].GetYLoc();
    // determine which grid number the new sensor location falls in
    myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow +1);

    myBuckets[myTempBin].AddNodeToBucket(0);

    ptr = myBuckets;
    ptr = ptr + 1;

    // generate new location for every sensor
    for (int i = 1; i < myNumSensors ; ++i) {

        myXNum = myBuckets[i].GetXLoc();
        myYNum = myBuckets[i].GetYLoc();

        if (!myBuckets[i].IsFailed()){

            // determine which grid number the new sensor location falls in
            myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow + 1);

            // move sensor into a bin for this new location
            myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());
        }


        if (myBuckets[i].IsReplaced()){

            int myTempReg = myBuckets[i].GetSubregion();

            double tempXLow = myBuckets[myTempReg].GetXLow();
            double tempXHigh = myBuckets[myTempReg].GetXHigh() + 0.001;
            double tempYLow = myBuckets[myTempReg].GetYLow();
            double tempYHigh = myBuckets[myTempReg].GetYHigh() + 0.001;

            uniform_real_distribution<> myUnifX(tempXLow, fmin(tempXHigh, 1.0));
            uniform_real_distribution<> myUnifY(tempYLow, fmin(tempYHigh, 1.0));

            myXNum = myUnifX(myDevX);
            myYNum = myUnifY(myDevY);

            ptr->ResetFailed();
            ptr->ResetReplaced();
            ptr->SetNewLoc(myXNum, myYNum);

            // determine which grid number the new sensor location falls in
            myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow + 1);

            // move sensor into a bin for this new location
            myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());

            numReplaced = numReplaced + 1;
            maintOccur = true;

        }

        ptr = ptr + 1;
    }

    // place target nodes in new bin
    for (int k = myNumSensors; k < myNumElements ; ++k) {

        myXNum = ptr->GetXLoc() + 0.0001;
        myYNum = ptr->GetYLoc() + 0.0001;

        // determine which grid number the new sensor location falls in
        myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow +1);

        // move sensor into a bin for this new location
        myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());

        ptr = ptr + 1;

    }


    // Calculate cost of replacing sensors
    if (numReplaced > 0){
        maintCost = ((double) numReplaced) * (double) costVar + costFixed;
        policyCost = maintCost;

    }
    else{
        policyCost = 0.0;
    }
    numDeployed = numReplaced;

    // reset failed nodes and covered nodes
    ptr = myBuckets;
    ptr = ptr + 1;
    for (int m = 1; m < myNumElements; ++m) {
//        ptr->ResetFailed();
        ptr->ResetCovered();
        ptr = ptr + 1;
    }

    int myXBucket;
    int myYBucket;
    double dist;
    vector<int> myNodesInCurBucket;

    int curBucket = myBucketsPerRow + 1;
    for (int l = 0; l < pow(myBucketsPerRow,2) ; ++l) {
        // determine which grid cell we are currently in
        myXBucket = curBucket % myBucketsPerRow;
        myYBucket = (curBucket / myBucketsPerRow) % myBucketsPerRow ;

        if (myXBucket == 0){
            myXBucket = myBucketsPerRow;
            if (curBucket / myBucketsPerRow == myBucketsPerRow){
                myYBucket = myBucketsPerRow - 1;
            }
            else {
                myYBucket = myYBucket - 1;
            }
        }

        if (myYBucket == 0){
            myYBucket = myBucketsPerRow;
        }

        // all within bin comparison
        for (int i = 0; i < (myBuckets[l].GetNumberInCell()) ; ++i) {

            int startNode = myBuckets[l].GetNodeInBucket(i);

            // if the starting node is the sink node, then we only need to worry about connecting to sensor nodes, which is based on communication range
            if (startNode == 0) {
                for (int j = (i+1); j < myBuckets[l].GetNumberInCell() ; ++j) {
                    int endNode = myBuckets[l].GetNodeInBucket(j);
                    // determine if end node is a sensor node or a target node
                    if (endNode < myNumSensors){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        if (dist <= myCommunicationRange) {
                            myBuckets[startNode].AddReachableNode(endNode);
                        }
                    }
                }
            }
                // if starting node is a sensor node
            else if (startNode > 0 && startNode < myNumSensors){

                for (int j = (i+1); j < myBuckets[l].GetNumberInCell() ; ++j) {
                    int endNode = myBuckets[l].GetNodeInBucket(j);

                    // start node is a sensor node, end node is sink node, check for reverse arc
                    if (endNode == 0){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        // sensor to sink comparison, we can add both arcs
                        if (dist <= myCommunicationRange) {
                            myBuckets[endNode].AddReachableNode(startNode);
                        }

                    }
                    else if (endNode < myNumSensors){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        // sensor to sensor comparison, we can add both arcs
                        if (dist <= myCommunicationRange) {
                            myBuckets[startNode].AddReachableNode(endNode);
                            myBuckets[endNode].AddReachableNode(startNode);
                        }
                    }
                    else {
                        // end node must be a target node
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        if (dist <= mySensorRange){
                            myBuckets[startNode].AddReachableNode(endNode);
                        }
                    }
                }
            }
        }

        int myAdjacentCell;

        // look to the cell immediately above current cell
        int myTopBucket = myYBucket + 1;
        myAdjacentCell = myXBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow +1);
        if (l < myAdjacentCell && myTopBucket <= myBucketsPerRow){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell immediately to the right
        int myRightBucket = myXBucket + 1;
        myAdjacentCell = myRightBucket + (myYBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myRightBucket <= myBucketsPerRow){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell immediately below
        int myBottomBucket = myYBucket - 1;
        myAdjacentCell = myXBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myBottomBucket < 0){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to left of current cell
        int myLeftBucket = myXBucket - 1;
        myAdjacentCell = myLeftBucket + (myYBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myLeftBucket > 0){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the top right
        myAdjacentCell = myRightBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myRightBucket <= myBucketsPerRow) && (myTopBucket <= myBucketsPerRow)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the bottom right
        myAdjacentCell = myRightBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myRightBucket <= myBucketsPerRow) && (myBottomBucket > 0)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the top left
        myAdjacentCell = myLeftBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l< myAdjacentCell) && (myLeftBucket > 0 ) && (myTopBucket <= myBucketsPerRow)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the bottom left
        myAdjacentCell = myLeftBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myLeftBucket > 0 ) && (myBottomBucket > 0)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        curBucket = curBucket + 1;
    }

}

// used to generate a new RGG in a maintenance policy, but also records the number of the new sensors that are deployed, and the number of sensors that are
// disconnected from the sink node at the time of maintenance
void NodeLinkedListMC::GenerateNewRGG_ADPMaintAction(){

    uniform_real_distribution<> myUnif(0,1);

    double myXNum;
    double myYNum;
    int myTempBin;

    int myBucketsPerRow = (int) ceil(1 / myCommunicationRange);

    SimulationNodeMC *ptr = myBuckets;
    //Sink Location
    ptr->SetNewLoc(0.5,0.5);
    // clear the forward start of the node, and nodes in bin zero
    ptr->ResetReachableNodes();
    ptr->ResetNodesInBucket();

    ptr = ptr + 1;

    // sets the pointers for a node to be the node immediately prior and immediately after the node
    for (int i = 1; i < myNumSensors ; ++i) {
        ptr->ResetReachableNodes();
        ptr->ResetNodesInBucket();

        ptr = ptr + 1;

    }

    // Update value for target nodes
    for (int j = myNumSensors ; j < myNumElements ; ++j) {
//        ptr->ResetNode();
        ptr->ResetReachableNodes();
        ptr->ResetNodesInBucket();
        ptr = ptr + 1;
    }

    // start placing sensor in bins, starting with sink node
    myXNum = myBuckets[0].GetXLoc();
    myYNum = myBuckets[0].GetYLoc();
    // determine which grid number the new sensor location falls in
    myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow +1);

    myBuckets[myTempBin].AddNodeToBucket(0);

    ptr = myBuckets;
    ptr = ptr + 1;

    // generate new location for every sensor
    for (int i = 1; i < myNumSensors ; ++i) {

        myXNum = myBuckets[i].GetXLoc();
        myYNum = myBuckets[i].GetYLoc();

        if (!myBuckets[i].IsFailed() && myBuckets[i].IsActive()){

            // determine which grid number the new sensor location falls in
            myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow + 1);

            // move sensor into a bin for this new location
            myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());
        }


        if (myBuckets[i].IsReplaced()){

            int myTempReg = myBuckets[i].GetSubregion();

            double tempXLow = myBuckets[myTempReg].GetXLow();
            double tempXHigh = myBuckets[myTempReg].GetXHigh() + 0.001;
            double tempYLow = myBuckets[myTempReg].GetYLow();
            double tempYHigh = myBuckets[myTempReg].GetYHigh() + 0.001;

            uniform_real_distribution<> myUnifX(tempXLow, fmin(tempXHigh, 1.0));
            uniform_real_distribution<> myUnifY(tempYLow, fmin(tempYHigh, 1.0));

            myXNum = myUnifX(myDevX);
            myYNum = myUnifY(myDevY);

            ptr->ResetFailed();
            ptr->ResetReplaced();
            ptr->SetActive();
            ptr->SetNewLoc(myXNum, myYNum);

            // determine which grid number the new sensor location falls in
            myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow + 1);

            // move sensor into a bin for this new location
            myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());

        }

        ptr = ptr + 1;
    }

    // place target nodes in new bin
    for (int k = myNumSensors; k < myNumElements ; ++k) {

        myXNum = ptr->GetXLoc() + 0.0001;
        myYNum = ptr->GetYLoc() + 0.0001;

        // determine which grid number the new sensor location falls in
        myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow +1);

        // move sensor into a bin for this new location
        myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());

        ptr = ptr + 1;

    }

    // reset failed nodes and covered nodes
    ptr = myBuckets;
    ptr = ptr + 1;
    for (int m = 1; m < myNumElements; ++m) {
//        ptr->ResetFailed();
        ptr->ResetCovered();
        ptr = ptr + 1;
    }

    int myXBucket;
    int myYBucket;
    double dist;
    vector<int> myNodesInCurBucket;

    int curBucket = myBucketsPerRow + 1;
    for (int l = 0; l < pow(myBucketsPerRow,2) ; ++l) {
        // determine which grid cell we are currently in
        myXBucket = curBucket % myBucketsPerRow;
        myYBucket = (curBucket / myBucketsPerRow) % myBucketsPerRow ;

        if (myXBucket == 0){
            myXBucket = myBucketsPerRow;
            if (curBucket / myBucketsPerRow == myBucketsPerRow){
                myYBucket = myBucketsPerRow - 1;
            }
            else {
                myYBucket = myYBucket - 1;
            }
        }

        if (myYBucket == 0){
            myYBucket = myBucketsPerRow;
        }

        // all within bin comparison
        for (int i = 0; i < (myBuckets[l].GetNumberInCell()) ; ++i) {

            int startNode = myBuckets[l].GetNodeInBucket(i);

            // if the starting node is the sink node, then we only need to worry about connecting to sensor nodes, which is based on communication range
            if (startNode == 0) {
                for (int j = (i+1); j < myBuckets[l].GetNumberInCell() ; ++j) {
                    int endNode = myBuckets[l].GetNodeInBucket(j);
                    // determine if end node is a sensor node or a target node
                    if (endNode < myNumSensors){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        if (dist <= myCommunicationRange) {
                            myBuckets[startNode].AddReachableNode(endNode);
                        }
                    }
                }
            }
                // if starting node is a sensor node
            else if (startNode > 0 && startNode < myNumSensors){

                for (int j = (i+1); j < myBuckets[l].GetNumberInCell() ; ++j) {
                    int endNode = myBuckets[l].GetNodeInBucket(j);

                    // start node is a sensor node, end node is sink node, check for reverse arc
                    if (endNode == 0){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        // sensor to sink comparison, we can add both arcs
                        if (dist <= myCommunicationRange) {
                            myBuckets[endNode].AddReachableNode(startNode);
                        }

                    }
                    else if (endNode < myNumSensors){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        // sensor to sensor comparison, we can add both arcs
                        if (dist <= myCommunicationRange) {
                            myBuckets[startNode].AddReachableNode(endNode);
                            myBuckets[endNode].AddReachableNode(startNode);
                        }
                    }
                    else {
                        // end node must be a target node
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        if (dist <= mySensorRange){
                            myBuckets[startNode].AddReachableNode(endNode);
                        }
                    }
                }
            }
        }

        int myAdjacentCell;

        // look to the cell immediately above current cell
        int myTopBucket = myYBucket + 1;
        myAdjacentCell = myXBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow +1);
        if (l < myAdjacentCell && myTopBucket <= myBucketsPerRow){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell immediately to the right
        int myRightBucket = myXBucket + 1;
        myAdjacentCell = myRightBucket + (myYBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myRightBucket <= myBucketsPerRow){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell immediately below
        int myBottomBucket = myYBucket - 1;
        myAdjacentCell = myXBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myBottomBucket < 0){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to left of current cell
        int myLeftBucket = myXBucket - 1;
        myAdjacentCell = myLeftBucket + (myYBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myLeftBucket > 0){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the top right
        myAdjacentCell = myRightBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myRightBucket <= myBucketsPerRow) && (myTopBucket <= myBucketsPerRow)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the bottom right
        myAdjacentCell = myRightBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myRightBucket <= myBucketsPerRow) && (myBottomBucket > 0)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the top left
        myAdjacentCell = myLeftBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l< myAdjacentCell) && (myLeftBucket > 0 ) && (myTopBucket <= myBucketsPerRow)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the bottom left
        myAdjacentCell = myLeftBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myLeftBucket > 0 ) && (myBottomBucket > 0)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        curBucket = curBucket + 1;
    }

}

// used to generate a new RGG in a maintenance policy, but also records the number of the new sensors that are deployed, and the number of sensors that are
// disconnected from the sink node at the time of maintenance
void NodeLinkedListMC::GenerateNewRGG_ADPMaintAction_SingleRegion(){

    uniform_real_distribution<> myUnif(0,1);

    double myXNum;
    double myYNum;
    int myTempBin;

    int myBucketsPerRow = (int) ceil(1 / myCommunicationRange);

    SimulationNodeMC *ptr = myBuckets;
    //Sink Location
    ptr->SetNewLoc(0.5,0.5);
    // clear the forward start of the node, and nodes in bin zero
    ptr->ResetReachableNodes();
    ptr->ResetNodesInBucket();

    ptr = ptr + 1;

    // sets the pointers for a node to be the node immediately prior and immediately after the node
    for (int i = 1; i < myNumSensors ; ++i) {
        ptr->ResetReachableNodes();
        ptr->ResetNodesInBucket();

        ptr = ptr + 1;

    }

    // Update value for target nodes
    for (int j = myNumSensors ; j < myNumElements ; ++j) {
//        ptr->ResetNode();
        ptr->ResetReachableNodes();
        ptr->ResetNodesInBucket();
        ptr = ptr + 1;
    }

    // start placing sensor in bins, starting with sink node
    myXNum = myBuckets[0].GetXLoc();
    myYNum = myBuckets[0].GetYLoc();
    // determine which grid number the new sensor location falls in
    myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow +1);

    myBuckets[myTempBin].AddNodeToBucket(0);

    ptr = myBuckets;
    ptr = ptr + 1;

    // generate new location for every sensor
    for (int i = 1; i < myNumSensors ; ++i) {

        myXNum = myBuckets[i].GetXLoc();
        myYNum = myBuckets[i].GetYLoc();

        if (!myBuckets[i].IsFailed() && myBuckets[i].IsActive()){

            // determine which grid number the new sensor location falls in
            myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow + 1);

            // move sensor into a bin for this new location
            myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());
        }


        if (myBuckets[i].IsReplaced()){

            myXNum = myUnif(myDevX);
            myYNum = myUnif(myDevY);

            ptr->ResetFailed();
            ptr->ResetReplaced();
            ptr->SetActive();
            ptr->SetNewLoc(myXNum, myYNum);

            // determine which grid number the new sensor location falls in
            myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow + 1);

            // move sensor into a bin for this new location
            myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());

        }

        ptr = ptr + 1;
    }

    // place target nodes in new bin
    for (int k = myNumSensors; k < myNumElements ; ++k) {

        myXNum = ptr->GetXLoc() + 0.0001;
        myYNum = ptr->GetYLoc() + 0.0001;

        // determine which grid number the new sensor location falls in
        myTempBin = ((int) ceil(myXNum / myCommunicationRange) + (int) (ceil(myYNum / myCommunicationRange))*myBucketsPerRow) - (myBucketsPerRow +1);

        // move sensor into a bin for this new location
        myBuckets[myTempBin].AddNodeToBucket(ptr->GetNodeID());

        ptr = ptr + 1;

    }

    // reset failed nodes and covered nodes
    ptr = myBuckets;
    ptr = ptr + 1;
    for (int m = 1; m < myNumElements; ++m) {
//        ptr->ResetFailed();
        ptr->ResetCovered();
        ptr = ptr + 1;
    }

    int myXBucket;
    int myYBucket;
    double dist;
    vector<int> myNodesInCurBucket;

    int curBucket = myBucketsPerRow + 1;
    for (int l = 0; l < pow(myBucketsPerRow,2) ; ++l) {
        // determine which grid cell we are currently in
        myXBucket = curBucket % myBucketsPerRow;
        myYBucket = (curBucket / myBucketsPerRow) % myBucketsPerRow ;

        if (myXBucket == 0){
            myXBucket = myBucketsPerRow;
            if (curBucket / myBucketsPerRow == myBucketsPerRow){
                myYBucket = myBucketsPerRow - 1;
            }
            else {
                myYBucket = myYBucket - 1;
            }
        }

        if (myYBucket == 0){
            myYBucket = myBucketsPerRow;
        }

        // all within bin comparison
        for (int i = 0; i < (myBuckets[l].GetNumberInCell()) ; ++i) {

            int startNode = myBuckets[l].GetNodeInBucket(i);

            // if the starting node is the sink node, then we only need to worry about connecting to sensor nodes, which is based on communication range
            if (startNode == 0) {
                for (int j = (i+1); j < myBuckets[l].GetNumberInCell() ; ++j) {
                    int endNode = myBuckets[l].GetNodeInBucket(j);
                    // determine if end node is a sensor node or a target node
                    if (endNode < myNumSensors){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        if (dist <= myCommunicationRange) {
                            myBuckets[startNode].AddReachableNode(endNode);
                        }
                    }
                }
            }
                // if starting node is a sensor node
            else if (startNode > 0 && startNode < myNumSensors){

                for (int j = (i+1); j < myBuckets[l].GetNumberInCell() ; ++j) {
                    int endNode = myBuckets[l].GetNodeInBucket(j);

                    // start node is a sensor node, end node is sink node, check for reverse arc
                    if (endNode == 0){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        // sensor to sink comparison, we can add both arcs
                        if (dist <= myCommunicationRange) {
                            myBuckets[endNode].AddReachableNode(startNode);
                        }

                    }
                    else if (endNode < myNumSensors){
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        // sensor to sensor comparison, we can add both arcs
                        if (dist <= myCommunicationRange) {
                            myBuckets[startNode].AddReachableNode(endNode);
                            myBuckets[endNode].AddReachableNode(startNode);
                        }
                    }
                    else {
                        // end node must be a target node
                        dist = sqrt(pow(myBuckets[startNode].GetXLoc() - myBuckets[endNode].GetXLoc(), 2) +
                                    pow(myBuckets[startNode].GetYLoc() - myBuckets[endNode].GetYLoc(), 2));

                        if (dist <= mySensorRange){
                            myBuckets[startNode].AddReachableNode(endNode);
                        }
                    }
                }
            }
        }

        int myAdjacentCell;

        // look to the cell immediately above current cell
        int myTopBucket = myYBucket + 1;
        myAdjacentCell = myXBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow +1);
        if (l < myAdjacentCell && myTopBucket <= myBucketsPerRow){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell immediately to the right
        int myRightBucket = myXBucket + 1;
        myAdjacentCell = myRightBucket + (myYBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myRightBucket <= myBucketsPerRow){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell immediately below
        int myBottomBucket = myYBucket - 1;
        myAdjacentCell = myXBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myBottomBucket < 0){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to left of current cell
        int myLeftBucket = myXBucket - 1;
        myAdjacentCell = myLeftBucket + (myYBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if (l < myAdjacentCell && myLeftBucket > 0){
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the top right
        myAdjacentCell = myRightBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myRightBucket <= myBucketsPerRow) && (myTopBucket <= myBucketsPerRow)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the bottom right
        myAdjacentCell = myRightBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myRightBucket <= myBucketsPerRow) && (myBottomBucket > 0)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the top left
        myAdjacentCell = myLeftBucket + (myTopBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l< myAdjacentCell) && (myLeftBucket > 0 ) && (myTopBucket <= myBucketsPerRow)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        // look to the cell to the bottom left
        myAdjacentCell = myLeftBucket + (myBottomBucket * myBucketsPerRow) - (myBucketsPerRow + 1);
        if ((l < myAdjacentCell) && (myLeftBucket > 0 ) && (myBottomBucket > 0)) {
            this->CompareAdjacentCells(l, myAdjacentCell);
        }

        curBucket = curBucket + 1;
    }
}

// Used to partition the region into certain bins for the purpose of determining the density of functioning sensors in a subregion
void NodeLinkedListMC::SetBinLimits(int numRegionsPerRow) {
//    int myRegionsPerRow = (int) ceil(1 / myCommunicationRange);
//    double myRegionWidth = 1 / (double) myBucketsPerRow;

    int myRegionsPerRow = numRegionsPerRow;
    myNumSubregions = numRegionsPerRow * numRegionsPerRow;
    double myRegionWidth = 1 / (double) myRegionsPerRow;

    int bucketIndex;

    double tempXLow;
    double tempXHigh;
    double tempYLow;
    double tempYHigh;

    for (int i = 1; i <= myRegionsPerRow; ++i) {
        for (int j = 1; j <= myRegionsPerRow; ++j) {
            tempXLow = (double) (j-1) * myRegionWidth;
            tempXHigh = (double)j * myRegionWidth;

            tempYLow = (double)(i-1) * myRegionWidth;
            tempYHigh = (double)i * myRegionWidth;

            bucketIndex = j + (i * myRegionsPerRow) - (myRegionsPerRow + 1);

            myBuckets[bucketIndex].SetXLimits(tempXLow, tempXHigh);
            myBuckets[bucketIndex].SetYLimits(tempYLow, tempYHigh);

//            cout << " Bin : " << bucketIndex << " -- " << tempXLow << " , " << tempXHigh << " -- " << tempYLow << " , " << tempYHigh << endl;

        }
    }
}

// Used to partition the region into certain bins for the purpose of determining the density of functioning sensors in a subregion
void NodeLinkedListMC::CustomBins(){

    myNumSubregions = 16;

    myBuckets[0].SetXLimits(0,0.2);
    myBuckets[0].SetYLimits(0,0.8);

    myBuckets[1].SetXLimits(0.2,1.0);
    myBuckets[1].SetYLimits(0,0.2);

    myBuckets[2].SetXLimits(0.8,1);
    myBuckets[2].SetYLimits(0.2,1);

    myBuckets[3].SetXLimits(0,0.8);
    myBuckets[3].SetYLimits(0.8,1);

    myBuckets[4].SetXLimits(0.2,0.35);
    myBuckets[4].SetYLimits(0.2,0.35);

    myBuckets[5].SetXLimits(0.35,0.65);
    myBuckets[5].SetYLimits(0.2,0.35);

    myBuckets[6].SetXLimits(0.65,0.8);
    myBuckets[6].SetYLimits(0.2,0.35);

    myBuckets[7].SetXLimits(0.2,0.35);
    myBuckets[7].SetYLimits(0.35,0.65);

    myBuckets[8].SetXLimits(0.2,0.35);
    myBuckets[8].SetYLimits(0.65,0.8);

    myBuckets[9].SetXLimits(0.35,0.65);
    myBuckets[9].SetYLimits(0.65,0.8);

    myBuckets[10].SetXLimits(0.65,0.8);
    myBuckets[10].SetYLimits(0.65,0.8);

    myBuckets[11].SetXLimits(0.65,0.8);
    myBuckets[11].SetYLimits(0.35,0.65);

    myBuckets[12].SetXLimits(0.35,0.5);
    myBuckets[12].SetYLimits(0.35,0.5);

    myBuckets[13].SetXLimits(0.5,0.65);
    myBuckets[13].SetYLimits(0.35,0.5);

    myBuckets[14].SetXLimits(0.35,0.5);
    myBuckets[14].SetYLimits(0.5,0.65);

    myBuckets[15].SetXLimits(0.5,0.65);
    myBuckets[15].SetYLimits(0.5,0.65);


}

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

NetworkSignature::NetworkSignature() {}

NetworkSignature::NetworkSignature(int numSensors) {
    myNumSensors = numSensors;
}

void NetworkSignature::SetNumSensors(int numSensors) {
    myNumSensors = numSensors;
}

int NetworkSignature::GetNumSensors() {
    return myNumSensors;
}

void NetworkSignature::UpdateDSpec(int numFailed, double failProb) {
    DSpecElements.push_back(numFailed);
    DSpecProb.push_back(failProb);
}

void NetworkSignature::GetSignature(vector<int> &failCount, vector<double> &failProb) {
    for (int i = 0; i < DSpecElements.size() ; ++i) {
        failCount.push_back(DSpecElements.at(i));
        failProb.push_back(DSpecProb.at(i));
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////

SignatureCollection::SignatureCollection(int numSignatures, int minSize, int maxSize, int numReg) {
    mySignatures = new NetworkSignature[numSignatures];
    myNumSignatures = numSignatures;
    minNetSize = minSize;
    maxNetSize = maxSize;
    numRegions = numReg;
}

void SignatureCollection::ReadSignatures(char* filename) {

    ifstream file;
    int numRows;
    int failCount;
    double failProb;

    int prevValue;
    int curValue;

    file.open(filename);

    file >> numRows;

    NetworkSignature *ptr = mySignatures;

    file >> prevValue;
    file >> failCount;
    file >> failProb;

    ptr->SetNumSensors(prevValue);
    ptr->UpdateDSpec(failCount, failProb);

    for (int i = 1; i < numRows; ++i) {
        file >> curValue;
        if (curValue == prevValue){

            prevValue = curValue;

            file >> failCount;
            file >> failProb;

            ptr->UpdateDSpec(failCount, failProb);

        }
        else {
            ptr = ptr + 1;
            prevValue = curValue;
            ptr->SetNumSensors(curValue);

            file >> failCount;
            file >> failProb;

            ptr->UpdateDSpec(failCount, failProb);
        }
    }
}

void SignatureCollection::ReturnSignature_Approximation(int netSize, vector<int> &failCount, vector<double> &failProb) {

    NetworkSignature *ptr = mySignatures;

    int minSize = ptr->GetNumSensors();

    int startPtr = (netSize - minSize) / numRegions;

    ptr = ptr + startPtr;

    ptr->GetSignature(failCount, failProb);

}

void SignatureCollection::ReturnSignature(int netSize, vector<int> &failCount, vector<double> &failProb) {

    NetworkSignature *ptr = mySignatures;

    int minSize = ptr->GetNumSensors();

    int startPtr = netSize - minSize;

    ptr = ptr + startPtr;

    ptr->GetSignature(failCount, failProb);

}


double SignatureCollection::ReliabilityEstimate_Approximation(double delta, int nSize, vector<int> ageVector) {

    double t1 = ((double) nSize / (double) numRegions);

    int lowerLimit = max((int) floor(t1) * numRegions, minNetSize);
    int upperLimit = min((int) ceil(t1) * numRegions, maxNetSize);

    double rel = 0;

    double failProb = ApproxStableResidualLifeDist(delta, ageVector, myShape, myScale);

    if (lowerLimit == upperLimit){

        vector<int> sigCount;
        vector<double> sigProb;

        this->ReturnSignature_Approximation(nSize, sigCount, sigProb);

        rel = ADPReliabilityEstimate(nSize, failProb, sigCount, sigProb);

    }
    else {

        vector<int> sigCount;
        vector<double> sigProb;

        this->ReturnSignature_Approximation(lowerLimit, sigCount, sigProb);

        double relLower = ADPReliabilityEstimate(lowerLimit, failProb, sigCount, sigProb);

        vector<int> sigCountU;
        vector<double> sigProbU;

        this->ReturnSignature_Approximation(upperLimit, sigCountU, sigProbU);

        double relUpper = ADPReliabilityEstimate(upperLimit, failProb, sigCountU, sigProbU);

        rel = ((relUpper - relLower) / (upperLimit - lowerLimit)) * (nSize - lowerLimit) + relLower;

    }

    return rel;
}

double SignatureCollection::ReliabilityEstimate(double delta, int nSize, vector<int> ageVector) {

    double rel = 0;

    double failProb = ApproxStableResidualLifeDist(delta, ageVector, myShape, myScale);

    vector<int> sigCount;
    vector<double> sigProb;

    this->ReturnSignature(nSize, sigCount, sigProb);

    rel = ADPReliabilityEstimate(nSize, failProb, sigCount, sigProb);

    return rel;
}

void SignatureCollection::SetFailureDist(double shape, double scale){
    myShape = shape;
    myScale = scale;
}

SingleRegionSignature::SingleRegionSignature(char* filename){
    this->ReadSignature(filename);
}

void SingleRegionSignature::ReadSignature(char *filename) {

    ifstream file;
    int numRows;
    int nMax;
    int failCount;
    double failP;

    file.open(filename);

    file >> numRows;
    file >> nMax;

    maxSize = nMax;

    for (int i = 0; i < numRows; ++i) {

        file >> failCount;
        file >> failP;

        failNum.push_back(failCount);
        failProb.push_back(failP);
    }
}

void SingleRegionSignature::SetFailureDist(double shape, double scale){
    myShape = shape;
    myScale = scale;
}

double SingleRegionSignature::ReliabilityEstimate(double delta, int nSize, vector<int> ageVect) {

    double rel = 0;

    double senFailProb = ApproxStableResidualLifeDist(delta, ageVect, myShape, myScale);

    vector<int> sigCount;
    vector<double> sigProb;

    this->SignautreUpdate(nSize, sigCount, sigProb);

    rel = ADPReliabilityEstimate(nSize, senFailProb, sigCount, sigProb);

    return rel;

}

void SingleRegionSignature::SignautreUpdate(int nSize, vector<int> &failN, vector<double> &failP) {

    int sizeDiff = maxSize - nSize;

    if (sizeDiff < 0) {
        cout << " The new network size is larger than initial network" << endl;
    }
    else {

        int firstFail = failNum.at(0);

        if ((firstFail - sizeDiff) <= 0){
            failN.push_back(0);
            failP.push_back(failProb.at(0));
        }

        for (int i = 1; i < failNum.size(); ++i) {
            if ((failNum.at(i) - sizeDiff) <= 0){
                failP[0] = failP[0] + failProb.at(i);
            }
            else{
                failN.push_back(failNum.at(i) - sizeDiff);
                failP.push_back(failProb.at(i));
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Value Functions
/////////////////////////////////////////////////////////////////////////////////////////////////////

ValueFunctionCostCollection::ValueFunctionCostCollection(int numMissions, double learningRate){
    myValueFunctions = new CostValueFunction[numMissions];
    myNumFunctions = numMissions;
    myLearningRate = learningRate;
}

double ValueFunctionCostCollection::GetStateValue(int curMission, double avgAge, int netSize){
    return myValueFunctions[curMission].GetStateValue(avgAge, netSize);
}

double ValueFunctionCostCollection::GetTempStateValue(int curMission, double avgAge, int netSize){
    return myValueFunctions[curMission].GetTempStateValue(avgAge, netSize);
}

double ValueFunctionCostCollection::GetStepSize(int curMission, double avgAge, int netSize){
    return myValueFunctions[curMission].GetStepSize(avgAge, netSize);
}

void ValueFunctionCostCollection::UpdateValueFunctions() {
    for (int i = 0; i < myNumFunctions; ++i) {
        myValueFunctions[i].UpdateValueFunction(myLearningRate);
    }
}

void ValueFunctionCostCollection::UpdateTempStateValue(int curMission, double avgAge, int netSize, double newValue){
    myValueFunctions[curMission].UpdateTempStateValue(avgAge, netSize, newValue);
}

void ValueFunctionCostCollection::UpdateStepSize(int curMission, double avgAge, int netSize, double newStep){
    myValueFunctions[curMission].UpdateStepSize(avgAge, netSize, newStep);
}

void ValueFunctionCostCollection::UpdateAdjacentStates(int curMission, double postDecisionAge, int postDecisionSize,
                                                   double newValue) {

    if (postDecisionSize - 3 >= 0){
        double tAlpha = this->GetStepSize(curMission, postDecisionAge, postDecisionSize -3);
        double tValue = this->GetStateValue(curMission, postDecisionAge, postDecisionSize - 3);

        double updateValue = ((1.0 - tAlpha) * tValue) + (tAlpha * newValue);
        this->UpdateTempStateValue(curMission, postDecisionAge, postDecisionSize - 3, updateValue);
    }

    if (postDecisionSize - 2 >= 0){
        double tAlpha = this->GetStepSize(curMission, postDecisionAge, postDecisionSize - 2);
        double tValue = this->GetStateValue(curMission, postDecisionAge, postDecisionSize - 2);

        double updateValue = ((1.0 - tAlpha) * tValue) + (tAlpha * newValue);
        this->UpdateTempStateValue(curMission, postDecisionAge, postDecisionSize - 2, updateValue);
    }

    if (postDecisionSize - 1 >= 0){
        double tAlpha = this->GetStepSize(curMission, postDecisionAge, postDecisionSize - 1);
        double tValue = this->GetStateValue(curMission, postDecisionAge, postDecisionSize - 1);

        double updateValue = ((1.0 - tAlpha) * tValue) + (tAlpha * newValue);
        this->UpdateTempStateValue(curMission, postDecisionAge, postDecisionSize - 1, updateValue);
    }

    if (postDecisionSize + 1 <= 567){
        double tAlpha = this->GetStepSize(curMission, postDecisionAge, postDecisionSize + 1);
        double tValue = this->GetStateValue(curMission, postDecisionAge, postDecisionSize + 1);

        double updateValue = ((1.0 - tAlpha) * tValue) + (tAlpha * newValue);
        this->UpdateTempStateValue(curMission, postDecisionAge, postDecisionSize + 1, updateValue);
    }

    if (postDecisionSize + 2 <= 567){
        double tAlpha = this->GetStepSize(curMission, postDecisionAge, postDecisionSize + 2);
        double tValue = this->GetStateValue(curMission, postDecisionAge, postDecisionSize + 2);

        double updateValue = ((1.0 - tAlpha) * tValue) + (tAlpha * newValue);
        this->UpdateTempStateValue(curMission, postDecisionAge, postDecisionSize + 2, updateValue);
    }

    if (postDecisionSize + 3 <= 567){
        double tAlpha = this->GetStepSize(curMission, postDecisionAge, postDecisionSize + 3);
        double tValue = this->GetStateValue(curMission, postDecisionAge, postDecisionSize + 3);

        double updateValue = ((1.0 - tAlpha) * tValue) + (tAlpha * newValue);
        this->UpdateTempStateValue(curMission, postDecisionAge, postDecisionSize + 3, updateValue);
    }

}

void ValueFunctionCostCollection::ADP_MonotoneUpdate(int curMission, double postDecisionAge, int postDecisionSize,
                                                 double newValue, double cVar){

    double baseAlpha = this->GetStepSize(curMission, postDecisionAge, postDecisionSize);
    double initialValue = this->GetStateValue(curMission, postDecisionAge, postDecisionSize);
    double baseValue = ((1.0 - baseAlpha) * initialValue) + (baseAlpha * newValue);

    if (postDecisionSize - 3 >= 0){
        double tAlpha = this->GetStepSize(curMission, postDecisionAge, postDecisionSize - 3);
        double tValue = this->GetStateValue(curMission, postDecisionAge, postDecisionSize - 3);

        double updateValue = ((1.0 - tAlpha) * tValue) + (tAlpha * (newValue - 3*cVar));
        updateValue = max(updateValue, baseValue);

        this->UpdateTempStateValue(curMission, postDecisionAge, postDecisionSize - 3, updateValue);
    }

    if (postDecisionSize - 2 >= 0){
        double tAlpha = this->GetStepSize(curMission, postDecisionAge, postDecisionSize - 2);
        double tValue = this->GetStateValue(curMission, postDecisionAge, postDecisionSize - 2);

        double updateValue = ((1.0 - tAlpha) * tValue) + (tAlpha * (newValue - 2*cVar));
        updateValue = max(updateValue, baseValue);

        this->UpdateTempStateValue(curMission, postDecisionAge, postDecisionSize - 2, updateValue);
    }

    if (postDecisionSize - 1 >= 0){
        double tAlpha = this->GetStepSize(curMission, postDecisionAge, postDecisionSize - 1);
        double tValue = this->GetStateValue(curMission, postDecisionAge, postDecisionSize - 1);

        double updateValue = ((1.0 - tAlpha) * tValue) + (tAlpha * (newValue - cVar));
        updateValue = max(updateValue, baseValue);

        this->UpdateTempStateValue(curMission, postDecisionAge, postDecisionSize - 1, updateValue);
    }

    if (postDecisionSize + 1 <= 567){
        double tAlpha = this->GetStepSize(curMission, postDecisionAge, postDecisionSize + 1);
        double tValue = this->GetStateValue(curMission, postDecisionAge, postDecisionSize + 1);

        double updateValue = ((1.0 - tAlpha) * tValue) + (tAlpha * (newValue + cVar));
        updateValue = min(updateValue, baseValue);

        this->UpdateTempStateValue(curMission, postDecisionAge, postDecisionSize + 1, updateValue);
    }

    if (postDecisionSize + 2 <= 567){
        double tAlpha = this->GetStepSize(curMission, postDecisionAge, postDecisionSize + 2);
        double tValue = this->GetStateValue(curMission, postDecisionAge, postDecisionSize + 2);

        double updateValue = ((1.0 - tAlpha) * tValue) + (tAlpha * (newValue + 2*cVar));
        updateValue = min(updateValue, baseValue);

        this->UpdateTempStateValue(curMission, postDecisionAge, postDecisionSize + 2, updateValue);
    }

    if (postDecisionSize + 3 <= 567){
        double tAlpha = this->GetStepSize(curMission, postDecisionAge, postDecisionSize + 3);
        double tValue = this->GetStateValue(curMission, postDecisionAge, postDecisionSize + 3);

        double updateValue = ((1.0 - tAlpha) * tValue) + (tAlpha * (newValue + 3*cVar));
        updateValue = min(updateValue, baseValue);

        this->UpdateTempStateValue(curMission, postDecisionAge, postDecisionSize + 3, updateValue);
    }

}

void ValueFunctionCostCollection::PrintValueFunctions() {

    ofstream myState0;
        myState0.open ("StateValues_t0.txt");
        myValueFunctions[0].PrintValueFunction(myState0);
    myState0.close();

    ofstream myState1;
    myState1.open ("StateValues_t1.txt");
    myValueFunctions[1].PrintValueFunction(myState1);
    myState1.close();

    ofstream myState2;
    myState2.open ("StateValues_t2.txt");
    myValueFunctions[2].PrintValueFunction(myState2);
    myState2.close();

    ofstream myState3;
    myState3.open ("StateValues_t3.txt");
    myValueFunctions[3].PrintValueFunction(myState3);
    myState3.close();

    ofstream myState4;
    myState4.open ("StateValues_t4.txt");
    myValueFunctions[4].PrintValueFunction(myState4);
    myState4.close();

    ofstream myState5;
    myState5.open ("StateValues_t5.txt");
    myValueFunctions[5].PrintValueFunction(myState5);
    myState5.close();

    ofstream myState6;
    myState6.open ("StateValues_t6.txt");
    myValueFunctions[6].PrintValueFunction(myState6);
    myState6.close();

    ofstream myState7;
    myState7.open ("StateValues_t7.txt");
    myValueFunctions[7].PrintValueFunction(myState7);
    myState7.close();

    ofstream myState8;
    myState8.open ("StateValues_t8.txt");
    myValueFunctions[8].PrintValueFunction(myState8);
    myState8.close();

    ofstream myState9;
    myState9.open ("StateValues_t9.txt");
    myValueFunctions[9].PrintValueFunction(myState9);
    myState9.close();

    ofstream myState10;
    myState10.open ("StateValues_t10.txt");
    myValueFunctions[10].PrintValueFunction(myState10);
    myState10.close();

    ofstream myState11;
    myState11.open ("StateValues_t11.txt");
    myValueFunctions[11].PrintValueFunction(myState11);
    myState11.close();

    ofstream myState12;
    myState12.open ("StateValues_t12.txt");
    myValueFunctions[12].PrintValueFunction(myState12);
    myState12.close();

    ofstream myState13;
    myState13.open ("StateValues_t13.txt");
    myValueFunctions[13].PrintValueFunction(myState13);
    myState13.close();

    ofstream myState14;
    myState14.open ("StateValues_t14.txt");
    myValueFunctions[14].PrintValueFunction(myState14);
    myState14.close();

    ofstream myState15;
    myState15.open ("StateValues_t15.txt");
    myValueFunctions[15].PrintValueFunction(myState15);
    myState15.close();

    ofstream myState16;
    myState16.open ("StateValues_t16.txt");
    myValueFunctions[16].PrintValueFunction(myState16);
    myState16.close();

    ofstream myState17;
    myState17.open ("StateValues_t17.txt");
    myValueFunctions[17].PrintValueFunction(myState17);
    myState17.close();

    ofstream myState18;
    myState18.open ("StateValues_t18.txt");
    myValueFunctions[18].PrintValueFunction(myState18);
    myState18.close();

    ofstream myState19;
    myState19.open ("StateValues_t19.txt");
    myValueFunctions[19].PrintValueFunction(myState19);
    myState19.close();

}

/////////////////////////////////////////////////////////////////////////////////////////////////////

CostValueFunction::CostValueFunction() {

    fill(*myStepSize, *myStepSize + (91*567), 1.0);
}

double CostValueFunction::GetStateValue(double avgAge, int netSize) {

    int ageIndex = (int) round(avgAge * 10.0);
    return myValues[ageIndex][netSize];
}

double CostValueFunction::GetTempStateValue(double avgAge, int netSize) {

    int ageIndex = (int) round(avgAge * 10.0);
    return myTempValues[ageIndex][netSize];
}

double CostValueFunction::GetStepSize(double avgAge, int netSize){

    int ageIndex = (int) round(avgAge * 10.0);
    return myStepSize[ageIndex][netSize];
}

void CostValueFunction::UpdateValueFunction(double learningRate) {
    for (int i = 0; i < ageStatesUpdates.size(); ++i) {
        myValues[ageStatesUpdates.at(i)][netSizeUpdates.at(i)] = myTempValues[ageStatesUpdates.at(i)][netSizeUpdates.at(i)];
        myStepSize[ageStatesUpdates.at(i)][netSizeUpdates.at(i)] = learningRate;
    }

    ageStatesUpdates.clear();
    netSizeUpdates.clear();
}

void CostValueFunction::UpdateStateValue(double avgAge, int netSize, double newValue){
    int ageIndex = (int) round(avgAge * 10.0);
    myValues[ageIndex][netSize] = newValue;
}

void CostValueFunction::UpdateTempStateValue(double avgAge, int netSize, double newValue){

    int ageIndex = (int) round(avgAge * 10.0);
    ageStatesUpdates.push_back(ageIndex);
    netSizeUpdates.push_back(netSize);

    myTempValues[ageIndex][netSize] = newValue;
}

void CostValueFunction::UpdateStepSize(double avgAge, int netSize, double newStep){

    int ageIndex = (int) round(avgAge * 10.0);
    myStepSize[ageIndex][netSize] = newStep;
}

void CostValueFunction::PrintValueFunction(ofstream &outputFile) {

    for (int i = 0; i < 91; ++i) {
        for (int j = 0; j < 567 ; ++j) {
            outputFile << myValues[i][j] << " " ;
        }
        outputFile << endl;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Reliability Value Functions
/////////////////////////////////////////////////////////////////////////////////////////////////////

ValueFunctionReliabilityCollection::ValueFunctionReliabilityCollection(int numMissions, double learningRate){
    myValueFunctions = new ReliabilityValueFunction[numMissions];
    myNumFunctions = numMissions;
    myLearningRate = learningRate;
}

ValueFunctionReliabilityCollection::ValueFunctionReliabilityCollection(int numMissions, double learningRate, double Budget, int nMin, int nMax){
    myValueFunctions = new ReliabilityValueFunction[numMissions];
    myNumFunctions = numMissions;
    myLearningRate = learningRate;

    minNetSize = nMin;
    maxNetSize = nMax;
    myBudget = Budget;
}

void ValueFunctionReliabilityCollection::ConstructValueFunction(int numMissions, double Budget, int nMin, int nMax) {
    for (int i = 0; i < numMissions ; ++i) {
        myValueFunctions[i].ConstructValueFunction(Budget, nMin, nMax);
    }
}

double ValueFunctionReliabilityCollection::GetStateValue(int curMission, double budget, int netSize){
    return myValueFunctions[curMission].GetStateValue(budget, netSize);
}

double ValueFunctionReliabilityCollection::GetTempStateValue(int curMission, double budget, int netSize){
    return myValueFunctions[curMission].GetTempStateValue(budget, netSize);
}

double ValueFunctionReliabilityCollection::GetStepSize(int curMission, double budget, int netSize){
    return myValueFunctions[curMission].GetStepSize(budget, netSize);
}

int ValueFunctionReliabilityCollection::GetNumStateVisits(int curMission, double budget, int netSize) {
    return myValueFunctions[curMission].GetNumStateVisists(budget, netSize);
}

void ValueFunctionReliabilityCollection::UpdateValueFunctionsConstant() {
    for (int i = 0; i < myNumFunctions; ++i) {
        myValueFunctions[i].UpdateValueFunctionConstant(myLearningRate);
    }
}

void ValueFunctionReliabilityCollection::UpdateValueFunctionsStC(double initialLearningRate){
    for (int i = 0; i < myNumFunctions; ++i) {
        myValueFunctions[i].UpdateValueFunctionStC(initialLearningRate);
    }
}

void ValueFunctionReliabilityCollection::UpdateValueFunctionsGHarmonic(double initialLearningRate){
    for (int i = 0; i < myNumFunctions; ++i) {
        myValueFunctions[i].UpdateValueFunctionsGHarmonic(initialLearningRate);
    }
}

void ValueFunctionReliabilityCollection::UpdateTempStateValue(int curMission, double budget, int netSize, double newValue){
    myValueFunctions[curMission].UpdateTempStateValue(budget, netSize, newValue);
}

void ValueFunctionReliabilityCollection::UpdateStepSize(int curMission, double budget, int netSize, double newStep){
    myValueFunctions[curMission].UpdateStepSize(budget, netSize, newStep);
}

void ValueFunctionReliabilityCollection::PrintValueFunctions() {

    char *filenameStart = "StateValues_t";
    char *filenameEnd = ".txt";

    stringstream ss;

    for (int i = 0; i < myNumFunctions ; ++i) {

        ss << filenameStart << i << filenameEnd;

        ofstream myState;
        myState.open(ss.str());
        myValueFunctions[i].PrintValueFunction(myState);
        myState.close();

        ss.str("");

    }


/*
    ofstream myState0;
    myState0.open ("StateValues_t0.txt");
    myValueFunctions[0].PrintValueFunction(myState0);
    myState0.close();

    ofstream myState1;
    myState1.open ("StateValues_t1.txt");
    myValueFunctions[1].PrintValueFunction(myState1);
    myState1.close();

    ofstream myState2;
    myState2.open ("StateValues_t2.txt");
    myValueFunctions[2].PrintValueFunction(myState2);
    myState2.close();

    ofstream myState3;
    myState3.open ("StateValues_t3.txt");
    myValueFunctions[3].PrintValueFunction(myState3);
    myState3.close();

    ofstream myState4;
    myState4.open ("StateValues_t4.txt");
    myValueFunctions[4].PrintValueFunction(myState4);
    myState4.close();

    ofstream myState5;
    myState5.open ("StateValues_t5.txt");
    myValueFunctions[5].PrintValueFunction(myState5);
    myState5.close();

    ofstream myState6;
    myState6.open ("StateValues_t6.txt");
    myValueFunctions[6].PrintValueFunction(myState6);
    myState6.close();

    ofstream myState7;
    myState7.open ("StateValues_t7.txt");
    myValueFunctions[7].PrintValueFunction(myState7);
    myState7.close();

    ofstream myState8;
    myState8.open ("StateValues_t8.txt");
    myValueFunctions[8].PrintValueFunction(myState8);
    myState8.close();

    ofstream myState9;
    myState9.open ("StateValues_t9.txt");
    myValueFunctions[9].PrintValueFunction(myState9);
    myState9.close();

    ofstream myState10;
    myState10.open ("StateValues_t10.txt");
    myValueFunctions[10].PrintValueFunction(myState10);
    myState10.close();

    ofstream myState11;
    myState11.open ("StateValues_t11.txt");
    myValueFunctions[11].PrintValueFunction(myState11);
    myState11.close();

    ofstream myState12;
    myState12.open ("StateValues_t12.txt");
    myValueFunctions[12].PrintValueFunction(myState12);
    myState12.close();

    ofstream myState13;
    myState13.open ("StateValues_t13.txt");
    myValueFunctions[13].PrintValueFunction(myState13);
    myState13.close();

    ofstream myState14;
    myState14.open ("StateValues_t14.txt");
    myValueFunctions[14].PrintValueFunction(myState14);
    myState14.close();

    ofstream myState15;
    myState15.open ("StateValues_t15.txt");
    myValueFunctions[15].PrintValueFunction(myState15);
    myState15.close();

    ofstream myState16;
    myState16.open ("StateValues_t16.txt");
    myValueFunctions[16].PrintValueFunction(myState16);
    myState16.close();

    ofstream myState17;
    myState17.open ("StateValues_t17.txt");
    myValueFunctions[17].PrintValueFunction(myState17);
    myState17.close();

    ofstream myState18;
    myState18.open ("StateValues_t18.txt");
    myValueFunctions[18].PrintValueFunction(myState18);
    myState18.close();

    ofstream myState19;
    myState19.open ("StateValues_t19.txt");
    myValueFunctions[19].PrintValueFunction(myState19);
    myState19.close();
*/
}

void ValueFunctionReliabilityCollection::ADP_MonotoneUpdate(int curMission, double postDecisionBudget, int postDecisionSize,
                                                     double newValue){

    double baseAlpha = this->GetStepSize(curMission, postDecisionBudget, postDecisionSize);
    double initialValue = this->GetStateValue(curMission, postDecisionBudget, postDecisionSize);
    double baseValue = ((1.0 - baseAlpha) * initialValue) + (baseAlpha * newValue);

    if (postDecisionSize - 3 >= 0){
        double tAlpha = this->GetStepSize(curMission, postDecisionBudget, postDecisionSize - 3);
        double tValue = this->GetStateValue(curMission, postDecisionBudget, postDecisionSize - 3);

        double updateValue = ((1.0 - tAlpha) * tValue) + (tAlpha *newValue);
        updateValue = min(updateValue, baseValue);

        this->UpdateTempStateValue(curMission, postDecisionBudget, postDecisionSize - 3, updateValue);
    }

    if (postDecisionSize - 2 >= 0){
        double tAlpha = this->GetStepSize(curMission, postDecisionBudget, postDecisionSize - 2);
        double tValue = this->GetStateValue(curMission, postDecisionBudget, postDecisionSize - 2);

        double updateValue = ((1.0 - tAlpha) * tValue) + (tAlpha * newValue);
        updateValue = min(updateValue, baseValue);

        this->UpdateTempStateValue(curMission, postDecisionBudget, postDecisionSize - 2, updateValue);
    }

    if (postDecisionSize - 1 >= 0){
        double tAlpha = this->GetStepSize(curMission, postDecisionBudget, postDecisionSize - 1);
        double tValue = this->GetStateValue(curMission, postDecisionBudget, postDecisionSize - 1);

        double updateValue = ((1.0 - tAlpha) * tValue) + (tAlpha * newValue);
        updateValue = min(updateValue, baseValue);

        this->UpdateTempStateValue(curMission, postDecisionBudget, postDecisionSize - 1, updateValue);
    }

    if (postDecisionSize + 1 < (maxNetSize - minNetSize + 1)){
        double tAlpha = this->GetStepSize(curMission, postDecisionBudget, postDecisionSize + 1);
        double tValue = this->GetStateValue(curMission, postDecisionBudget, postDecisionSize + 1);

        double updateValue = ((1.0 - tAlpha) * tValue) + (tAlpha *newValue);
        updateValue = max(updateValue, baseValue);

        this->UpdateTempStateValue(curMission, postDecisionBudget, postDecisionSize + 1, updateValue);
    }

    if (postDecisionSize + 2 < (maxNetSize - minNetSize + 1)){
        double tAlpha = this->GetStepSize(curMission, postDecisionBudget, postDecisionSize + 2);
        double tValue = this->GetStateValue(curMission, postDecisionBudget, postDecisionSize + 2);

        double updateValue = ((1.0 - tAlpha) * tValue) + (tAlpha * newValue);
        updateValue = max(updateValue, baseValue);

        this->UpdateTempStateValue(curMission, postDecisionBudget, postDecisionSize + 2, updateValue);
    }

    if (postDecisionSize + 3 < (maxNetSize - minNetSize + 1)){
        double tAlpha = this->GetStepSize(curMission, postDecisionBudget, postDecisionSize + 3);
        double tValue = this->GetStateValue(curMission, postDecisionBudget, postDecisionSize + 3);

        double updateValue = ((1.0 - tAlpha) * tValue) + (tAlpha * newValue);
        updateValue = max(updateValue, baseValue);

        this->UpdateTempStateValue(curMission, postDecisionBudget, postDecisionSize + 3, updateValue);
    }

}

void ValueFunctionReliabilityCollection::InitializeValueFunction(char *filename) {

    ifstream file;
    int numRows;
    int numMissions;
    double initialValue;

    file.open(filename);

    file >> numRows;
    file >> numMissions;

    for (int i = 0; i < numRows; ++i) {

        for (int j = 0; j < (numMissions-1); ++j) {

            file >> initialValue;

            if (initialValue > 0){
                myValueFunctions[j].SetInitialStateValue(i, initialValue, myLearningRate);
            }
        }
    }

    file.close();

}

void ValueFunctionReliabilityCollection::InitializeValueFunctionStC(char *filename) {

    ifstream file;
    int numRows;
    int numMissions;
    double initialValue;

    file.open(filename);

    file >> numRows;
    file >> numMissions;

    int readNum = min(numRows, (int) myBudget / 10);

    for (int i = 0; i < readNum; ++i) {

        for (int j = 0; j < (numMissions-1); ++j) {

            file >> initialValue;

            if (initialValue > 0){
                myValueFunctions[j].SetInitialStateValueStC(i, initialValue, myLearningRate);
            }
        }
    }

    file.close();

}

void ValueFunctionReliabilityCollection::InitializeValueFunctionGHarmonic(char *filename) {

    ifstream file;
    int numRows;
    int numMissions;
    double initialValue;

    file.open(filename);

    file >> numRows;
    file >> numMissions;

    int readNum = min(numRows, (int) myBudget / 10);

    for (int i = 0; i < readNum; ++i) {

        for (int j = 0; j < (numMissions-1); ++j) {

            file >> initialValue;

            if (initialValue > 0){
                myValueFunctions[j].SetInitialStateValueGHarmonic(i, initialValue, myLearningRate);
            }
        }
    }

    file.close();

}

void ValueFunctionReliabilityCollection::ReadValueFunctions() {

    char *filename = "C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t0.txt";
    ifstream file;

    file.open(filename);
    double tValue;

    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[0].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t1.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[1].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t2.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[2].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t3.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[3].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t4.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[4].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t5.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[5].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t6.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[6].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t7.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[7].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t8.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[8].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t9.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[9].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t10.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[10].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t11.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[11].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t12.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[12].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t13.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[13].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t14.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[14].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t15.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[15].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t16.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[16].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t17.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[17].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t18.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[18].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t19.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[19].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t20.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[20].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t21.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[21].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t22.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[22].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t23.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[23].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t24.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[24].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t25.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[25].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t26.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[26].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t27.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[27].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t28.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[28].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

    file.open("C:\\Users\\nboardma\\Documents\\Personal\\InputValueFunctions\\StateValues_t29.txt");
    for (int i = 0; i < 700 ; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1) ; ++j) {
            file >> tValue;
            myValueFunctions[29].ReadValueFunction(i,j,tValue);
        }
    }
    file.close();

}

void ValueFunctionReliabilityCollection::TempOutput() {

    ofstream myState0;
    myState0.open ("StateValuesInitialize.txt");
    myValueFunctions[1].PrintValueFunction(myState0);
    myState0.close();
}

int ValueFunctionReliabilityCollection::BoltzmannExploration(int curMission, int startSize, vector<int> actions, vector<double> stateValues, vector<double> stateBudget, double bestValue) {

    double beta;
    vector<double> boltValues;

    for (int i = 0; i < actions.size() ; ++i) {

        int sizeIndex = startSize + actions.at(i) - minNetSize;
        if (sizeIndex < 0){
            sizeIndex = 0;
        }

        beta = (double) this->GetNumStateVisits(curMission, stateBudget.at(i), sizeIndex);
        beta = beta / (abs(stateValues.at(i) - bestValue));

        boltValues.push_back(exp(beta*stateValues.at(i)));

        cout << "                      " << this->GetNumStateVisits(curMission, stateBudget.at(i), sizeIndex)<<" " << (abs(stateValues.at(i) - bestValue)) << endl;

    }

    discrete_distribution<> actionDist(boltValues.begin(), boltValues.end());
    int act = actionDist(myExplorationGen);

    cout << "      size : " << boltValues.size() << " action index : " << act << endl;

//    int actionSelected = actions.at(act);

    return act;

}

/////////////////////////////////////////////////////////////////////////////////////////////////////

ReliabilityValueFunction::ReliabilityValueFunction() {
//    fill(*myStepSize, *myStepSize + (720*(nMax - nMin + 1)), 1.0);
}

void ReliabilityValueFunction::ConstructValueFunction(double Budget, int nMin, int nMax) {

    minNetSize = nMin;
    maxNetSize = nMax;

    int x = (int) Budget / 10;
    int y = nMax - nMin + 1;

    myBudgetSize = x;

    myValues = new double*[x];
    myTempValues = new double*[x];
    myStepSize = new double*[x];
    myNumVisits = new int*[x];

    for (int i = 0; i < x; ++i) {
        myValues[i] = new double[y];
        myTempValues[i] = new double[y];
        myStepSize[i] = new double[y];
        myNumVisits[i] = new int[y];
    }

    for (int i = 0; i < x; ++i) {
        for (int j = 0; j < y ; ++j) {
            myValues[i][j] = 0;
            myTempValues[i][j] = 0;
            myStepSize[i][j] = 1;
        }
    }
}

double ReliabilityValueFunction::GetStateValue(double budget, int netSize) {

    int budgetIndex = (int) (budget / 10);
//    cout << budget << " - " << netSize << " - budgetIndex : " << budgetIndex << "   " << myValues[budgetIndex][netSize] << endl;
    return myValues[budgetIndex][netSize];
}

double ReliabilityValueFunction::GetTempStateValue(double budget, int netSize) {

    int budgetIndex = (int) (budget / 10);
    return myTempValues[budgetIndex][netSize];
}

double ReliabilityValueFunction::GetStepSize(double budget, int netSize){

    int budgetIndex = (int) (budget / 10);
    return myStepSize[budgetIndex][netSize];
}

int ReliabilityValueFunction::GetNumStateVisists(double budget, int netSize) {

    int budgetIndex = (int) (budget / 10);
    return myNumVisits[budgetIndex][netSize];

}

void ReliabilityValueFunction::UpdateValueFunctionConstant(double learningRate) {
    for (int i = 0; i < budgetStatesUpdates.size(); ++i) {
        myValues[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)] = myTempValues[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)];
        myStepSize[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)] = learningRate;
    }

    budgetStatesUpdates.clear();
    netSizeUpdates.clear();
}

void ReliabilityValueFunction::UpdateValueFunctionStC(double initialLearningRate) {
    for (int i = 0; i < budgetStatesUpdates.size(); ++i) {
        myValues[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)] = myTempValues[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)];
        myNumVisits[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)] = myNumVisits[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)] + 1;

        double n = (double) myNumVisits[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)];
        double tStep = initialLearningRate * (((b / n) + a) / ((b / n) + a + n));

        myStepSize[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)] = tStep;
        myNumVisits[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)] = myNumVisits[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)] + 1;
    }

    budgetStatesUpdates.clear();
    netSizeUpdates.clear();
}

void ReliabilityValueFunction::UpdateValueFunctionsGHarmonic(double initialLearningRate) {
    for (int i = 0; i < budgetStatesUpdates.size(); ++i) {
        myValues[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)] = myTempValues[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)];
        myNumVisits[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)] = myNumVisits[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)] + 1;

        double n = (double) myNumVisits[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)];
        double tStep = initialLearningRate * (generalizedHarmonicParam / (generalizedHarmonicParam + n -1));

        myStepSize[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)] = tStep;
        myNumVisits[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)] = myNumVisits[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)] + 1;
    }

    budgetStatesUpdates.clear();
    netSizeUpdates.clear();
}

void ReliabilityValueFunction::UpdateStateValue(double budget, int netSize, double newValue){
    int budgetIndex = (int) (budget / 10);
    myValues[budgetIndex][netSize] = newValue;
}

void ReliabilityValueFunction::UpdateTempStateValue(double budget, int netSize, double newValue){

    int budgetIndex = (int) (budget / 10);
    budgetStatesUpdates.push_back(budgetIndex);
    netSizeUpdates.push_back(netSize);

    myTempValues[budgetIndex][netSize] = newValue;

}

void ReliabilityValueFunction::UpdateStepSize(double budget, int netSize, double newStep){

    int budgetIndex = (int) (budget / 10);
    myStepSize[budgetIndex][netSize] = newStep;
}

void ReliabilityValueFunction::PrintValueFunction(ofstream &outputFile) {

    for (int i = 0; i < myBudgetSize; ++i) {
        for (int j = 0; j < (maxNetSize - minNetSize + 1); ++j) {
            outputFile << myValues[i][j] << " " ;
        }
        outputFile << endl;
    }
}

void ReliabilityValueFunction::SetInitialStateValue(int budgetLevel, double initialValue, double learningRate) {

    for (int i = 0; i < (maxNetSize - minNetSize + 1); ++i) {
        myValues[budgetLevel][i] = initialValue;
        myStepSize[budgetLevel][i] = learningRate;
    }
}

void ReliabilityValueFunction::SetInitialStateValueStC(int budgetLevel, double initialValue, double initialLearningRate) {

    for (int i = 0; i < (maxNetSize - minNetSize + 1); ++i) {
        myValues[budgetLevel][i] = initialValue;
        myNumVisits[budgetLevel][i] = 1;

        double n = (double) myNumVisits[budgetLevel][i];
        double tStep = initialLearningRate * (((b / n) + a) / ((b / n) + a + n));

        myStepSize[budgetLevel][i] = tStep;
    }
}

void ReliabilityValueFunction::SetInitialStateValueGHarmonic(int budgetLevel, double initialValue, double initialLearningRate) {

    for (int i = 0; i < (maxNetSize - minNetSize + 1); ++i) {
        myValues[budgetLevel][i] = initialValue;
        myNumVisits[budgetLevel][i] = 1;

        double n = (double) myNumVisits[budgetLevel][i];
        double tStep = initialLearningRate * (generalizedHarmonicParam / (generalizedHarmonicParam + n - 1.0));

        myStepSize[budgetLevel][i] = tStep;
    }
}

void ReliabilityValueFunction::ReadValueFunction(int budgetLevel, int netSize, double initialValue) {
    myValues[budgetLevel][netSize] = initialValue;
}





/*
ReliabilityValueFunction::ReliabilityValueFunction() {

    fill(*myStepSize, *myStepSize + (720*567), 1.0);
}

double ReliabilityValueFunction::GetStateValue(double budget, int netSize) {

    int budgetIndex = (int) (budget / 10);
//    cout << budget << " - " << netSize << " - budgetIndex : " << budgetIndex << "   " << myValues[budgetIndex][netSize] << endl;
    return myValues[budgetIndex][netSize];
}

double ReliabilityValueFunction::GetTempStateValue(double budget, int netSize) {

    int budgetIndex = (int) (budget / 10);
    return myTempValues[budgetIndex][netSize];
}

double ReliabilityValueFunction::GetStepSize(double budget, int netSize){

    int budgetIndex = (int) (budget / 10);
    return myStepSize[budgetIndex][netSize];
}

void ReliabilityValueFunction::UpdateValueFunctionConstant(double learningRate) {
    for (int i = 0; i < budgetStatesUpdates.size(); ++i) {
        myValues[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)] = myTempValues[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)];
        myStepSize[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)] = learningRate;
    }

    budgetStatesUpdates.clear();
    netSizeUpdates.clear();
}

void ReliabilityValueFunction::UpdateValueFunctionStC(double initialLearningRate) {
    for (int i = 0; i < budgetStatesUpdates.size(); ++i) {
        myValues[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)] = myTempValues[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)];
        myNumVisits[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)] = myNumVisits[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)] + 1;

        double n = (double) myNumVisits[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)];
        double tStep = initialLearningRate * (((b / n) + a) / ((b / n) + a + n));

        myStepSize[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)] = tStep;
        myNumVisits[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)] = myNumVisits[budgetStatesUpdates.at(i)][netSizeUpdates.at(i)] + 1;
    }

    budgetStatesUpdates.clear();
    netSizeUpdates.clear();
}

void ReliabilityValueFunction::UpdateStateValue(double budget, int netSize, double newValue){
    int budgetIndex = (int) (budget / 10);
    myValues[budgetIndex][netSize] = newValue;
}

void ReliabilityValueFunction::UpdateTempStateValue(double budget, int netSize, double newValue){

    int budgetIndex = (int) (budget / 10);
    budgetStatesUpdates.push_back(budgetIndex);
    netSizeUpdates.push_back(netSize);

    myTempValues[budgetIndex][netSize] = newValue;

}

void ReliabilityValueFunction::UpdateStepSize(double budget, int netSize, double newStep){

    int budgetIndex = (int) (budget / 10);
    myStepSize[budgetIndex][netSize] = newStep;
}

void ReliabilityValueFunction::PrintValueFunction(ofstream &outputFile) {

    for (int i = 0; i < 720; ++i) {
        for (int j = 0; j < 567 ; ++j) {
            outputFile << myValues[i][j] << " " ;
        }
        outputFile << endl;
    }
}

void ReliabilityValueFunction::SetInitialStateValue(int budgetLevel, double initialValue, double learningRate) {

    for (int i = 0; i < 567; ++i) {
        myValues[budgetLevel][i] = initialValue;
        myStepSize[budgetLevel][i] = learningRate;
    }
}

void ReliabilityValueFunction::SetInitialStateValueStC(int budgetLevel, double initialValue, double initialLearningRate) {

    for (int i = 0; i < 567; ++i) {
        myValues[budgetLevel][i] = initialValue;
        myNumVisits[budgetLevel][i] = 1;

        double n = (double) myNumVisits[budgetLevel][i];
        double tStep = initialLearningRate * (((b / n) + a) / ((b / n) + a + n));

        myStepSize[budgetLevel][i] = tStep;
    }
}*/
