//
// Created by nboardma on 11/12/2020.
//

#ifndef CPP_ADP_ADPRUNFILES_H
#define CPP_ADP_ADPRUNFILES_H

#include <vector>
#include <random>
using namespace std;

void InitialSensorDeployment(vector<double> &gradientVect, vector<int> &numInRegion, vector<double> subArea, vector<double> subWeights, int nSens);

double AgeAggregate(vector<int> ageVector);

void ADP_CBM_AVI_CostModel(int nSens, int numReps, int nMissions, double relRequirement, double delta, double Budget,
                           double cFixed, double cVar);

void ADP_CBM_AVI_CostModel_CAVE(int nSens, int numReps, int nMissions, double relRequirement, double delta, double Budget,
                 double cFixed, double cVar);

void ADP_CBM_AVI_CostModel_BinarySearch_CAVE(int nSens, int numReps, int nMissions, double relRequirement, double delta, double Budget,
                                double cFixed, double cVar);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ADP_CBM_AVI_ReliabilityModel(int nSens, int numReps, int nMissions, double delta, double Budget,
                                  double cFixed, double cVar);

void ADP_CBM_AVI_ReliabilityModel_CAVE(int nSens, int numReps, int nMissions, double delta, double Budget,
                                  double cFixed, double cVar);

void ADP_CBM_AVI_ReliabilityModel_CAVE_EvaluateMC(int nSens, int numReps, int nMissions, double delta, double Budget,
                                       double cFixed, double cVar, double covRequirement);

void ADP_CBM_InputValueFunction_EvaluateMC(int nSens, int numReps, int nMissions, double delta, double Budget,
                                                  double cFixed, double cVar, double covRequirement);

void ADP_CBM_AVI_ReliabilityModel_CAVE_EvaluateMC_MinRequirement(int nSens, int numReps, int nMissions, double delta, double Budget,
                                                  double cFixed, double cVar, double covRequirement, double relRequirement);

void ADP_CBM_AVI_ReliabilityModel_CAVE_MinRequirement(int nSens, int numReps, int nMissions, double delta, double Budget,
                                                                 double cFixed, double cVar, double covRequirement, double relRequirement);

void ADP_CBM_InputValueFunction_EvaluateMC_Requirement(int nSens, int numReps, int nMissions, double delta, double Budget,
                                           double cFixed, double cVar, double covRequirement, double relRequirement);

void ADP_CBM_AVI_ReliabilityModel_CAVE_EvaluateMC_MinRequirement_PenaltyCost(int nSens, int numReps, int nMissions, double delta, double Budget,
                                                                 double cFixed, double cVar, double covRequirement, double relRequirement);


void ADP_CBM_AVI_ReliabilityModel_CAVE_EvaluateMC_MinRequirement_Skip(int nSens, int numReps, int nMissions, double delta, double Budget,
                                                                 double cFixed, double cVar, double covRequirement, double relRequirement);

void DebugAge(int nSens, int numReps, int nMissions, double delta, double Budget, double cFixed, double cVar, double covRequirement, double relRequirement);

void ADP_CBM_AVI_ReliabilityModel_CAVE_EvaluateMC_MinRequirement_SingleRegion(int nSens, int numReps, int nMissions, double delta, double Budget,
                                                                              double cFixed, double cVar, double covRequirement, double relRequirement);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



#endif //CPP_ADP_ADPRUNFILES_H
