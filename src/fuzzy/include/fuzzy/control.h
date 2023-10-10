#ifndef CONTROL_H
#define CONTROL_H

#include <vector>

using namespace std;

// Function declaration
void myFunction();
float triangle_membership(float x, float a, float b, float c);
int getObstacleAngle(const vector<float>& array);
vector<float> USToBinary(const vector<float>& sensor);

// Constants
const int MY_CONSTANT = 42;

#endif