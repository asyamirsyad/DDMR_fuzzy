#include <ros/ros.h>
#include <vector>

using namespace std;

void myFunction() {
    ROS_INFO("This is my function!");
}

float triangle_membership(float x, float a, float b, float c) {
    if (x <= a || x >= c) {
        return 0.0;
    }
    else if (x >= b) {
        return (c - x) / (c - b);
    }
    else {
        return (x - a) / (b - a);
    }
}

// getting obstacle angle with binary value
int getObstacleAngle(const vector<float>& array) {
    int obstacleCount = 0;
    int obstacleAngle = -1;

    for (int i = 0; i < 8; i++) {
        if (array[i] != 0) {
            obstacleCount++;
            switch (i) {
            case 0:
                obstacleAngle = -90;
                break;
            case 1:
                obstacleAngle = -60;
                break;
            case 2:
                obstacleAngle = -30;
                break;
            case 3:
                obstacleAngle = -10;
                break;
            case 4:
                obstacleAngle = 10;
                break;
            case 5:
                obstacleAngle = 30;
                break;
            case 6:
                obstacleAngle = 60;
                break;
            case 7:
                obstacleAngle = 90;
                break;
            }
        }

        if (obstacleCount == 2) {
            if (array[2] > 0 && array[3] > 0) {
                obstacleAngle = -20;
            }
            else if (array[1] > 0 && array[2] > 0) {
                obstacleAngle = -45;
            }
            else if (array[0] > 0 && array[1] > 0) {
                obstacleAngle = -75;
            }
            else if (array[5] > 0 && array[6] > 0) {
                obstacleAngle = 45;
            }
            else if (array[6] > 0 && array[7] > 0) {
                obstacleAngle = 75;
            }
            if (array[3] > 0 && array[4] > 0) {
                obstacleAngle = 0;
            }
        }
        else {
            obstacleAngle = -1;
        }
    }


    // if (obstacleAngle < 0) {
    //     belok = -1;
    // }
    // else {
    //     belok = 1;
    // }

    // cout << obstacleAngle << " <- angle: ";

    return obstacleAngle;
}


vector<float> USToBinary(const vector<float>& sensor) {
    vector<float> binaryVector = { 0, 0, 0, 0, 0, 0, 0, 0 };
    float minVal1 = 1.5;
    float minVal2 = 1.5;
    int minIdx = -1;
    int minIdx2 = -1;

    for (int i = 0; i < 8; i++) {
        if (sensor[i] < minVal1) {
            minVal2 = minVal1;
            minIdx2 = minIdx;
            minVal1 = sensor[i];
            minIdx = i;
        }
        else if (sensor[i] < minVal2 && sensor[i] != minVal1) {
            minVal2 = sensor[i];
            minIdx2 = i;
        }
    }

    for (int i = 0; i < 8; i++) {
        if (sensor[i] == minVal1 && minVal1 != 0) {
            binaryVector[i] = minVal1;
        }
        else if (sensor[i] == minVal2 && minVal2 != 0 && abs(minVal1 - minVal2) < 0.05 && abs(minIdx - minIdx2) == 1) {
            binaryVector[i] = minVal2;
        }
        else {
            binaryVector[i] = 0;
        }
    }

    // for (int i = 0; i < 8; i++) {
    //     cout << binaryVector[i] << ",";
    // }
    // cout << endl;

    return binaryVector;
}
