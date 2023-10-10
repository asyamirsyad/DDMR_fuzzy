#include <math.h>

#include <cmath>
#include <iostream>
#include <vector>

#include "geometry_msgs/Pose2D.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"

#include "fuzzy/control.h"

using namespace std;

vector<float> arr_us(8);

vector<float> initial_speed = { 6.000, 6.000 };
vector<double> robot_position(3);
float hasil = 0.000;
float belok = 1;

void getUSvalues(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    for (int i = 0; i < 8; i++) {
        arr_us[i] = msg->data[i];
    }
}

void runThread() {
    vector<float> arr_biner = USToBinary(arr_us);
    float angleObs = getObstacleAngle(arr_biner);
    float distance = 0;

    float mu_angle[36];
    float mu_distance[36];
    float mu_total[36];
    float resultan[36];

    for (int i = 0; i < 8; i++) {
        if (arr_us[i] != 0) {
            distance = arr_us[i];
        }
    }

    if (angleObs < 0) {
        belok = -1;
    }
    else {
        belok = 1;
    }

    float VB = 10, B = 8, M = 6, S = 4, VS = 2;

    resultan[0] = S;
    resultan[1] = M;
    resultan[2] = B;
    resultan[3] = VB;
    resultan[4] = VB;
    resultan[5] = VB;
    resultan[6] = B;
    resultan[7] = M;
    resultan[8] = S;

    resultan[9] = VS;
    resultan[10] = S;
    resultan[11] = M;
    resultan[12] = B;
    resultan[13] = B;
    resultan[14] = B;
    resultan[15] = M;
    resultan[16] = S;
    resultan[17] = VS;

    resultan[18] = VS;
    resultan[19] = VS;
    resultan[20] = S;
    resultan[21] = S;
    resultan[22] = S;
    resultan[23] = S;
    resultan[24] = S;
    resultan[25] = VS;
    resultan[26] = VS;

    resultan[27] = VS;
    resultan[28] = VS;
    resultan[29] = VS;
    resultan[30] = VS;
    resultan[31] = VS;
    resultan[32] = VS;
    resultan[33] = VS;
    resultan[34] = VS;
    resultan[35] = VS;

    // membership jarak
    float VC = triangle_membership(distance, -10, 0, 0.5);  // VC
    float C = triangle_membership(distance, 0, 0.5, 1);     // C
    float F = triangle_membership(distance, 0.5, 1, 1.5);   // F
    float TF = triangle_membership(distance, 1, 1.5, 10);   // TF
    vector<float> mem_distance = { VC, C, F, TF };

    // membership sudut
    float M7 = triangle_membership(angleObs, 60, 90, 180);      // M7
    float M5 = triangle_membership(angleObs, 30, 60, 90);       // M5
    float M3 = triangle_membership(angleObs, 10, 30, 60);       // M3
    float M1 = triangle_membership(angleObs, 0, 10, 30);        // M1
    float M0 = triangle_membership(angleObs, -10, 0, 10);       // M0
    float iM1 = triangle_membership(angleObs, 0, -10, -30);     //-M1
    float iM3 = triangle_membership(angleObs, -10, -30, -60);   //-M3
    float iM5 = triangle_membership(angleObs, -30, -60, -90);   //-M5
    float iM7 = triangle_membership(angleObs, -60, -90, -180);  //-M7
    vector<float> mem_angle = { iM7, iM5, iM3, iM1, M0, M1, M3, M5, M7 };

    // inference system / rulebase
    for (int i = 0; i < 36; i++) {
        mu_distance[i] = mem_distance[i / 9];
        mu_angle[i] = mem_angle[i % 9];
        mu_total[i] = min(mu_angle[i], mu_distance[i]);
    }

    float Z = 1;
    float mu_z = 1;
    for (int i = 0; i < 36; i++) {
        Z += mu_total[i] * resultan[i];
        mu_z += mu_total[i];
    }

    if (angleObs == -1) {
        hasil = 0;
    }
    else {
        hasil = Z / mu_z;
    }

    cout << distance << "||" << angleObs << "||" << hasil << endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "index");
    ros::NodeHandle node_obj;

    ros::Publisher motor_publisher =
        node_obj.advertise<std_msgs::Float32MultiArray>("/set_motor", 10);

    ros::Subscriber ultrasound_subscriber =
        node_obj.subscribe("/ultrasound", 10, getUSvalues);

    ros::Rate loop_rate(20);

    float angularSpeed = 0.000;
    float linearSpeed = 6.000;

    while (ros::ok()) {
        ros::spinOnce();  // for subscribe loop
        std_msgs::Float32MultiArray msg;
        runThread();

        if (!isnan(hasil)) {
            angularSpeed = hasil * belok;
        }
        else {
            angularSpeed = 0;
        }

        msg.data = { linearSpeed, angularSpeed };

        motor_publisher.publish(msg);
        loop_rate.sleep();
    }

    return 0;
}