/*
 * EKF.h
 *
 *  Created on: May 7, 2012
 *      Author: engelhan
 */

#include "ds_control/EKF.h"

using namespace std;
using namespace Eigen;


void ExtendedKalmanFilter::initFilter(){
    state = Eigen::Vector3f(0,0,0);

    sigma = Eigen::Matrix3f::Zero();
    sigma(0,0) = sigma(1,1) = 1; sigma(2,2) = 1;

    Q = Eigen::Matrix3f::Zero();
    Q(0,0) = Q(1,1) = 0.002; Q(2,2) = 0.00002;

    R = Eigen::Matrix3f::Zero();
    R(0,0) = R(1,1) = 0.01; R(2,2) = 0.0001;
}

void ExtendedKalmanFilter::printState(){
    std::cout << "kalman state: " <<
                 state(0) << "  " <<
                 state(1) << " " <<
                 state(2)/M_PI*180 << std::endl;
}


// odometry:
// x: distance travelled in local x-direction
// y: distance travelled in local y-direction
// phi: rotation update
void ExtendedKalmanFilter::predictionStep(const Eigen::Vector3f& odometry){

    state(0) = state(0) + cos(state(2))*odometry(0) - sin(state(2))*odometry(1);
    state(1) = state(1) + sin(state(2))*odometry(0) + cos(state(2))*odometry(1);
    state(2) = state(2) + odometry(2);

    state(2) = atan2(sin(state(2)),cos(state(2))); // normalize angle

    // dg/dx:
    Eigen::Matrix3f G;

    G << 1, 0, -sin(state(2))*odometry(0) - cos(state(2))*odometry(1),
            0, 1,  cos(state(2))*odometry(0) - sin(state(2))*odometry(1),
            0, 0,  1;

    // cout << "G: " << endl << G << endl;

    sigma = G*sigma*G.transpose() + Q;

}



void ExtendedKalmanFilter::correctionStep(const Eigen::Vector3f& measurement, const Eigen::Vector3f& global_marker_pose){  // compare expected and measured values, update state and uncertainty


    // compute expected measurement:
    Vector3f z_exp; // z_exp = h(x)
    z_exp(0) = cos(state(2))*(global_marker_pose(0)- state(0)) + sin(state(2))*(global_marker_pose(1)-state(1));
    z_exp(1) = -sin(state(2))*(global_marker_pose(0) -state(0)) + cos(state(2))*(global_marker_pose(1)-state(1));
    z_exp(2) =  global_marker_pose(2)-state(2);

    Matrix3f H; // dh/dx
    H << -cos(state(2)), -sin(state(2)), -sin(state(2))*(global_marker_pose(0) - state(0)) + cos(state(2))*(global_marker_pose(1)-state(1)),
            sin(state(2)), -cos(state(2)), -cos(state(2))*(global_marker_pose(0) -state(0)) - sin(state(2))*(global_marker_pose(1)-state(1)),
            0,                  0, -1;

    // cout << "H: " << endl << H << endl;

    Matrix3f K = sigma * H.transpose() * ((H * sigma * H.transpose() + R).inverse()); // Kalman Gain

    // compute the difference between expectation and observation
    Vector3f z_diff = measurement - z_exp;
    // normalize angle
    z_diff(2) = atan2(sin(z_diff(2)),cos(z_diff(2)));

    // cout << "z_exp: " << z_exp(0) << " "<< z_exp(1) << " "<< z_exp(2) << endl;
    // cout << "diff: " << z_diff(0) << " "<< z_diff(1) << " "<< z_diff(2) << endl;

    // correct pose estimate
    state = state + K*( z_diff );
    sigma = (Matrix3f::Identity() - K*H)*sigma;


}





void ExtendedKalmanFilter::testFilter(){

    // creation of fake measurements:
    Eigen::MatrixXf U = createTestOdometry();
    Eigen::Vector3f z_global(5,5,0); // Global Marker pose
    Eigen::MatrixXf Z = createTestMeasurements();

    // cout << "U: " << endl << U << endl;
    // cout << "Z: " << endl << Z << endl;

    initFilter();

    ros::Rate r(5);

    for (uint i=0; i<44; ++i){
        r.sleep();


        Vector3f u(U(i,0),U(i,1),U(i,2));
        cout << (i+1) << ": odo: " << u << endl;
        predictionStep(u);

        cout << "new pose:" << endl << state << endl;
        cout << "new sigma:" << endl << sigma << endl;

        if (i>0 && (i+1) % 11 == 0){
            ROS_WARN("CORRECTION");
            Vector3f z(Z((i+1)/11-1,0),Z((i+1)/11-1,1),Z((i+1)/11-1,2));
            cout << "Measurement: " << endl << z << endl;

            correctionStep(z,z_global);

            cout << "after correction: " << endl << "state: " << endl << state << endl << "sigma:" << endl << sigma << endl;

        }

        marker.addFilterState(state, sigma, 0);

        marker.publish_last_n_states(10);

    }
}




Eigen::MatrixXf ExtendedKalmanFilter::createTestMeasurements(){

    Eigen::Matrix<float, 8,3> Z;
    Z << 5,5,-M_PI/2,
            5,5,-M_PI,
            5,5,-3*M_PI/2,
            5,5,0, // second round
            5,5,-M_PI/2,
            5,5,-M_PI,
            5,5,-3*M_PI/2,
            5,5,0;

    return Z;
}

Eigen::MatrixXf ExtendedKalmanFilter::createTestOdometry(){
    Eigen::Matrix<float, 88,3> U;
    U << 1,0,0 ,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            0,0,M_PI/2,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            0,0,M_PI/2,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            0,0,M_PI/2,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            0,0,M_PI/2, // second round
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            0,0,M_PI/2,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            0,0,M_PI/2,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            0,0,M_PI/2,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            1,0,0,
            0,0,M_PI/2;


    return U;
}



