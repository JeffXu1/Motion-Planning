#include "lec5_hw/visualizer.hpp"
#include "lec5_hw/trajectory.hpp"

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <math.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include "trajectory_generator_waypoint.h"
using namespace std;
using namespace Eigen;
TrajectoryGeneratorWaypoint trajectoryGeneratorWaypoint;
//vector<Vector3d> path_list;

struct Config
{
    std::string targetTopic;
    double clickHeight;
    std::vector<double> initialVel;
    std::vector<double> initialAcc;
    std::vector<double> terminalVel;
    std::vector<double> terminalAcc;
    double allocationSpeed;
    double allocationAcc;
    int maxPieceNum;

    Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("TargetTopic", targetTopic);
        nh_priv.getParam("ClickHeight", clickHeight);
        nh_priv.getParam("InitialVel", initialVel);
        nh_priv.getParam("InitialAcc", initialAcc);
        nh_priv.getParam("TerminalVel", terminalVel);
        nh_priv.getParam("TerminalAcc", terminalAcc);
        nh_priv.getParam("AllocationSpeed", allocationSpeed);
        nh_priv.getParam("AllocationAcc", allocationAcc);
        nh_priv.getParam("MaxPieceNum", maxPieceNum);
    }
};

double timeTrapzVel(const double dist,
                    const double vel,
                    const double acc)
{
    const double t = vel / acc;
    const double d = 0.5 * acc * t * t;

    if (dist < d + d)
    {
        return 2.0 * sqrt(dist / acc);
    }
    else
    {
        return 2.0 * t + (dist - 2.0 * d) / vel;
    }
}

void minimumJerkTrajGen(
    // Inputs:
    const int pieceNum,
    const Eigen::Matrix3Xd &waypoints,
    const Eigen::Vector3d &initialPos,
    const Eigen::Vector3d &initialVel,
    const Eigen::Vector3d &initialAcc,
    const Eigen::Vector3d &terminalPos,
    const Eigen::Vector3d &terminalVel,
    const Eigen::Vector3d &terminalAcc,
    const Eigen::Matrix3Xd &intermediatePositions,
    const Eigen::VectorXd &timeAllocationVector,
    // Outputs:
    Eigen::MatrixXd &coefficientMatrix)
{
    // coefficientMatrix is a matrix with 6*piece num rows and 3 columes
    // As for a polynomial c0+c1*t+c2*t^2+c3*t^3+c4*t^4+c5*t^5,
    // each 6*3 sub-block of coefficientMatrix is
    // --              --
    // | c0_x c0_y c0_z |
    // | c1_x c1_y c1_z |
    // | c2_x c2_y c2_z |
    // | c3_x c3_y c3_z |
    // | c4_x c4_y c4_z |
    // | c5_x c5_y c5_z |
    // --              --
    // Please computed coefficientMatrix of the minimum-jerk trajectory
    // in this function

    // ------------------------ Put your solution below ------------------------
    //Eigen::MatrixXd path_list = Eigen::MatrixXd::Zero(pieceNum+1, 3); 
    //Eigen::MatrixXd coefficientMatrix_g = Eigen::MatrixXd::Zero(6 * pieceNum, 3);
    Eigen::MatrixXd vel = Eigen::MatrixXd::Zero(2, 3); 
    Eigen::MatrixXd acc = Eigen::MatrixXd::Zero(2, 3);
    int dev_order=3;

    vel.row(0)=initialPos;


    vel.row(0)=initialVel;
    vel.row(1)=terminalVel;
    acc.row(0)=initialAcc;
    acc.row(1)=terminalAcc;

    //cout<< "Here is 620 " << endl;
    //coefficientMatrix = Eigen::Matrix3d::Zero(6 * pieceNum, 3);
    //cout << "coefficientMatrix 50 :" << endl << coefficientMatrix<< endl;

    //cout<< "Here is 600 " << endl;
    coefficientMatrix=trajectoryGeneratorWaypoint.PolyQPGeneration(dev_order, waypoints, vel, acc, timeAllocationVector);
    cout << "coefficientMatrix 50 :" << endl << coefficientMatrix<< endl;
    //cout<< "Here is 610 " << endl;
    // ------------------------ Put your solution above ------------------------
}

class ClickGen
{
private:
    Config config;

    ros::NodeHandle nh;
    ros::Subscriber targetSub;

    Visualizer visualizer;

    Eigen::Matrix3Xd positions;
    Eigen::VectorXd times;
    int positionNum;
    Trajectory<5> traj;

public:
    ClickGen(const Config &conf,
             ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          visualizer(nh),
          positions(3, config.maxPieceNum + 1),
          times(config.maxPieceNum),
          positionNum(0)
    {
        targetSub = nh.subscribe(config.targetTopic, 1,
                                 &ClickGen::targetCallBack, this,
                                 ros::TransportHints().tcpNoDelay());
    }

    void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (positionNum > config.maxPieceNum)
        {
            positionNum = 0;
            traj.clear();
        }

        positions(0, positionNum) = msg->pose.position.x;
        positions(1, positionNum) = msg->pose.position.y;
        positions(2, positionNum) = std::fabs(msg->pose.orientation.z) * config.clickHeight;

        if (positionNum > 0)
        {
            const double dist = (positions.col(positionNum) - positions.col(positionNum - 1)).norm();
            times(positionNum - 1) = timeTrapzVel(dist, config.allocationSpeed, config.allocationAcc);
        }

        ++positionNum;

        if (positionNum > 1)
        {
            const int pieceNum = positionNum - 1;
            const Eigen::Vector3d initialPos = positions.col(0);
            const Eigen::Vector3d initialVel(config.initialVel[0], config.initialVel[1], config.initialVel[2]);
            const Eigen::Vector3d initialAcc(config.initialAcc[0], config.initialAcc[1], config.initialAcc[2]);
            const Eigen::Vector3d terminalPos = positions.col(pieceNum);
            const Eigen::Vector3d terminalVel(config.terminalVel[0], config.terminalVel[1], config.terminalVel[2]);
            const Eigen::Vector3d terminalAcc(config.terminalAcc[0], config.terminalAcc[1], config.terminalAcc[2]);
            const Eigen::Matrix3Xd intermediatePositions = positions.middleCols(1, pieceNum - 1);
            const Eigen::VectorXd timeAllocationVector = times.head(pieceNum);

            Eigen::MatrixXd coefficientMatrix = Eigen::MatrixXd::Zero(6 * pieceNum, 3);
            Eigen::Matrix3Xd positions_transfer = Eigen::MatrixXd::Zero(3, positionNum);
            //cout<< "coefficientMatrix size " << coefficientMatrix.size()<<endl;
            //cout << "position value is:" << endl << positions << endl;

            //cout << "initialPos value is:" << endl << initialPos << endl;

            //cout << "terminalPos value is:" << endl << terminalPos << endl;

            //cout << "intermediatePositions value is:" << endl << intermediatePositions << endl;
            for (int j=0;j<positionNum;j++)
            {
              positions_transfer.col(j)= positions.col(j);
            }
            cout << "positions_col value is:" << endl << positions_transfer << endl;
            cout << "coefficientMatrix value is:" << endl << coefficientMatrix << endl;
            minimumJerkTrajGen(pieceNum,positions_transfer,
                               initialPos, initialVel, initialAcc,
                               terminalPos, terminalVel, terminalAcc,
                               intermediatePositions,
                               timeAllocationVector,
                               coefficientMatrix);

            traj.clear();
            traj.reserve(pieceNum);
            //cout << "here is 500" << endl;
            for (int i = 0; i < pieceNum; i++)
            {
                //cout << "here is 510" << endl;
                traj.emplace_back(timeAllocationVector(i),
                                  coefficientMatrix.block<6, 3>(6 * i, 0).transpose().rowwise().reverse());
                //cout << "here is 520" << endl;
            }
        }

        visualizer.visualize(traj, positions.leftCols(positionNum));

        return;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "click_gen_node");
    ros::NodeHandle nh_;
    ClickGen clickGen(Config(ros::NodeHandle("~")), nh_);
    ros::spin();
    return 0;
}
