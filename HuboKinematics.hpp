/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

#ifndef HUBO_KINEMATICS_H
#define HUBO_KINEMATICS_H

#include <Tabs/GRIPTab.h>
#include <Tabs/GRIPThread.h>

#include <robotics/Robot.h>
#include <robotics/Object.h>
#include <dynamics/BodyNodeDynamics.h>
#include <robotics/World.h>
#include <integration/EulerIntegrator.h>
#include <integration/RK4Integrator.h>

#include <kinematics/Joint.h>
#include <kinematics/Dof.h>

#include <iostream>
#include <time.h>

class HuboWalk
{
public:
    HuboWalk(double startTime, double dT, int numTimeSteps, int numWalkSteps);

    double mTimeInit;
    double mdT;
    int mNumTimeSteps;
    double getTimeEnd() { return mTimeInit + (mdT * mNumTimeSteps); }

    int mCurrentWalkFrame;

    void DoNextWalkFrame();
    void DoWalkFrame(double t);
    void DoWalkFrame(int timestep);

    int mNumWalkSteps;
    
    Eigen::VectorXd mLegVals;
    Eigen::VectorXi swingLegLinks;
    Eigen::VectorXi stanceLegLinks;
    
    double mBaseIncrement;
    double mHalfIncrement;
    double mSwayIncrement;
    
    double mForwardStepIncrement;
    double mForwardStepWalkIncrement;
    double mForwardStepStandIncrement;
};

class HuboKinematics
{
public:
    robotics::World* mWorld;
    int mHuboId;
    
    HuboKinematics(robotics::World* w, int HuboID);
    void InitLinkTable();
    void PrintLinkPositions();
    void WriteInitialPose(Viewer* viewer);
    void HuboWalk(GRIPFrame* frame, Viewer* viewer);

    Eigen::VectorXd initialConfig;

    Eigen::VectorXi mLeftLegLinks;
    Eigen::VectorXi mRightLegLinks;

    Eigen::VectorXi mLeftArmLinks;
    Eigen::VectorXi mRightArmLinks;

    Eigen::VectorXd mLeftLegConf;
    Eigen::VectorXd mRightLegConf;

    Eigen::VectorXd mLeftArmConf;
    Eigen::VectorXd mRightArmConf;

    string static const mLeftLegLinkNames[];
    string static const mRightLegLinkNames[];
    string static const mLeftArmLinkNames[];
    string static const mRightArmLinkNames[];

    string static const mLeftLegEEName;
    string static const mRightLegEEName;
    string static const mLeftArmEEName;
    string static const mRightArmEEName;

    int static const mLeftLegLinkNum = 6;
    int static const mRightLegLinkNum = 6;
    int static const mLeftArmLinkNum = 7;
    int static const mRightArmLinkNum = 7;


    int mLeftLegEEId;
    int mRightLegEEId;
    int mLeftArmEEId;
    int mRightArmEEId;
  
};


#endif // HUBO_KINEMATICS_H
