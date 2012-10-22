/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

#include <GUI/GRIPFrame.h>
#include <GUI/Viewer.h>
#include "HuboKinematics.hpp"
#include <string>

/////////////////////////////////////////////////////////////////////////////////////////////
//#########################################################################################//
//# hubo kinematics variables                                                             #//
//#########################################################################################//
/////////////////////////////////////////////////////////////////////////////////////////////


string const HuboKinematics::mLeftLegLinkNames[mLeftLegLinkNum] = { "leftHip", "leftHipPitchRoll", "leftKneeUpper", "leftKneeLower", "leftAnklePitch", "leftFoot" };
string const HuboKinematics::mRightLegLinkNames[mRightLegLinkNum] = { "rightHip", "rightHipPitchRoll", "rightKneeUpper", "rightKneeLower", "rightAnklePitch", "rightFoot" };
string const HuboKinematics::mLeftArmLinkNames[mLeftArmLinkNum] = { "Body_LSP", "Body_LSR", "Body_LSY", "Body_LEP", "Body_LWY", "leftUJoint", "leftPalmDummy" };
string const HuboKinematics::mRightArmLinkNames[mRightArmLinkNum] = { "Body_RSP", "Body_RSR", "Body_RSY", "Body_REP", "Body_RWY", "rightUJoint", "rightPalmDummy" };


string const HuboKinematics::mLeftLegEEName = "leftFoot";
string const HuboKinematics::mRightLegEEName = "rightFoot";
string const HuboKinematics::mLeftArmEEName = "leftPalmDummy";
string const HuboKinematics::mRightArmEEName = "leftFoot";


/////////////////////////////////////////////////////////////////////////////////////////////
//#########################################################################################//
//# hubo kinematics constructor                                                           #//
//#########################################################################################//
/////////////////////////////////////////////////////////////////////////////////////////////


HuboKinematics::HuboKinematics(robotics::World* w, int HuboID)
{
    mWorld = w;
    mHuboId = HuboID;

    initialConfig.resize(65);
    initialConfig <<
        -.132, 1.316, .93, 0.0, 0.0, 0.0, 0.0, 0.0, 
        -.785, .785, -.785, -.785, -.785, 
        -3.14, .785, -.785, -.785, -.785, 
        -.785, .785, -.785, -.785, -.785, 
        -.785, .785, -.785, -.785, -.785, 
        -.523, 0.0, -1.570, -0.523, 0.0, 
        0.0, 1.047, 0.523, 0.366, -0.174, 
        0.0, 0.0, 0.0, -0.523, 0.0, -1.570, 
        -0.523, 0.0, 0.0, 1.047, 0.523, -0.366, 
        0.174, 0.0, 0.0, 0.0, 0.0, -0.785, -0.785, 
        -0.785, -0.785, 0.785, -0.785, -0.785, -0.785, -0.785, 0.785;
}


/////////////////////////////////////////////////////////////////////////////////////////////
//#########################################################################################//
//# interacting with hubo world object                                                    #//
//#########################################################################################//
/////////////////////////////////////////////////////////////////////////////////////////////

void HuboKinematics::InitLinkTable() {
  
    int allDofs = mWorld->getRobot(mHuboId)->getNumDofs();

    // Left Leg Links
    mLeftLegLinks.resize( mLeftLegLinkNum );
    for( int i = 0; i < mLeftLegLinkNum; ++i ) {
        for( int j = 0; j < allDofs; ++j ) {

            if( strcmp( mWorld->getRobot(mHuboId)->getDof( j )->getJoint()->getChildNode()->getName(), 
                        mLeftLegLinkNames[i].c_str() ) == 0 ) {
                mLeftLegLinks[i] = j;
                break;
            } 

        } 
    } 

    mLeftLegEEId = mWorld->getRobot(mHuboId)->getNodeIndex( mLeftLegEEName.c_str() );

    // Right Leg
    mRightLegLinks.resize( mRightLegLinkNum );
    for( int i = 0; i < mRightLegLinkNum; ++i ) {
        for( int j = 0; j < allDofs; ++j ) {

            if( strcmp( mWorld->getRobot(mHuboId)->getDof( j )->getJoint()->getChildNode()->getName(), 
                        mRightLegLinkNames[i].c_str() ) == 0 ) {
                mRightLegLinks[i] = j;
                break;
            } 

        } 
    }

    mRightLegEEId = mWorld->getRobot(mHuboId)->getNodeIndex( mRightLegEEName.c_str() );

    // Left Arm Links
    mLeftArmLinks.resize( mLeftArmLinkNum );
    for( int i = 0; i < mLeftArmLinkNum; ++i ) {
        for( int j = 0; j < allDofs; ++j ) {

            if( strcmp( mWorld->getRobot(mHuboId)->getDof( j )->getJoint()->getChildNode()->getName(), 
                        mLeftArmLinkNames[i].c_str() ) == 0 ) {
                mLeftArmLinks[i] = j;
                break;
            } 

        } 
    } 

    mLeftArmEEId = mWorld->getRobot(mHuboId)->getNodeIndex( mLeftArmEEName.c_str() );

    // Right Arm
    mRightArmLinks.resize( mRightArmLinkNum );
    for( int i = 0; i < mRightArmLinkNum; ++i ) {
        for( int j = 0; j < allDofs; ++j ) {

            if( strcmp( mWorld->getRobot(mHuboId)->getDof( j )->getJoint()->getChildNode()->getName(), 
                        mRightArmLinkNames[i].c_str() ) == 0 ) {
                mRightArmLinks[i] = j;
                break;
            } 
      
        } 
    }
  
    mRightArmEEId = mWorld->getRobot(mHuboId)->getNodeIndex( mRightArmEEName.c_str() );


    // Print
    PrintLinkPositions();
}


void HuboKinematics::WriteInitialPose(Viewer* viewer)
{
    Eigen::MatrixXd lTransform;
    Eigen::VectorXd lXYZ(3),lXYZ2(3),dlXYZ(3);
    Eigen::VectorXi torsoLinks(3);
    torsoLinks << 0,1,2;
    dynamics::BodyNodeDynamics* foot = (dynamics::BodyNodeDynamics*)mWorld->getRobot(mHuboId)->getNode(mLeftLegEEName.c_str());
    
    // figure out stance foot's position in world coords
    lTransform = foot->getWorldTransform();
    lXYZ << lTransform(0,3), lTransform(1,3), lTransform(2,3);

    // keep our torso in the same place
    for(int i = 0; i < 6; i++)
        initialConfig[i] = mWorld->getRobot(mHuboId)->getDof(i)->getValue();

    // move everything
    int allDofs = mWorld->getRobot(mHuboId)->getNumDofs();
    for(int i=0; i<allDofs; i++){
        mWorld->getRobot(mHuboId)->getDof(i)->setValue(initialConfig[i]);
    }
    mWorld->getRobot(mHuboId)->update();

    // see where the foot moved
    lTransform = foot->getWorldTransform();
    lXYZ2 << lTransform(0,3), lTransform(1,3), lTransform(2,3);

    // negative of the foot's movement in world
    dlXYZ = lXYZ-lXYZ2;

    // move the entire robot so the foot remains stationary
    mWorld->getRobot(mHuboId)->setDofs(mWorld->getRobot(mHuboId)->getDofs(torsoLinks)+dlXYZ,torsoLinks);
    mWorld->getRobot(mHuboId)->update();

    // and finally display changes
    viewer->UpdateCamera();
}


/////////////////////////////////////////////////////////////////////////////////////////////
//#########################################################################################//
//# debug output                                                                          #//
//#########################################################################################//
/////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @function printLinks
 */
void HuboKinematics::PrintLinkPositions() {

    //-- Left Leg
    printf( "* Left Leg Links: %d \n", mLeftLegLinkNum );
    for( int i = 0; i < mLeftLegLinkNum; ++i ) {
        printf(" %d ", mLeftLegLinks[i] );
    }
    printf("\n");

    //-- Right Leg
    printf( "* Right Leg Links: %d \n", mRightLegLinkNum );
    for( int i = 0; i < mRightLegLinkNum; ++i ) {
        printf(" %d ", mRightLegLinks(i) );
    }
    printf("\n");

    //-- Left Arm
    printf( "* Left Arm Links: %d \n", mLeftArmLinkNum );
    for( int i = 0; i < mLeftArmLinkNum; ++i ) {
        printf(" %d ", mLeftArmLinks[i] );
    }
    printf("\n");


    //-- Right Arm
    printf( "* Right Arm Links: %d \n", mRightArmLinkNum );
    for( int i = 0; i < mRightArmLinkNum; ++i ) {
        printf(" %d ", mRightArmLinks[i] );
    }
    printf("\n");
}

/////////////////////////////////////////////////////////////////////////////////////////////
//#########################################################################################//
//# solitary kinematic walk - write straight to tab timeline                              #//
//#########################################################################################//
/////////////////////////////////////////////////////////////////////////////////////////////

void HuboKinematics::HuboWalk(GRIPFrame* frame, Viewer* viewer){
    // The next two lines are only here for making the video... if you incorporate this into your
    // video, then you probably want to set this globally.
    double increment = .05;
    frame->InitTimer( string("Hubo Walk"),increment );

    Eigen::VectorXd legVals[2];
    Eigen::VectorXi swingLegLinks;
    Eigen::VectorXi stanceLegLinks;
    double stepSize = .01;
    double halfStep = stepSize/2.0;
    double swayStep = stepSize/4.0;
    double forwardStep;
    double forwardStepStand = stepSize/5.0;
    double forwardStepWalk = stepSize/2.5;

    dynamics::BodyNodeDynamics *footNode[2];
    footNode[0] = (dynamics::BodyNodeDynamics*)  mWorld->getRobot(mHuboId)->getNode( mLeftLegEEName.c_str() );
    footNode[1] = (dynamics::BodyNodeDynamics*) mWorld->getRobot(mHuboId)->getNode( mRightLegEEName.c_str() );

    legVals[0] = mWorld->getRobot( mHuboId )->getDofs(mLeftLegLinks);
    legVals[1] = mWorld->getRobot( mHuboId )->getDofs(mRightLegLinks);
	
    Eigen::MatrixXd lTransform;
    Eigen::VectorXd lXYZ(3),lXYZ2(3),dlXYZ(3);
    Eigen::VectorXi torsoLinks(3);
    Eigen::VectorXd torsoVals(3);
    torsoLinks << 0,1,2;

    int stanceLeg = 0;
    int swingLeg = 1;
    swingLegLinks = mRightLegLinks;
    stanceLegLinks = mLeftLegLinks;

    int numSteps = 6;

    for(int n=0; n<numSteps; n++){
        // forwardstepstand is half forwardstepwalk, so
        // last step is half length
        if(n == 0)              // first step: half length to get stride started
            forwardStep = forwardStepStand;
        else if(n == numSteps - 1) // last step: half length to end stride
            forwardStep = forwardStepStand;
        else                    // middle stpes; full length
            forwardStep = forwardStepWalk;

        // swing's major axis is 0 to .34
        for(double i=0.0; i<0.34; i+=stepSize){

            // swinging leg - move up and slightly forward
            legVals[swingLeg][2] -= halfStep;
            legVals[swingLeg][3] += stepSize;
            legVals[swingLeg][4] -= halfStep;

            legVals[swingLeg][2] -= forwardStep;
            legVals[swingLeg][4] += forwardStep;

            // update swing leg
            mWorld->getRobot( mHuboId )->setDofs(legVals[swingLeg], swingLegLinks );
            mWorld->getRobot(mHuboId)->update();

            // figure out stance foot's position in world coords
            lTransform = footNode[stanceLeg]->getWorldTransform();
            lXYZ << lTransform(0,3), lTransform(1,3), lTransform(2,3);

            // stance leg - move back slightly
            torsoVals = mWorld->getRobot( mHuboId )->getDofs(torsoLinks);
            legVals[stanceLeg][1] -= swayStep;
            legVals[stanceLeg][5] += swayStep;

            legVals[stanceLeg][2] += forwardStep;
            legVals[stanceLeg][4] -= forwardStep;

            // update stance leg
            mWorld->getRobot( mHuboId )->setDofs(legVals[stanceLeg], stanceLegLinks );
            mWorld->getRobot(mHuboId)->update();

            // stance foot's initial position in world
            lTransform = footNode[stanceLeg]->getWorldTransform();
            lXYZ2 << lTransform(0,3), lTransform(1,3), lTransform(2,3);

            // negative of stance foot's movement in world
            dlXYZ = lXYZ-lXYZ2;

            // move the entire robot so the stance foot remains stationary
            mWorld->getRobot( mHuboId )->setDofs(torsoVals+dlXYZ,torsoLinks);
            mWorld->getRobot(mHuboId)->update();

            // update camera and proceed
            viewer->UpdateCamera();
            frame->AddWorld( mWorld );
        }
        for(double i=0.0; i<0.34; i+=stepSize){
            // swinging leg - move forward and down
            legVals[swingLeg][2] += halfStep;
            legVals[swingLeg][3] -= stepSize;
            legVals[swingLeg][4] += halfStep;

            legVals[swingLeg][2] -= forwardStep;
            legVals[swingLeg][4] += forwardStep;

            // update swing leg
            mWorld->getRobot( mHuboId )->setDofs(legVals[swingLeg], swingLegLinks );
            mWorld->getRobot(mHuboId)->update();

            // stance foot's initial position in world
            lTransform = footNode[stanceLeg]->getWorldTransform();
            lXYZ << lTransform(0,3), lTransform(1,3), lTransform(2,3);

            // stance leg - move backward slightly
            torsoVals = mWorld->getRobot( mHuboId )->getDofs(torsoLinks);
            legVals[stanceLeg][1] += swayStep;
            legVals[stanceLeg][5] -= swayStep;

            legVals[stanceLeg][2] += forwardStep;
            legVals[stanceLeg][4] -= forwardStep;

            // update stance foo
            mWorld->getRobot( mHuboId )->setDofs(legVals[stanceLeg], stanceLegLinks );
            mWorld->getRobot(mHuboId)->update();

            // get stance foot's new position in world
            lTransform = footNode[stanceLeg]->getWorldTransform();
            lXYZ2 << lTransform(0,3), lTransform(1,3), lTransform(2,3);

            // negative of stance foot's movement in world
            dlXYZ = lXYZ-lXYZ2;

            // move robot so stance foot remains stationary
            mWorld->getRobot( mHuboId )->setDofs(torsoVals+dlXYZ,torsoLinks);
            mWorld->getRobot(mHuboId)->update();

            // update camera and proceed
            viewer->UpdateCamera();
            frame->AddWorld( mWorld );
        }
		
        // switch legs
        if(stanceLeg == 0){ 
            stanceLeg = 1;
            swingLeg = 0;
            swayStep = -swayStep;
            swingLegLinks = mLeftLegLinks;
            stanceLegLinks = mRightLegLinks;
        } else {
            stanceLeg = 0;
            swingLeg = 1;
            swayStep = -swayStep;
            swingLegLinks = mRightLegLinks;
            stanceLegLinks = mLeftLegLinks;
        }
    }
}


/////////////////////////////////////////////////////////////////////////////////////////////
//#########################################################################################//
//# hubowalk object for walking during other tasks                                        #//
//#########################################################################################//
/////////////////////////////////////////////////////////////////////////////////////////////

HuboWalk::HuboWalk(double startTime, double dT, int numTimeSteps, int numWalkSteps)
{
    mTimeInit = startTime;
    mdT = dT;
    mNumTimeSteps = numTimeSteps;
    mNumWalkSteps = numWalkSteps;
    
    mBaseIncrement = 0.34/mNumTimeSteps;
    mHalfIncrement = mBaseIncrement / 2.0;
    mSwayIncrement = mBaseIncrement / 4.0;
    
    mForwardStepIncrement;
    mForwardStepWalkIncrement = mBaseIncrement/2.5;
    mForwardStepStandIncrement = mBaseIncrement/5.0;
}

void DoNextWalkFrame()
{
}

void DoWalkFrame(double t)
{
}

void DoWalkFrame(int timestep)
{
}
