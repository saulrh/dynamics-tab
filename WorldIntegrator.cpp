/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

#include "WorldIntegrator.hpp"
#include <robotics/Robot.h>
#include <robotics/Object.h>
#include <dynamics/BodyNodeDynamics.h>
#include <robotics/World.h>
#include <iostream>
#include <iomanip>
#include <dynamics/SkeletonDynamics.h>
#include <integration/EulerIntegrator.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <Tabs/AllTabs.h>
#include <GRIPApp.h>

/////////////////////////////////////////////////////////////////////////////////////////////
//#########################################################################################//
//# world state                                                                           #//
//#########################################################################################//
/////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////
// constructors

/**
 * @function WorldState
 * @brief default constructor
 */
WorldState::WorldState()
{
    mPosVects.resize(0);
    mVelVects.resize(0);
    mT = 0;
}

/**
 * @function WorldState
 * @brief creates a world state from the given world. Note that it
 * assumes that the world is not moving!
 */
WorldState::WorldState(robotics::World* w)
{
    readFromWorld(w);
    mT = 0;
    mId = getUID();
}

/**
 * @function WorldState
 * @brief constructs a world state from a state that's been
 * serialized/compacted/whatever into a VectorXd for simulation
 * purposes.
 */
WorldState::WorldState(robotics::World* w, Eigen::VectorXd& serState)
{
    readFromVector(w, serState);
    mT = 0;
    mId = getUID();
}

/**
 * @function WorldState
 * @brief copy constructor
 */
WorldState::WorldState(WorldState& other)
{
    mPosVects.resize(other.mPosVects.size());
    mVelVects.resize(other.mVelVects.size());
    for(unsigned int i = 0; i < mPosVects.size(); i++)
        mPosVects[i] = Eigen::VectorXd(other.mPosVects[i]);
    for(unsigned int i = 0; i < mVelVects.size(); i++)
        mVelVects[i] = Eigen::VectorXd(other.mVelVects[i]);
    mT = other.mT;
    mId = getUID();
}

/**
 * @function WorldState
 * @brief copy constructor
 */
WorldState::WorldState(WorldState* other)
{
    mPosVects.resize(other->mPosVects.size());
    mVelVects.resize(other->mVelVects.size());
    for(unsigned int i = 0; i < mPosVects.size(); i++)
        mPosVects[i] = Eigen::VectorXd(other->mPosVects[i]);
    for(unsigned int i = 0; i < mVelVects.size(); i++)
        mVelVects[i] = Eigen::VectorXd(other->mVelVects[i]);
    mT = other->mT;
    mId = getUID();
}

/**
 * @function ~WorldState
 * @brief destructor
 */
WorldState::~WorldState()
{
}

////////////////////////////////////////////////////////////////
// converters

/**
 * @function readFromVector

 * @brief takes a serialized/compacted/whatever state in VectorXd and
 * unpacks and copies it into this worldstate. Uses the given world to
 * figure out how long everything should be.
 */
void WorldState::readFromVector(robotics::World* w, Eigen::VectorXd& serState)
{
    // std::cout << "vect to state" << std::endl;
    // printVectToStdout(serState);

    int currentIndex = 0;
    mPosVects.resize(w->getNumSkeletons());
    mVelVects.resize(w->getNumSkeletons());
    for(int s = 0; s < w->getNumSkeletons(); s++)
    {
        for (int d = 0; d < w->getSkeleton(s)->getNumDofs(); d++)
        {
            mPosVects[s][d] = serState[currentIndex++];
            mVelVects[s][d] = serState[currentIndex++];
        }
    }
}

/**
 * @function writeToVector
 * @brief takes this world state and compacts/serializes/whatevers it
 * into the given VectorXd for simulation.
 */
void WorldState::writeToVector(Eigen::VectorXd& serState)
{
    // std::cout << "state to vect" << std::endl;
    // printToStdout();

    int currentIndex;

    int nDofs = 0;
    for(unsigned int i = 0; i < mPosVects.size(); i++) nDofs += mPosVects[i].size();
    for(unsigned int i = 0; i < mVelVects.size(); i++) nDofs += mVelVects[i].size();
    serState.resize(nDofs);

    currentIndex = 0;
    for(unsigned int i = 0; i < mPosVects.size(); i++)
        for(unsigned int j = 0; j < mPosVects[i].size(); j++)
        {
            serState[currentIndex] = mPosVects[i][j];
            currentIndex += 2;
        }
    currentIndex = 1;
    for(unsigned int i = 0; i < mVelVects.size(); i++)
        for(unsigned int j = 0; j < mVelVects[i].size(); j++)
        {
            serState[currentIndex] = mVelVects[i][j];
            currentIndex += 2;
        }
}

/**
 * @function readFromWorld
 * @brief Copies the state out of the given world. Note that
 * velocities are assumed to be zero! Don't clobbery anything.
 */
void WorldState::readFromWorld(robotics::World* w)
{
    // std::cout << "world to state" << std::endl;
    // printWorldToStdout(w);
    
    mPosVects.resize(w->getNumSkeletons());
    mVelVects.resize(w->getNumSkeletons());

    for(int s = 0; s < w->getNumSkeletons(); s++ ) {
        w->getSkeleton(s)->getPose(mPosVects[s]);
        mVelVects[s] = Eigen::VectorXd::Zero(mPosVects[s].size());
    }

    // printToStdout();
}

/**
 * @function readFromWorld
 * @brief Copies this state into the given world. Note that velocities
 * are ignored in this function - the world can't handle them.
 */
void WorldState::writeToWorld(robotics::World* w)
{
    // std::cout << "state to world" << std::endl;
    // printToStdout();

    for(int s = 0; s < w->getNumSkeletons(); s++) {
        // std::cout << "DEBUG: setting pose of skel " << s << " to" << std::endl;
        // std::cout << "       ";
        // for(unsigned int i = 0; i < mPosVects[s].size(); i++)
        //     std::cout << mPosVects[s][i] << " ";
        // std::cout << std::endl;
        w->getSkeleton(s)->setPose(mPosVects[s], true, true);
    }
    for(int r = 0; r < w->getNumRobots(); r++)
        w->getRobot(r)->update();
    for(int o = 0; o < w->getNumObjects(); o++)
        w->getObject(o)->update();

    // printWorldToStdout(w);
}

////////////////////////////////////////////////////////////////
// utility functions

void WorldState::printToStdout()
{
    std::cout << "  state" << std::endl;
    for(unsigned int s = 0; s < mPosVects.size(); s++)
    {
        std::cout << "    skel " << s << std::endl << "      ";
        for (unsigned int d = 0; d < mPosVects[s].size(); d++)
            std::cout << std::fixed << std::setw(7) << setprecision(3) << mPosVects[s][d];
        std::cout << std::endl << "      ";
        for (unsigned int d = 0; d < mVelVects[s].size(); d++)
            std::cout << std::fixed << std::setw(7) << setprecision(3)  << mVelVects[s][d];
        std::cout << std::endl;
    }
}

void WorldState::printVectToStdout(Eigen::VectorXd& v)
{
    std::cout << "  vect" << std::endl << "    ";
    for(unsigned int i = 0; i < v.size(); i++)
        std::cout << setiosflags(ios::fixed) << std::setw(7) << setprecision(3) << v[i] << " ";
    std::cout << std::endl;
}

void WorldState::printWorldToStdout(robotics::World* w)
{
    std::cout << "  world" << std::endl;
    for(int s = 0; s < w->getNumSkeletons(); s++)
    {
        std::cout << "    skel " << s << std::endl << "      ";
        Eigen::VectorXd pose;
        w->getSkeleton(s)->getPose(pose);
        for (unsigned int d = 0; d < pose.size(); d++)
            std::cout << std::fixed << std::setw(7) << setprecision(3) << pose[d];
        std::cout << std::endl;
    }
}


/**
 * @function getNumberOfDoFs
 * @brief Figures out how many dofs there are in the given world
 */
int WorldState::getNumberOfDoFs(robotics::World* w)
{
    int result = 0;
    for(int i = 0; i < w->getNumSkeletons(); ++i ) {
        result += w->getSkeleton(i)->getNumDofs();
    }
    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//#########################################################################################//
//# world integrator                                                                      #//
//#########################################################################################//
/////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// constructors

/**
 * @function WorldIntegrator
 * @brief Constructor
 */
WorldIntegrator::WorldIntegrator()
{
    mWorld = NULL;
    mTimeStep = 0.01;
    mWorldState = NULL;
}

/**
 * @function WorldIntegrator
 * @brief Constructor with args. Sets mTimeStep and remembers w and state.
 */
WorldIntegrator::WorldIntegrator(double timeStep, robotics::World* w, Eigen::Vector3d gravityVector)
{
    mWorld = w;
    mTimeStep = timeStep;
    mWorldState = new WorldState();
    mGravity = gravityVector;
}

/**
 * @function ~WorldIntegrator
 * @brief Destructor. Doesn't delete anything! Worlds are persistent,
 * and you're probably using this with a list of vectors somewhere
 */
WorldIntegrator::~WorldIntegrator()
{
}

////////////////////////////////////////////////////////////////
// get state

Eigen::VectorXd WorldIntegrator::getState()
{
    Eigen::VectorXd state;
    mWorldState->writeToVector(state);
    // std::cout << "DEBUG: outputting state" << std::endl;
    // std::cout << state << std::endl;
    return state;
}

////////////////////////////////////////////////////////////////
// set state


void WorldIntegrator::setState(Eigen::VectorXd state)
{
    // std::cout << "DEBUG: inputting state" << std::endl;
    // std::cout << state << std::endl;
    mWorldState->readFromVector(mWorld, state);
}

////////////////////////////////////////////////////////////////
// eval derivative

Eigen::VectorXd WorldIntegrator::evalDeriv()
{
    // update the model
    mWorldState->writeToWorld(mWorld);
    // std::cout << "DEBUG: calculate contact forces" << std::endl;

    // calulate contact forces
    mWorld->mCollisionHandle->applyContactForces();

    Eigen::VectorXd deriv = Eigen::VectorXd::Zero(WorldState::getNumberOfDoFs(mWorld) * 2);
    int currentIndex = 0;

    // std::cout << "DEBUG: start processing skeletons" << std::endl;

    // for each skeleton, calculate how that skeleton is going to try to behave
    for(int i = 0; i < mWorld->getNumSkeletons(); i++)
    {
        dynamics::SkeletonDynamics* skel = mWorld->getSkeleton(i);
        
        if(skel->getImmobileState())
        {
            // std::cout << "DEBUG: immobile object" << std::endl;
            currentIndex += skel->getNumDofs() * 2;
        }
        else
        {
            // std::cout << "DEBUG: compute dynamics" << std::endl;
            skel->computeDynamics(mGravity, mWorldState->mVelVects[i], true);
            
            // std::cout << "DEBUG: compute qddot" << std::endl;
            Eigen::MatrixXd invMassMatrix = skel->getInvMassMatrix();
            // std::cout << "DEBUG: inv mass matrix has "
            //           << invMassMatrix.rows()
            //           << " rows and "
            //           << invMassMatrix.cols()
            //           << " cols"
            //           << std::endl;
            Eigen::MatrixXd forces = (-skel->getCombinedVector()
                                    + skel->getExternalForces()
                                    + mWorld->mCollisionHandle->getConstraintForce(i));
            // std::cout << "DEBUG: forces matrix  has "
            //           << forces.rows()
            //           << " rows and "
            //           << forces.cols()
            //           << " cols"
            //           << std::endl;

            // std::cout << "DEBUG: compute qddot" << std::endl;
            Eigen::VectorXd qddot = invMassMatrix * forces;

            // std::cout << "DEBUG: clamp rotations to [-pi, pi]" << std::endl;
            skel->clampRotation(mWorldState->mPosVects[i], mWorldState->mVelVects[i]);

            // std::cout << "DEBUG: set velocity component of derivative" << std::endl;
            Eigen::VectorXd velUpdate = mWorldState->mVelVects[i] + (qddot * mTimeStep);
            // std::cout << "       ";
            // for(unsigned int i = 0; i < velUpdate.size(); i++)
            //     std::cout << velUpdate[i] << " ";
            // std::cout << std::endl;
            // std::cout << "       "
            //           << currentIndex
            //           << skel->getNumDofs()
            //           << velUpdate.size()
            //           << std::endl;
            deriv.segment(currentIndex, skel->getNumDofs()) = velUpdate;
            currentIndex += skel->getNumDofs();

            // std::cout << "DEBUG: set acceleration component of derivative" << std::endl;
            // std::cout << "       ";
            // for(unsigned int i = 0; i < qddot.size(); i++)
            //     std::cout << qddot[i] << " ";
            // std::cout << std::endl;
            // std::cout << "       "
            //           << currentIndex
            //           << skel->getNumDofs()
            //           << qddot.size()
            //           << std::endl;
            deriv.segment(currentIndex, skel->getNumDofs()) = qddot;
            currentIndex += skel->getNumDofs();
            // std::cout << "DEBUG: velupdate" << std::endl;
            // std::cout << velUpdate << std::endl;
            // std::cout << "DEBUG: qddot" << std::endl;
            // std::cout << qddot << std::endl;
        }
        // std::cout << "DEBUG: derivative" << std::endl;
        // std::cout << deriv << std::endl;
    }
    
    // std::cout << "DEBUG: done processing skeletons" << std::endl;

    // std::cout << "DEBUG: update state's time" << std::endl;

    // update the time counter
    mWorldState->mT += mTimeStep;

    // and finally return the result
    return deriv;
}
