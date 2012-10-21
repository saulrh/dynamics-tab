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
}

/**
 * @function WorldState
 * @brief creates a world state from the given world. Note that it
 * assumes that the world is not moving!
 */
WorldState::WorldState(robotics::World* w)
{
    readFromWorld(w);
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
}

/**
 * @function WorldState
 * @brief copy constructor
 */
WorldState::WorldState(WorldState& other)
{
    mPosVects.resize(other.mPosVects.size());
    mVelVects.resize(other.mVelVects.size());
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
    mPosVects.resize(w->getNumSkeletons());
    mVelVects.resize(w->getNumSkeletons());

    for(int s = 0; s < w->getNumSkeletons(); s++ ) {
        w->getSkeleton(s)->getPose(mPosVects[s]);
        mVelVects[s] = Eigen::VectorXd::Zero(mPosVects[s].size());
    }
}

/**
 * @function readFromWorld
 * @brief Copies this state into the given world. Note that velocities
 * are ignored in this function - the world can't handle them.
 */
void WorldState::writeToWorld(robotics::World* w)
{
    for(int s = 0; s < w->getNumSkeletons(); s++) {
        // std::cout << "DEBUG: setting pose of skel " << s << " to" << std::endl;
        // std::cout << "       ";
        // for(unsigned int i = 0; i < mPosVects[s].size(); i++)
        //     std::cout << mPosVects[s][i] << " ";
        // std::cout << std::endl;
        w->getSkeleton(s)->setPose(mPosVects[s]);
    }
    for(int r = 0; r < w->getNumRobots(); r++)
        w->getRobot(r)->update();
    for(int o = 0; o < w->getNumObjects(); o++)
        w->getObject(o)->update();
}

////////////////////////////////////////////////////////////////
// utility functions

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
WorldIntegrator::WorldIntegrator(robotics::World* w, double timeStep)
{
    mWorld = w;
    mTimeStep = timeStep;
    mWorldState = new WorldState();
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
    Eigen::VectorXd result;
    mWorldState->writeToVector(result);
    return result;
}

////////////////////////////////////////////////////////////////
// set state


void WorldIntegrator::setState(Eigen::VectorXd state)
{
    mWorldState->readFromVector(mWorld, state);
}

////////////////////////////////////////////////////////////////
// eval derivative

Eigen::VectorXd WorldIntegrator::evalDeriv()
{
    // update the model
    mWorldState->writeToWorld(mWorld);
    
    // calulate contact forces
    mWorld->mCollisionHandle->applyContactForces();

    Eigen::VectorXd deriv = Eigen::VectorXd::Zero(WorldState::getNumberOfDoFs(mWorld) * 2);
    int currentIndex;

    // for each skeleton, calculate how that skeleton is going to try to behave
    for(int i = 0; i < mWorld->getNumSkeletons(); i++)
    {
        dynamics::SkeletonDynamics* skel = mWorld->getSkeleton(i);
        if(skel->getImmobileState())
        {
            currentIndex += skel->getNumDofs() * 2;
        }
        else
        {
            Eigen::VectorXd qddot =
                skel->getInvMassMatrix() *
                (skel->getExternalForces()
                 + mWorld->mCollisionHandle->getConstraintForce(i)
                 - skel->getCombinedVector());
            skel->clampRotation(mWorldState->mPosVects[i], mWorldState->mVelVects[i]);
            deriv.segment(currentIndex, skel->getNumDofs()) = mWorldState->mVelVects[i] + (qddot * mTimeStep);
            currentIndex += skel->getNumDofs();
            deriv.segment(currentIndex, skel->getNumDofs()) = qddot;
            currentIndex += skel->getNumDofs();
        }
    }
    
    // update the time counter
    mWorldState->mT += mTimeStep;

    // and finally return the result
    return deriv;
}
