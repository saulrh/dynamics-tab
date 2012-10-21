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

/**
 * @function WorldState
 * @brief default constructor
 */
WorldState::WorldState()
{
    mPoss.resize(0);
    mVels.resize(0);
}

/**
 * @function WorldState
 * @brief creates a world state from the given world. Note that it
 * assumes that the world is not moving!
 */
WorldState::WorldState(robotics::World* w)
{
    mPoss.resize(getNumberOfDoFs(w));
    mVels.resize(getNumberOfDoFs(w));
}

/**
 * @function WorldState
 * @brief constructs a world state from a state that's been
 * serialized/compacted/whatever into a VectorXd for simulation
 * purposes.
 */
WorldState::WorldState(Eigen::VectorXd& serState)
{
    readFromVector(serState);
}

/**
 * @function WorldState
 * @brief copy constructor
 */
WorldState::WorldState(WorldState& other)
{
    mPoss = Eigen::VectorXd(other.mPoss);
    mVels = Eigen::VectorXd(other.mVels);
}


/**
 * @function readFromVector
 * @brief takes a serialized/compacted/whatever state in VectorXd and
 * unpacks and copies it into this worldstate
 */
void WorldState::readFromVector(Eigen::VectorXd& serState)
{
    mPoss.resize(serState.size() / 2);
    mVels.resize(serState.size() / 2);
    for(int i = 0; i < mPoss.size(); i++)
    {
        mPoss[i] = serState[2*i    ];
        mVels[i] = serState[2*i + 1];
    }
}

/**
 * @function writeToVector
 * @brief takes this world state and compacts/serializes/whatevers it
 * into the given VectorXd for simulation.
 */
void WorldState::writeToVector(Eigen::VectorXd& serState)
{
    serState.resize(mPoss.size() + mVels.size());
    for(int i = 0; i < mPoss.size(); i++)
    {
        serState[2*i    ] = mPoss[i];
        serState[2*i + 1] = mVels[i];
    }
}

/**
 * @function readFromWorld
 * @brief Copies the state out of the given world. Note that
 * velocities are assumed to be zero! Don't clobbery anything.
 */
void WorldState::readFromWorld(robotics::World* w)
{
    int nDofs = WorldState::getNumberOfDoFs(w);
    mPoss.resize(nDofs);
    mVels.resize(nDofs);
    int currentIndex;
    for(int s = 0; s < w->getNumSkeletons(); s++ ) {
        Eigen::VectorXd skelDofs;
        w->getSkeleton(s)->getPose(skelDofs);
        
        for(int dof = 0; dof < skelDofs.size(); dof++) {
            mPoss[currentIndex] = skelDofs[dof];
            mVels[currentIndex] = 0.0;
            currentIndex++;
        }
    }
}

/**
 * @function readFromWorld
 * @brief Copies this state into the given world. Note that velocities
 * are ignored in this function - the world can't handle them.
 */
void WorldState::writeToWorld(robotics::World* w)
{
    int curStateIndex = 0;
    for(int s = 0; s < w->getNumSkeletons(); s++ ) {
        Eigen::VectorXd skelState;
        kinematics::Skeleton* skel = w->getSkeleton(s);
        int nDofs = skel->getNumDofs();
        skelState.resize(nDofs);
        for(int dof = 0; dof < nDofs; dof++)
            skelState[dof] = mPoss[curStateIndex++];
        skel->setPose(skelState);
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

Eigen::VectorXd WorldIntegrator::getState()
{
    Eigen::VectorXd result;
    mWorldState->writeToVector(result);
    return result;
}

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
            currentIndex += skel->getNumDofs();
        }
        else
        {
            Eigen::VectorXd qddot =
                skel->getInvMassMatrix() *
                (skel->getExternalForces()
                 + mWorld->mCollisionHandle->getConstraintForce(i)
                 - skel->getCombinedVector());
            // skel->clampRotation(
        }
    }
}

void WorldIntegrator::setState(Eigen::VectorXd state)
{
    mWorldState->readFromVector(state);
}


// void DynamicSimulationTab::EvaluateSimulationDerivative()
// {
//     // apply contact forces for this frame to each thing in the world
//     mWorld->mCollisionChecker->applyContactForces();
    
//     VectorXd deriv = VectorXd::Zero(ComputeNumDofsInWorld(mWorld) * 2);
//     int currentIndex;
    
//     for (unsigned int i = 0; i < mWorld->mRobots->size(); i++)
//     {
//         if(mWorld->mRobots->getImmobileState())
//             continue;
//         int numIndices = mRobots[i]->getNumQuickDofs();
//         VectorXd qddot = mRobots[i]->getInvMassMatrix() *
//             (-mRobots[i]->getCombinedVector()
//              + mRobots[i]->getExternalForces()
//              + mCollisionHandle->getConstraintForce(i));
//     }
//     for (unsigned int i = 0; i < mWorld.mObjects.size(); i++)
//     {
//     }
    
// }
