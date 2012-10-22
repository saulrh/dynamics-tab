/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

#ifndef DYNAMIC_SIMULATION_TAB
#define DYNAMIC_SIMULATION_TAB

#include <Tabs/GRIPTab.h>
#include <Tabs/GRIPThread.h>

#include <robotics/Robot.h>
#include <robotics/Object.h>
#include <dynamics/BodyNodeDynamics.h>
#include <robotics/World.h>
#include <integration/EulerIntegrator.h>
#include <integration/RK4Integrator.h>

#include "WorldIntegrator.hpp"

#include <iostream>

#define SIMULATION_TIMESTEP 0.005

/**
 * @class DynamicSimulationTab
 * @brief Uses DART's dynamic simulation capabilities
 */
class DynamicSimulationTab : public GRIPTab
{
public:
    std::vector< WorldState* > mSimHistory;
    std::vector< WorldState* > mSavedStates;

    // sizer for whole tab
    wxBoxSizer* sizerFull;

    // public vars to capture external selection stuff 
    robotics::Object* mSelectedObject;
    robotics::Robot* mSelectedRobot;
    dynamics::BodyNodeDynamics* mSelectedNode;

    /// Functions
    DynamicSimulationTab(){};
    DynamicSimulationTab( wxWindow * parent, wxWindowID id = -1,
                          const wxPoint & pos = wxDefaultPosition,
                          const wxSize & size = wxDefaultSize,
                          long style = wxTAB_TRAVERSAL);
    virtual ~DynamicSimulationTab(){}

    void SimulateFrame(double dt=SIMULATION_TIMESTEP);
    void OnSlider(wxCommandEvent &evt);
    void OnButton(wxCommandEvent &evt);
    void OnListBox(wxCommandEvent &evt);
    void OnCheckBox(wxCommandEvent &evt);
    void OnTimer(wxTimerEvent &evt);
    void PopulateTimeline();
    void UpdateListBox();
    void GRIPStateChange();

    // wx events
    DECLARE_DYNAMIC_CLASS( DynamicSimulationTab )
    DECLARE_EVENT_TABLE()
    
private:
    integration::EulerIntegrator mEuIntegrator;
    integration::RK4Integrator mRK4Integrator;

    wxListBox* mStateListBox;
    int mListBoxSelectedState;
    
    wxCheckBox* mStaticSkeletonCheckbox;
    wxTimer* mSimTimer;
    
    WorldState* mCurrentSimState;
};

#endif /** DYNAMIC_SIMULATION_TAB */
