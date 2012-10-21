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

#include <iostream>

/**
 * @class DynamicSimulationTab
 * @brief Uses DART's dynamic simulation capabilities
 */
class DynamicSimulationTab : public GRIPTab
{
public:
    std::vector< Eigen::VectorXd > mSimHistory;
    std::vector< int > mSavedStates;

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

    void SimulateFrame();
    void OnSlider(wxCommandEvent &evt);
    void OnRadio(wxCommandEvent &evt);
    void OnButton(wxCommandEvent &evt);
    void OnCheckBox(wxCommandEvent &evt);
    void OnListBox(wxCommandEvent &evt);
    void PopulateTimeline();
    void GRIPStateChange();

    // wx events
    DECLARE_DYNAMIC_CLASS( DynamicSimulationTab )
    DECLARE_EVENT_TABLE()
    
private:
    integration::Integrator* mIntegrator;
};

#endif /** DYNAMIC_SIMULATION_TAB */
