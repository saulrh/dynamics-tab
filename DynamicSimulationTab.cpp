/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <Tabs/GRIPTab.h>
#include <Tabs/AllTabs.h>
#include <GRIPApp.h>

#include <iostream>
#include <iomanip>
#include <cstdio>

#include <robotics/World.h>
#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/BodyNode.h>

#include "DynamicSimulationTab.hpp"

/////////////////////////////////////////////////////////////////////////////////////////////
//#########################################################################################//
//# wx events stuff                                                                       #//
//#########################################################################################//
/////////////////////////////////////////////////////////////////////////////////////////////

// Control IDs (used for event handling - be sure to start with a non-conflicted id)
enum DynamicSimulationTabEvents {
    id_button_RunSim = 8345, // just to be safe
    id_button_RunFrame,
    id_button_StopSim,
    id_button_ReloadSimulation,
    id_button_SaveState,
    id_button_LoadState,
    id_button_LoadWorkingState,
    id_button_DeleteState,
    id_button_WriteHistory,
    id_button_InitDynamics,
    id_checkbox_SkeletonFixed,
    id_listbox_SavedStates,
    id_timer_Simulation
};

//Add a handler for any events that can be generated by the widgets you add here (sliders, radio, checkbox, etc)
BEGIN_EVENT_TABLE(DynamicSimulationTab, wxPanel)
EVT_BUTTON(id_button_RunSim, DynamicSimulationTab::OnButton)
EVT_BUTTON(id_button_RunFrame, DynamicSimulationTab::OnButton)
EVT_BUTTON(id_button_StopSim, DynamicSimulationTab::OnButton)
EVT_BUTTON(id_button_ReloadSimulation, DynamicSimulationTab::OnButton)
EVT_BUTTON(id_button_SaveState, DynamicSimulationTab::OnButton)
EVT_BUTTON(id_button_LoadState, DynamicSimulationTab::OnButton)
EVT_BUTTON(id_button_LoadWorkingState, DynamicSimulationTab::OnButton)
EVT_BUTTON(id_button_DeleteState, DynamicSimulationTab::OnButton)
EVT_BUTTON(id_button_WriteHistory, DynamicSimulationTab::OnButton)
EVT_BUTTON(id_button_InitDynamics, DynamicSimulationTab::OnButton)
EVT_CHECKBOX(id_checkbox_SkeletonFixed, DynamicSimulationTab::OnCheckBox)
EVT_COMMAND(wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, DynamicSimulationTab::OnSlider)
EVT_LISTBOX(id_listbox_SavedStates, DynamicSimulationTab::OnListBox)
EVT_TIMER(id_timer_Simulation, DynamicSimulationTab::OnTimer)
END_EVENT_TABLE()

// Class constructor for the tab: Each tab will be a subclass of RSTTab
IMPLEMENT_DYNAMIC_CLASS(DynamicSimulationTab, GRIPTab)

/////////////////////////////////////////////////////////////////////////////////////////////
//#########################################################################################//
//# DynamicSimulationTab constructor                                                      #//
//#########################################################################################//
/////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @function RipTabPlanner
 * @brief Constructor
 */
DynamicSimulationTab::DynamicSimulationTab( wxWindow *parent, const wxWindowID id,
                                            const wxPoint& pos, const wxSize& size, long style) :
GRIPTab(parent, id, pos, size, style)
{
    sizerFull = new wxBoxSizer( wxHORIZONTAL );

    wxStaticBox* tabBox = new wxStaticBox(this, -1, wxT("Dynamics"));
    wxStaticBoxSizer* tabBoxSizer = new wxStaticBoxSizer(tabBox, wxHORIZONTAL);
    wxBoxSizer* simulationControlSizer = new wxBoxSizer(wxVERTICAL);
    wxBoxSizer* stateSizer = new wxBoxSizer(wxHORIZONTAL);
    wxBoxSizer* stateControlSizer = new wxBoxSizer(wxVERTICAL);
    wxBoxSizer* simulationPropertySizer = new wxBoxSizer(wxVERTICAL);

    mSimTimer = new wxTimer(this, id_timer_Simulation);
    mCurrentSimState = NULL;

    mSavedStates.resize(0);
    mStateListBox = new wxListBox(this, id_listbox_SavedStates);
    mListBoxSelectedState = -1;

    // mStaticSkeletonCheckbox = new wxCheckBox(this, id_checkbox_SkeletonFixed, wxT("Static object"));

    // simulationPropertySizer->Add(mStaticSkeletonCheckbox,
    //                              0,     // do not resize to fit proportions vertically
    //                              wxALL, // border all around
    //                              1);    // border width is 1 so buttons are close together
    simulationPropertySizer->Add(new wxButton(this, id_button_InitDynamics, wxT("Init Dynamics")),
                                0,     // do not resize to fit proportions vertically
                                wxALL, // border all around
                                1);    // border width is 1 so buttons are close together
    
    simulationControlSizer->Add(new wxButton(this, id_button_RunSim, wxT("Toggle Simulation")),
                                0,     // do not resize to fit proportions vertically
                                wxALL, // border all around
                                1);    // border width is 1 so buttons are close together
    simulationControlSizer->Add(new wxButton(this, id_button_RunFrame, wxT("Simulate One Frame")),
                                0,     // do not resize to fit proportions vertically
                                wxALL, // border all around
                                1);    // border width is 1 so buttons are close together
    simulationControlSizer->Add(new wxButton(this, id_button_ReloadSimulation, wxT("Reload Sim")),
                                0,     // do not resize to fit proportions vertically
                                wxALL, // border all around
                                1);    // border width is 1 so buttons are close together
    simulationControlSizer->Add(new wxButton(this, id_button_WriteHistory, wxT("Save History")),
                                0,     // do not resize to fit proportions vertically
                                wxALL, // border all around
                                1);    // border width is 1 so buttons are close together

    stateControlSizer->Add(new wxButton(this, id_button_SaveState, wxT("Save State")),
                           0,     // do not resize to fit proportions vertically
                           wxALL, // border all around
                           1);    // border width is 1 so buttons are close together
    stateControlSizer->Add(new wxButton(this, id_button_LoadState, wxT("Load Selected State")),
                           0,     // do not resize to fit proportions vertically
                           wxALL, // border all around
                           1);    // border width is 1 so buttons are close together
    stateControlSizer->Add(new wxButton(this, id_button_LoadWorkingState, wxT("Load Working State")),
                           0,     // do not resize to fit proportions vertically
                           wxALL, // border all around
                           1);    // border width is 1 so buttons are close together
    stateControlSizer->Add(new wxButton(this, id_button_DeleteState, wxT("Delete Selected State")),
                           0,     // do not resize to fit proportions vertically
                           wxALL, // border all around
                           1);    // border width is 1 so buttons are close together
    
    stateSizer->Add(stateControlSizer,
                    1,                          // take up 1/4 of stateSizer
                    wxEXPAND | wxALIGN_CENTER,  // expand and center
                    0);                         // no border
    stateSizer->Add(mStateListBox,
                    3,                                   // take up 3/4 of stateSizer
                    wxEXPAND | wxALIGN_CENTER | wxALL,   // borders all over, expand to fit
                    1);                                  // 1-pixel border

    tabBoxSizer->Add(simulationPropertySizer,
                     1,         // take up 2/6 of the tab
                     wxEXPAND | wxALIGN_CENTER | wxALL,
                     1);
    tabBoxSizer->Add(simulationControlSizer,
                     1,         // take up 1/6 of the tab
                     wxEXPAND | wxALIGN_CENTER | wxALL,
                     1);
    tabBoxSizer->Add(stateSizer,
                     4,         // take up 3/6 of the tab
                     wxEXPAND | wxALIGN_CENTER | wxALL,
                     1);

    sizerFull->Add(tabBoxSizer, 1, wxEXPAND | wxALL, 6);

    SetSizer(sizerFull);
}

/////////////////////////////////////////////////////////////////////////////////////////////
//#########################################################################################//
//# DynamicSimulationTab event handlers                                                   #//
//#########################################################################################//
/////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////
// normal buttons
////////////////////////////////////////////////////////////////

/**
 * @function OnButton
 * @brief Handle Button Events
 */
void DynamicSimulationTab::OnButton(wxCommandEvent &evt) {
    int button_num = evt.GetId();
  
    switch (button_num)
    {
    case id_button_RunSim:         /** Start or stop Simulating */
        {
        if ( mWorld == NULL ) {
            std::cout << "(!) Must have a world loaded to simulate (!)" << std::endl;
            break;
        }
        std::cout << "(I) Simulating ten frames." << std::endl;
        for(int i = 0; i < 10; i++)
            SimulateFrame();
        std::cout << "(I) Simulated ten frames." << std::endl;
        
        // if (mSimTimer->IsRunning())
        // {
        //     std::cout << "(I) Stopping simulation." << std::endl;
        //     mSimTimer->Stop();
        // }
        // else
        // {
        //     std::cout << "(I) Starting simulation." << std::endl;
        //     // save the start
        //     if (mCurrentSimState == NULL)
        //         mCurrentSimState = new WorldState(mWorld);
        //     mSavedStates.push_back(mCurrentSimState);
        //     UpdateListBox();
        //     // start timer, milliseconds
        //     mTimerSetAt = clock();
        //     bool result = mSimTimer->Start(mWorld->mTimeStep * 5000, true);
        //     if (!result)
        //         std::cout << "(!) Could not start timer." << std::endl;
        // }
        break;
    }
    case id_button_RunFrame:         /** Simulate one step */
    {
        if ( mWorld == NULL ) {
            std::cout << "(!) Must have a world loaded to simulate (!)" << std::endl;
            break;
        }
        std::cout << "(I) Simulating one frame." << std::endl;
        SimulateFrame();
        std::cout << "(I) Simulated one frame." << std::endl;
        break;
    }
    case id_button_SaveState:         /** save the current state as a WorldState */
    {
        if ( mWorld == NULL ) {
            std::cout << "(!) Must have a world loaded to work with states (!)" << std::endl;
            break;
        }
        std::cout << "(I) Saving state" << std::endl;
        if (mCurrentSimState == NULL)
            mCurrentSimState = new WorldState(mWorld);
        mSavedStates.push_back(mCurrentSimState);
        UpdateListBox();
        std::cout << "(I) Saved state" << std::endl;
        break;
    }
    case id_button_LoadState:
    {
        if ( mWorld == NULL ) {
            std::cout << "(!) Must have a world loaded to work with states (!)" << std::endl;
            break;
        }
        if ( mListBoxSelectedState == -1 ) {
            std::cout << "(!) Must have a state selected to load a state (!)" << std::endl;
            break;
        }
        std::cout << "(I) Loading state" << std::endl;
        if (mCurrentSimState == NULL)
            mCurrentSimState = new WorldState(mWorld);
        mCurrentSimState = new WorldState(mSavedStates[mListBoxSelectedState]);
        mCurrentSimState->writeToWorld(mWorld);
        viewer->UpdateCamera();
        std::cout << "(I) Loaded state" << std::endl;
        break;
    }
    case id_button_DeleteState:
    {
        if ( mWorld == NULL ) {
            std::cout << "(!) Must have a world loaded to work with states (!)" << std::endl;
            break;
        }
        if ( mListBoxSelectedState == -1 ) {
            std::cout << "(!) Must have a state selected to delete a state (!)" << std::endl;
            break;
        }
        if (mSavedStates[mListBoxSelectedState] == mCurrentSimState)
        {
            std::cout << "(!) Sorry, pointer fail means that you can't delete that (!)" << std::endl;
        }
        std::cout << "(I) Deleting state" << std::endl;
        mSavedStates.erase(mSavedStates.begin() + mListBoxSelectedState);
        UpdateListBox();
        std::cout << "(I) Deleted state" << std::endl;
        break;
    }
    case id_button_ReloadSimulation:
    {
        if ( mWorld == NULL ) {
            std::cout << "(!) Must have a world loaded to reload (!)" << std::endl;
            break;
        }
        std::cout << "(I) Reloading simulation stuff" << std::endl;
        mCurrentSimState = new WorldState(mWorld);
        mSavedStates.resize(0);
        UpdateListBox();
        std::cout << "(I) Reloaded simulation" << std::endl;
        break;
    }
    case id_button_WriteHistory:
    {
        if ( mWorld == NULL ) {
            std::cout << "(!) Must have a world loaded to check collisions (!)" << std::endl;
            break;
        }
        PopulateTimeline();
        break;
    }
    case id_button_InitDynamics:
    {
        if ( mWorld == NULL ) {
            std::cout << "(!) Must have a world loaded to check collisions (!)" << std::endl;
            break;
        }
        mCurrentSimState = new WorldState(mWorld);
        mWorld->mTimeStep = .001;
        mWorld->mGravity = Eigen::Vector3d(0, 0, -9.8);
        for(int i = 0; i < mWorld->getNumSkeletons(); i++)
        {
            dynamics::SkeletonDynamics* skel = mWorld->getSkeleton(i);
            skel->initDynamics();
            skel->setPose(mCurrentSimState->mPosVects[i], true, true);
            skel->computeDynamics(mWorld->mGravity, mCurrentSimState->mVelVects[i], true);
        }
        mWorld->getSkeleton(0)->setImmobileState(true);
        // mWorld->getSkeleton(1)->setImmobileState(true);
        // mWorld->getSkeleton(2)->setImmobileState(true);
        // mWorld->getSkeleton(3)->setImmobileState(true);
        // mWorld->getSkeleton(4)->setImmobileState(true);
        mWorld->rebuildCollision();
    }
    }
}

////////////////////////////////////////////////////////////////
// listbox
////////////////////////////////////////////////////////////////


/**
 * @function UpdateListBox
 * @brief Updates the listbox to reflect the mSavedState list
 */
void DynamicSimulationTab::UpdateListBox()
{
    wxString listboxItems[mSavedStates.size()];
    for(unsigned int i = 0; i < mSavedStates.size(); i++)
    {
        wxString id;
        wxString time;
        id << mSavedStates[i]->mId;
        time << mSavedStates[i]->mT;
        listboxItems[i] = id + wxT(": T=") + time;
    }
    mStateListBox->Set(mSavedStates.size(), listboxItems);
}


/**
 * @function OnListBox
 * @brief Handle selection of listbox items
 */
void DynamicSimulationTab::OnListBox(wxCommandEvent &evt) {
    int listbox_num = evt.GetId();
    int item = evt.GetSelection();
    switch(listbox_num) {
    case id_listbox_SavedStates:
        mListBoxSelectedState = item;
        break;
    }
}

////////////////////////////////////////////////////////////////
// slider changed
////////////////////////////////////////////////////////////////

/**
 * @function OnSlider
 * @brief Handle slider changes
 */
void DynamicSimulationTab::OnSlider(wxCommandEvent &evt) {
    int slnum = evt.GetId();
    double pos = *(double*) evt.GetClientData();
  
    switch (slnum) {
    // case slider_Time:
    //     break;
    default:
        break;
    }

    // if (frame != NULL)
    //     frame->SetStatusText(wxString(numBuf, wxConvUTF8));
}

////////////////////////////////////////////////////////////////
// timer tick
////////////////////////////////////////////////////////////////

/**
 * @function OnTimer
 * @brief Handle timer ticks
 */
void DynamicSimulationTab::OnTimer(wxTimerEvent &evt) {
    SimulateFrame();

    bool cont = true;
    for(int i = 0; i < mCurrentSimState->mPosVects.size(); i++)
        if(std::isnan(mCurrentSimState->mPosVects[i][0]))
            cont = false;
    if (cont)
    {
        int timerFiredAt = clock();
        double timeTaken = ((double)(mTimerSetAt - timerFiredAt)) / CLOCKS_PER_SEC;
        int msToWait = timeTaken / 10000;
        std::cout << "waiting " << msToWait << " ms for next tick" << std::endl;
        mTimerSetAt = clock();
        mSimTimer->Start(msToWait, true);
    }
    else
        mSimTimer->Stop();
}


////////////////////////////////////////////////////////////////
// Checkbox toggled
////////////////////////////////////////////////////////////////

/**
 * @function OnCheckBox
 * @brief Handle CheckBox Events
 */ 
void DynamicSimulationTab::OnCheckBox( wxCommandEvent &evt ) {
  int checkbox_num = evt.GetId();
  bool checkbox_value = (bool)evt.GetSelection();

  switch (checkbox_num) {
  case id_checkbox_SkeletonFixed:
  {
        if ( mWorld == NULL ) {
            std::cout << "(!) Must have a world loaded (!)" << std::endl;
            break;
        }
        if (mSelectedObject != NULL) {
            // mSelectedObject->setImmobileState(checkbox_value);
        }
        else if (mSelectedRobot != NULL) {
            // mSelectedRobot->setImmobileState(checkbox_value);
        }
        else {
            std::cout << "(!) Must have a robot or object selected to set fixedness (!)" << std::endl;
            break;
        }
  }
  }
}
    
/////////////////////////////////////////////////////////////////////////////////////////////
//#########################################################################################//
//# helper functions                                                                      #//
//#########################################################################################//
/////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// simulate one tick forward
////////////////////////////////////////////////////////////////

void DynamicSimulationTab::SimulateFrame()
{
    std::cout << "Simulating frame" << std::endl;
    
    if (mCurrentSimState == NULL)
    {
        std::cout << "(!) must init dynamics before simulating (!)" << std::endl;
        return;
    }

    WorldIntegrator wi = WorldIntegrator(mWorld);
    mCurrentSimState = new WorldState(mCurrentSimState);

    wi.mWorldState = mCurrentSimState;
    mEuIntegrator.integrate(&wi, mWorld->mTimeStep);
    wi.mWorldState->writeToWorld(mWorld);
    viewer->UpdateCamera();

    mSimHistory.push_back(wi.mWorldState);

    std::cout << "Simulated frame T="
              << std::fixed
              << setprecision(3)
              << mCurrentSimState->mT
              << (mWorld->checkCollision() ? " Colliding" : "")
              << std::endl;
}

////////////////////////////////////////////////////////////////
// timeline population
////////////////////////////////////////////////////////////////

/**
 * @function PopulateTimeline
 * @brief Copies the simulation history into the timeline for easy review
 */
void DynamicSimulationTab::PopulateTimeline()
{
    std::cout << " (+) Populating Timeline. dt: " << mWorld->mTimeStep << " T: " << mSimHistory.back()->mT << std::endl;

    frame->InitTimer(string("Simulation_History"), mWorld->mTimeStep);

    for( std::vector<WorldState*>::iterator it = mSimHistory.begin(); it != mSimHistory.end(); it++)
    {
        // set each robot and object to the position recorded for that frame
        (*it)->writeToWorld(mWorld);
        
        
        
        // and record that world configuration
        viewer->UpdateCamera();
        frame->AddWorld( mWorld );
    }
}

////////////////////////////////////////////////////////////////
// grip selector state change
////////////////////////////////////////////////////////////////

/**
 * @function GRIPStateChange
 * @brief This function is called when an object is selected in the Tree View or other
 *        global changes to the RST world. Use this to capture events from outside the tab.
 */
void DynamicSimulationTab::GRIPStateChange() {
    if ( selectedTreeNode == NULL ) {
        return;
    }
  
    std::string statusBuf;
    std::string buf, buf2;
  
    switch (selectedTreeNode->dType) {
    
    case Return_Type_Object:
        mSelectedObject = (robotics::Object*) ( selectedTreeNode->data );
        mSelectedRobot = NULL;
        mSelectedNode = NULL;
        statusBuf = " Selected Object: " + mSelectedObject->getName();
        buf = "You clicked on object: " + mSelectedObject->getName();
        
        // mStaticSkeletonCheckbox->SetValue(mSelectedObject->getImmobileState());
        
        // Enter action for object select events here:
    
        break;
    case Return_Type_Robot:
        mSelectedObject = NULL;
        mSelectedRobot = (robotics::Robot*) ( selectedTreeNode->data );
        mSelectedNode = NULL;
        statusBuf = " Selected Robot: " + mSelectedRobot->getName();
        buf = " You clicked on robot: " + mSelectedRobot->getName();
    
        // mStaticSkeletonCheckbox->SetValue(mSelectedRobot->getImmobileState());

        // Enter action for Robot select events here:
    
        break;
    case Return_Type_Node:
        mSelectedObject = NULL;
        mSelectedRobot = NULL;
        mSelectedNode = (dynamics::BodyNodeDynamics*) ( selectedTreeNode->data );
        statusBuf = " Selected Body Node: " + string(mSelectedNode->getName()) + " of Robot: "
            + ( (robotics::Robot*) mSelectedNode->getSkel() )->getName();
        buf = " Node: " + std::string(mSelectedNode->getName()) + " of Robot: " + ( (robotics::Robot*) mSelectedNode->getSkel() )->getName();
    
        // Enter action for link select events here:
    
        break;
    default:
        fprintf(stderr, "--( :D ) Someone else's problem!\n");
        assert(0);
        exit(1);
    }
  
    //cout << buf << endl;
    frame->SetStatusText(wxString(statusBuf.c_str(), wxConvUTF8));
    sizerFull->Layout();
}
