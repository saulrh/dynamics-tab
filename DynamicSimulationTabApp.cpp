/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

#include "GRIPApp.h"
#include "DynamicSimulationTab.hpp"

extern wxNotebook* tabView;

/**
 * @class RipPlannerTabApp
 */
class RipPlannerTabApp : public GRIPApp {
    virtual void AddTabs() {
        tabView->AddPage(new DynamicSimulationTab(tabView), wxT("Dynamic Simulation"));
    }
};

IMPLEMENT_APP(RipPlannerTabApp)
