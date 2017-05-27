///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
// Copyright 2017 TiWinDeTea.                                                                    //
// This file is part of LO41 Drone Sim project which is released under the                       //
// European Union Public License v1.1. If a copy of the EUPL was                                 //
// not distributed with this software, you can obtain one at :                                   //
// https://joinup.ec.europa.eu/community/eupl/og_page/european-union-public-licence-eupl-v11     //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef DRONE_SIM_DRONE
#define DRONE_SIM_DRONE

#include "typedefs.h"

typedef struct {
    power_unit_t fuel;
    power_unit_t max_fuel;
    weigth_t max_load;
    package_t* package;
    distance_t client_distance;
    distance_t mothership_distance;
    // client_distance + mothership_distance should be a constant sum at runtime.
} drone_t;

#endif /* ifndef DRONE_SIM_DRONE */

