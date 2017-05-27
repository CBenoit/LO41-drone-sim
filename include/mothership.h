///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
// Copyright 2017 TiWinDeTea.                                                                    //
// This file is part of LO41 Drone Sim project which is released under the                               //
// European Union Public License v1.1. If a copy of the EUPL was                                 //
// not distributed with this software, you can obtain one at :                                   //
// https://joinup.ec.europa.eu/community/eupl/og_page/european-union-public-licence-eupl-v11     //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef DRONE_SIM_MOTHERSHIP
#define DRONE_SIM_MOTHERSHIP

#include "typedefs.h"

typedef struct {
    unsigned int nb_package_loading_slots;
    unsigned int nb_power_loading_slots;
    power_unit_t power_unit_per_tick;
    addressee_t* addressees;
} mothership_t;

#endif /* ifndef DRONE_SIM_MOTHERSHIP */


