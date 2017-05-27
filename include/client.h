///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
// Copyright 2017 TiWinDeTea.                                                                    //
// This file is part of LO41 Drone Sim project which is released under the                       //
// European Union Public License v1.1. If a copy of the EUPL was                                 //
// not distributed with this software, you can obtain one at :                                   //
// https://joinup.ec.europa.eu/community/eupl/og_page/european-union-public-licence-eupl-v11     //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef DRONE_SIM_CLIENT
#define DRONE_SIM_CLIENT

#include "typedefs.h"

typedef struct {
    client_id_t client_id;
    distance_t mothership_distance; // distance to the mothership
    airway_t airway;
} client_t;

#endif /* ifndef DRONE_SIM_CLIENT */

