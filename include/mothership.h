///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
// Copyright 2017 TiWinDeTea.                                                                    //
// This file is part of LO41 Drone Sim project which is released under the                       //
// European Union Public License v1.1. If a copy of the EUPL was                                 //
// not distributed with this software, you can obtain one at :                                   //
// https://joinup.ec.europa.eu/community/eupl/og_page/european-union-public-licence-eupl-v11     //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef DRONE_SIM_MOTHERSHIP
#define DRONE_SIM_MOTHERSHIP

#include "typedefs.h"
#include "client.h"
#include "package.h"

typedef struct {
    unsigned long int package_throughput; // number of packages per tick
    unsigned long int power_loading_slots;  //
    power_t power_throughput;
    unsigned int client_nbr;
    client_t* clients;
    unsigned int package_nbr;
    package_t* packages;
} mothership_t;

#endif /* ifndef DRONE_SIM_MOTHERSHIP */

