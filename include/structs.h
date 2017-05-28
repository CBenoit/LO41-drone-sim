///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
// Copyright 2017 TiWinDeTea.                                                                    //
// This file is part of LO41 Drone Sim project which is released under the                       //
// European Union Public License v1.1. If a copy of the EUPL was                                 //
// not distributed with this software, you can obtain one at :                                   //
// https://joinup.ec.europa.eu/community/eupl/og_page/european-union-public-licence-eupl-v11     //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef DRONE_SIM_STRUCTS
#define DRONE_SIM_STRUCTS

#include "typedefs.h"

typedef struct {
    ammo_t ammo;
    ticks_t shoot_interval;
} hunter_t;

typedef struct {
    client_id_t client_id;
    distance_t mothership_distance; // distance to the mothership
    airway_t airway;
} client_t;

typedef struct {
    priority_t priority;
    weigth_t weight;
    volume_t volume;
    client_id_t client_id;
} package_t;

typedef struct {
    power_t fuel;
    power_t max_fuel;
    weigth_t max_package_weight;
    volume_t max_package_volume;
    package_t* package;
    distance_t client_distance;
    distance_t mothership_distance;
    // client_distance + mothership_distance should be a constant sum at runtime.
} drone_t;

typedef struct {
    unsigned long int package_throughput; // number of packages per tick
    unsigned long int power_loading_slots;  //
    power_t power_throughput;
    unsigned int client_nbr;
    client_t* clients;
    unsigned int package_nbr;
    package_t* packages;
} mothership_t;

#endif /* ifndef DRONE_SIM_STRUCTS */
