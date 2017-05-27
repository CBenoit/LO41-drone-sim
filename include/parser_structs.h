///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
// Copyright 2017 TiWinDeTea.                                                                    //
// This file is part of LO41 Drone Sim project which is released under the                       //
// European Union Public License v1.1. If a copy of the EUPL was                                 //
// not distributed with this software, you can obtain one at :                                   //
// https://joinup.ec.europa.eu/community/eupl/og_page/european-union-public-licence-eupl-v11     //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef DRONE_SIM_PARSER_STRUCTS
#define DRONE_SIM_PARSER_STRUCTS

#include "typedefs.h"

enum {
    parser_x = 0,
    parser_y = 1
};

typedef struct {
    double weight_capacity;
    double volume_capacity;
} parser_trunk_type;

typedef struct {
    double power_capacity;
    parser_trunk_type trunk;
} parser_drone_type;

typedef struct {
    double coord[2];
    long int id;
} parser_client_type;

typedef struct {
    unsigned long int ammo;
    unsigned long int reload_time;
} parser_hunter_type;

typedef struct {
    double   weight;
    double   volume;
    long int target;
    long int priority;
} parser_package_type;

typedef struct {
    double coord[2];
    unsigned long int package_throughput;
    double reloader_throughput;
    unsigned long int reloader_nbr;
} parser_mothership_type;

typedef struct {
    parser_mothership_type mothership;
    unsigned int drone_nbr;
    parser_drone_type* drones;
    unsigned int client_nbr;
    parser_client_type* clients;
    unsigned int hunter_nbr;
    parser_hunter_type* hunters;
    unsigned int package_nbr;
    parser_package_type* packages;
} parser_data;

#endif //DRONE_SIME_PARSER_STRUCTS

