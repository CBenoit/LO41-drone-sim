///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
// Copyright 2017 TiWinDeTea.                                                                    //
// This file is part of LO41 Drone Sim project which is released under the                       //
// European Union Public License v1.1. If a copy of the EUPL was                                 //
// not distributed with this software, you can obtain one at :                                   //
// https://joinup.ec.europa.eu/community/eupl/og_page/european-union-public-licence-eupl-v11     //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef DRONE_SIM_STATS
#define DRONE_SIM_STATS

typedef struct {
    unsigned int initial_nb_drone;
    unsigned int nb_powered_off_drone;
    unsigned int initial_nb_package;
    unsigned int nb_delivered_package;
    unsigned int nb_package_still_in_mothership;
} sim_stats;

void print_stats(sim_stats stats);

#endif /* ifndef DRONE_SIM_STATS */
