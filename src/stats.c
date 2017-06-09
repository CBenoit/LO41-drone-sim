///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
// Copyright 2017 TiWinDeTea.                                                                    //
// This file is part of LO41 Drone Sim project which is released under the                       //
// European Union Public License v1.1. If a copy of the EUPL was                                 //
// not distributed with this software, you can obtain one at :                                   //
// https://joinup.ec.europa.eu/community/eupl/og_page/european-union-public-licence-eupl-v11     //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>

#include "stats.h"
#include "colors.h"

void print_stats(sim_stats stats) {
    printf("\n== Simulation statistics ==\n\n");
    printf("\t"FGREEN"Number of drones powered off:    "FYELLOW""BOLD"%u/%u"RESET"\n",
            stats.initial_nb_drone - stats.crashed_drones - stats.shot_drones, stats.initial_nb_drone);
    printf("\t"FLYELLOW"Number of drones shot:           "FLYELLOW""BOLD"%u/%u"RESET"\n", stats.shot_drones, stats.initial_nb_drone);
    printf("\t"FLRED"Number of crashed drones:        "FLYELLOW""BOLD"%u/%u"RESET"\n", stats.crashed_drones, stats.initial_nb_drone);

    printf("\n\t"FGREEN"Number of delivered packages:                   "FLYELLOW""BOLD"%u/%u"RESET"\n", stats.nb_delivered_package, stats.initial_nb_package);
    printf("\t"FLYELLOW"Number of packages still in the mothership:     "FLYELLOW""BOLD"%u/%u"RESET"\n", stats.nb_package_still_in_mothership, stats.initial_nb_package);
    printf("\t"FLRED"Number of packages lost:                        "FLYELLOW""BOLD"%u/%u"RESET"\n",
            stats.initial_nb_package - stats.nb_package_still_in_mothership - stats.nb_delivered_package, stats.initial_nb_package);

    printf("\nConsummed %.2lf power units over %lu ticks.\n", stats.power_consumption, stats.tick_count);
}

