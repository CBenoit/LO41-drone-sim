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
    printf("\n== Simulation statistics ==\n");
    printf("\tInitial number of drones:\t"FYELLOW""BOLD"%u"RESET"\n", stats.initial_nb_drone);
    printf("\t"FGREEN"Number of drones powered off:\t"FYELLOW""BOLD"%u"RESET"\n", stats.nb_powered_off_drone);
    printf("\t"FLRED"Number of drones lost:\t\t"FLYELLOW""BOLD"%u"RESET"\n",
            stats.initial_nb_drone - stats.nb_powered_off_drone);

    printf("\n\tInitial number of packages:\t\t\t"FLYELLOW""BOLD"%u"RESET"\n", stats.initial_nb_package);
    printf("\t"FGREEN"Number of delivered packages:\t\t\t"FLYELLOW""BOLD"%u"RESET"\n", stats.nb_delivered_package);
    printf("\t"FYELLOW"Number of packages still in the mothership:\t"FLYELLOW""BOLD"%u"RESET"\n", stats.nb_package_still_in_mothership);
    printf("\t"FLRED"Number of packages lost:\t\t\t"FLYELLOW""BOLD"%u"RESET"\n",
            stats.initial_nb_package - stats.nb_package_still_in_mothership - stats.nb_delivered_package);
}

