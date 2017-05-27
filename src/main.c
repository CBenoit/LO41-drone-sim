///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
// Copyright 2017 TiWinDeTea.                                                                    //
// This file is part of LO41 Drone Sim project which is released under the                       //
// European Union Public License v1.1. If a copy of the EUPL was                                 //
// not distributed with this software, you can obtain one at :                                   //
// https://joinup.ec.europa.eu/community/eupl/og_page/european-union-public-licence-eupl-v11     //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <stdio.h>

#include <parser.h>

int main(int argc, char *argv[]) {
    parser_data data;
    load(&data, fopen("test.csv", "r"));

    printf("Mother Ship :\n\tx : %lf\n\ty : %lf\n\tPackage throughput : %lu\n\tReloader throughput : %lf\n\t%lu reloaders.\n\n",
            data.mothership.coord[parser_x], data.mothership.coord[parser_y], data.mothership.package_throughput, data.mothership.reloader_throughput, data.mothership.reloader_nbr);

    printf("%d Drones.", data.drone_nbr);
    for (unsigned long int i = 0 ; i < data.drone_nbr ; ++i) {
        printf("\n\tDrone #%ld :\n\t\tPower trunk : %lf\n\t\tVolume capa : %lf\n\t\tWeight capa : %lf", i,
            data.drones[i].power_capacity, data.drones[i].trunk.volume_capacity, data.drones[i].trunk.weight_capacity);
    }

    printf("\n\n%d Packages.", data.package_nbr);
    for (unsigned long int i = 0 ; i < data.package_nbr ; ++i) {
        printf("\n\tPackage #%ld :\n\t\tTarget :   %ld\n\t\tWeight :   %lf\n\t\tVolume :   %lf\n\t\tPriority : %ld", i,
            data.packages[i].target, data.packages[i].weight, data.packages[i].volume, data.packages[i].priority);
    }

    printf("\n\n%d Clients.", data.client_nbr);
    for (unsigned long int i = 0 ; i < data.client_nbr ; ++i) {
        printf("\n\tClient #%ld :\n\t\tID : %ld\n\t\tx :  %lf\n\t\ty :  %lf", i,
            data.clients[i].id, data.clients[i].coord[parser_x], data.clients[i].coord[parser_y]);
    }

    printf("\n\n%d Hunters.", data.hunter_nbr);
    for (unsigned long int i = 0 ; i < data.hunter_nbr ; ++i) {
        printf("\n\tHunter #%ld :\n\t\tAmmo :       %ld\n\t\tReload time : %ld", i,
            data.hunters[i].ammo, data.hunters[i].reload_time);
    }

    printf("\n\n");

    unload(&data);

    return EXIT_SUCCESS;
}

