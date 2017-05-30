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
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "parser.h"
#include "typedefs.h"

#define AIRWAY_SIZE ((double) 0.2) // in radians

static void count(parser_data* data, FILE* file);
static void parse(parser_data* data, FILE* file);
static identity_t find(unsigned long value, unsigned long* array, unsigned long size);

void load(parser_data* data, const char* file_path) {
    FILE* file = fopen(file_path, "r");
    count(data, file);
    data->drones   = (parser_drone_type*)   malloc(data->drone_nbr   * sizeof(parser_drone_type));
    data->clients  = (parser_client_type*)  malloc(data->client_nbr  * sizeof(parser_client_type));
    data->hunters  = (parser_hunter_type*)  malloc(data->hunter_nbr  * sizeof(parser_hunter_type));
    data->packages = (parser_package_type*) malloc(data->package_nbr * sizeof(parser_package_type));
    parse(data, file);
    fclose(file);
}

void unload(parser_data* data) {
    data->drone_nbr = 0;
    free(data->drones);
    data->drones = NULL;

    data->client_nbr = 0;
    free(data->clients);
    data->clients = NULL;

    data->hunter_nbr = 0;
    free(data->hunters);
    data->hunters = NULL;

    data->package_nbr = 0;
    free(data->packages);
    data->packages = NULL;
}


void count(parser_data* data, FILE* file) {
    data->drone_nbr = data->package_nbr = data->client_nbr = data->hunter_nbr = 0;
    fseek(file, 0, SEEK_SET);
    size_t len;
    char* line = NULL;
    while(getline(&line, &len, file) != -1) {
        char* val = strtok(line,",;");
        if (strcmp(val, "drone") == 0) {
            ++data->drone_nbr;
        } else if (strcmp(val, "colis") == 0) {
            ++data->package_nbr;
        } else if (strcmp(val, "client") == 0) {
            ++data->client_nbr;
        } else if (strcmp(val, "chasseur") == 0) {
            ++data->hunter_nbr;
        }

        free(line);
        line = NULL;
        len = 0;
    }
    free(line);
}

void parse(parser_data* data, FILE* file) {
    fseek(file, 0, SEEK_SET);
    size_t len;
    char* line = NULL;
    data->drone_nbr = data->package_nbr = data->client_nbr = data->hunter_nbr = 0;
    while(getline(&line, &len, file) != -1) {
        char* val = strtok(line,",;");
        if (strcmp(val, "drone") == 0) {
            data->drones[data->drone_nbr].power_capacity = strtod(strtok(NULL, ",;"), NULL);
            data->drones[data->drone_nbr].trunk.weight_capacity = strtod(strtok(NULL,",;"), NULL);
            data->drones[data->drone_nbr].trunk.volume_capacity = strtod(strtok(NULL,",;"), NULL);
            ++data->drone_nbr;
        } else if (strcmp(val, "colis") == 0) {
            data->packages[data->package_nbr].weight = strtod(strtok(NULL, ",;"), NULL);
            data->packages[data->package_nbr].volume = strtod(strtok(NULL, ",;"), NULL);
            data->packages[data->package_nbr].target = strtoul(strtok(NULL, ",;"), NULL, 10);
            data->packages[data->package_nbr].priority = strtoul(strtok(NULL, ",;"), NULL, 10);
            ++data->package_nbr;
        } else if (strcmp(val, "client") == 0) {
            data->clients[data->client_nbr].coord[parser_x] = strtod(strtok(NULL,",;"), NULL);
            data->clients[data->client_nbr].coord[parser_y] = strtod(strtok(NULL,",;"), NULL);
            data->clients[data->client_nbr].id = strtoul(strtok(NULL,",;"), NULL, 10);
            ++data->client_nbr;
        } else if (strcmp(val, "chasseur") == 0) {
            data->hunters[data->hunter_nbr].ammo = strtoul(strtok(NULL,",;"), NULL, 10);
            data->hunters[data->hunter_nbr].reload_time = strtoul(strtok(NULL,",;"), NULL, 10);
            ++data->hunter_nbr;
        } else if (strcmp(val, "vaisseau") == 0) {
            data->mothership.coord[parser_x] = strtod(strtok(NULL,",;"), NULL);
            data->mothership.coord[parser_y] = strtod(strtok(NULL,",;"), NULL);
            data->mothership.package_throughput = strtoul(strtok(NULL,",;"), NULL, 10);
            data->mothership.reloader_throughput = strtod(strtok(NULL,",;"), NULL);
            data->mothership.reloader_nbr = strtoul(strtok(NULL,",;"), NULL, 10);
        }

        free(line);
        line = NULL;
        len = 0;
    }
    free(line);
}

void load_simulation_data(parser_data* input, sim_data* output) {
    // Loading the Mother Ship
    output->mothership.package_throughput = input->mothership.package_throughput;
    output->mothership.power_loading_slots = input->mothership.reloader_nbr;
    output->mothership.power_throughput = input->mothership.reloader_throughput;
    output->mothership.client_nbr = input->client_nbr;
    output->mothership.package_nbr = input->package_nbr;

    identity_t* clients_ids_map = malloc(output->mothership.client_nbr * sizeof(identity_t));

    output->mothership.clients  = malloc(output->mothership.client_nbr  * sizeof(client_t));
    for (uint_fast32_t i = output->mothership.client_nbr ; i-- ;) {
        double rx = input->clients[i].coord[parser_x] - input->mothership.coord[parser_x]; // relative x
        double ry = input->clients[i].coord[parser_y] - input->mothership.coord[parser_y]; // relative y

        output->mothership.clients[i].id = i;
        output->mothership.clients[i].mothership_distance = sqrt(rx * rx + ry * ry);
        output->mothership.clients[i].airway = (airway_t) (atan2(ry, rx) / AIRWAY_SIZE);

        clients_ids_map[i] = input->clients[i].id;
    }

    output->mothership.packages = malloc(output->mothership.package_nbr * sizeof(package_t));
    for (uint_fast32_t i = output->mothership.package_nbr ; i-- ;) {
        output->mothership.packages[i].priority = input->packages[i].priority;
        output->mothership.packages[i].weight = input->packages[i].weight;
        output->mothership.packages[i].volume = input->packages[i].volume;
        output->mothership.packages[i].client_id = find(input->packages[i].target, clients_ids_map, output->mothership.client_nbr);
    }

    free(clients_ids_map);

    // Loading Drones
    output->drone_nbr = input->drone_nbr;
    output->drones = malloc(output->drone_nbr * sizeof(drone_t));
    for (uint_fast32_t i = output->drone_nbr ; i-- ;) {
        output->drones[i].id = i;
        output->drones[i].max_fuel = input->drones[i].power_capacity;
        output->drones[i].fuel = output->drones[i].max_fuel;
        output->drones[i].max_package_weight = input->drones[i].trunk.weight_capacity;
        output->drones[i].max_package_volume = input->drones[i].trunk.volume_capacity;
        output->drones[i].package = NULL;
        output->drones[i].client_distance = 0;
        output->drones[i].mothership_distance = 0;
    }

    // Loading Hunters
    output->hunter_nbr = input->hunter_nbr;
    output->hunters = malloc(output->hunter_nbr * sizeof(hunter_t));
    for (uint_fast32_t i = output->hunter_nbr ; i-- ;) {
        output->hunters[i].ammo = input->hunters[i].ammo;
        output->hunters[i].shoot_interval = input->hunters[i].reload_time;
    }
}

void unload_simulation_data(sim_data* data) {
    data->mothership.client_nbr = 0;
    free(data->mothership.clients);
    data->mothership.clients = NULL;

    data->mothership.package_nbr = 0;
    free(data->mothership.packages);
    data->mothership.packages = NULL;

    data->drone_nbr = 0;
    free(data->drones);
    data->drones = NULL;

    data->hunter_nbr = 0;
    free(data->hunters);
    data->hunters = NULL;
}

identity_t find(unsigned long value, unsigned long* array, unsigned long size) {
    for (unsigned long i = size ; i-- ;) {
        if (array[i] == value) {
            return i;
        }
    }
    printf("Client %lu doesn't exist! Aborting...\n", value);
    abort();
}

void load_simulation(sim_data* data, const char* file_path) {
    parser_data pdata;
    load(&pdata, file_path);
    load_simulation_data(&pdata, data);
    unload(&pdata);
}

void unload_simulation(sim_data* data) {
    unload_simulation_data(data);
}

void print_parsed_data(parser_data* data) {
    printf("Mother Ship :\n\tx : %.2lf\n\ty : %.2lf\n\tPackage throughput : %lu\n\tReloader throughput : %.2lf\n\t%lu reloaders.\n\n",
            data->mothership.coord[parser_x], data->mothership.coord[parser_y], data->mothership.package_throughput, data->mothership.reloader_throughput, data->mothership.reloader_nbr);

    printf("%d Drones.", data->drone_nbr);
    for (unsigned long int i = 0 ; i < data->drone_nbr ; ++i) {
        printf("\n\tDrone #%ld :\n\t\tPower trunk : %.2lf\n\t\tVolume capa : %.2lf\n\t\tWeight capa : %.2lf", i,
            data->drones[i].power_capacity, data->drones[i].trunk.volume_capacity, data->drones[i].trunk.weight_capacity);
    }

    printf("\n\n%d Packages.", data->package_nbr);
    for (unsigned long int i = 0 ; i < data->package_nbr ; ++i) {
        printf("\n\tPackage #%ld :\n\t\tTarget :   %ld\n\t\tWeight :   %.2lf\n\t\tVolume :   %.2lf\n\t\tPriority : %ld", i,
            data->packages[i].target, data->packages[i].weight, data->packages[i].volume, data->packages[i].priority);
    }

    printf("\n\n%d Clients.", data->client_nbr);
    for (unsigned long int i = 0 ; i < data->client_nbr ; ++i) {
        printf("\n\tClient #%ld :\n\t\tID : %ld\n\t\tx :  %.2lf\n\t\ty :  %.2lf", i,
            data->clients[i].id, data->clients[i].coord[parser_x], data->clients[i].coord[parser_y]);
    }

    printf("\n\n%d Hunters.", data->hunter_nbr);
    for (unsigned long int i = 0 ; i < data->hunter_nbr ; ++i) {
        printf("\n\tHunter #%ld :\n\t\tAmmo :        %ld\n\t\tReload time : %ld", i,
            data->hunters[i].ammo, data->hunters[i].reload_time);
    }

    printf("\n\n");
}

void print_simulation_data(sim_data* data) {
    printf("\n\n\033[33mMother Ship :\n\t%lu packages can be loaded at once\n\t%lu power loading slots\n\t%.2lf power throughput per slot\n\t%u clients\n\t%u packages",
            data->mothership.package_throughput,
            data->mothership.power_loading_slots,
            data->mothership.power_throughput,
            data->mothership.client_nbr,
            data->mothership.package_nbr);
    printf("\n\n\t\033[31mClients :");
    for (unsigned long int i = 0 ; i < data->mothership.client_nbr ; ++i) {
        printf("\n\t\tClient #%lu :\n\t\t\tID :                        %lu\n\t\t\tDistance from Mother Ship : %.2lf\n\t\t\tAirway :                    %ld",
                i,
                data->mothership.clients[i].id,
                data->mothership.clients[i].mothership_distance,
                data->mothership.clients[i].airway);
    }

    printf("\n\n\t\033[32mPackages :");
    for (unsigned long int i = 0 ; i < data->mothership.package_nbr ; ++i) {
        printf("\n\t\tPackage #%lu :\n\t\t\tPriority :        %lu\n\t\t\tWeight :          %.2lf\n\t\t\tVolume :          %.2lf\n\t\t\tTarget :  client #%lu",
                i,
                data->mothership.packages[i].priority,
                data->mothership.packages[i].weight,
                data->mothership.packages[i].volume,
                data->mothership.packages[i].client_id);
    }

    printf("\n\n\033[0m-------------------------\n\033[34mDrones :");
    for (unsigned long int i = 0 ; i < data->drone_nbr ; i++) {
        printf("\n\tDrone #%lu :\n\t\tFuel :                    %.2lf/%.2lf\n\t\tWeight max :              %.2lf\n\t\tVolume max :              %.2lf\n\t\tDistance to client :      %.2lf\n\t\tDistance to Mother Ship : %.2lf\n\t\t%s",
                i,
                data->drones[i].fuel,
                data->drones[i].max_fuel,
                data->drones[i].max_package_weight,
                data->drones[i].max_package_volume,
                data->drones[i].client_distance,
                data->drones[i].mothership_distance,
                data->drones[i].package != NULL ? "Has a package" : "Hasn't got any package");
    }


    printf("\n\n\033[0m-------------------------\n\033[35mHunters :");
    for (unsigned long int i = 0 ; i < data->hunter_nbr ; ++i) {
        printf("\n\tHunter #%ld :\n\t\tAmmo :        %ld\n\t\tReload time : %ld", i,
            data->hunters[i].ammo, data->hunters[i].shoot_interval);
    }

    printf("\n\n\033[0m");
    fflush(stdout);
}
