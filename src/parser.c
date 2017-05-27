#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "parser.h"

static void count(parser_data* data, FILE* file);
static void parse(parser_data* data, FILE* file);
static client_id_t find(unsigned long value, unsigned long* array, unsigned long size);

void load(parser_data* data, FILE* file) {
    count(data, file);
    data->drones   = (parser_drone_type*)   malloc(data->drone_nbr   * sizeof(parser_drone_type));
    data->clients  = (parser_client_type*)  malloc(data->client_nbr  * sizeof(parser_client_type));
    data->hunters  = (parser_hunter_type*)  malloc(data->hunter_nbr  * sizeof(parser_hunter_type));
    data->packages = (parser_package_type*) malloc(data->package_nbr * sizeof(parser_package_type));
    parse(data, file);
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

    client_id_t* clients_ids_map = malloc(output->mothership.client_nbr * sizeof(client_id_t));

    output->mothership.clients  = malloc(output->mothership.client_nbr  * sizeof(client_t));
    for (uint_fast32_t i = output->mothership.client_nbr ; i-- ;) {
        output->mothership.clients[i].client_id = i;
        output->mothership.clients[i].mothership_distance =
            sqrt(input->clients[i].coord[parser_x] * input->clients[i].coord[parser_x] + input->clients[i].coord[parser_y] * input->clients[i].coord[parser_y]);
        output->mothership.clients[i].airway = /* TODO */ -1;

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

client_id_t find(unsigned long value, unsigned long* array, unsigned long size) {
    for (unsigned long i = size ; i-- ;) {
        if (array[i] == value) {
            return i;
        }
    }
    printf("Client %lu doesn't exist! Aborting...\n", value);
    abort();
}
