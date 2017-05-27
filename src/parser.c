#include <stdlib.h>
#include <string.h>

#include "parser.h"

static void count(parser_data* data, FILE* file);
static void parse(parser_data* data, FILE* file);

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
    data->client_nbr = 0;
    free(data->clients);
    data->hunter_nbr = 0;
    free(data->hunters);
    data->package_nbr = 0;
    free(data->packages);
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
            data->packages[data->package_nbr].target = strtol(strtok(NULL, ",;"), NULL, 10);
            data->packages[data->package_nbr].priority = strtol(strtok(NULL, ",;"), NULL, 10);
            ++data->package_nbr;
        } else if (strcmp(val, "client") == 0) {
            data->clients[data->client_nbr].coord[parser_x] = strtod(strtok(NULL,",;"), NULL);
            data->clients[data->client_nbr].coord[parser_y] = strtod(strtok(NULL,",;"), NULL);
            data->clients[data->client_nbr].id = strtol(strtok(NULL,",;"), NULL, 10);
            ++data->client_nbr;
        } else if (strcmp(val, "chasseur") == 0) {
            data->hunters[data->hunter_nbr].ammo = strtoul(strtok(NULL,",;"), NULL, 10);
            data->hunters[data->hunter_nbr].reload_time = strtoul(strtok(NULL,",;"), NULL, 10);
            ++data->hunter_nbr;
        } else if (strcmp(val, "vaisseau") == 0) {
            data->mothership.coord[parser_x] = strtod(strtok(NULL,",;"), NULL);
            data->mothership.coord[parser_y] = strtod(strtok(NULL,",;"), NULL);
            data->mothership.package_throughput = strtod(strtok(NULL,",;"), NULL);
            data->mothership.reloader_throughput = strtod(strtok(NULL,",;"), NULL);
            data->mothership.reloader_nbr = strtoul(strtok(NULL,",;"), NULL, 10);
        }

        free(line);
        line = NULL;
        len = 0;
    }
}
