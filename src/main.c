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
#include <unistd.h>

#include "parser.h"

void start(sim_data*);
void mothership_main(sim_data*, pid_t*, pid_t*, pid_t*);
void drone_main(drone_t, int*, unsigned int, int*);
void client_main(int, int*, unsigned int);
void hunter_main(hunter_t);

int* open_pipes(unsigned int);
void close_pipes(unsigned int, int*);

int* open_pipes(unsigned int number_of_pipes) {
    int* pipes = (int*) malloc(2 * number_of_pipes * sizeof(int));
    for (unsigned int i = number_of_pipes ; i-- ;) {
        if (pipe(pipes + 2*i)) {
            printf ("Failed to create pipes. Aborting.\n");
            abort();
        }
    }
    return pipes;
}

void close_pipes(unsigned int number_of_pipes, int* pipes) {
    while(number_of_pipes--) {
        close(pipes[number_of_pipes * 2 + 1]);
        close(pipes[number_of_pipes * 2]);
    }
}

int main(int argc, char *argv[]) {
    sim_data data;
    load_simulation(&data, "test.csv");
    printf("\n\n----- Simulation data recap -----\n\n");
    start(&data);
    unload_simulation(&data);
    return EXIT_SUCCESS;
}

void start(sim_data* sdata) {
    // Creating hunters
    pid_t* hunters_p = (pid_t*) malloc(sdata->hunter_nbr * sizeof(pid_t));
    for (unsigned int i = sdata->hunter_nbr ; i-- ;) {
        hunters_p[i] = fork();
        if (hunters_p[i] == 0) {
            // I'm a hunter !
            free(hunters_p);
            hunter_main(sdata->hunters[i]);
        }
    }

    // Creating pipes so that drones and clients can communicate
    int* clients_pipes = open_pipes(sdata->mothership.client_nbr);
    int* drones_pipes  = open_pipes(sdata->drone_nbr);
    
    // Creating drones
    pid_t* drones_p = (pid_t*) malloc(sdata->drone_nbr * sizeof(pid_t));
    for (unsigned int i = sdata->drone_nbr ; i-- ;) {
        drones_p[i] = fork();
        if (drones_p[i] == 0) {
            // I'm a drone !
            free(drones_p);
            close_pipes(i, drones_pipes);
            close_pipes(sdata->drone_nbr - i - 1, drones_pipes + 2*i + 2);
            close(drones_pipes[2*i]);
            int this_drone_pipes[2] = { drones_pipes[2*i], drones_pipes[2*i + 1] };
            free(drones_pipes);
            // drone_main(...) should free clients_pipes and close this_drone_pipes
            drone_main(sdata->drones[i], clients_pipes, sdata->mothership.client_nbr, this_drone_pipes);
            exit(0);
        }
    }

    // Creating clients
    pid_t* clients_p = (pid_t*) malloc(sdata->mothership.client_nbr * sizeof(pid_t));
    for (unsigned int i = sdata->mothership.client_nbr ; i-- ;) {
        clients_p[i] = fork();
        if (clients_p[i] == 0) {
            // I'm a client !
            // Cleaning unused pipes
            close_pipes(i, clients_pipes);
            close_pipes(sdata->mothership.client_nbr - i - 1, clients_pipes + 2*i + 2);
            close(clients_pipes[2*i + 1]);
            int filedesc = clients_pipes[2*i];

            // Freeing unused memory
            free(clients_pipes);
            free(drones_p);
            free(clients_p);

            // Client ! Go !
            client_main(filedesc, drones_pipes, sdata->drone_nbr);
            exit(0);
        }
    }

    close_pipes(sdata->mothership.client_nbr, clients_pipes);
    free(clients_pipes);
    close_pipes(sdata->drone_nbr, drones_pipes);
    free(drones_pipes);
    // mothership_main(...) should free all that.
    mothership_main(sdata, drones_p, clients_p, hunters_p);
}

void mothership_main(sim_data* sdata, pid_t* drones_p, pid_t* clients_p, pid_t* hunters_p) {
    free(drones_p);
    free(clients_p);
    free(hunters_p);
}

void drone_main(drone_t me, int* clients_pipes, unsigned int number_of_clients, int my_pipes[2]) {
    // Waiting for the Mother Ship to be ready
    pause();

    close_pipes(number_of_clients, clients_pipes);
    close(my_pipes[1]);
    free(clients_pipes);
}

void client_main(int read_filedesc, int* drones_pipes, unsigned int number_of_drones) {
    // Waiting for the Mother Ship to be ready
    pause();

    close_pipes(number_of_drones, drones_pipes);
    free(drones_pipes);
}

void hunter_main(hunter_t me) {
    // Waiting for the Mother Ship to be ready
    pause();
}
