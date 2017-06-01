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
#include <fcntl.h>
#include <semaphore.h>
#include <sys/msg.h>

#include "parser.h"
#include "parser_structs.h"
#include "mothership.h"
#include "drone.h"
#include "client.h"
#include "hunter.h"
#include "utility.h"
#include "shm_communication.h"

void start(sim_data*);

int main(int argc, char *argv[]) {
    sim_data data;
    load_simulation(&data, "test.csv");
    printf("\n\n----- Simulation data recap -----\n\n");
    print_simulation_data(&data);
    printf("\n\n------ Starting simulation ------\n\n");
    start(&data);
    return EXIT_SUCCESS;
}

void start(sim_data* sdata) {
    initialize_shared_memory(sdata->drone_nbr);

    mother_sem = sem_open(MOTHER_SEM_NAME, O_CREAT | O_EXCL, 0644, 0);
    if (mother_sem == SEM_FAILED) {
        perror("Failed to create semaphore.");
        exit(EXIT_FAILURE);
    }

    // free once everyone closed the semaphore
    sem_unlink(MOTHER_SEM_NAME);

    // creating message queue
    int msqid = msgget(IPC_PRIVATE, IPC_CREAT | IPC_EXCL | 0600);
    if (msqid == -1) {
        sem_unlink(MOTHER_SEM_NAME);
        sem_close(mother_sem);
        sem_destroy(mother_sem);
        clean_shared_memory();
        perror("msgget");
        exit(EXIT_FAILURE);
    }

    // Creating hunters
    pid_t* hunters_p = (pid_t*) malloc(sdata->hunter_nbr * sizeof(pid_t));
    for (unsigned int i = sdata->hunter_nbr ; i-- ;) {
        hunters_p[i] = fork();
        if (hunters_p[i] == 0) {
            // I'm a hunter !
            free(hunters_p);
            hunter_t hunter = sdata->hunters[i];
            unload_simulation(sdata);
            hunter_main(hunter, msqid);
            exit(EXIT_SUCCESS);
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
            drone_main(sdata->drones[i], clients_pipes, sdata->mothership.client_nbr, this_drone_pipes, msqid, sdata);
            exit(EXIT_SUCCESS);
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

            fcntl(filedesc, F_SETFL, fcntl(filedesc, F_GETFL) | O_NONBLOCK);

            // Freeing unused memory
            free(clients_pipes);
            free(drones_p);
            free(clients_p);

            unload_simulation(sdata);

            // Client ! Go !
            client_main(filedesc, drones_pipes, sdata->drone_nbr);
            exit(EXIT_SUCCESS);
        }
    }

    close_pipes(sdata->mothership.client_nbr, clients_pipes);
    free(clients_pipes);
    close_pipes(sdata->drone_nbr, drones_pipes);
    free(drones_pipes);
    // mothership_main function should free all that.
    mothership_main(sdata, drones_p, clients_p, hunters_p, msqid);
}

