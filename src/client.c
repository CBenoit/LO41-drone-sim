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
#include <unistd.h>
#include <signal.h>
#include <stdlib.h>

#include "utility.h"
#include "client.h"

void client_main(int read_filedesc, int* drones_pipes, unsigned int number_of_drones) {
    sigset_t mask;
    sigaddset(&mask, MOTHERSHIP_SIGNAL);
    sigprocmask(SIG_BLOCK, &mask, NULL);

    // I am ready
    sem_post(mother_sem);

    // Waiting for the Mother Ship to be ready
    wait_mothership_signal();

    int val = 0;
    for (;;) {
        // 1 tick

        ++val;
        printf("Client:\t%s\n", val%2 ? "tic" : "tac");
        // TODO

        sem_post(mother_sem);
        wait_mothership_signal();
    }


    close_pipes(number_of_drones, drones_pipes);
    free(drones_pipes);
}

