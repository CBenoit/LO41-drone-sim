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
#include <unistd.h>
#include <stdio.h>
#include <sys/msg.h>

#include "utility.h"
#include "drone.h"
#include "mq_communication.h"

void drone_main(drone_t me, int* clients_pipes, unsigned int number_of_clients, int my_pipes[2], int msqid) {
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
        printf("Drone: \t%s\n", val%2 ? "tic" : "tac");
        // TODO

        printf("drone: sending message to mothership.\n");
        message_t message = create_empty_message(getppid(), ASK_DEPARTURE_MSG);
        if (msgsnd(msqid, &message, sizeof(message_t), IPC_NOWAIT) == -1) {
            perror("msgsnd");
            abort();
        }

        sem_post(mother_sem);
        wait_mothership_signal();
    }

    close_pipes(number_of_clients, clients_pipes);
    close(my_pipes[1]);
    free(clients_pipes);
}

