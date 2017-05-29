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
#include <unistd.h>
#include <stdio.h>
#include <sys/msg.h>

#include "utility.h"
#include "drone.h"
#include "mq_communication.h"

void drone_main(drone_t me, int* clients_pipes, unsigned int number_of_clients, int my_pipes[2], int msqid) {
    // Waiting for the Mother Ship to be ready
    wait_mothership_signal();

    for (;;) {
        // 1 tick

        printf("drone tick\n");

        printf("drone: sending message to mothership.\n");
        message_t message = create_empty_message(getppid(), DRONE_MSG);
        if (msgsnd(msqid, &message, sizeof(message_t), IPC_NOWAIT) == -1) {
            perror("msgsnd");
            abort();
        }

        wait_mothership_signal();
    }

    close_pipes(number_of_clients, clients_pipes);
    close(my_pipes[1]);
    free(clients_pipes);
}

