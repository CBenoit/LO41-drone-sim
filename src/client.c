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
#include <time.h>
#include <string.h>

#include "utility.h"
#include "client.h"

void client_main(int pipes[2]) {
    sigset_t mask;
    sigaddset(&mask, MOTHERSHIP_SIGNAL);
    sigprocmask(SIG_BLOCK, &mask, NULL);
    srand((unsigned int)time(NULL));

    // I am ready
    sem_post(mother_sem);

    // Waiting for the Mother Ship to be ready
    wait_mothership_signal();

    char msg[256];
    forever {
        if (read(pipes[0], msg, 256) != -1) {
            double volume = strtod(msg, NULL);
            dprintf(pipes[1], "%lu", (unsigned long) (volume / 10. + 1 + rand() % 4));
        }

        sem_post(mother_sem);
        wait_mothership_signal();
    }
}

