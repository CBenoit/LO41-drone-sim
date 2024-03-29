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
#include <stdlib.h> /* srand, rand */
#include <time.h>   /* time */

#include "colors.h"
#include "utility.h"
#include "hunter.h"
#include "shm_communication.h"

void hunter_main(hunter_t me) {
    sigset_t mask;
    sigaddset(&mask, MOTHERSHIP_SIGNAL);
    sigprocmask(SIG_BLOCK, &mask, NULL);

    ticks_t waiting_time = me.shoot_interval;

    srand((unsigned int) time(NULL));

    map_shared_memory();

    // I am ready
    sem_post(mother_sem);

    // Waiting for the Mother Ship to be ready
    wait_mothership_signal();

    unsigned long tick_count = 0;
    forever {
        if (waiting_time == 0) {
            size_t nb_flying_drones = get_number_of_flying_drones();
            if (nb_flying_drones > 0) {
                printf(FRED"tick %lu - Hunter "BOLD""FYELLOW"#H%d"RESET""FRED" shoots ... "RESET, tick_count, getpid());
                --me.ammo;
                if ((rand() % 100 + 1) <= me.accuracy) { // successful shot!
                    pid_t* flying_drones_p = get_flying_drones();
                    pid_t selected_pid = flying_drones_p[(size_t) rand() % nb_flying_drones];
                    printf(FRED"and hits a drone (pid %d)!"RESET"\n", selected_pid);
                    kill(selected_pid, SIGKILL);
                } else {
                    printf(FRED"but fails."RESET"\n");
                }

                if (me.ammo == 0) {
                    sem_post(mother_sem);
                    wait_mothership_signal();
                    printf(FRED"Hunter "BOLD""FYELLOW"#H%d"RESET""FRED" has no more ammo and goes back home."RESET"\n", getpid());
                    unmap_shared_memory();
                    sem_close(mother_sem);
                    exit(GO_HOME);
                }

                waiting_time = me.shoot_interval;
                printf("\n");
            }
        } else {
            --waiting_time;
        }

        sem_post(mother_sem);
        wait_mothership_signal();
        ++tick_count;
    }
}

