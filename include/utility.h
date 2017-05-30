///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
// Copyright 2017 TiWinDeTea.                                                                    //
// This file is part of LO41 Drone Sim project which is released under the                       //
// European Union Public License v1.1. If a copy of the EUPL was                                 //
// not distributed with this software, you can obtain one at :                                   //
// https://joinup.ec.europa.eu/community/eupl/og_page/european-union-public-licence-eupl-v11     //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef DRONE_SIM_UTILITY
#define DRONE_SIM_UTILITY

#include <semaphore.h>
#include <signal.h>

#define MOTHER_SEM_NAME "/tiwindesem_mother"
#define forever for(;;)

enum /*signals*/ {
    MOTHERSHIP_SIGNAL = SIGUSR1
};

enum /*return values*/{
    GRACEFULLY_STOPPED,
    EXPLODED,
    UNEXPECTEDLY_STOPPED,
    DIED
};

sem_t* mother_sem;


int* open_pipes(unsigned int);

void close_pipes(unsigned int, int*);

void empty_handler(int sig);

void wait_mothership_signal(void);

#endif /* ifndef DRONE_SIM_UTILITY */

