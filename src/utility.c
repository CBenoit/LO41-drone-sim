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
#include <signal.h>

#include "utility.h"

int* open_pipes(unsigned int number_of_pipes) {
    int* pipes = (int*) malloc(2 * number_of_pipes * sizeof(int));
    for (unsigned int i = number_of_pipes ; i-- ;) {
        if (pipe(pipes + 2*i)) {
            printf ("Failed to create pipes. Aborting.\n");
            exit(EXIT_FAILURE);
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

void empty_handler(int sig) {
}

void wait_mothership_signal() {
    sigset_t mask;
    sigfillset(&mask);
    sigdelset(&mask, MOTHERSHIP_SIGNAL);
    signal(MOTHERSHIP_SIGNAL, &empty_handler);
    sigsuspend(&mask);
}

