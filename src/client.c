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

#include "client.h"
#include "utility.h"

void client_main(int read_filedesc, int* drones_pipes, unsigned int number_of_drones) {
    // Waiting for the Mother Ship to be ready
    pause();

    // TODO

    close_pipes(number_of_drones, drones_pipes);
    free(drones_pipes);
}

