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

#include "hunter.h"
#include "utility.h"

void hunter_main(hunter_t me) {
    // Waiting for the Mother Ship to be ready
    wait_mothership_signal();

    for (;;) {
        // 1 tick

        printf("hunter tick\n");
        // TODO

        wait_mothership_signal();
    }
}

