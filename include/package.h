///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
// Copyright 2017 TiWinDeTea.                                                                    //
// This file is part of LO41 Drone Sim project which is released under the                       //
// European Union Public License v1.1. If a copy of the EUPL was                                 //
// not distributed with this software, you can obtain one at :                                   //
// https://joinup.ec.europa.eu/community/eupl/og_page/european-union-public-licence-eupl-v11     //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef DRONE_SIM_PACKAGE
#define DRONE_SIM_PACKAGE

#include "typedefs.h"

typedef struct {
    priority_t priority;
    weigth_t weight;
    volume_t volume;
    client_id_t client_id;
} package_t;

#endif /* ifndef DRONE_SIM_PACKAGE */

