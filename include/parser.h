///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
// Copyright 2017 TiWinDeTea.                                                                    //
// This file is part of LO41 Drone Sim project which is released under the                       //
// European Union Public License v1.1. If a copy of the EUPL was                                 //
// not distributed with this software, you can obtain one at :                                   //
// https://joinup.ec.europa.eu/community/eupl/og_page/european-union-public-licence-eupl-v11     //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef DRONE_SIM_PARSER
#define DRONE_SIM_PARSER

#include <stdio.h>
#include "parser_structs.h"

void load(parser_data* data, FILE* file);

void unload(parser_data* data);

#endif // DRONE_SIM_PARSER