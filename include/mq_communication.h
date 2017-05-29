///////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                               //
// Copyright 2017 TiWinDeTea.                                                                    //
// This file is part of LO41 Drone Sim project which is released under the                       //
// European Union Public License v1.1. If a copy of the EUPL was                                 //
// not distributed with this software, you can obtain one at :                                   //
// https://joinup.ec.europa.eu/community/eupl/og_page/european-union-public-licence-eupl-v11     //
//                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef DRONE_SIM_MQ_COMMUNICATION
#define DRONE_SIM_MQ_COMMUNICATION

#include <sys/types.h>
#include <stdint.h>

#include "typedefs.h"

#define MSG_CONTENT_SIZE_MAX 256

enum {
    MOTHERSHIP_MSG,
    DRONE_MSG,
    HUNTER_MSG
};

typedef struct {
    long type;
    pid_t pid;
    mq_msg_id_t msg_id;
    uint8_t content[MSG_CONTENT_SIZE_MAX];
} message_t;

message_t create_empty_message(pid_t dest, mq_msg_id_t msg_id);
message_t create_initialized_message(pid_t dest, mq_msg_id_t msg_id, uint8_t content[], size_t content_size);

#endif /* ifndef DRONE_SIM_MOTHERSHIP_DRONE_COMMUNICATION */

