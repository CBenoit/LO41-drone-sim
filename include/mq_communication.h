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

typedef enum {
    // mothership messages
    LOAD_PACKAGE_MSG,
    EXPLOSE_MSG,
    REFUEL_DRONE_MSG,
    DEPART_DRONE_MSG,
    POWER_OFF_MSG,

    // hunter messages
    SHOOT_DRONE_MSG,

    // drone messages
    ASK_DEPARTURE_MSG,
    ASK_PACKAGE_MSG,
    ASK_REFUEL_MSG,
    END_REFUEL_MSG,
    NOTIFY_ARRIVAL_MSG,

    // misc
    INVALID
} mq_msg_id_t;

typedef struct {
    long type;
    pid_t pid;
    mq_msg_id_t msg_id;
    union {
        int int_value;
        double double_value;
    };
} message_t;

message_t make_message(pid_t dest, mq_msg_id_t msg_id);
message_t make_int_message(pid_t dest, mq_msg_id_t msg_id, int value);
message_t make_double_message(pid_t dest, mq_msg_id_t msg_id, double value);


#endif /* ifndef DRONE_SIM_MQ_COMMUNICATION */

