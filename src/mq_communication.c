#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "mq_communication.h"

message_t create_empty_message(pid_t dest, mq_msg_id_t msg_id) {
    message_t message;
    message.type = dest;
    message.pid = getpid();
    message.msg_id = msg_id;
    return message;
}

message_t create_message(pid_t dest, mq_msg_id_t msg_id, int value) {
    message_t message = create_empty_message(dest, msg_id);
    message.value = value;
    return message;
}

