#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "mq_communication.h"

message_t make_message(pid_t dest, mq_msg_id_t msg_id) {
    message_t message;
    message.type = dest;
    message.pid = getpid();
    message.msg_id = msg_id;
    return message;
}

message_t make_int_message(pid_t dest, mq_msg_id_t msg_id, int value) {
    message_t message = make_message(dest, msg_id);
    message.int_value = value;
    return message;
}

message_t make_double_message(pid_t dest, mq_msg_id_t msg_id, double value) {
    message_t message = make_message(dest, msg_id);
    message.double_value = value;
    return message;
}

