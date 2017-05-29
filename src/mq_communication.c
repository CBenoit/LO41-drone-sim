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

message_t create_initialized_message(pid_t dest, mq_msg_id_t msg_id, uint8_t content[], size_t content_size) {
    if (content_size > MSG_CONTENT_SIZE_MAX) {
        printf("error: cannot create a message with a content greater than the content size max.");
        abort();
    }

    message_t message = create_empty_message(dest, msg_id);
    memcpy(&message.content, &content, content_size);
    return message;
}

