//
// Created by phil on 21/07/18.
//

#ifndef MAVLINKMSGHANDLER_H
#define MAVLINKMSGHANDLER_H
#include <standard/mavlink.h>
#include <common/mavlink.h>
#include <jevois/Debug/Log.H>
#include <jevois/Core/StdioInterface.H>
class MavLinkMsgHandler{
public:
    virtual void handle_msg(mavlink_message_t *msg){ LINFO("MavLink Message Received. No Handler");}
};

#endif //MAVLINKMSGHANDLER_H
