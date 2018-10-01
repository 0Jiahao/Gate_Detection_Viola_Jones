//
// Created by phil on 13/08/18.
//

#ifndef CONFIG_H
#define CONFIG_H

#ifdef TARGET_JEVOIS
#include "ConfigJevois.h"
#elif TARGET_SIMULATOR
#include "ConfigSimulator.h"
#endif

#define F_PERIODIC_HZ 500.0 //Filter + Control
#define F_SEND_CMD_HZ 200 //Sending setpoint
#define F_SEND_STATE_HZ 50 //Send state estimation
#define F_LOG_HZ 100



#endif //CONFIG_H
