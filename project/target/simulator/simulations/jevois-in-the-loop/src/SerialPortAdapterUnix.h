//
// Created by phil on 21/07/2018.
//

#ifndef SERIALPORTADAPTERUNIX_H
#define SERIALPORTADAPTERUNIX_H
#include <cstdlib>
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <termios.h> // POSIX terminal control definitions
#include <pthread.h> // This uses POSIX Threads
#include "SerialPortAdapter.h"
#include <iostream>

class SerialPortAdapterUnix : public SerialPortAdapter{
public:
    int  open_port(const char *port) override ;
    bool setup(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control) override ;
    bool close_port() override ;
    int  read_port(uint8_t *ch) override ;
    int write_port(char *buf, unsigned len) override ;
    SerialPortAdapterUnix();
    ~SerialPortAdapterUnix();
protected:
    int  fd;
    pthread_mutex_t  lock;
    bool status;
};


#endif //SERIAL_SERIALPORTADAPTERUNIX_H
