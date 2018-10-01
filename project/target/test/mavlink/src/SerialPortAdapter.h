//
// Created by phil on 21/07/2018.
//

#ifndef SERIALPORTADAPTER_H
#define SERIALPORTADAPTER_H

#include <stdint.h>

class SerialPortAdapter{
public:
    virtual int  open_port(const char *port)=0;
    virtual bool setup(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)=0;
    virtual bool close_port()=0;
    virtual int  read_port(uint8_t *ch)=0;
    virtual int write_port(uint8_t *buf, unsigned len)=0;
protected:
};
#endif //SERIALPORTADAPTER_H
