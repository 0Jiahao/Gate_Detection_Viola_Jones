//
// Created by phil on 21/07/2018.
//

#ifndef SERIALPORTADAPTERWIN_H
#define SERIALPORTADAPTERWIN_H

#include <Windows.h>
#include "SerialPortAdapter.h"

class SerialPortAdapterWin : public SerialPortAdapter{

public:
    int  open_port(const char *port) override ;
    bool setup(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control) override ;
    bool close_port() override ;
    int  read_port(uint8_t *ch) override ;
    int write_port(uint8_t *buf, unsigned len) override ;
protected:

    HANDLE hComm{};                          // Handle to the Serial port
    BOOL  Status{};                          // Status of the various operations
    DWORD dwEventMask{};                     // Event mask to trigger
    DWORD NoBytesRead{};                     // Bytes read by ReadFile()
};


#endif //SERIALPORTADAPTERWIN_H
