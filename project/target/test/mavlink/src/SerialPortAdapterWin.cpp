//
// Created by phil on 21/07/2018.
//

#include <cstdio>
#include "SerialPortAdapterWin.h"

int SerialPortAdapterWin::open_port(const char *port) {
    printf("\n\n +==========================================+");
    printf("\n |    Serial Port  Reception (Win32 API)    |");
    printf("\n +==========================================+\n");
    /*---------------------------------- Opening the Serial Port -------------------------------------------*/

    hComm = CreateFile( port,                  // Name of the Port to be Opened
                        GENERIC_READ | GENERIC_WRITE, // Read/Write Access
                        0,                            // No Sharing, ports cant be shared
                        NULL,                         // No Security
                        OPEN_EXISTING,                // Open existing port only
                        0,                            // Non Overlapped I/O
                        NULL);                        // Null for Comm Devices

    if (hComm == INVALID_HANDLE_VALUE)
        printf("\n    Error! - Port %s can't be opened\n", port);
    else
        printf("\n    Port %s Opened\n ", port);
    return 0;
}

bool SerialPortAdapterWin::setup(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control) {



    /*------------------------------- Setting the Parameters for the SerialPort ------------------------------*/

    DCB dcbSerialParams = { 0 };                         // Initializing DCB structure
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    Status = GetCommState(hComm, &dcbSerialParams);      //retreives  the current settings

    if (Status == FALSE)
        printf("\n    Error! in GetCommState()");

    switch (baud){
        case 9600:
            dcbSerialParams.BaudRate = CBR_9600;
            break;
        case 115200:
            dcbSerialParams.BaudRate = CBR_115200;
            break;
        default:
            printf("Baud Rate Unknown! Choosing 9600");
            dcbSerialParams.BaudRate = CBR_9600;

    }
    dcbSerialParams.ByteSize = 8;             // Setting ByteSize = 8
    dcbSerialParams.StopBits = ONESTOPBIT;    // Setting StopBits = 1

    if (parity){
        dcbSerialParams.Parity = ODDPARITY;

    }else{
        dcbSerialParams.Parity = NOPARITY;        // Setting Parity = None

    }

    Status = SetCommState(hComm, &dcbSerialParams);  //Configuring the port according to settings in DCB

    if (Status == FALSE)
    {
        printf("\n    Error! in Setting DCB Structure");
    }
    else //If Successfull display the contents of the DCB Structure
    {
        printf("\n\n    Setting DCB Structure Successfull\n");
        printf("\n       Baudrate = %d", dcbSerialParams.BaudRate);
        printf("\n       ByteSize = %d", dcbSerialParams.ByteSize);
        printf("\n       StopBits = %d", dcbSerialParams.StopBits);
        printf("\n       Parity   = %d", dcbSerialParams.Parity);
    }

    /*------------------------------------ Setting Timeouts --------------------------------------------------*/

    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout         = 50;
    timeouts.ReadTotalTimeoutConstant    = 50;
    timeouts.ReadTotalTimeoutMultiplier  = 10;
    timeouts.WriteTotalTimeoutConstant   = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (SetCommTimeouts(hComm, &timeouts) == FALSE)
        printf("\n\n    Error! in Setting Time Outs");
    else
        printf("\n\n    Setting Serial Port Timeouts Successfull");

    /*------------------------------------ Setting Receive Mask ----------------------------------------------*/

    Status = SetCommMask(hComm, EV_RXCHAR); //Configure Windows to Monitor the serial device for Character Reception

    if (Status == FALSE)
        printf("\n\n    Error! in Setting CommMask");
    else
        printf("\n\n    Setting CommMask successfull");
    return static_cast<bool>(Status);
}

int SerialPortAdapterWin::write_port(uint8_t *buf, unsigned len) {
    DWORD  dNoOFBytestoWrite;              // No of bytes to write into the port
    DWORD  dNoOfBytesWritten = 0;          // No of bytes written to the port

    dNoOFBytestoWrite = sizeof(buf); // Calculating the no of bytes to write into the port

    Status = WriteFile(hComm,               // Handle to the Serialport
                       buf,            // Data to be written to the port
                       len,   // No of bytes to write into the port
                       &dNoOfBytesWritten,  // No of bytes written to the port
                       NULL);

    return Status;
}

int SerialPortAdapterWin::read_port(uint8_t *ch) {
    Status = WaitCommEvent(hComm, &dwEventMask, NULL); //Wait for the character to be received

    /*-------------------------- Program will Wait here till a Character is received ------------------------*/

    if (Status == FALSE)
    {
        printf("\n    Error! in Setting WaitCommEvent()");
        return 0;

    }
    else //If  WaitCommEvent()==True Read the RXed data using ReadFile();
    {
        Status = ReadFile(hComm, ch, sizeof(uint8_t), &NoBytesRead, NULL);
        return static_cast<int>(NoBytesRead);

    }
}

bool SerialPortAdapterWin::close_port() {
    Status = CloseHandle(hComm);//Closing the Serial Port
    return static_cast<bool>(Status);
}
