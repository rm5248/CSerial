/*
   Copyright 2016 rm5248

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
*/

/*
 * Platform-specific definitions 
 */
#ifdef _WIN32

    #include <windows.h>

    #define close( x ) CloseHandle( x )
    #define SPEED_SWITCH(SPD,io) case SPD: io.BaudRate = CBR_##SPD; break;
    #define GET_SPEED_SWITCH(SPD,io) case CBR_##SPD: return SPD;
	
    #define GET_SERIAL_PORT_STRUCT( port, io_name ) DCB io_name = {0};\
       io_name.DCBlength = sizeof( io_name ); \
       if (!GetCommState( port, &io_name ) ) { \
         printf("bad get comm\n");\
         return -1;\
       }
    #define SET_SERIAL_PORT_STRUCT( port, io_name ) 	if( !SetCommState( port, &io_name ) ){\
        printf("bad set comm\n");\
        return 0;\
    }

typedef HANDLE c_serial_mutex_t;

#else
    #include <termios.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <unistd.h>
    #include <dirent.h>
    #include <pthread.h>
    #include <sys/ioctl.h>
    #include <errno.h>
    #include <poll.h>

    #ifndef ENOMEDIUM
    #define ENOMEDIUM ENODEV
    #endif

    #ifdef CRTSCTS
        #define HW_FLOW CRTSCTS
    #elif CNEW_RTSCTS
        #define HW_FLOW CNEW_RTSCTS
    #endif
	
    #define SPEED_SWITCH(SPD,io) case SPD: cfsetospeed( &io, B##SPD ); cfsetispeed( &io, B##SPD ); break;
    #define GET_SPEED_SWITCH(SPD,io) case B##SPD: return SPD;
    #define GET_SERIAL_PORT_STRUCT( port, io_name )	struct termios io_name; \
        if( tcgetattr( port, &io_name ) < 0 ){ \
            return -1; \
        }
    #define SET_SERIAL_PORT_STRUCT( port, io_name ) 	if( tcsetattr( port, TCSANOW, &io_name ) < 0 ){\
        return -1;\
    }

typedef pthread_mutex_t c_serial_mutex_t;

#endif /* _WIN32 */

#include <stdlib.h>
#include <errno.h>
#include <string.h>

#include "c_serial.h"

/*
 * Struct Definitions
 */
struct c_serial_port {
    c_serial_handle_t port;
    c_serial_mutex_t mutex;
#ifdef _WIN32
    int winDTR;
    int winRTS;
#endif
    const char* port_name;
    enum CSerial_Baud_Rate baud_rate;
    enum CSerial_Data_Bits data_bits;
    enum CSerial_Stop_Bits stop_bits;
    enum CSerial_Parity parity;
    enum CSerial_Flow_Control flow;
    void* user_data;
};

/*
 * Method Implementations
 */
