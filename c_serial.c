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
#include <stdio.h>

#include "c_serial.h"

/*
 * Local defines
 */
#define GLOBAL_LOG( severity, message, port ) \
  do{ \
  if( global_log_function != NULL ){ \
    (*global_log_function)( severity,\
                            message,\
                            __FILE__,\
                            __LINE__,\
                            __func__,\
                            port );\
  }\
  }while( 0 )
#define GLOBAL_LOG_TRACE( message, port ) \
        GLOBAL_LOG( CSERIAL_TRACE, message, port )
#define GLOBAL_LOG_DEBUG( message, port ) \
        GLOBAL_LOG( CSERIAL_DEBUG, message, port )
#define GLOBAL_LOG_INFO( message, port ) \
        GLOBAL_LOG( CSERIAL_INFO, message, port )
#define GLOBAL_LOG_WARNING( message, port ) \
        GLOBAL_LOG( CSERIAL_WARNING, message, port )
#define GLOBAL_LOG_ERROR( message, port ) \
        GLOBAL_LOG( CSERIAL_ERROR, message, port )

#define LOG( severity, message, port ) \
        do{ \
        if( port->log_function ){\
            port->log_function( severity, \
                                message, \
                                __FILE__, \
                                __LINE__, \
                                __func__, \
                                port ); \
        }else{ \
            GLOBAL_LOG( severity, message, port ); \
        } \
        }while( 0 )
#define LOG_TRACE( message, port ) \
        LOG( CSERIAL_TRACE, message, port )
#define LOG_DEBUG( message, port ) \
        LOG( CSERIAL_DEBUG, message, port )
#define LOG_INFO( message, port ) \
        LOG( CSERIAL_INFO, message, port )
#define LOG_WARNING( message, port ) \
        LOG( CSERIAL_WARNING, message, port )
#define LOG_ERROR( message, port ) \
        LOG( CSERIAL_ERROR, message, port )

#define CHECK_INVALID_PORT( port ) \
  if( port == NULL ){\
    return CSERIAL_ERROR_INVALID_PORT;\
  }

static c_serial_log_function global_log_function = NULL;

/*
 * Struct Definitions
 */
struct c_serial_port {
    c_serial_handle_t port;
    c_serial_mutex_t mutex;
    c_serial_errnum_t last_errnum;
#ifdef _WIN32
    int winDTR;
    int winRTS;
#endif
    char* port_name;
    enum CSerial_Baud_Rate baud_rate;
    enum CSerial_Data_Bits data_bits;
    enum CSerial_Stop_Bits stop_bits;
    enum CSerial_Parity parity;
    enum CSerial_Flow_Control flow;
    void* user_data;
    c_serial_log_function log_function;
    int is_open;
};

/*
 * Local Methods
 */
static int set_raw_input( c_serial_port_t* port ){
    GET_SERIAL_PORT_STRUCT( port->port, newio );

#ifdef _WIN32
    newio.fBinary = TRUE;
    newio.fParity = TRUE;
    newio.fOutxCtsFlow = FALSE;
    newio.fOutxDsrFlow = FALSE;
    newio.fDtrControl = DTR_CONTROL_DISABLE;
    newio.fDsrSensitivity = FALSE;
    newio.fOutX = FALSE;
    newio.fInX = FALSE;
    newio.fNull = FALSE;
    newio.fRtsControl = FALSE;
	
    /* Set timeouts */
    {
        COMMTIMEOUTS timeouts = {0};
        timeouts.ReadIntervalTimeout = MAXDWORD;
        timeouts.ReadTotalTimeoutMultiplier = 0;
        timeouts.ReadTotalTimeoutConstant = 0;
        timeouts.WriteTotalTimeoutMultiplier = 0;
        timeouts.WriteTotalTimeoutConstant = 0;
        if( SetCommTimeouts( port->port, &timeouts ) == 0 ){
            port->last_error = GetLastError();
            LOG_ERROR( "Unable to set comm timeouts", port );
            return 0;
        }
    }
#else
    newio.c_iflag |= IGNBRK;
    newio.c_iflag &= ~BRKINT;
    newio.c_iflag &= ~ICRNL;
    newio.c_oflag = 0;
    newio.c_lflag = 0;
    newio.c_cc[VTIME] = 0;
    newio.c_cc[VMIN] = 1;
#endif
	
    SET_SERIAL_PORT_STRUCT( port->port, newio );

    return 1;
}

static int set_baud_rate( c_serial_port_t* desc, int baud_rate ){
    GET_SERIAL_PORT_STRUCT( desc->port, newio );

    switch( baud_rate ){
#ifndef _WIN32
        /* Note that Windows only supports speeds of 110 and above */
        SPEED_SWITCH(0,newio);
        SPEED_SWITCH(50,newio);
        SPEED_SWITCH(75,newio);
#endif
        SPEED_SWITCH(110,newio);
#ifndef _WIN32
        /* Windows does not support speeds of 134, 150, or 200 */
        SPEED_SWITCH(134,newio);
        SPEED_SWITCH(150,newio);
        SPEED_SWITCH(200,newio);
#endif
        SPEED_SWITCH(300,newio);
        SPEED_SWITCH(600,newio);
        SPEED_SWITCH(1200,newio);
#ifndef _WIN32
        /* Windows does not support 1800 */
        SPEED_SWITCH(1800,newio);
#endif
        SPEED_SWITCH(2400,newio);
        SPEED_SWITCH(4800,newio);
        SPEED_SWITCH(9600,newio);
        SPEED_SWITCH(19200,newio);
        SPEED_SWITCH(38400,newio);
        SPEED_SWITCH(115200,newio);
    }

    SET_SERIAL_PORT_STRUCT( desc->port, newio );

    return 1;
}

/**
 * @param data_bits The number of data bits
 */
static int set_data_bits( c_serial_port_t* desc, 
                          enum CSerial_Data_Bits data_bits ){
    GET_SERIAL_PORT_STRUCT( desc->port, newio );

#ifdef _WIN32 
    newio.ByteSize = data_bits;
#else
    newio.c_cflag &= ~CSIZE;
    if( data_bits == CSERIAL_BITS_8 ){
        newio.c_cflag |= CS8;
    }else if( data_bits == CSERIAL_BITS_7 ){
        newio.c_cflag |= CS7;
    }else if( data_bits == CSERIAL_BITS_6 ){
        newio.c_cflag |= CS6;
    }else if( data_bits == CSERIAL_BITS_5 ){
        newio.c_cflag |= CS5;
    }
#endif
	
    SET_SERIAL_PORT_STRUCT( desc->port, newio );

    return 1;
}

/**
 * @param stop_bits 1 for 1, 2 for 2
 */
static int set_stop_bits( c_serial_port_t* desc, 
                          enum CSerial_Stop_Bits stop_bits ){
    GET_SERIAL_PORT_STRUCT( desc->port, newio );

#ifdef _WIN32 
    if( stop_bits == CSERIAL_STOP_BITS_1 ){
        newio.StopBits = ONESTOPBIT;
    }else if( stop_bits == CSERIAL_STOP_BITS_2 ){
        newio.StopBits = TWOSTOPBITS;
    }
#else
    if( stop_bits == CSERIAL_STOP_BITS_1 ){
        newio.c_cflag &= ~CSTOPB;
    }else if( stop_bits == CSERIAL_STOP_BITS_2 ){
        newio.c_cflag |= CSTOPB;
    }
#endif
	
    SET_SERIAL_PORT_STRUCT( desc->port, newio );

    return 1;
}

/**
 * @param parity 0 for no parity, 1 for odd parity, 2 for even parity
 */
static int set_parity( c_serial_port_t* desc, enum CSerial_Parity parity ){
    GET_SERIAL_PORT_STRUCT( desc->port, newio );

#ifdef _WIN32 
    if( parity == CSERIAL_PARITY_NONE ){
        newio.Parity = NOPARITY;
    }else if( parity == CSERIAL_PARITY_ODD ){
        newio.Parity = ODDPARITY;
    }else if( parity == CSERIAL_PARITY_EVEN ){
        newio.Parity = EVENPARITY;
    }
#else
    newio.c_iflag &= ~IGNPAR; 
    newio.c_cflag &= ~( PARODD | PARENB );
    if( parity == CSERIAL_PARITY_NONE ){
        newio.c_iflag |= IGNPAR;
    }else if( parity == CSERIAL_PARITY_ODD ){
        newio.c_cflag |= PARODD;
    }else if( parity == CSERIAL_PARITY_EVEN ){
        newio.c_cflag |= PARENB;
    }
#endif
	
    SET_SERIAL_PORT_STRUCT( desc->port, newio );

    return 1;
}

/**
 * @param flow_control 0 for none, 1 for hardware, 2 for software
 */
static int set_flow_control( c_serial_port_t* desc, 
                             enum CSerial_Flow_Control flow_control ){
    GET_SERIAL_PORT_STRUCT( desc->port, newio );

#ifdef _WIN32
    if( flow_control == CSERIAL_FLOW_NONE ){
        newio.fOutxCtsFlow = FALSE;
        newio.fRtsControl = FALSE;
        newio.fOutX = FALSE;
        newio.fInX = FALSE;
    }else if( flow_control == CSERIAL_FLOW_HARDWARE ){
        newio.fOutxCtsFlow = TRUE;
        newio.fRtsControl = TRUE;
        newio.fOutX = FALSE;
        newio.fInX = FALSE;
    }else if( flow_control == CSERIAL_FLOW_SOFTWARE ){
        newio.fOutxCtsFlow = FALSE;
        newio.fRtsControl = FALSE;
        newio.fOutX = TRUE;
        newio.fInX = TRUE;
    }
#else
    newio.c_iflag &= ~( IXON | IXOFF | IXANY );
    newio.c_cflag &= ~HW_FLOW;
    if( flow_control == CSERIAL_FLOW_NONE ){
        newio.c_iflag &= ~( IXON | IXOFF | IXANY );
    }else if( flow_control == CSERIAL_FLOW_HARDWARE ){
        newio.c_cflag |= HW_FLOW;
    }else if( flow_control == CSERIAL_FLOW_SOFTWARE ){
        newio.c_iflag |= ( IXON | IXOFF | IXANY );
    }
#endif
	
    SET_SERIAL_PORT_STRUCT( desc->port, newio );

    return 1;
}

/*
 * Method Implementations
 */

int c_serial_new( c_serial_port_t** port, c_serial_errnum_t* errnum ){
    c_serial_port_t* new_port;

    if( *port == NULL ){
        return CSERIAL_ERROR_CANT_CREATE;
    }

    new_port = malloc( sizeof( struct c_serial_port ) );
    if( new_port == NULL ){
        return CSERIAL_ERROR_CANT_CREATE;
    }

    /* Clear our memory and set some sane defaults */
    memset( new_port, 0, sizeof( struct c_serial_port ) );
    new_port->baud_rate = CSERIAL_BAUD_9600;
    new_port->data_bits = CSERIAL_BITS_8;
    new_port->stop_bits = CSERIAL_STOP_BITS_1;
    new_port->parity = CSERIAL_PARITY_NONE;
    new_port->flow = CSERIAL_FLOW_NONE;

#ifdef _WIN32
    new_port->mutex = CreateMutex( NULL, FALSE, NULL );
    if( new_port->mutex == NULL ){
        /* Unable to create mutex for some reason, error out */
        if( errnum != NULL ) *errnum = GetLastError();
        free( new_port );
        return CSERIAL_ERROR_CANT_CREATE;
    }
#else
    pthread_mutex_init( &(new_port->mutex), NULL );
#endif /* _WIN32 */

    *port = new_port;

    return CSERIAL_OK;
}

void c_serial_free( c_serial_port_t* port ){
    if( port == NULL ){
        return;
    }

    c_serial_close( port );

    free( port->port_name );
    free( port );
}

void c_serial_close( c_serial_port_t* port ){
    if( port == NULL ){
        return;
    }
    close( port->port );
    port->is_open = 0;
}

int c_serial_open( c_serial_port_t* port ){
    CHECK_INVALID_PORT( port );

    if( port->is_open ) return CSERIAL_ERROR_ALREADY_OPEN;
    if( port->port_name == NULL ) return CSERIAL_ERROR_NO_PORT;

#ifdef _WIN32
    port->port = CreateFile( port->port_name,
                             GENERIC_READ | GENERIC_WRITE,
                             0, 0,
                             OPEN_EXISTING,
                             FILE_FLAG_OVERLAPPED,
                             0 );
    if( port->port == INVALID_HANDLE_VALUE ){
        port->last_errnum = GetLastError();
        if( port->last_errnum == ERROR_FILE_NOT_FOUND ){
            return CSERIAL_ERROR_NO_SUCH_SERIAL_PORT;
        }
        return CSERIAL_ERROR_GENERIC;
    }
#else
    port->port = open( port->port_name, O_RDWR );
    if( port->port < 0 ){
        port->last_errnum = errno;
        if( port->last_errnum == ENOENT ){
            return CSERIAL_ERROR_NO_SUCH_SERIAL_PORT;
        }
        return CSERIAL_ERROR_GENERIC;
    }

    {
        struct termios io_tmp;
        if( tcgetattr( port->port, &io_tmp ) < 0 ){
            port->last_errnum = errno;
            if( port->last_errnum == ENOTTY ){
                return CSERIAL_ERROR_NOT_A_SERIAL_PORT;
            }
            return CSERIAL_ERROR_GENERIC;
        } 
    }
#endif /* _WIN32 */

    port->is_open = 1;

    set_raw_input( port );
    set_baud_rate( port, port->baud_rate );
    set_data_bits( port, port->data_bits );
    set_stop_bits( port, port->stop_bits );
    set_parity( port, port->parity );
    set_flow_control( port, port->flow );

    return CSERIAL_OK;
}

int c_serial_is_open( c_serial_port_t* port ){
    CHECK_INVALID_PORT( port );

    return port->is_open;
}

int c_serial_set_port_name( c_serial_port_t* port,
                            const char* port_name ){
    int port_name_len;
    int port_name_offset;
    CHECK_INVALID_PORT( port );

    port_name_len = strlen( port_name );
#ifdef _WIN32
    port_name_offset = 4;
    port_name_len += 5; /* add in \\.\ to the front and have NULL terminator */
#else
    port_name_offset = 0;
    port_name_len += 1;
#endif

    port->port_name = malloc( port_name_len );
    memset( port->port_name, 0, port_name_len );
    memcpy( port->port_name + port_name_offset,
            port_name,
            strlen( port_name ) );
#ifdef _WIN32
    memcpy( port->port_name, "\\\\.\\", 4 );
#endif

   return CSERIAL_OK;
}

const char* c_serial_get_port_name( c_serial_port_t* port ){
    if( port == NULL ){
        return NULL;
    }

    return port->port_name;
}

int c_serial_set_baud_rate( c_serial_port_t* port, 
                            enum CSerial_Baud_Rate baud ){
    CHECK_INVALID_PORT( port );

    port->baud_rate = baud;
    if( port->is_open ){
        set_baud_rate( port, port->baud_rate );
    }

    return CSERIAL_OK;
}

enum CSerial_Baud_Rate c_serial_get_baud_rate( c_serial_port_t* port ){
    CHECK_INVALID_PORT( port );

    if( !port->is_open ){
        return port->baud_rate;
    }
}

int c_serial_set_data_bits( c_serial_port_t* port,
                            enum CSerial_Data_Bits bits ){
    CHECK_INVALID_PORT( port );
}

enum CSerial_Data_Bits c_serial_get_data_bits( c_serial_port_t* port ){
    CHECK_INVALID_PORT( port );
}

int c_serial_set_stop_bits( c_serial_port_t* port,
                            enum CSerial_Stop_Bits bits ){
    CHECK_INVALID_PORT( port );
}

enum CSerial_Stop_Bits c_serial_get_stop_btis( c_serial_port_t* port ){
    CHECK_INVALID_PORT( port );
}

int c_serial_set_parity( c_serial_port_t* port,
                         enum CSerial_Parity parity ){
    CHECK_INVALID_PORT( port );
}

enum CSerial_Parity c_serial_get_parity( c_serial_port_t* port ){
    CHECK_INVALID_PORT( port );
}

int c_serial_set_flow_control( c_serial_port_t* port,
                               enum CSerial_Flow_Control contol ){
    CHECK_INVALID_PORT( port );
}

enum CSerial_Flow_Control c_serial_get_flow_control( c_serial_port_t* port ){
    CHECK_INVALID_PORT( port );
}

int c_serial_write_data( c_serial_port_t* port,
                         void* data,
                         int length ){
    CHECK_INVALID_PORT( port );
}

int c_serial_read_data( c_serial_port_t* port,
                        void* data,
                        int* data_length,
                        c_serial_control_lines_t* lines ){
    CHECK_INVALID_PORT( port );
}

c_serial_handle_t c_serial_get_native_handle( c_serial_port_t* port ){
    CHECK_INVALID_PORT( port );
}

int c_serial_set_control_line( c_serial_port_t* port,
                               c_serial_control_lines_t* lines ){
    CHECK_INVALID_PORT( port );
}

int c_serial_get_control_lines( c_serial_port_t* port,
                                c_serial_control_lines_t* lines ){
    CHECK_INVALID_PORT( port );
}

int c_serial_get_available( c_serial_port_t* port,
                            int* available ){
    CHECK_INVALID_PORT( port );
}

int c_serial_set_serial_line_change_flags( c_serial_port_t* port,
                                           int flags ){
    CHECK_INVALID_PORT( port );
}

int c_serial_get_serial_line_change_flags( c_serial_port_t* port ){
    CHECK_INVALID_PORT( port );
}

void c_serial_set_user_data( c_serial_port_t* port, void* data ){
    if( port == NULL ){
        return;
    }

    port->user_data = data;
}

void* c_serial_get_user_data( c_serial_port_t* port ){
    if( port == NULL ){
        return NULL;
    }

    return port->user_data;
}

const char* c_serial_get_error_string( int errnum ){
    switch( errnum ){
        case CSERIAL_ERROR_INVALID_PORT: 
            return "Invalid port(was NULL)";
        case CSERIAL_ERROR_NOT_A_SERIAL_PORT: 
            return "Attempting to open something that is not a serial port";
        case CSERIAL_ERROR_NO_SUCH_SERIAL_PORT:
            return "No such serial port";
        case CSERIAL_ERROR_INCORRECT_READ_PARAMS:
            return "Invalid parameters to read from serial port";
        case CSERIAL_ERROR_CANT_CREATE:
            return "Unable to create serial port";
        default: 
            return "Unknown error";
    }
}

int c_serial_set_log_function( c_serial_port_t* port,
                               c_serial_log_function func ){
    CHECK_INVALID_PORT( port );
    port->log_function = func;
}

int c_serial_set_global_log_function( c_serial_log_function func ){
    GLOBAL_LOG_TRACE( "Foobar", NULL );

    global_log_function = func;

    GLOBAL_LOG_TRACE( "test?", NULL );
}

void c_serial_stderr_log_function( enum CSerial_Log_Level logLevel,
                                   const char* logMessage,
                                   const char* fileName,
                                   int lineNumber,
                                   const char* functionName,
                                   c_serial_port_t* port ){
    char* level_string;
    
    switch( logLevel ){
        case CSERIAL_TRACE: level_string = "trace"; break;
        case CSERIAL_DEBUG: level_string = "debug"; break;
        case CSERIAL_INFO: level_string = "info"; break;
        case CSERIAL_WARNING: level_string = "warning"; break;
        case CSERIAL_ERROR: level_string = "error"; break;
    }

    fprintf( stderr, "c_serial: [%s] %s - %s \n", 
        level_string, 
        functionName,
        logMessage );
    fflush( stderr );
}

c_serial_errnum_t c_serial_get_last_native_errnum( c_serial_port_t* port ){
    CHECK_INVALID_PORT( port );
    return port->last_errnum;
}

const char** c_serial_get_serial_ports_list(){
}

void c_serial_free_serial_ports_list(){
}
