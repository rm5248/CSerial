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

static c_serial_log_function global_log_function = NULL;

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
    c_serial_log_function log_function;
};

/*
 * Method Implementations
 */

int c_serial_new( c_serial_port_t** port ){
    c_serial_port_t* new_port;

    if( *port == NULL ){
        return CSERIAL_ERROR_CANT_CREATE;
    }

    new_port = malloc( sizeof( struct c_serial_port ) );
    if( new_port == NULL ){
        return CSERIAL_ERROR_CANT_CREATE;
    }

    memset( new_port, 0, sizeof( struct c_serial_port ) );

    *port = new_port;

    return CSERIAL_OK;
}

void c_serial_free( c_serial_port_t* port ){

}

void c_serial_close( c_serial_port_t* port ){
}

int c_serial_open( c_serial_port_t* port ){
}

int c_serial_is_open( c_serial_port_t* port ){
}

int c_serial_set_port_name( c_serial_port_t* port,
                            const char* port_name ){
}

const char* c_serial_get_port_name( c_serial_port_t* port ){
}

int c_serial_set_baud_rate( c_serial_port_t* port, 
                            enum CSerial_Baud_Rate baud ){
}

enum CSerial_Baud_Rate c_serial_get_baud_rate( c_serial_port_t* port ){
}

int c_serial_set_data_bits( c_serial_port_t* port,
                            enum CSerial_Data_Bits bits ){
}

enum CSerial_Data_Bits c_serial_get_data_bits( c_serial_port_t* port ){
}

int c_serial_set_stop_bits( c_serial_port_t* port,
                            enum CSerial_Stop_Bits bits ){
}

enum CSerial_Stop_Bits c_serial_get_stop_btis( c_serial_port_t* port ){
}

int c_serial_set_parity( c_serial_port_t* port,
                         enum CSerial_Parity parity ){
}

enum CSerial_Parity c_serial_get_parity( c_serial_port_t* port ){
}

int c_serial_set_flow_control( c_serial_port_t* port,
                               enum CSerial_Flow_Control contol ){
}

enum CSerial_Flow_Control c_serial_get_flow_control( c_serial_port_t* port ){
}

int c_serial_write_data( c_serial_port_t* port,
                         void* data,
                         int length ){
}

int c_serial_read_data( c_serial_port_t* port,
                        void* data,
                        int* data_length,
                        c_serial_control_lines_t* lines ){
}

c_serial_handle_t c_serial_get_native_handle( c_serial_port_t* port ){
}

int c_serial_set_control_line( c_serial_port_t* port,
                               c_serial_control_lines_t* lines ){
}

int c_serial_get_control_lines( c_serial_port_t* port,
                                c_serial_control_lines_t* lines ){
}

int c_serial_get_available( c_serial_port_t* port,
                            int* available ){
}

int c_serial_set_serial_line_change_flags( c_serial_port_t* port,
                                           int flags ){
}

int c_serial_get_serial_line_change_flags( c_serial_port_t* port ){
}

void c_serial_set_user_data( c_serial_port_t* port, void* data ){
}

void* c_serial_get_user_data( c_serial_port_t* port ){
}

const char* c_serial_get_error_string( int errnum ){
}

int c_serial_set_log_function( c_serial_port_t* port,
                               c_serial_log_function func ){
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

const char** c_serial_get_serial_ports_list(){
}

void c_serial_free_serial_ports_list(){
}
