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
#ifndef C_SERIAL_H
#define C_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef _WIN32
 
  #ifdef cserial_EXPORTS
    #define CSERIAL_EXPORT __declspec( dllexport )
  #else
    #define CSERIAL_EXPORT __declspec( dllimport )
  #endif /* CSERIAL_LIB */

typedef HANDLE c_serial_handle_t;
typedef DWORD c_serial_errnum_t;

#else
    /* Linux/POSIX zone */
    #define CSERIAL_EXPORT

typedef int c_serial_handle_t;
typedef int c_serial_errnum_t;

#endif /* _WIN32 */

/*
 * Error code definitions
 */
/** Everything OK */
#define CSERIAL_OK 0
/** Generic error */
#define CSERIAL_ERROR_GENERIC -1
/** The port we tried to open is not actually a serial port */
#define CSERIAL_ERROR_NOT_A_SERIAL_PORT -2
/** The port that we tried to open does not exist */
#define CSERIAL_ERROR_NO_SUCH_SERIAL_PORT -3
/** The parameters to cserial_read were incorrect */
#define CSERIAL_ERROR_INCORRECT_READ_PARAMS -4
/** Unable to create new serial port */
#define CSERIAL_ERROR_CANT_CREATE -5
/** No port defined to open */
#define CSERIAL_ERROR_NO_PORT -6
/** Port already open */
#define CSERIAL_ERROR_ALREADY_OPEN -7
/** invalid c_serial_port_t, e.g. it is NULL */
#define CSERIAL_ERROR_INVALID_PORT -8

/**
 * Serial line flags
 */
#define CSERIAL_LINE_FLAG_CTS     ( 0x01 << 0 )
#define CSERIAL_LINE_FLAG_DSR     ( 0x01 << 1 )
#define CSERIAL_LINE_FLAG_DTR     ( 0x01 << 2 )
#define CSERIAL_LINE_FLAG_RTS     ( 0x01 << 3 )
#define CSERIAL_LINE_FLAG_RI      ( 0x01 << 4 )
#define CSERIAL_LINE_FLAG_CD      ( 0x01 << 5 )
#define CSERIAL_LINE_FLAG_NONE    0x00
#define CSERIAL_LINE_FLAG_ALL     ( CSERIAL_LINE_FLAG_CTS | \
                                    CSERIAL_LINE_FLAG_DSR | \
                                    CSERIAL_LINE_FLAG_DTR | \
                                    CSERIAL_LINE_FLAG_RTS | \
                                    CSERIAL_LINE_FLAG_RI  | \
                                    CSERIAL_LINE_FLAG_CD )

/**
 * Struct representing the control line state.
 */
struct c_serial_control_lines {
    /** CD - Carrier Detect */
    int cd;
    /** CTS - Clear To Send */
    int cts;
    /** DSR - Data Set Ready */
    int dsr;
    /** DTR - Data Terminal Ready */
    int dtr;
    /** RTS - Request To Send */
    int rts;
    /** Ring Indicator(is the phone ringing?) */
    int ri;
};
typedef struct c_serial_control_lines c_serial_control_lines_t;

/** 
 * Enum for logging callback.
 */
enum CSerial_Log_Level{
    CSERIAL_TRACE,
    CSERIAL_DEBUG,
    CSERIAL_INFO,
    CSERIAL_WARNING,
    CSERIAL_ERROR
};   

/**
 * Enum for baud rate
 */
enum CSerial_Baud_Rate {
    /** Not supported by Windows */
    CSERIAL_BAUD_0,
    /** Not supported by Windows */
    CSERIAL_BAUD_50,
    /** Not supported by Windows */
    CSERIAL_BAUD_100,

    CSERIAL_BAUD_110,

    /** Not supported by Windows */
    CSERIAL_BAUD_134,
    /** Not supported by Windows */
    CSERIAL_BAUD_150,
    /** Not supported by Windows */
    CSERIAL_BAUD_200,
    
    CSERIAL_BAUD_300,
    CSERIAL_BAUD_600,
    CSERIAL_BAUD_1200,

    /** Not supported by Windows */
    CSERIAL_BAUD_1800,
 
    CSERIAL_BAUD_2400,
    CSERIAL_BAUD_4800,
    CSERIAL_BAUD_9600,
    CSERIAL_BAUD_19200,
    CSERIAL_BAUD_38400,
    CSERIAL_BAUD_115200
}; 

enum CSerial_Data_Bits{
    CSERIAL_BITS_5 = 5,
    CSERIAL_BITS_6,
    CSERIAL_BITS_7,
    CSERIAL_BITS_8
};

enum CSerial_Stop_Bits{
    CSERIAL_STOP_BITS_1 = 1,
    CSERIAL_STOP_BITS_2
};

enum CSerial_Parity{
    CSERIAL_PARITY_NONE = 0,
    CSERIAL_PARITY_ODD,
    CSERIAL_PARITY_EVEN
};

enum CSerial_Flow_Control{
    CSERIAL_FLOW_NONE = 0,
    CSERIAL_FLOW_HARDWARE,
    CSERIAL_FLOW_SOFTWARE
};

/*
 * Opaque type for interfacing with a serial port.
 */
typedef struct c_serial_port c_serial_port_t;

/**
 * Logging function pointer type
 */
typedef void (*c_serial_log_function)( enum CSerial_Log_Level logLevel,
                                       const char* logMessage,
                                       const char* fileName,
                                       int lineNumber,
                                       const char* functionName,
                                       c_serial_port_t* port );

/**
 * Cerate a new C Serial object with default settings.
 * 
 * @param port A pointer to the pointer that will be filled in with the 
 *  new data.
 * @param errnum A pointer to the native error type if extended error
 *  information is required
 * @return CSERIAL_OK or an error code.  If there was an error, the port will
 *  not be valid.
 */
CSERIAL_EXPORT int c_serial_new( c_serial_port_t** port,
                                 c_serial_errnum_t* errnum );

/**
 * Close the serial port(if it is not already closed) and free all resources.
 * The pointer will be invalid after this call
 */
CSERIAL_EXPORT void c_serial_free( c_serial_port_t* port );

/**
 * Close the port associated with this serial port.  The port may be opened
 * again by calling c_serial_open()
 */
CSERIAL_EXPORT void c_serial_close( c_serial_port_t* port );

/**
 * Open the port on the system.
 */
CSERIAL_EXPORT int c_serial_open( c_serial_port_t* port );

/**
 * Open the port on the system, optionally keeping all settings.
 * 
 * @param port The port to open
 * @param keepSettings 1 if current serial port settings should be kept,
 *  0 otherwise
 */
CSERIAL_EXPORT int c_serial_open_keep_settings( c_serial_port_t* port,
                                                int keepSettings );

/**
 * Returns 1 if the port is open, 0 otherwise
 */
CSERIAL_EXPORT int c_serial_is_open( c_serial_port_t* port );

/**
 * Set the name of the port to open.
 * The value sent is platform-dependent, e.g. COM1 on Windows and /dev/ttyS1
 * on Linux and Linux-like systems.
 *
 * The port name is copied to internal memory and may be freed after.
 */
CSERIAL_EXPORT int c_serial_set_port_name( c_serial_port_t* port,
                                           const char* port_name );

/**
 * Get the port name that this serial port represents, e.g. COM1 on Windows
 * and /dev/ttyS1 on Linux and Linux-like systems
 */
CSERIAL_EXPORT const char* c_serial_get_port_name( c_serial_port_t* port );

/**
 * Set the baud rate of the serial port
 */
CSERIAL_EXPORT int c_serial_set_baud_rate( c_serial_port_t* port, 
                                           enum CSerial_Baud_Rate baud );

/**
 * Get the baud rate of the serial port.  Note that this function returns 
 * a cached value if the port is not open, otherwise it reads directly
 * from the port and returns the proper value.
 */
CSERIAL_EXPORT enum CSerial_Baud_Rate c_serial_get_baud_rate( 
                                               c_serial_port_t* port );

/**
 * Set the number of data bits.
 */
CSERIAL_EXPORT int c_serial_set_data_bits( c_serial_port_t* port,
                                           enum CSerial_Data_Bits bits );

/**
 * Get the number of data bits
 */
CSERIAL_EXPORT enum CSerial_Data_Bits c_serial_get_data_bits( 
                                                   c_serial_port_t* port );

/**
 * Set the number of stop bits
 */
CSERIAL_EXPORT int c_serial_set_stop_bits( c_serial_port_t* port,
                                           enum CSerial_Stop_Bits bits );

/**
 * Get the number of stop bits
 */
CSERIAL_EXPORT enum CSerial_Stop_Bits c_serial_get_stop_btis( 
                                                   c_serial_port_t* port );

/**
 * Set the parity 
 */
CSERIAL_EXPORT int c_serial_set_parity( c_serial_port_t* port,
                                        enum CSerial_Parity parity );

/**
 * Get the parity.
 */
CSERIAL_EXPORT enum CSerial_Parity c_serial_get_parity( 
                                                  c_serial_port_t* port );

/**
 * Set the flow control
 */
CSERIAL_EXPORT int c_serial_set_flow_control( c_serial_port_t* port,
                                             enum CSerial_Flow_Control contol );

/**
 * Get the flow control
 */
CSERIAL_EXPORT enum CSerial_Flow_Control c_serial_get_flow_control(
                                                  c_serial_port_t* port );

/**
 * Write a block of data to the serial port.
 * Acts the same as write() in POSIX systems.
 *
 * @param port The port to write data out to.
 * @param data The data to write out to the port.
 * @param length How long(in bytes) the data is.  Set to the number of bytes written on return.
 * @return status code
 */
CSERIAL_EXPORT int c_serial_write_data( c_serial_port_t* port,
                                        void* data,
                                        int* length );

/**
 * Read data from the serial port.
 * Acts like read() in POSIX systems, except that it will also return the 
 * state of the serial lines.  This function will block for data before it 
 * returns.  If a mask of serial lines to listen to has been set using
 * c_serial_set_serial_line_change_flags, then this function may return
 * without having read any data into the data buffer.
 *
 * If data is NULL and lines is non-NULL, but no change_flags have been set,
 * this function returns immediately without having read any data.
 * 
 * @param port The port to read data from
 * @param data The buffer to write data to.  May be NULL if you are only
 *  interested in listening for serial line changes.
 * @param data_length On entry, how long the data is.  On exit, how many bytes
 *  were read. 
 * @param lines The state of the serial lines on read of data.  If waiting for
 *  the line state to change, cannot be NULL
 * @return CSERIAL_OK on success, error code otherwise
 */
CSERIAL_EXPORT int c_serial_read_data( c_serial_port_t* port,
                                       void* data,
                                       int* data_length,
                                       c_serial_control_lines_t* lines );

/**
 * Get the native handle(int or HANDLE) that is used by this serial port.
 * This is intended for use with some sort of poll()-like function.
 */
CSERIAL_EXPORT c_serial_handle_t c_serial_get_native_handle( 
                                                 c_serial_port_t* port );

/**
 * Set the state of the control lines, optionally returning the current 
 * state of the lines after setting them.
 * 
 * @param port The port to set the line state for.
 * @param lines The line state to set.  Note that only DTR and RTS can be set.
 * @param return_state If true, modify lines on return with the current state
 *  of the serial lines.
 */
CSERIAL_EXPORT int c_serial_set_control_line( c_serial_port_t* port,
                                              c_serial_control_lines_t* lines,
                                              int return_state );

/**
 * Get the current state of the serial lines
 */
CSERIAL_EXPORT int c_serial_get_control_lines( c_serial_port_t* port,
                                              c_serial_control_lines_t* lines );

/**
 * Get the number of bytes currently available
 * 
 * @param port The port to get the number of bytes currently available from
 * @param available The number of bytes available
 * @return CSERIAL_OK or an error code
 */
CSERIAL_EXPORT int c_serial_get_available( c_serial_port_t* port,
                                           int* available );

/**
 * Set the serial lines that will cause a return from c_serial_read
 *
 * @param port The port to set the line bitmask for
 * @param flags Bitwise-OR of any CSERIAL_LINE_FLAG_XXX
 */
CSERIAL_EXPORT int c_serial_set_serial_line_change_flags( c_serial_port_t* port,
                                                          int flags );

/**
 * Get the serial line flags that will cause a return from c_serial_read
 */
CSERIAL_EXPORT int c_serial_get_serial_line_change_flags( 
                                                    c_serial_port_t* port );

/**
 * Set any user data that should be associated with this serial port 
 */
CSERIAL_EXPORT void c_serial_set_user_data( c_serial_port_t* port, void* data );

/**
 * Get any user data associated with this serial port
 */
CSERIAL_EXPORT void* c_serial_get_user_data( c_serial_port_t* port );

/**
 * Get the text of a C Serial error number
 */
CSERIAL_EXPORT const char* c_serial_get_error_string( int errnum );

/**
 * Set the log function for a specific serial port
 */
CSERIAL_EXPORT int c_serial_set_log_function( c_serial_port_t* port,
                                              c_serial_log_function func );

/**
 * Set the global log function.  This function is used if the log function
 * for a specified port has not been set.
 */
CSERIAL_EXPORT int c_serial_set_global_log_function( 
                                                 c_serial_log_function func );

/**
 * A simple implementation of a log function which outputs log information
 * to stderr.
 */
CSERIAL_EXPORT void c_serial_stderr_log_function( 
                                       enum CSerial_Log_Level logLevel,
                                       const char* logMessage,
                                       const char* fileName,
                                       int lineNumber,
                                       const char* functionName,
                                       c_serial_port_t* port );

/**
 * Get the last error number associated with this port.  This corresponds
 * to errno on Linux-like systems, and GetLastError() on Windows.
 */
CSERIAL_EXPORT c_serial_errnum_t c_serial_get_last_native_errnum( 
                                                       c_serial_port_t* port );

/**
 * Get a list of all serial ports that are on the system.
 * This must be freed with c_serial_free_serial_ports_list()
 *
 * @return An array of char* listing all of the serial ports on the system
 *  e.g. /dev/ttyS1, /dev/ttyS2 on Linux; COM1,COM2 on Windows.  This array is 
 *  NULL-terminated.
 */
CSERIAL_EXPORT const char** c_serial_get_serial_ports_list();

/**
 * Free the list of serial ports retrieved with c_serial_get_serial_ports
 */
CSERIAL_EXPORT void c_serial_free_serial_ports_list( const char** port_list );
                    
#ifdef __cplusplus
} /* end extern "C" */
#endif /* __cplusplus */

#endif /* C_SERIAL_H */
