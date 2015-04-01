/**
 * @file scanner.h
 *
 * Program to scan the CAN bus of my car and output interesting data such as
 * engine RPM, throttle position, temperatures, voltage, etc.
 *
 * @author Andrew Mass
 */

#define _BSD_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>

#define TTY "/dev/ttyUSB0"

#define READ_BUF_SIZE 32

#define ELM_DEFAULTS "AT D\r"
#define ELM_DESCRIBE_PROTOCOL "AT DP\r"
#define ELM_ECHO_OFF "AT E0\r"
#define ELM_INFO "AT I\r"
#define ELM_LINEFEED_ON "AT L1\r"
#define ELM_RESET "AT Z\r"
#define ELM_VOLTAGE "AT RV\r"

#define COLOR_RED_BOLD "\033[1;31m"
#define COLOR_BLUE_BOLD "\033[1;34m"
#define COLOR_RESET "\033[0m"

#define UNUSED(x) (void)(x)

/*
 * Writes the character string to the ELM over the serial port connection.
 *
 * @note This function must not be called before the port is opened.
 * @param command The string command to send to the ELM.
 */
void writeElm(const char* command);

/*
 * Reads all characters on the stream and concatenates them into strings that
 * represent the message that the ELM returned to us.
 *
 * @note This function is run continuously in its own thread.
 * @param ptr A pointer to nothing, required by pthread_create().
 * @returns Nothing, required by pthread_create().
 */
void* pollRead(void* ptr);

