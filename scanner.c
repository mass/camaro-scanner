/**
 * @file scanner.c
 *
 * Program to scan the CAN bus of my car and output interesting data such as
 * engine RPM, throttle position, temperatures, voltage, etc.
 *
 * @author Andrew Mass
 */
#include "scanner.h"

// Integer representing the serial port used to talk to the ELM.
int elm_port;

// Boolean flag which, when set to 0, will terminate the pollRead thread.
volatile unsigned char continue_read = 1;

int main() {
  /*
   * O_RDWR - Read/Write Mode.
   * O_NOCITY - Tell the OS that we don't want to be the controlling terminal.
   * O_NDELAY - Non-blocking open of the port.
   */
  elm_port = open(TTY, O_RDWR | O_NOCTTY | O_NDELAY);

  if (elm_port < 0) {
    perror("Failed to open serial port");
    return 1;
  }

  if (!isatty(elm_port)) {
    perror("Serial port not correct");
    return 2;
  }

  struct termios config;

  // Check to see if we can get current termios configuration.
  if (tcgetattr(elm_port, &config) < 0) {
    perror("Failed to read serial port configuration");
    return 3;
  }

  // Clear old termios configuration.
  memset(&config, 0, sizeof(config));
  config.c_iflag = 0;
  config.c_oflag = 0;
  config.c_cflag = 0;
  config.c_lflag = 0;
  config.c_cc[VDISCARD] = 0;
  config.c_cc[VEOF] = 0;
  config.c_cc[VEOL] = 0;
  config.c_cc[VEOL2] = 0;
  config.c_cc[VERASE] = 0;
  config.c_cc[VINTR] = 0;
  config.c_cc[VKILL] = 0;
  config.c_cc[VLNEXT] = 0;
  config.c_cc[VMIN] = 0;
  config.c_cc[VQUIT] = 0;
  config.c_cc[VREPRINT] = 0;
  config.c_cc[VSTART] = 0;
  config.c_cc[VSTOP] = 0;
  config.c_cc[VSUSP] = 0;
  config.c_cc[VSWTC] = 0;
  config.c_cc[VTIME] = 0;
  config.c_cc[VWERASE] = 0;

  // IXON?
  config.c_iflag = IGNBRK | IGNPAR | INLCR | IUTF8;
  config.c_iflag &= ~(BRKINT | INPCK | ISTRIP | IGNCR |
      ICRNL | IUCLC | IXON | IMAXBEL);

  // OCRNL? VTDLY?
  config.c_oflag = OPOST;
  config.c_oflag &= ~(OLCUC | ONLCR | OCRNL | ONOCR |
      ONLRET | OFILL | OFDEL);

  // CS8? CREAT? PARODD?
  config.c_cflag = CS8 | CREAD | CLOCAL;
  config.c_cflag &= ~(CSTOPB | PARENB | PARODD | HUPCL);

  // ICANON? NOFLSH? TOSTOP? IEXTEN?
  config.c_lflag = ISIG | TOSTOP | IEXTEN;
  config.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHONL | NOFLSH | ICANON);

  config.c_cc[VDISCARD] = 0;
  config.c_cc[VEOF] = 4;
  config.c_cc[VEOL] = 0;
  config.c_cc[VEOL2] = 0;
  config.c_cc[VERASE] = 0;
  config.c_cc[VINTR] = 3;
  config.c_cc[VKILL] = 0;
  config.c_cc[VLNEXT] = 0;
  config.c_cc[VQUIT] = 0;
  config.c_cc[VREPRINT] = 0;
  config.c_cc[VSTART] = 0;
  config.c_cc[VSTOP] = 0;
  config.c_cc[VSUSP] = 0;
  config.c_cc[VSWTC] = 0;
  config.c_cc[VWERASE] = 0;

  // VMIN is the minimum number of characters to receive before returning.
  // VTIME is the number of deciseconds to wait before returning.
  config.c_cc[VMIN] = 0;
  config.c_cc[VTIME] = 0;

  // Set the BAUD rate to 38400.
  if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0) {
    perror("Failed to set BAUD rate to 38400");
    return 4;
  }

  // Apply the configuration struct.
  if (tcsetattr(elm_port, TCSAFLUSH, &config) < 0) {
    perror("Failed to apply new configuration to serial port");
    return 5;
  }

  // Make sure we can still retrieve the configuration.
  if (tcgetattr(elm_port, &config) < 0) {
    perror("Failed to retrieve new serial port configuration");
    return 6;
  }

  // Create a separate thread for reading from the port.
  pthread_t readThread;
  pthread_create(&readThread, NULL, pollRead, NULL);

  writeElm(ELM_RESET);
  sleep(1);

  writeElm(ELM_DEFAULTS);
  usleep(INTERVAL);

  writeElm(ELM_LINEFEED);
  usleep(INTERVAL);

  writeElm(ELM_ECHO);
  usleep(INTERVAL);

  //TODO: Switch to proper SAE protocol. "AT SP 0"
  //TODO: Verify connection to car.

  while (1) {
    writeElm(ELM_INFO);
    usleep(INTERVAL);

    writeElm(ELM_VOLTAGE);
    usleep(INTERVAL);
  }

  continue_read = 0;
  pthread_join(readThread, NULL);
  close(elm_port);
}

void writeElm(const char* command) {
  ssize_t num = write(elm_port, command, strlen(command) + 1);
  if (num < 0) {
    perror("Writing Failed!");
  } else {
    fprintf(stdout, "\nC: %s\n", command);
  }
}

void* pollRead(void* ptr) {
  UNUSED(ptr);
  char* buf = (char*) calloc(sizeof(char), READ_BUF_SIZE);
  char* result = (char*) calloc(sizeof(char), 2);
  char* itr = buf;

  while (continue_read) {
    ssize_t num = read(elm_port, result, 1);
    if (num > 0) {
      if (result[0] != 0) {
        if (result[0] == '>') {
          puts(buf);
          memset(buf, 0, sizeof(char) * READ_BUF_SIZE);
          itr = buf;
        } else {
          //TODO: Protect against buffer overflow.
          *itr++ = result[0];
        }
      }
    }
    memset(result, 0, sizeof(char) * 2);
  }

  free(buf);
  buf = NULL;
  free(result);
  result = NULL;
  itr = NULL;

  return NULL;
}
