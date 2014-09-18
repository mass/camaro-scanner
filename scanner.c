/*
 * @file scanner.c
 *
 * Program to scan the CAN bus of my car and output interesting data such as
 * engine RPM, throttle position, temperatures, voltage, etc.
 *
 * @author Andrew Mass
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#define TTY "/dev/ttyUSB0"

int main() {
  int port = open(TTY, O_RDWR | O_NOCTTY | O_NDELAY);

  if(port < 0) {
    perror("1");
    return 1;
  }

  if(!isatty(port)) {
    perror("2");
    return 2;
  }

  struct termios config;

  if(tcgetattr(port, &config) < 0) {
    perror("3");
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
  config.c_cc[VMIN] = 1;
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
  config.c_cc[VMIN] = 0;
  config.c_cc[VQUIT] = 0;
  config.c_cc[VREPRINT] = 0;
  config.c_cc[VSTART] = 0;
  config.c_cc[VSTOP] = 0;
  config.c_cc[VSUSP] = 0;
  config.c_cc[VSWTC] = 0;
  config.c_cc[VTIME] = 0;
  config.c_cc[VWERASE] = 0;

  // Set the BAUD rate to 38400.
  if(cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0) {
    perror("4");
    return 4;
  }

  // Apply the configuration struct.
  if(tcsetattr(port, TCSAFLUSH, &config) < 0) {
    perror("5");
    return 5;
  }

  // Make sure we can still retrieve the configuration.
  if(tcgetattr(port, &config) < 0) {
    perror("6");
    return 6;
  }

  // This is the ASCII command that tells the ELM327 to give info about itself.
  const char * command = "AT I\r";

  while(1) {
    sleep(3);

    // Send the command to the ELM327.
    ssize_t num = 0;
    num = write(port, command, strlen(command) + 1);
    if(num < 0) {
      perror("Writing Failed");
    } else {
      puts("\nCommand sent");
    }

    // Read the response and print it to the screen.
    char* buf = (char*) malloc(sizeof(char) * 64);
    num = read(port, buf, 63);
    if(num < 0) {
      perror("Reading Failed");
    } else {
      puts(buf);
    }

    free(buf);
    buf = NULL;
  }

  close(port);
}
