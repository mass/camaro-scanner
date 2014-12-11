MAKEFLAGS += --warn-undefined-variables
SHELL = /bin/sh

CC = gcc
CFLAGS = -O2 --std=c99 -Wall -Wextra -Werror -pedantic -g
LDFLAGS =
DEPS =
TARGET = camaro-scanner
LIBS = -lpthread

.PHONY: all
all: $(TARGET)

$(TARGET): scanner.o $(DEPS)
	$(CC) $(LDFLAGS) $^ -o $@ $(LIBS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@ $(LIBS)

.PHONY: clean
clean:
	$(RM) *.o $(TARGET)
