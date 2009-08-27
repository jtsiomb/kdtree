kdlib = ../libkdtree.a

CC = gcc
CFLAGS = -std=c89 -pedantic -Wall -g -I..
LDFLAGS = $(kdlib) -lm

.PHONY: all
all: test test2

test: test.c $(kdlib)
test2: test2.c $(kdlib)

.PHONY: clean
clean:
	rm -f test test2
