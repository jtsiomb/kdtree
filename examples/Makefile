kdlib = ../libkdtree.a

CC = gcc
CFLAGS = -std=c89 -pedantic -Wall -g -I..
LDFLAGS = $(kdlib) -lm

.PHONY: all
all: test test2 test3

test: test.c $(LDFLAGS)
test2: test2.c $(LDFLAGS)
test3: test3.c $(LDFLAGS)

.PHONY: clean
clean:
	rm -f test test2 test3
