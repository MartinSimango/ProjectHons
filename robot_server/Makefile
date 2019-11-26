# Makefile for the socket programming example
#

CC = gcc

CFLAGS=-c -Wall

simple_server_objects = simple_server_main.o

all : simple_server

simple_server: $(simple_server_objects)
	gcc -o simple_server_ADC $(simple_server_objects) -lpthread -static
 
simple_server_main.o: simple_server_main.c
	gcc -c simple_server_main.c -static

clean:
	rm -f *.o simple_server
