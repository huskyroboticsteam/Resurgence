CC=gcc
CFLAGS=-I.
DEPS = CANPacket.h
OBJ = test.o CANPacket.o

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

TestCAN: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) ; rm *.o