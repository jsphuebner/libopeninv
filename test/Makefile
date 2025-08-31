# Option to allow signed reception of CAN variables
CAN_SIGNED ?= 0

CC		= gcc
CPP		= g++
LD		= g++
CFLAGS    = -std=c99 -ggdb -fpermissive -DSTM32F1 -DCAN_SIGNED=$(CAN_SIGNED) -Itest-include -I../include -I../../libopencm3/include
CPPFLAGS    = -ggdb -fpermissive -DSTM32F1 -DCAN_SIGNED=$(CAN_SIGNED) -Itest-include -I../include -I../../libopencm3/include
LDFLAGS     = -g
BINARY		= test_libopeninv
OBJS		= test_main.o fu.o test_fu.o test_fp.o my_fp.o my_string.o params.o \
			  stub_canhardware.o test_canmap.o canmap.o test_linbus.o linbus.o \
			  stub_libopencm3.o
VPATH = ../src ../libopeninv/src

# Check if the variable GITHUB_RUN_NUMBER exists. When running on the github actions running, this
# variable is automatically available.
# Create a compiler define with the content of the variable. Or, if it does not exist, use replacement value 99999.
CPPFLAGS += $(shell \
    if [ -z "$$GITHUB_RUN_NUMBER" ]; then echo "-DGITHUB_RUN_NUMBER=0"; else echo "-DGITHUB_RUN_NUMBER=$$GITHUB_RUN_NUMBER"; fi )

all: $(BINARY)

$(BINARY): $(OBJS)
	$(LD) $(LDFLAGS) -o $(BINARY) $(OBJS)

%.o: ../%.cpp
	$(CPP) $(CPPFLAGS) -o $@ -c $<

%.o: ../%.c
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(OBJS) $(BINARY)
