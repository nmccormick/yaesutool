CC              = gcc

VERSION         = 1.1
CFLAGS          = -g -O -Wall -Werror -DVERSION='"$(VERSION)"'
LDFLAGS         =
LIBS            =

# Optional macOS extras:
# CFLAGS        += -I/usr/local/opt/gettext/include
# LIBS          += -L/usr/local/opt/gettext/lib -lintl

# Detect VX-1 source (accept lowercase or uppercase filename)
VX1SRC          := $(or $(wildcard vx-1.c),$(wildcard vx-1.c))

# Source files
SRCS            = main.c util.c radio.c ft-60.c vx-2.c $(VX1SRC)

# Objects derived from sources
OBJS            = $(SRCS:.c=.o)

# Auto header dependencies
CFLAGS          += -MMD -MP
DEPS            = $(OBJS:.o=.d)

.PHONY: all clean install

all:    yaesutool

yaesutool: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)

# Generic pattern rule: builds any .o from matching .c
%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

# Header dependencies (kept for clarity; pattern rule compiles them)
ft-60.o: ft-60.c radio.h util.h
main.o:  main.c  radio.h util.h
radio.o: radio.c radio.h util.h
util.o:  util.c  util.h
vx-2.o:  vx-2.c  radio.h util.h
# If you want an explicit rule for vx-1 too:
vx-1.o vx-1.c: radio.h util.h

clean:
	$(RM) *~ *.o *.d core yaesutool yaesutool.linux

install: yaesutool
	install -c -s yaesutool /usr/local/bin/yaesutool

yaesutool.linux: yaesutool
	cp -p $< $@
	strip $@

# Include auto-generated dependency files
-include $(DEPS)
