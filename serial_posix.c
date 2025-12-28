/*
 * serial_posix.c
 *
 * POSIX serial implementation with Yaesu-friendly defaults:
 * - raw mode
 * - 8 data bits, no parity
 * - configurable stop bits (default 2)
 * - raises DTR/RTS by default (for powered clone cables)
 *
 * Build:
 *   cc -O2 -Wall -Wextra -o vx1 vx-1.updated.c serial_posix.c <other .c files>
 *
 * If your project already has serial_* implementations, port the changes in
 * serial_open() instead of compiling this file to avoid duplicate symbols.
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include "util.h"

int serial_verbose __attribute__((weak)) = 0; /* allow override by main program */

static speed_t baud_to_speed(int baud)
{
    switch (baud) {
        case 1200: return B1200;
        case 2400: return B2400;
        case 4800: return B4800;
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
#ifdef B57600
        case 57600: return B57600;
#endif
#ifdef B115200
        case 115200: return B115200;
#endif
        default: return 0;
    }
}

static int set_modem_lines(int fd)
{
#if SERIAL_RAISE_DTR || SERIAL_RAISE_RTS
    int status = 0;
    if (ioctl(fd, TIOCMGET, &status) < 0) {
        return -1;
    }
#if SERIAL_RAISE_DTR
    status |= TIOCM_DTR;
#endif
#if SERIAL_RAISE_RTS
    status |= TIOCM_RTS;
#endif
    if (ioctl(fd, TIOCMSET, &status) < 0) {
        return -1;
    }
#else
    (void)fd;
#endif
    return 0;
}

int serial_open(const char *portname, int baud)
{
    speed_t speed = baud_to_speed(baud);
    if (speed == 0) {
        fprintf(stderr, "serial_open: unsupported baud rate %d\n", baud);
        return -1;
    }

    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        fprintf(stderr, "serial_open: open(%s) failed: %s\n", portname, strerror(errno));
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "serial_open: tcgetattr failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    cfmakeraw(&tty);

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~PARODD;
    tty.c_cflag &= ~CRTSCTS;

#if SERIAL_STOP_BITS == 2
    tty.c_cflag |= CSTOPB;
#else
    tty.c_cflag &= ~CSTOPB;
#endif

    /* Non-blocking semantics via select() in serial_read(). */
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "serial_open: tcsetattr failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    if (set_modem_lines(fd) != 0) {
        fprintf(stderr, "serial_open: failed to set DTR/RTS: %s\n", strerror(errno));
        /* Not fatal for all cables; keep going. */
    }

    /* Small settling delay for some USB-serial adapters. */
    usleep(100000);

    return fd;
}

void serial_close(int fd)
{
    if (fd >= 0) close(fd);
}

void serial_flush(int fd)
{
    if (fd >= 0) tcflush(fd, TCIFLUSH);
}

static int wait_readable(int fd, int timeout_ms)
{
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);

    struct timeval tv;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    for (;;) {
        int rc = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (rc >= 0) return rc;
        if (errno == EINTR) continue;
        return -1;
    }
}

int serial_read(int fd, unsigned char *data, int len)
{
    if (len <= 0) return 0;

    int got = 0;

    /* First byte: allow up to 200ms. */
    int rc = wait_readable(fd, 200);
    if (rc <= 0) return 0;

    while (got < len) {
        ssize_t n = read(fd, data + got, (size_t)(len - got));
        if (n > 0) {
            got += (int)n;
            if (got >= len) break;

            /* After first bytes arrive, keep trying briefly to fill the request. */
            rc = wait_readable(fd, 50);
            if (rc <= 0) break;
            continue;
        }

        if (n == 0) break;
        if (errno == EINTR) continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK) break;

        fprintf(stderr, "serial_read: read failed: %s\n", strerror(errno));
        break;
    }

    return got;
}

void serial_write(int fd, const void *data, int len)
{
    const unsigned char *p = (const unsigned char *)data;
    int remaining = len;

    while (remaining > 0) {
        ssize_t n = write(fd, p, (size_t)remaining);
        if (n > 0) {
            p += n;
            remaining -= (int)n;
            continue;
        }
        if (n < 0 && errno == EINTR) continue;

        fprintf(stderr, "serial_write: write failed: %s\n", strerror(errno));
        return;
    }

    /* Ensure it hits the wire promptly. */
    tcdrain(fd);
}
