/*
 * FLOSS - Free, Lightweight, Open Short-range Stack
 * SPDX-License-Identifier: Unlicense
 *
 * floss_serial.c - Non-blocking serial port stream for Windows and POSIX.
 *
 * Overview
 * --------
 * The H4 transport polls its stream on every floss_poll(), so reads must
 * never wait. Both back ends are configured for "return what is there":
 *
 *   Windows  COMMTIMEOUTS with ReadIntervalTimeout = MAXDWORD and zero total
 *            timeouts makes ReadFile return immediately with the bytes
 *            already buffered. Writes get a short total timeout so a peer
 *            holding CTS low cannot hang the stack forever; a short write is
 *            reported and the H4 driver retries the rest.
 *   POSIX    termios raw mode (8N1, no echo, no line discipline, VMIN =
 *            VTIME = 0) on a descriptor opened with O_NONBLOCK; EAGAIN maps
 *            to "0 bytes".
 *
 * Baud rates above 230400 are not POSIX; they are used when the platform
 * defines them (Linux B460800..B4000000, macOS via IOSSIOSPEED).
 */
#ifndef _WIN32
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif
#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE /* glibc: CRTSCTS and the high B* constants */
#endif
#ifndef _DARWIN_C_SOURCE
#define _DARWIN_C_SOURCE /* macOS: CRTSCTS, IOSSIOSPEED */
#endif
#endif

#include "floss_serial.h"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <stdio.h>
#else
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#ifdef __APPLE__
#include <IOKit/serial/ioss.h>
#endif
#endif

/* ======================================================================== */
/* Windows                                                                  */
/* ======================================================================== */
#ifdef _WIN32

static int serial_write(void *ctx, const uint8_t *data, uint16_t len)
{
    floss_serial_t *s = (floss_serial_t *)ctx;
    DWORD written = 0;

    if (!WriteFile((HANDLE)s->handle, data, len, &written, NULL)) {
        return FLOSS_ERR_IO;
    }
    return (int)written;
}

static int serial_read(void *ctx, uint8_t *buf, uint16_t max)
{
    floss_serial_t *s = (floss_serial_t *)ctx;
    DWORD got = 0;

    if (!ReadFile((HANDLE)s->handle, buf, max, &got, NULL)) {
        return FLOSS_ERR_IO;
    }
    return (int)got;
}

static floss_err_t configure(floss_serial_t *s, uint32_t baud, bool rtscts)
{
    DCB dcb;
    COMMTIMEOUTS to;

    memset(&dcb, 0, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState((HANDLE)s->handle, &dcb)) {
        return FLOSS_ERR_IO;
    }
    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = rtscts ? TRUE : FALSE;
    dcb.fRtsControl = rtscts ? RTS_CONTROL_HANDSHAKE : RTS_CONTROL_ENABLE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fNull = FALSE;
    dcb.fAbortOnError = FALSE;
    if (!SetCommState((HANDLE)s->handle, &dcb)) {
        return FLOSS_ERR_UNSUPPORTED; /* usually: baud rate not supported */
    }

    memset(&to, 0, sizeof(to));
    to.ReadIntervalTimeout = MAXDWORD; /* with zero totals: never wait */
    to.ReadTotalTimeoutMultiplier = 0;
    to.ReadTotalTimeoutConstant = 0;
    to.WriteTotalTimeoutMultiplier = 0;
    to.WriteTotalTimeoutConstant = 20; /* ms; bounded stall under flow control */
    if (!SetCommTimeouts((HANDLE)s->handle, &to)) {
        return FLOSS_ERR_IO;
    }
    return FLOSS_OK;
}

floss_err_t floss_serial_open(floss_serial_t *s, const char *port, const floss_serial_config_t *cfg)
{
    char path[64];
    HANDLE h;
    floss_err_t err;

    if (s == NULL || port == NULL || cfg == NULL) {
        return FLOSS_ERR_INVALID;
    }
    memset(s, 0, sizeof(*s));
    /* "\\.\COMn" is required for COM10 and above and harmless below. */
    if (strncmp(port, "\\\\.\\", 4) == 0) {
        snprintf(path, sizeof(path), "%s", port);
    } else {
        snprintf(path, sizeof(path), "\\\\.\\%s", port);
    }
    h = CreateFileA(path, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (h == INVALID_HANDLE_VALUE) {
        return (GetLastError() == ERROR_FILE_NOT_FOUND) ? FLOSS_ERR_NOT_FOUND : FLOSS_ERR_IO;
    }
    s->handle = h;
    SetupComm(h, 8192, 8192);
    err = configure(s, cfg->baud, cfg->rtscts);
    if (err != FLOSS_OK) {
        CloseHandle(h);
        s->handle = NULL;
        return err;
    }
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

    s->open = true;
    s->stream.ctx = s;
    s->stream.write = serial_write;
    s->stream.read = serial_read;
    return FLOSS_OK;
}

floss_err_t floss_serial_set_baud(floss_serial_t *s, uint32_t baud)
{
    DCB dcb;

    if (s == NULL || !s->open) {
        return FLOSS_ERR_STATE;
    }
    memset(&dcb, 0, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState((HANDLE)s->handle, &dcb)) {
        return FLOSS_ERR_IO;
    }
    dcb.BaudRate = baud;
    return SetCommState((HANDLE)s->handle, &dcb) ? FLOSS_OK : FLOSS_ERR_UNSUPPORTED;
}

void floss_serial_close(floss_serial_t *s)
{
    if (s == NULL || !s->open) {
        return;
    }
    CloseHandle((HANDLE)s->handle);
    s->handle = NULL;
    s->open = false;
}

/* ======================================================================== */
/* POSIX                                                                    */
/* ======================================================================== */
#else

static int serial_write(void *ctx, const uint8_t *data, uint16_t len)
{
    floss_serial_t *s = (floss_serial_t *)ctx;
    ssize_t n = write(s->fd, data, len);

    if (n < 0) {
        return (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) ? 0 : FLOSS_ERR_IO;
    }
    return (int)n;
}

static int serial_read(void *ctx, uint8_t *buf, uint16_t max)
{
    floss_serial_t *s = (floss_serial_t *)ctx;
    ssize_t n = read(s->fd, buf, max);

    if (n < 0) {
        return (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) ? 0 : FLOSS_ERR_IO;
    }
    return (int)n;
}

/** Map a numeric baud rate to a termios speed constant (0 if unknown). */
static speed_t baud_to_speed(uint32_t baud)
{
    switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
#ifdef B57600
    case 57600: return B57600;
#endif
#ifdef B115200
    case 115200: return B115200;
#endif
#ifdef B230400
    case 230400: return B230400;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B500000
    case 500000: return B500000;
#endif
#ifdef B576000
    case 576000: return B576000;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
#ifdef B1000000
    case 1000000: return B1000000;
#endif
#ifdef B1500000
    case 1500000: return B1500000;
#endif
#ifdef B2000000
    case 2000000: return B2000000;
#endif
#ifdef B2500000
    case 2500000: return B2500000;
#endif
#ifdef B3000000
    case 3000000: return B3000000;
#endif
#ifdef B3500000
    case 3500000: return B3500000;
#endif
#ifdef B4000000
    case 4000000: return B4000000;
#endif
    default: return 0;
    }
}

static floss_err_t apply_baud(int fd, struct termios *tio, uint32_t baud)
{
    speed_t sp = baud_to_speed(baud);

    if (sp != 0) {
        cfsetispeed(tio, sp);
        cfsetospeed(tio, sp);
        return (tcsetattr(fd, TCSANOW, tio) == 0) ? FLOSS_OK : FLOSS_ERR_IO;
    }
#if defined(__APPLE__) && defined(IOSSIOSPEED)
    {
        /* macOS accepts arbitrary rates through an ioctl after tcsetattr. */
        speed_t custom = (speed_t)baud;
        cfsetispeed(tio, B9600);
        cfsetospeed(tio, B9600);
        if (tcsetattr(fd, TCSANOW, tio) != 0) {
            return FLOSS_ERR_IO;
        }
        return (ioctl(fd, IOSSIOSPEED, &custom) == 0) ? FLOSS_OK : FLOSS_ERR_UNSUPPORTED;
    }
#else
    return FLOSS_ERR_UNSUPPORTED;
#endif
}

floss_err_t floss_serial_open(floss_serial_t *s, const char *port, const floss_serial_config_t *cfg)
{
    struct termios tio;
    floss_err_t err;
    int fd;

    if (s == NULL || port == NULL || cfg == NULL) {
        return FLOSS_ERR_INVALID;
    }
    memset(s, 0, sizeof(*s));
    s->fd = -1;
    fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        return (errno == ENOENT) ? FLOSS_ERR_NOT_FOUND : FLOSS_ERR_IO;
    }
    if (tcgetattr(fd, &tio) != 0) {
        close(fd);
        return FLOSS_ERR_IO;
    }

    /* Raw mode by hand (cfmakeraw is not POSIX). */
    tio.c_iflag &= ~(tcflag_t)(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
    tio.c_oflag &= ~(tcflag_t)OPOST;
    tio.c_lflag &= ~(tcflag_t)(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tio.c_cflag &= ~(tcflag_t)(CSIZE | PARENB | CSTOPB);
    tio.c_cflag |= CS8 | CLOCAL | CREAD;
#ifdef CRTSCTS
    if (cfg->rtscts) {
        tio.c_cflag |= CRTSCTS;
    } else {
        tio.c_cflag &= ~(tcflag_t)CRTSCTS;
    }
#else
    if (cfg->rtscts) {
        close(fd);
        return FLOSS_ERR_UNSUPPORTED;
    }
#endif
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    err = apply_baud(fd, &tio, cfg->baud);
    if (err != FLOSS_OK) {
        close(fd);
        return err;
    }
    tcflush(fd, TCIOFLUSH);

    s->fd = fd;
    s->open = true;
    s->stream.ctx = s;
    s->stream.write = serial_write;
    s->stream.read = serial_read;
    return FLOSS_OK;
}

floss_err_t floss_serial_set_baud(floss_serial_t *s, uint32_t baud)
{
    struct termios tio;

    if (s == NULL || !s->open) {
        return FLOSS_ERR_STATE;
    }
    if (tcgetattr(s->fd, &tio) != 0) {
        return FLOSS_ERR_IO;
    }
    return apply_baud(s->fd, &tio, baud);
}

void floss_serial_close(floss_serial_t *s)
{
    if (s == NULL || !s->open) {
        return;
    }
    close(s->fd);
    s->fd = -1;
    s->open = false;
}

#endif /* _WIN32 */

const floss_stream_t *floss_serial_stream(floss_serial_t *s)
{
    return (s != NULL && s->open) ? &s->stream : NULL;
}
