#include "serial.h"

#define DEFAULT_BAUDRATE    B115200


int putchar_t(int fd, uint8_t c)
{
    char buf[2];
    buf[0] = c;
    return  write(fd, buf, 1);
}

int putbuffer_t(int fd, uint8_t *buff, int len)
{
    return write(fd, buff, len);

}

int getchar_t(int fd, uint8_t *c)
{
    uint8_t buff[1];
    int rc = read(fd, buff, 1);

    *c = buff[0];
    return rc;
}

int getbuffer_t(int fd, uint8_t *buff, int len)
{
    int numRead=0;
    int response_length = 0;
    int retry=10;

    while (retry-->0) {
        numRead = read(fd, &buff[response_length], 1);
        if (numRead <= 0) {
            usleep(1000);
            retry--;
        } else {
            response_length = response_length + numRead;
        }

        if (response_length == len)
            break;
    }

    return response_length;
}

int free_uart(int fd)
{
    return close(fd);
}

static int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    memset (&tty, 0, sizeof tty);

    if (tcgetattr(fd, &tty) != 0) {
        printf("ERROR %d from tcgetattr\n", errno);
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    /*
     * Enable the receiver and set local mode...
     */
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag |= CS8;                                     /* 8-bit */
    tty.c_cflag &= ~(PARENB | PARODD);                      /* Disable Parity */
    tty.c_cflag &= ~CSIZE;                                  /* Mask the character size bits */
    tty.c_cflag &= ~CSTOPB;                                 /* Disable stop bit */
    tty.c_cflag &= ~CRTSCTS;                                /* Disable Hardware flow control */

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);         /* Non-Canonical mode */

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                 /* Disable Software flow control */
#if 1
    /* Not sure it working */
    tty.c_iflag &= ~IGNBRK;                                 // disable break processing
    tty.c_iflag &= ~(INLCR | ICRNL);
#endif

    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN]  = 0;                                    // read doesn't block
    tty.c_cc[VTIME] = 5;                                    // 0.5 seconds read timeout

    tcflush(fd, TCIOFLUSH); //FLUSH IO

    if (tcsetattr (fd, TCSANOW, &tty) != 0) {
        printf  ("ERROR %d from tcsetattr\n", errno);
        return -1;
    }
    return fd;
}

int init_uart(char *dev)
{
    int n;
    int fd;

    fd  = open(dev, O_RDWR | O_NDELAY | O_NOCTTY);
    if (fd >= 0) {
        // Cancel the O_NDELAY flag.
        n = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, n & ~O_NDELAY);
    }

    if (fd < 0) {
        printf  ("error %d opening %s: %s\n", errno, dev, strerror (errno));
        return -1;
    }

    return set_interface_attribs(fd, DEFAULT_BAUDRATE);
}
