//
// Simple Xiegu G90 CAT/CI-V protocol simulator
//

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/time.h>

uint32_t frequency = 0;

// Set serial port to 19200 8N1 raw
int serial_open(const char *path)
{
    struct termios tty;
    int sfd = open(path, O_RDWR | O_NOCTTY);
    if (sfd < 0) {
        perror("open");
        return -1;
    }

    if (tcgetattr(sfd, &tty) != 0) {
        perror("tcgetattr");
        close(sfd);
        return -1;
    }

    cfmakeraw(&tty);

    // Baud
    cfsetispeed(&tty, B19200);
    cfsetospeed(&tty, B19200);

    // 8N1
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    // No flow control
    tty.c_cflag &= ~CRTSCTS;

    // Enable receiver, local mode
    tty.c_cflag |= (CLOCAL | CREAD);

    // No canonical, no echo, no signals
    tty.c_lflag = 0;
    tty.c_iflag = 0;
    tty.c_oflag = 0;

    // Read blocking: return as soon as at least 1 byte is available
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 10;

    tcflush(sfd, TCIOFLUSH);
    if (tcsetattr(sfd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        close(sfd);
        return -1;
    }
    return sfd;
}

int main(int argc, char **argv)
{
    // Hard coded responses
    const uint8_t getFreqResp[] = { 0xFE, 0xFE, 0xE0, 0x70, 0x03, 0x04, 0x32, 0x21, 0x14, 0x00, 0xFD };
    const uint8_t setFreqResp[] = { 0xFE, 0xFE, 0xE0, 0x70, 0xFB, 0xFD};

    int fd = -1;
    uint8_t buffer[32];
    ssize_t len = 0;
    const char *dev = "/dev/ttyS0";

    // Disable stdout buffering
    setvbuf(stdout, NULL, _IONBF, 0);

    // Optionally the device may be specified as the first parameter
    if (argc >= 2)
        dev = argv[1];

    fd = serial_open(dev);
    if (fd < 0) {
        fprintf(stderr, "Failed to open serial device %s\n", dev);
        return 1;
    }

    printf("Xiegu G90 CAT/CI-V protocol simulator\n");

    for (;;) {

        len = 0;

        // Receive the first new bytes enought to determine what kind of message it is
        for (int i = 0; i < 6; i++) {
            if (read(fd, &buffer[i], 1) < 1) {
                break;
            }
            len++;
        }

        // Validate lenght
        if (len < 6) {
            continue;
        }

        // Validate the message header
        if ( (buffer[0] != 0xFE)
            ||
             (buffer[1] != 0xFE)
            ||
             (buffer[2] != 0x70)
            ||
             (buffer[3] != 0xE0)
           )
        {
            continue;
        }

        // Get the command byte
        uint8_t cmd = buffer[4];

        // Get ActiveVFO frequency command?
        if (cmd == 0x03) {
            // Proper message termination?
            if (buffer[5] == 0xFD) {
                // Send the response to the controler
                write(fd, buffer, len);
                write(fd, getFreqResp, sizeof(getFreqResp));
            }
        }
        else if (cmd == 0x05) {
            // receive the rest of the bytes
            for (int i = 0; i < 5; i++) {
                if (read(fd, &buffer[6 + i], 1) < 1) {
                    break;
                }
                len++;
            }

            // Validate length
            if (len < 11) {
                continue;
            }

            frequency = 0;

            // Check message termination
            if (buffer[10] == 0xFD)
            {
                // Decode BCD to freq
                uint8_t *in = &buffer[5];
                uint8_t decimals[10];

                decimals[0] = in[0] & 0x0F;
                decimals[1] = (in[0] >> 4) & 0x0F;
                decimals[2] = in[1] & 0x0F;
                decimals[3] = (in[1]>>4) & 0x0F;
                decimals[4] = in[2] & 0x0F;
                decimals[5] = (in[2]>>4) & 0x0F;
                decimals[6] = in[3] & 0x0F;
                decimals[7] = (in[3] >> 4) & 0x0F;
                decimals[8] = in[4] & 0x0F;
                decimals[9] = (in[4] >> 4) & 0x0F;

                uint32_t mul=1;

                for(int i=0;i<10;i++) {
                    frequency += decimals[i] * mul;
                    mul *= 10;
                }
            }

            // Print the decoded frequency
            printf("Frequency: %d Hz\r", frequency);

            // Send the response to the controler
            write(fd, buffer, sizeof(len));
            write(fd, setFreqResp, sizeof(setFreqResp));
        }
        else if (cmd == 0x06) {

            // receive the rest of the bytes
            for (int i = 0; i < 4; i++) {
                if (read(fd, &buffer[6 + i], 1) < 1) {
                    break;
                }
                len++;
            }

            // Validate length
            if (len < 10) {
                continue;
            }

            printf("frequency: %d \t %d \t %d                               \r", frequency, (buffer[5] << 8) | buffer[6], (buffer[7] << 8) | buffer[8]);
        }
    }
    close(fd);
    return 0;
}

