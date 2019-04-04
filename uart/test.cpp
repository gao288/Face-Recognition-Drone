#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <iterator>

using namespace std;

template<typename Out>
void split(const std::string &s, char delim, Out result) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        *(result++) = item;
    }
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag &= ~PARENB;             /* no parity bit */
    tty.c_cflag &= ~CSTOPB;             /* only need 1 stop bit */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;                 /* 8-bit characters */

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 5;
    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CRTSCTS;            /* no hardware flowcontrol */

    /* Make raw */
    cfmakeraw(&tty);

    /* Flush Port, then applies attributes */
    tcflush( fd, TCIFLUSH );
    if ( tcsetattr ( fd, TCSANOW, &tty ) != 0) {
        std::cout << "Error " << errno << " from tcsetattr" << std::endl;
        return -1;
    }

    return 0;
}

void writeToUART(int port, string s) {
    int n_written = 0, spot = 0;

    do {
        n_written = write(port, &s[spot], 1);
        spot += n_written;
    } while (s[spot-1] != '\r' && n_written > 0);
}

int main()
{
    char *portname = "/dev/ttyUSB0";
    int fd;

    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }

    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    set_interface_attribs(fd, B115200);

    writeToUART(fd, string("x0") + "\r");
    writeToUART(fd, string("y0") + "\r");
    writeToUART(fd, string("z0") + "\r");

    sleep(1);

    writeToUART(fd, string("x-100") + "\r");
    writeToUART(fd, string("y-100") + "\r");
    writeToUART(fd, string("z-100") + "\r");

    sleep(1);

    /* x axis */
    cout << "moving x axis" << endl;
    for(int i = -100;i <= 100;i++){
        writeToUART(fd, "x" + to_string(i) + "\r");
        usleep(10000);
    } 
    sleep(1);
    writeToUART(fd, string("x0") + "\r");
    sleep(1);

    /* y axis */
    cout << "moving y axis" << endl;
    for(int i = -100;i <= 100;i++){
        writeToUART(fd, "y" + to_string(i) + "\r");
        usleep(10000);
    } 
    sleep(1);
    writeToUART(fd, string("y0") + "\r");
    sleep(1);

    /* z axis */
    cout << "moving z axis" << endl;
    for(int i = -100;i <= 100;i++){
        writeToUART(fd, "z" + to_string(i) + "\r");
        usleep(10000);
    } 
    sleep(1);
    writeToUART(fd, string("z0") + "\r");
}
