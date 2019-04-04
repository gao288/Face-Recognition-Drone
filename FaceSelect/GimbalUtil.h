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
void split(const std::string &s, char delim, Out result);
std::vector<std::string> split(const std::string &s, char delim);
int set_interface_attribs(int fd, int speed);
void writeToUART(int port, string s);
int set_interface_attribs(int fd, int speed);
void writeToUART(int port, string s);
int gimbal_init();
