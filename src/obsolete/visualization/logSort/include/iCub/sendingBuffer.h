// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco, Charles Clercq
 * email:   francesco.rea@iit.it, charles.clercq@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/**
 * @file sendingBuffer.h
 * @brief A class inherited from the bufferefPort class created in order to read events.
 */


#ifndef _SENDING_BUFFER
#define _SENDING_BUFFER

//#include <yarp/os/Portable.h>
#include <yarp/os/all.h>
#include <iCub/config.h>
//#define SIZE_OF_DATA 65536
//#include "config.h"

/*
#ifndef __linux__                 // posix compliant and for GCC
typedef unsigned long uint32_t;
#endif 

#define u32 uint32_t

struct aer {
    u32 timestamp;
    u32 address;z
};
*/

class sendingBuffer : public yarp::os::Portable {
public:
    sendingBuffer();
    sendingBuffer(char*, int);
    ~sendingBuffer();
    virtual bool write(yarp::os::ConnectionWriter&);
    virtual bool read(yarp::os::ConnectionReader&);

    void set_data(char*, int);

    char* get_packet(){return packet;};
    int get_sizeOfPacket(){return size_of_the_packet;};
private:
    char* packet;
    int size_of_the_packet;
};

#endif //C_SENDING_BUFFER
//----- end-of-file --- ( next line intentionally left blank ) ------------------

