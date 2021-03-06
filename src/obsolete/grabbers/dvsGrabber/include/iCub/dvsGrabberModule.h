// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
 * email:   francesco.rea@iit.it
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
 * @file dvsGrabbberModule.h
 * @brief A module that extracts independent event-driven responses coming from dvs camera Dynamic Vision Sensor, an event-based camera developed at INI)
 */

#ifndef _DVS_GRABBER_MODULE_H_
#define _DVS_GRABBER_MODULE_H_

/** 
 *
 * \defgroup icub_dvsGrabber dvsGrabber
 * @ingroup icub_eMorph
 *
 *
 * This is a module that extracts independent event-driven response to changes in the luminance sensed by DiffeDVS 
 * 
 * Event driven asynchronous sensors transmit the local pixel-level changes caused by movement in a scene at the time they occur.
 * The result is a stream of events at microsecond time resolution with very low redundancy that drastically reduces power, data storage and computational requirements.
 * 
 * \section reference
 * The address-event representation communication protocol AER 0.02, Caltech, Pasadena, CA, Internal Memo, Feb. 1993 [Online]. Available:
 * http://www.ini.uzh.ch/~amw/scx/std002.pdf
 * 
 * S. R. Deiss, T. Delbr�ck, R. J. Douglas, M. Fischer, M. Mahowald, T. Matthews, and A. M. Whatley, Address-event asynchronous local broadcast protocol, Inst. Neuroinform., Zurich, Switzerland, 1994 [Online].
 * Available: http://www.ini.uzh.ch/~amw/scx/aeprotocol.html
 * 
 * A. M. Whatley, PCI-AER Board Driver, Library & Documentation, Inst. Neuroinform., Zurich, Switzerland, 2007 [Online]. Available:
 * http://www.ini.uzh.ch/~amw/pciaer/
 * 
 * S. R. Deiss, R. J. Douglas, and A. M. Whatley, "A pulse-coded communications infrastructure for neuromorphic systems", in Pulsed Neural Networks, W. Maass and C. M. Bishop, Eds. Cambridge, MA: MIT Press, 1998, ch. 6, pp. 157�178.
 * 
 * V. Dante, P. Del Giudice, and A. M. Whatley, �PCI-AER�hardware and software for interfacing to address-event based neuromorphic systems,� The Neuromorphic Engineer vol. 2, no. 1, pp.
 * 5�6, 2005 [Online]. Available: http://ine-web.org/research/newsletters/index.html
 * 
 *
 * \section Description
 *
 * \section lib_sec Libraries
 *
 * YARP.
 *
 * \section parameters_sec Parameters
 * 
 * <b>Command-line Parameters</b> 
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - \c from \c dvsGrabber.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c dvsGrabber/conf \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c dvsGrabber \n 
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * - \c robot \c icub \n 
 *   specifies the name of the robot (used to form the root of robot port names)
 *
 *
 * <b>Configuration File Parameters</b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 *
 * 
 * \section portsa_sec Ports Accessed
 * 
 * - None
 *                          
 * \section portsc_sec Ports Created
 *
 *  <b>Input ports</b>
 *
 *  - \c /dvsGrabber \n
 *    This port is used to change the parameters of the module at run time or stop the module. \n
 *    The following commands are available
 * 
 *  -  \c help \n
 *  -  \c quit \n
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /visualFilter
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - \c /dvsGrabber/image:i \n
 *
 * <b>Output ports</b>
 *
 *  - \c /dvsGrabber \n
 *    see above
 *
 *  - \c /dvsGrabber/image:o \n
 *
 * <b>Port types</b>
 *
 *
 * \section in_files_sec Input Data Files
 *
 * None
 *
 * \section out_data_sec Output Data Files
 *
 * None
 *
 * \section conf_file_sec Configuration Files
 *
 * \c dvsGrabber.ini  in \c $ICUB_ROOT/app/dvsGrabber/conf \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows, Linux
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>dvsGrabber --name dvsGrabber --context dvsGrabber/conf --from dvsGrabber.ini --robot icub</tt>
 *
 * \author Rea Francesco
 *
 * Copyright (C) 2010 RobotCub Consortium\n
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
 * This file can be edited at \c $ICUB_ROOT/main/src/modules/visualFilter/include/iCub/visualFilter.h
 * 
 */

/**
 * CHANGE LOG
 * 30/07/2011 : added verbosity parameter and forcing the resource finder to find the name of the dumpFile             @author Rea
 */

#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>

//within project includes
#include <iCub/device2yarp.h>

class dvsGrabberModule:public yarp::os::RFModule {
    std::string moduleName;                     // name of the module (rootname of ports)
    std::string robotName;                      // name of the robot
    std::string robotPortName;                  // reference to the head of the robot
    std::string deviceName;                     // name of the device
    std::string devicePortName;                 // reference to the device port
    std::string handlerPortName;                // name of the handler port (comunication with respond function)
    int deviceNumber;                           // number to append to the port name for multiple devices
    int ratethread;                             // time constant for ratethread

    yarp::os::Port handlerPort;                 // a port to handle messages 

    device2yarp* D2Y;                          //reference to the ratethread that reads the dvs camera

public:
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod(); 
    bool updateModule();
};


#endif // __DVS_GRABBER_MODULE_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

