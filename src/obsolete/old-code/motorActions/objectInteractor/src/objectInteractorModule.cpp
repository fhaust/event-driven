// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea
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
 * @file oInteractorModule.cpp
 * @brief Implementation of the oInteractorModule (see header file).
 */

#include <iCub/objectInteractorModule.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool oInteractorModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("/oInteractor"), 
                           "module name (string)").asString();
    /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
    setName(moduleName.c_str());

    /*
    * get the robot name which will form the stem of the robot ports names
    * and append the specific part and device required
    */
    robotName             = rf.check("robot", 
                           Value("icub"), 
                           "Robot name (string)").asString();
    robotPortName         = "/" + robotName + "/head";

    /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */
    handlerPortName =  "/";
    handlerPortName += getName();         // use getName() rather than a literal 

    if (!handlerPort.open(handlerPortName.c_str())) {           
        cout << getName() << ": Unable to open port " << handlerPortName << endl;  
        return false;
    }

    attach(handlerPort);                  // attach to port

    oiThread = 0;
    /*
    oiThread = new oInteractorThread();
    oiThread->setName(getName().c_str());
    oiThread->setRobotName(robotName);
    oiThread->start();
    */

    gThread = 0;
    gThread = new graspThread(rf);    
    gThread->setName(getName().c_str());    
    gThread->start();
    ////gThread->configure(rf);
    ////gThread->setRobotName(robotName);

    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool oInteractorModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool oInteractorModule::close() {
    handlerPort.close();
    /* stop the thread */
    
    if(oiThread != 0) {
        oiThread->stop();
        delete oiThread;
    }

    if(gThread!=0) {
        gThread->stop();
        delete gThread;
    }

    return true;
}

bool oInteractorModule::respond(const Bottle& command, Bottle& reply) {
    string helpMessage =  string(getName().c_str()) + 
                        " commands are: \n" +  
                        "help \n" + 
                        "quit \n";

    reply.clear(); 

    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;     
    }
    else if (command.get(0).asString()=="help") {
        cout << helpMessage;
        reply.addString("ok");
    }
    return true;
}

/* Called periodically every getPeriod() seconds */
bool oInteractorModule::updateModule() {
    return true;
}

double oInteractorModule::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */
    return 1.0;
}

