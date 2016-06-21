/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Florian Hofmann
 * Email:  fhofmann@techfak.uni-bielefeld.de
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

#include "vRate.h"

/**********************************************************/
bool vRateModule::configure(yarp::os::ResourceFinder &rf)
{
    //set the name of the module
    std::string moduleName =
            rf.check("name", yarp::os::Value("vRate")).asString();
    setName(moduleName.c_str());

    eventManager.setTemporalSize(rf.check("temporalSize",
                                          yarp::os::Value(1)).asDouble());
    eventManager.open(moduleName);

    return true ;
}

/**********************************************************/
bool vRateModule::interruptModule()
{
    eventManager.interrupt();
    yarp::os::RFModule::interruptModule();
    return true;
}

/**********************************************************/
bool vRateModule::close()
{
    eventManager.close();
    yarp::os::RFModule::close();
    return true;
}

/**********************************************************/
bool vRateModule::updateModule()
{
    return true;
}

/**********************************************************/
double vRateModule::getPeriod()
{
    return 1.0;
}

/**********************************************************/
vRateIO::vRateIO()
{
    //here we should initialise the module
    windowSize = 1e6; // one second

}
/**********************************************************/
bool vRateIO::open(const std::string &name)
{
    //and open the input port

    this->useCallback();

    std::string inPortName = "/" + name + "/vBottle:i";
    yarp::os::BufferedPort<emorph::vBottle>::open(inPortName);

    std::string outPortName = "/" + name + "/vBottle:o";
    outPort.open(outPortName);

    return true;
}

/**********************************************************/
void vRateIO::close()
{
    //close ports
    outPort.close();
    yarp::os::BufferedPort<emorph::vBottle>::close();
}

/**********************************************************/
void vRateIO::interrupt()
{
    //pass on the interrupt call to everything needed
    outPort.interrupt();
    yarp::os::BufferedPort<emorph::vBottle>::interrupt();
}

/**********************************************************/
void vRateIO::onRead(emorph::vBottle &bot)
{
    //create event queue
    yarp::os::Stamp yts;
    this->getEnvelope(yts);
    emorph::vQueue q = bot.getAll();
    //create queue iterator
    emorph::vQueue::iterator qi, wi;

    // prepare output vBottle with address events extended with cluster ID (aec) and cluster events (clep)
    emorph::vBottle &outBottle = outPort.prepare();
    outBottle.clear();
    outPort.setEnvelope(yts);

    // current last timestamp
    int last = window.back().getStamp();

    // add all events in the bottle to the event window
    for(qi = q.begin(); qi != q.end(); qi++)
    {

        //leftWindow.addEvent(**qi);
        emorph::AddressEvent *v = (*qi)->getAs<emorph::AddressEvent>();
        if(!v) continue;

        // add event to deque
        window.push_back(*v);

    }

    // window could be empty
    if( window.empty() ) return;


    // remove all events from the window that don't qualify the time constraint

    // current time
    int now = window.back().getStamp();
    //printf("now: %dus, ", now);

    // drop events that are out of the time window

    while( !window.empty() ) {
      int then = window.front().getStamp();
      //printf("now: %d, then: %d\n", now, then);
      if( then > now ) then -= 0xFFFFFF; // 24bit resolution
      if( then < now - windowSize ) {
        window.pop_front(); // remove event
      } else {
        break;
      }
    }

    // event count
    int eventCount = window.size();
    //outBottle.addInt(eventCount);

    if( eventCount < 250000 ) {
      for( qi = q.begin(); qi != q.end(); qi++ ) {
        outBottle.addEvent(**qi);
      }
    }

    //printf("rate: %fs\n", double(eventCount) / (double(windowSize) / 1e6));


    //send on the processed events
    outPort.write();

}


void vRateIO::setTemporalSize(double seconds)
{
  this->windowSize = int(seconds * 1e6);
}

//empty line to make gcc happy
