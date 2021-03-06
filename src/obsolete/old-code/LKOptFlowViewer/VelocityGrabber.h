/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Fouzhan Hosseini
 * email:  fouzhan.hosseini@iit.it
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

#ifndef __VELOCITYGRABBER_H__
#define __VELOCITYGRABBER_H__

#include <queue>

#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Semaphore.h>


using namespace std;


class VelocityGrabber: public BufferedPort<VelocityBuffer>{

    unsigned int lostCntr;
    yarp::os::Semaphore bfrMutex;
    queue < VelocityBuffer * > velBfr;
    double frame_intv;
    double camClkFreq;
    unsigned long startTS; 
    bool startNewBuff;
    VelocityBuffer * crntVelbuff;
    
public:
    VelocityGrabber(){
        startNewBuff = true;
        crntVelbuff = NULL;
        lostCntr = 0;
    }
    void setParam(double fIntv, double camRes){
        frame_intv = fIntv;
        camClkFreq = camRes;

    }

    virtual void onRead(VelocityBuffer & data){
        int sz;
        unsigned long timeDur;
        //start a new buffer
        sz = data.getSize();       
                
        if (startNewBuff){
            startNewBuff = false;
            crntVelbuff = new VelocityBuffer();
            startTS = data.getTs(0);                
        }
        

        timeDur = data.getTs(0) - startTS;
        if (timeDur*camClkFreq > frame_intv){
            bfrMutex.wait();
            velBfr.push(crntVelbuff);
            bfrMutex.post();
            crntVelbuff = new VelocityBuffer();
            startTS = data.getTs(0);
        }

        crntVelbuff -> addData(data);


        timeDur = data.getTs(sz-1) - startTS;
        if (timeDur*camClkFreq > frame_intv){
            startNewBuff = true;
            bfrMutex.wait();
            velBfr.push(crntVelbuff);
            bfrMutex.post();    
            crntVelbuff = NULL;
        }                        
        
    }


    VelocityBuffer *  getVelocities(){
        static double last_read = 0;        
        VelocityBuffer * res = NULL;              
        double curr_time = yarp::os::Time::now();
        bfrMutex.wait();
        cout << velBfr.size() << endl;
        if (velBfr.size() > 0){
           res = velBfr.front();
           velBfr.pop();
        }
        else{
            if (curr_time - last_read > frame_intv){
                res = crntVelbuff;
                crntVelbuff = NULL;
                startNewBuff = true;
            }
        }
        bfrMutex.post();

        last_read = curr_time;
        
        return res;

    }


    ~VelocityGrabber(){
        VelocityBuffer * vb;

        for (int i = 0; i < velBfr.size(); ++i) {
            vb = velBfr.front();
            delete vb;
            velBfr.pop();
        }
        if (crntVelbuff != NULL)
            delete crntVelbuff;

        cout << "Sorry! " << lostCntr <<  " Velocity bufferes were lost." << endl;
    }
};



//class VelocityGrabber: public yarp::os::BufferedPort<VelocityBuffer>{
//
//    unsigned int lostCntr;
//    yarp::os::Semaphore bfrMutex;
//    // queue < VelocityBuffer * > velBfr;
//    VelocityBuffer * velBuf;
//
//public:
//    VelocityGrabber(){
//        lostCntr = 0;
//        velBuf = new VelocityBuffer();
//    }
//
//    virtual void onRead(VelocityBuffer & data){
//        //VelocityBuffer * vb = new VelocityBuffer();
//        //vb->setData(data);
//        //bfrMutex.wait();
//        //velBfr.push(vb);
//        //bfrMutex.post();
//
//        bfrMutex.wait();
//        velBuf -> addData(data);
//        bfrMutex.post();
//    }
//
//
//
//    VelocityBuffer *  getVelocities(){
////        VelocityBuffer * res = NULL;
////        bfrMutex.wait();
////        if (velBfr.size() > 0){
////           res = velBfr.front();
////           velBfr.pop();
////        }
////        bfrMutex.post();
////        return res;
//
//       VelocityBuffer * res = NULL;
//       bfrMutex.wait();
//       if (velBuf->getSize() > 0){
//          res = velBuf;
//          velBuf = new VelocityBuffer();
//       }
//       bfrMutex.post();
//       return res;
//
//
//    }
//
//    ~VelocityGrabber(){
////        VelocityBuffer * vb;
////
////        for (int i = 0; i < velBfr.size(); ++i) {
////            vb = velBfr.front();
////            delete vb;
////            velBfr.pop();
////        }
//
//        delete velBuf;
//        cout << "Sorry! " << lostCntr <<  " Velocity bufferes were lost." << endl;
//    }
//};

#endif
