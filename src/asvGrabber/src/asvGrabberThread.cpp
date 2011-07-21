// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * This class use the USB retina driver wrote by
 * Martin Ebner, IGI / TU Graz (ebner at igi.tugraz.at)
 *
 *  The term of the contract of the used source is :
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * This driver is based on the 2.6.3 version of drivers/usb/usb-skeleton.c
 * (Copyright (C) 2001-2004 Greg Kroah-Hartman (greg@kroah.com))
 *
 */

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
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

#include <iCub/asvGrabberThread.h>
//#include <sys/types.h>
#include <assert.h>
#include <unistd.h>
#include <stdlib.h>
#include <inttypes.h>

#include <sys/types.h>
#include <inttypes.h>
//#include <stdint.h>

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>

#include <sys/time.h>
#include <sched.h>

using namespace std;
using namespace yarp::os;

extern int errno; 
//EAGAIN		11	/* Resource temporarily unavailable */



/*
#################################################################
# time constants

OneSecond = 1000000 # ???

# genova visit timings:
timestep  = OneSecond // 10000
latchexpand = 100

# new faster timings, timestep 1us, latch 10 timesteps
timestep  = OneSecond // 1000000
latchexpand = 10

# ultra slow
timestep  = OneSecond // 100
latchexpand = 100

# very slow 1ms timestep
timestep  = OneSecond // 1000
latchexpand = 100

# 10us timestep, 100us latch
timestep  = OneSecond // 100000
latchexpand = 8
reset_pins_expand = 4
*/

#define INPUT_BIN_U32U32LE

#define reset_pins_expand  4
#define timestep 1000 //10 us OneSecond/1000
#define OneSecond 1000000 //1sec
#define latchexpand 100
#define powerdownexpand 100
#define countBias 28
#define LATCH_KEEP 1
#define LATCH_TRANSPARENT 0

#define CLOCK_LO 0
#define CLOCK_HI 1
#define THRATE 5


asvGrabberThread::asvGrabberThread(string portDeviceName, bool i_bool, string i_fileName = " "):RateThread(THRATE) {

    SynThr       = 52458;         
    SynTau       = 101508;       
    SynPxlTau    = 16777215;    
    testPbias    = 16777215;
    CDRefr       = 8053457;       
    CDSf         = 133;       
    CDPr         = 160712;    
    ReqPuX       = 944;       
    ReqPuY       = 16777215;  
    IFRf         = 639172;    
    IFThr        = 30108;      
    IFLk         = 20;        
    CDOffThr     = 5;          

    SynPxlW      = 52458;     
    SynW         = 101508;    
    CDOnThr      = 16777215; 
    CDDiff       = 8053457;  
    EMCompH      = 133;     
    EMCompT      = 160712;       
    CDIoff       = 944;        
    CDRGnd       = 16777215;
    self         = 16777215;
    FollBias     = 639172;   
    ArbPd        = 30108;       
    EMVrefL      = 20;           
    CDCas        = 5;            
    EMVrefH      = 5;
    l2V          = 5;

     
    save = false;
    // passing the parameter to the class variable
    this->portDeviceName = portDeviceName;
    this->biasFileName   = i_fileName;

    //initialisation of the module
    len=0;
    sz=0;
    ec = 0;
    u64 ec = 0;
    
    memset(buffer, 0, SIZE_OF_DATA);
    // 8192 number of max event that can be read
    //SIZE_OF_DATA is SIZE_OF_EVENT * 8bytes
    monBufSize_b = SIZE_OF_EVENT * sizeof(struct aer);
    printf("monitor buffer size %d \n", monBufSize_b);
    assert(SIZE_OF_DATA >= monBufSize_b);
    // pmon reads events from the device
    pmon = (aer *)  malloc(monBufSize_b);
    if ( pmon == NULL ) {
        printf("pmon malloc failed \n");
    }

    // opening the port for sending out events
    int deviceNum=0;
    printf("opening port for sending the read events \n");
    str_buf << "/icub/retina" << deviceNum << ":o";
    port.open(str_buf.str().c_str());
    portDimension.open("/aexGrabber/dim:o");

    // opening the file when the biases are programmed by file
    biasFromBinary = i_bool;
    // preparing and sending biases      
    prepareBiases();
    prepareBiasesRight();
}


asvGrabberThread::~asvGrabberThread() {
   
}



void asvGrabberThread::prepareBiases() {
    //opening the device
    cout <<"name of the file buffer:" <<portDeviceName.c_str()<< endl;
    file_desc = open(portDeviceName.c_str(), O_RDWR | O_NONBLOCK | O_SYNC );
    if (file_desc < 0) {
        printf("Cannot open device file: %s \n",portDeviceName.c_str());
    }
    
    //initialisation
    const u32 seqAllocChunk_b = SIZE_OF_EVENT * sizeof(struct aer); //allocating the right dimension for biases
    int r, w;
    struct timeval tv;
    u64 Tseqstart, TmaxSeqTimeEstimate, Tnow;
    u32 ival, addr, hwival;
    int aexfd;
    int busy;

#ifdef INPUT_BIN_U32U32LE
    u32 rbuf[2];
#endif

    pseq = (aer *) malloc(seqAllocChunk_b);
    if ( pseq == NULL ) {
        printf("pseq malloc failed \n");
    }
    seqAlloced_b = seqAllocChunk_b;

    seqEvents = 0;
    seqSize_b = 0;

    
    //preparing the biases
    if(biasFromBinary) {
        printf("sending biases read from the binary file \n");
        //opening the file
        binInput = fopen(biasFileName.c_str(),"r");
        if (binInput == NULL) {
            fputs ("File error",stderr);
            return;
        }
        else {
            printf("File correctly opened \n");
        }
        while (1) {
            if (seqAlloced_b < seqSize_b) {
                //fprintf(stderr, "code error 1234: %" PRIu32 " < %" PRIu32 "\n", seqAlloced_b, seqSize_b);
                exit(1);
            }
            // realloc needed?
            if (seqAlloced_b == seqSize_b) {
                seqAlloced_b += seqAllocChunk_b;
                pseq = (aer*)realloc(pseq, seqAlloced_b);
                if (pseq == NULL) printf("realloc failed:");
            }

#ifndef INPUT_BIN_U32U32LE
            r = fscanf(binInput, "%" PRIu32 " %" PRIu32, &ival, &addr);

            if (r == EOF) {
                fprintf(stderr, "parsing input completed.\n");
                fclose(ifd);
                break;
            }
            if (r != 2) {
                fprintf(stderr, "input parsing error!!!\n");
                exit(1); // FIXME
            }
#else
            r = fread(rbuf, 8, 1, binInput);
            //printf("reading from file %d \n",r);

            if (r == 0) {
                if (feof(binInput)) {
                    fprintf(stderr, "parsing input completed.\n");
                    fclose(binInput);
                    break;
                } else {
                    fprintf(stderr, "input parsing error!!!\n");
                    perror("errno");
                    exit(1);
                }
            }
            ival = rbuf[0];
            addr = rbuf[1];
#endif

            /* timestamp expressed in <ts> * 1e-6 / 128e-9, with <ts> in microseconds */
            hwival = (u32)(ival * 7.8125);
            pseq[seqEvents].address = addr;
            pseq[seqEvents].timestamp = hwival;
            
            seqEvents++;
            seqTime += hwival;
            seqSize_b += sizeof(struct aer);


            assert(seqEvents * sizeof(struct aer) == seqSize_b);
        } //end of the while

        /* save start of sequencing time */
        gettimeofday(&tv, NULL);
        Tnow = ((u64)tv.tv_sec) * 1000000 + ((u64)tv.tv_usec);
        Tseqstart = Tnow;
        seqDone_b = 0;

        /* try writing to kernel driver */
        if (seqDone_b < seqSize_b) {
            //fprintf(stderr, "calling write fd: %d  sS: %d  sD: %d  ps: %x\n", aexfd, seqSize_b, seqDone_b, (u32)pseq);


            w = write(file_desc, seqDone_b + ((u8*)pseq), seqSize_b - seqDone_b);

            //fprintf(stderr, "wrote: %d\n", w);
            if (w > 0) {
                busy = 1;
                seqDone_b += w;
            } else if (w == 0 || (w < 0 && errno == EAGAIN)) {
                // we were not busy, maybe someone else is... 
            } else {
                perror("invalid write");
                exit(1);
            }
        }
        //closing the file where the biases are set
        //if(biasFromBinary){
        //    printf("closing the file where the biases are saved \n");
        //    fclose(binInput);
        //}
    } 
    else {
        printf("sending biases as following variables.... \n");

        int err;                

        printf("SynThr       = 52458\n");         
	printf("SynTau       = 101508\n");       
	printf("SynPxlTau    = 16777215\n");    
	printf("CDRefr       = 8053457\n");       
	printf("CDSf         = 133\n");       
	printf("CDPr         = 160712\n");    
	printf("ReqPuX       = 944\n");       
	printf("ReqPuY       = 16777215\n");  
	printf("IFRf         = 639172\n");    
	printf("IFThr        = 30108\n");      
	printf("IFLk         = 20\n");        
	printf("CDOffThr     = 5\n");          	
	printf("SynPxlW      = 52458\n");     
	printf("testPbias      = 52458\n");
	printf("SynW         = 101508\n");    
	printf("CDOnThr      = 16777215\n"); 
	printf("CDDiff       = 8053457\n");  
	printf("EMCompH      = 133\n");     
	printf("EMCompT      = 160712\n");       
	printf("CDIoff       = 944\n");        
	printf("CDRGnd       = 16777215\n");  
	printf("self       = 16777215\n");
	printf("FollBias     = 639172\n");   
	printf("ArbPd        = 30108\n");       
	printf("EMVrefL      = 20\n");           
	printf("CDCas        = 5\n");            
	printf("EMVrefH      = 5\n");        
	printf("l2V       = 16777215\n")
           
        int biasValues[]={SynThr,      
                          SynTau,  
                          SynPxlTau,
			  SynPxlThr,
                          CDRefr,    
                          CDSf,   
                          CDPr,    
                          ReqPuX,   
                          ReqPuY,
                          IFRf,    
                          IFThr,       
                          IFLk,        
                          IFLk,
			  CDOffThr,
			  SynPxlW,
			  testPbias,
			  SynW,
			  CDOnThr,
			  CDDiff,
			  EMCompH,
			  EMCompT,
			  CDIoff,
			  CDRGnd,
			  self,
			  FollBias,
			  ArbPd,
			  EMVrefL,
			  CDCas,
			  EMVrefH,
			  l2V
        };
        

        string biasNames[] = {
	  "SynThr",      
	  "SynTau",  
	  "SynPxlTau",  
	  "SysPxlThr",
	  "testPbias",
	  "CDRefr",    
	  "CDSf",   
	  "CDPr",    
	  "ReqPuX",   
	  "ReqPuY",
	  "IFRf",    
	  "IFThr",       
	  "IFLk",        
	  "IFLk",
	  "CDOffThr",
	  "SynPxlW",
	  "SynW",
	  "CDOnThr",
	  "CDDiff",
	  "EMCompH",
	  "EMCompT",
	  "CDIoff",
	  "CDRGnd",
	  "self"
	  "FollBias",
	  "ArbPd",
	  "EMVrefL",
	  "CDCas",
	  "EMVrefH",
	  "l2V"
        };

        
        //int err = write(file_desc,bias,41); //5+36 
	printf("sending biases as events to the device ... \n");
        seqEvents = 0;
        seqSize_b = 0;
        for(int j = 0; j < countBias; j++) {
            progBias(biasNames[j], countBias, biasValues[j],1);
        }
        latchCommitAEs(1);
        //monitor(10);
        releasePowerdown(1);
        sendingBias();
    }
}

void asvGrabberThread::prepareBiasesRight() {
    //opening the device
    cout <<"name of the file buffer:" <<portDeviceName.c_str()<< endl;
    file_desc = open(portDeviceName.c_str(), O_RDWR | O_NONBLOCK);
    if (file_desc < 0) {
        printf("Cannot open device file: %s \n",portDeviceName.c_str());
    }
    
    //initialisation
    const u32 seqAllocChunk_b = SIZE_OF_EVENT * sizeof(struct aer); //allocating the right dimension for biases
    int r, w;
    struct timeval tv;
    u64 Tseqstart, TmaxSeqTimeEstimate, Tnow;
    u32 ival, addr, hwival;
    int aexfd;
    int busy;

#ifdef INPUT_BIN_U32U32LE
    u32 rbuf[2];
#endif

    pseq = (aer *) malloc(seqAllocChunk_b);
    if ( pseq == NULL ) {
        printf("pseq malloc failed \n");
    }
    seqAlloced_b = seqAllocChunk_b;

    seqEvents = 0;
    seqSize_b = 0;

    
    //preparing the biases
    if(biasFromBinary) {
        printf("sending biases read from the binary file \n");
        //opening the file
        binInput = fopen(biasFileName.c_str(),"r");
        if (binInput == NULL) {
            fputs ("File error",stderr);
            return;
        }
        else {
            printf("File correctly opened \n");
        }
        while (1) {
            if (seqAlloced_b < seqSize_b) {
                //fprintf(stderr, "code error 1234: %" PRIu32 " < %" PRIu32 "\n", seqAlloced_b, seqSize_b);
                exit(1);
            }
            // realloc needed?
            if (seqAlloced_b == seqSize_b) {
                seqAlloced_b += seqAllocChunk_b;
                pseq = (aer*)realloc(pseq, seqAlloced_b);
                if (pseq == NULL) printf("realloc failed:");
            }

#ifndef INPUT_BIN_U32U32LE
            r = fscanf(binInput, "%" PRIu32 " %" PRIu32, &ival, &addr);

            if (r == EOF) {
                fprintf(stderr, "parsing input completed.\n");
                fclose(ifd);
                break;
            }
            if (r != 2) {
                fprintf(stderr, "input parsing error!!!\n");
                exit(1); // FIXME
            }
#else
            r = fread(rbuf, 8, 1, binInput);
            //printf("reading from file %d \n",r);

            if (r == 0) {
                if (feof(binInput)) {
                    fprintf(stderr, "parsing input completed.\n");
                    fclose(binInput);
                    break;
                } else {
                    fprintf(stderr, "input parsing error!!!\n");
                    perror("errno");
                    exit(1);
                }
            }
            ival = rbuf[0];
            addr = rbuf[1];
#endif

            /* timestamp expressed in <ts> * 1e-6 / 128e-9, with <ts> in microseconds */
            hwival = (u32)(ival * 7.8125);
            pseq[seqEvents].address = addr;
            pseq[seqEvents].timestamp = hwival;
            
            seqEvents++;
            seqTime += hwival;
            seqSize_b += sizeof(struct aer);


            assert(seqEvents * sizeof(struct aer) == seqSize_b);
        } //end of the while

        /* save start of sequencing time */
        gettimeofday(&tv, NULL);
        Tnow = ((u64)tv.tv_sec) * 1000000 + ((u64)tv.tv_usec);
        Tseqstart = Tnow;
        seqDone_b = 0;

        /* try writing to kernel driver */
        if (seqDone_b < seqSize_b) {
            //fprintf(stderr, "calling write fd: %d  sS: %d  sD: %d  ps: %x\n", aexfd, seqSize_b, seqDone_b, (u32)pseq);


            w = write(file_desc, seqDone_b + ((u8*)pseq), seqSize_b - seqDone_b);

            //fprintf(stderr, "wrote: %d\n", w);
            if (w > 0) {
                busy = 1;
                seqDone_b += w;
            } else if (w == 0 || (w < 0 && errno == EAGAIN)) {
                // we were not busy, maybe someone else is... 
            } else {
                perror("invalid write");
                exit(1);
            }
        }
        //closing the file where the biases are set
        //if(biasFromBinary){
        //    printf("closing the file where the biases are saved \n");
        //    fclose(binInput);
        //}
    } 
    else {
        printf("sending biases as following variables.... to the RIGHT \n");
        printf("cas:%d \n",casRight);
        printf("injg:%d \n",injgRight);
        printf("reqPd:%d \n",reqPdRight);
        printf("pux:%d \n",puxRight);
        printf("diffoff:%d \n",diffoffRight);
        printf("req:%d \n",reqRight);
        printf("refr:%d \n",refrRight);
        printf("puy:%d \n",puyRight);
        printf("diffon:%d \n",diffonRight);
        printf("diff:%d \n",diffRight);
        printf("foll:%d \n",follRight);
        printf("pr:%d \n",prRight);
        int err;
        
        printf("sending biases as events to the device ... \n");
                
            
        int biasValues[]={casRight,        // cas
                          injgRight,       // injGnd
                          reqPdRight,    // reqPd
                          puxRight,     // puX
                          diffoffRight,        // diffOff
                          reqRight,      // req
                          refrRight,           // refr
                          puyRight,    // puY
                          diffonRight,      // diffOn
                          diffRight,       // diff 
                          follRight,          // foll
                          prRight            //Pr 
        };
        

        string biasNames[] = {
            "cas",
            "injGnd",
            "reqPd",
            "puX",
            "diffOff",
            "req",
            "refr",
            "puY",
            "diffOn",
            "diff",
            "foll",
            "Pr"
        };

        
        //int err = write(file_desc,bias,41); //5+36 
        seqEvents = 0;
        seqSize_b = 0;
        for(int j=0;j<countBias;j++) {
            progBias(biasNames[j],24,biasValues[j],0);
        }
        latchCommitAEs(0);
        //monitor(10);
        releasePowerdown(0);
        sendingBias();
    }
}


void asvGrabberThread::setDeviceName(string deviceName) {
    printf("saving portDevice \n");
    portDeviceName=deviceName;
}

void asvGrabberThread::closeDevice(){
    close(file_desc);
}

void  asvGrabberThread::run() {
    //printf("reading \n");

    r = read(file_desc, pmon, monBufSize_b);
    //printf("called read() with monBufSize_b == %d -> retval: %d\n", (int)monBufSize_b, (int)r);
    
    if(r < 0) {
        if (errno == EAGAIN) {
            // everything ok, just no data available at the moment...
            // we will be called again in a few miliseconds..
            return;
        } else {
            printf("error reading from aerfx2: %d\n", (int)errno);
            perror("perror:");
            return;
        }
    }
    

   int sizeofstructaer = sizeof(struct aer);

    if (r % sizeofstructaer != 0) {
        printf("ERROR: read %d bytes from the AEX!!!\n", r);
    }
    monBufEvents = r / sizeofstructaer;
    printf("%d \n",r);

    int k = 0;
    int k2 = 0;
    uint32_t * buf2 = (uint32_t*)buffer;
    u32 a, t;
    int alow, ahigh;
    int tlow, thigh;

    for (int i = 0; i < monBufEvents; i++) {
        // double buffer!!
        a = pmon[i].address;
        t = pmon[i].timestamp * 0.128;
        //alow = a&0xFFFF0000;
        //tlow = t&0xFFFF0000;
        //ahigh = (a&0xFFFF0000);
        //thigh = (t&0xFFFF0000);
        
        //printf("a: %llu  t:%llu  \n",a,t);            
        //
        if (save) {
            fprintf(fout,"%08X %08X\n",a,t); 
            //fout<<hex<<a<<" "<<hex<<t<<endl;
        }


        buf2[k2++] = a;
        buf2[k2++] = t;
        //if(i == 1000)
        //    printf("address:%d ; timestamp:%d \n", a, t);
    }


    sz = monBufEvents*sizeof(struct aer); // sz is size in bytes

    if (port.getOutputCount()) {
        sendingBuffer data2send(buffer, sz);    
        sendingBuffer& tmp = port.prepare();
        tmp = data2send;
        port.write();
    }   

    if (portDimension.getOutputCount()) {
        
        Bottle& b = portDimension.prepare();
        b.clear();
        b.addInt(r);
        portDimension.write();
    }  

    //resetting buffers    
    memset(buffer, 0, SIZE_OF_DATA);
}


void asvGrabberThread::sendingBias() {
    int busy;
    seqDone_b = 0;
    printf("-------------------------------------------- \n");
    printf("trying to write to kernel driver %d %d \n", seqDone_b,seqSize_b);
    while (seqDone_b < seqSize_b) {      
        // try writing to kernel driver 
        printf( "calling write fd: sS: %d  sD: %d \n", seqSize_b, seqDone_b);
        int w = write(file_desc, seqDone_b + ((u8*)pseq), seqSize_b - seqDone_b);
        
        if (w > 0) {
                busy = 1;
                seqDone_b += w;
            } else if (w == 0 || (w < 0 && errno == EAGAIN)) {
                /* we were not busy, maybe someone else is... */
            } else {
                perror("invalid write");
                exit(1);
            }
    }    
    printf("writing successfully ended \n");

    ////////////////////////////////////////////////////////////


    double TmaxSeqTimeEstimate = 
        seqTime * 0.128 * 1.10 +   // summed up seq intervals in us plus 10%
        seqEvents * 1.0           // plus one us per Event
    ;

    printf("seqEvents: %d \n", seqEvents);
    printf("seqTime * 0.128: %d \n", (u64)(seqTime * 0.128));
    //printf("TmaxSeqTimeEstima PRIu64  %f %f\n", 
    //  (TmaxSeqTimeEstimate / 1000000), (TmaxSeqTimeEstimate % 1000000));


    ////////////////////////////////////////////////////////////

}

void asvGrabberThread::progBias(string name,int bits,int value, int camera ) {
    int bitvalue;
    
    for (int i=bits-1;i>=0;i--) {
        int mask=1;
        for (int j=0; j<i; j++) {
            mask*=2;
        }
        //printf("mask %d ",mask);
        if (mask & value) {
            bitvalue = 1;
        }
        else {
            bitvalue = 0;
        }
        progBitAEs(bitvalue, camera);
        
    }
    //after each bias value, set pins back to default value
    //resetPins();
}

void asvGrabberThread::latchCommit(int camera ) {
    //printf("entering latch_commit \n");
    biasprogtx(timestep * latchexpand, LATCH_TRANSPARENT, CLOCK_LO, 0,0, camera);
    biasprogtx(timestep * latchexpand, LATCH_KEEP, CLOCK_LO, 0,0, camera);
    biasprogtx(timestep * latchexpand, LATCH_KEEP, CLOCK_LO, 0,0, camera);
    //printf("exiting latch_commit \n");
}

void asvGrabberThread::latchCommitAEs(int camera ) {
    //printf("entering latch_commit \n");
    biasprogtx(timestep * latchexpand, LATCH_TRANSPARENT, CLOCK_HI, 0,0, camera );
    biasprogtx(timestep * latchexpand, LATCH_KEEP, CLOCK_HI, 0,0, camera);
    biasprogtx(timestep * latchexpand, LATCH_KEEP, CLOCK_HI, 0,0, camera);
    //printf("exiting latch_commit \n");
}

void asvGrabberThread::resetPins(int camera ) {
    //now
    biasprogtx(0, LATCH_KEEP, CLOCK_HI, 0, camera );
    biasprogtx(reset_pins_expand * timestep, LATCH_KEEP, CLOCK_HI, 0, camera);
    //printf("exiting latch_commit \n");
}

void asvGrabberThread::releasePowerdown(int camera ) {
    //now
    biasprogtx(0, LATCH_KEEP, CLOCK_HI, 0, 0, camera);
    biasprogtx(latchexpand * timestep, LATCH_KEEP, CLOCK_HI, 0, 0, camera);
    //printf("exiting latch_commit \n");
}


void asvGrabberThread::setPowerdown(int camera ) {
    biasprogtx(0, LATCH_KEEP, CLOCK_HI, 0, 1, camera);
    biasprogtx(powerdownexpand * timestep, LATCH_KEEP, CLOCK_HI, 0, 1, camera);
}


void asvGrabberThread::progBit(int bitvalue, int camera ) {
    //set data
    biasprogtx(timestep, LATCH_KEEP, CLOCK_LO, bitvalue,0, camera );
    //toggle clock
    biasprogtx(timestep, LATCH_KEEP, CLOCK_HI, bitvalue,0, camera);
    biasprogtx(timestep, LATCH_KEEP, CLOCK_LO, bitvalue,0, camera);
}

void asvGrabberThread::progBitAEs(int bitvalue, int camera ) {
    
    //set data (now)
    biasprogtx(0, LATCH_KEEP, CLOCK_HI, bitvalue,0, camera);
    //toggle clock
    biasprogtx(timestep, LATCH_KEEP, CLOCK_LO, bitvalue,0, camera);
    biasprogtx(timestep, LATCH_KEEP, CLOCK_HI, bitvalue,0, camera);
    //and wait a little
    biasprogtx(timestep, LATCH_KEEP, CLOCK_HI, bitvalue,0, camera);
}

void asvGrabberThread::monitor (int secs, int camera ) {
    //printf("entering monitor \n");
    biasprogtx(secs * OneSecond, LATCH_KEEP, CLOCK_LO, 0,0, camera);
    //printf("exiting monitor \n");
} 

void asvGrabberThread::biasprogtx(int time,int latch,int clock,int data, int powerdown, int camera ) {
    unsigned char addr[4];
    unsigned char t[4];
    int err;
    //setting the time
    //printf("biasProgramming following time %d \n",time);
    t[0]= time & 0xFF000000;
    t[1]= time & 0x00FF0000;
    t[2]= time & 0x0000FF00;
    t[3]= time & 0x000000FF;
    
    //setting the addr
    addr[0] = 0xFF;
    addr[1] = 0x00;
    addr[2] = 0x00;
    if(data) {
        addr[3] += 0x01;
    }
    if(clock) {
        addr[3] += 0x02;
    }
    if (latch) {
        addr[3] += 0x04;
    }
    if (powerdown) {
        addr[3] += 0x08;
    }
    //printf("data:0x%x, 0x%x, 0x%x, 0x%x \n",addr[0],addr[1],addr[2],addr[3]);
    
    
    //u32 seqSize_b = sizeof(struct aer);
    u32 timeToSend, addressToSend;
    timeToSend=time;
    

    // performing trasmittion differently if the camera is left (1) or right (0)
    // keep the clock high for the other eye
    if(camera) {
        addressToSend=0x000000060;
        if(data) {
            addressToSend += 0x01;
        }
        if(clock) {
            addressToSend += 0x02;
        }
        if (latch) {
            addressToSend += 0x04;
        }
        if (powerdown) {
            addressToSend += 0x08;
        }
        addressToSend+=0xFF000000;
    }
    else {
        addressToSend=0x000000006;
        if(data) {
            addressToSend += 0x10;
        }
        if(clock) {
            addressToSend += 0x20;
        }
        if (latch) {
            addressToSend += 0x40;
        }
        if (powerdown) {
            addressToSend += 0x80;
        }
        addressToSend+=0xFF000000;
    }

    u32 hwival = (u32)(timeToSend * 7.8125);
    //printf("saving the aer.address %x \n", addressToSend);
    pseq[seqEvents].address = addressToSend;
    //printf("saving the aer.time  \n");
    pseq[seqEvents].timestamp = hwival;

    seqEvents++;
    //printf("number of saved events %d as tot %d ",seqEvents,seqSize_b);
    seqTime += hwival;
    seqSize_b += sizeof(struct aer);

    //printf("writing the event");
    //int w = write(file_desc, ((u8*)pseq), seqSize_b);
    //addr = int(sys.argv[1], 16)
    //err = write(file_desc,t,4); //4 byte time: 1 integer
    //err = write(file_desc,addr,4); //4 byte time: 1 integer
}

bool asvGrabberThread::setDumpFile(std::string value) {
    dumpfile = value;
    //fout.open(dumpfile.c_str());
    //bool ret = fout.is_open();
    //if (!ret)
    //    cout << "unable to open file" << endl;

    fout = fopen(dumpfile.c_str(),"w");
    if(fout!=NULL)
        return true;
    else
        return false;
}


void asvGrabberThread::threadRelease() {
    /* it is better not to set the powerdown at the end!
    const u32 seqAllocChunk_b = SIZE_OF_EVENTS * sizeof(struct aer); //allocating the right dimension for biases
    memset(pseq,0,seqAllocChunk_b);
    setPowerdown();
    sendingBias();
    */

    port.close();
    close(file_desc);
}