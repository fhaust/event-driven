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
 * @file cartesianFrameConverter.cpp
 * @brief A class inherited from the bufferefPort (see header cartesianFrameConverter.h)
 */

#include <iCub/cartesianFrameConverter.h>
#include <cassert>
#include <cstdlib>


//#define CHUNKSIZE 8192
//#define TH1 8192 
//#define TH2 16384
//#define TH3 24576
//#define BUFFERDIM 32768 

#define CHUNKSIZE 32768 
#define TH1       32768  
#define TH2       65536
#define TH3       98304
#define BUFFERDIM 131072

//#define VERBOSE

//#define CHUNKSIZE 1024 
//#define TH1       1024
//#define TH2       2048
//#define TH3       3072
//#define BUFFERDIM 4096

using namespace yarp::os;
using namespace yarp::sig;
using namespace emorph::ebuffer;
using namespace std;


cFrameConverter::cFrameConverter() {
    valid       = false;
    retinalSize = 128;
    totDim = 0;
    pcRead = 0;
    state  = 0;
    receivedBuffer = 0;
    printf ("allocating memory \n");
    converterBuffer_copy = (char*) malloc(BUFFERDIM); // allocates bytes
    converterBuffer = converterBuffer_copy;
    if (converterBuffer == 0) {
        printf("null pointer \n");
    }
    pcBuffer = converterBuffer;
    printf("setting memory \n");
    memset(converterBuffer,0,BUFFERDIM);              // set unsigned char
    pcRead = converterBuffer;
    //unmask_events.start();
    printf("unmask event just started");
    previousTimeStamp = 0;
    readEvents = fopen("./eventFeatureExtractor.readEvents.txt","w+");
}

cFrameConverter::~cFrameConverter() {
    printf("cFrameConverter:stopping the unmasker \n");
    //unmask_events.stop();
    //delete &unmask_events;
    //delete &convert_events;
    printf("cFrameConverter:freeing converterBuffer \n");
    free(converterBuffer_copy);
    printf("cFrameConverter:success in freeing converterBuffer \n");
}

void cFrameConverter::copyChunk(char* bufferCopy) {        
    mutex.wait();
    if(pcRead >= converterBuffer +  BUFFERDIM - CHUNKSIZE) {
        memcpy(bufferCopy, pcRead, converterBuffer + BUFFERDIM - pcRead );
        pcRead = converterBuffer;
    }
    else {
        memcpy(bufferCopy, pcRead, CHUNKSIZE);
        pcRead += CHUNKSIZE;
    }
    mutex.post();
}

// reading out from a circular buffer with 2 entry points and wrapping
void cFrameConverter::onRead(eventBuffer& i_ub) {
    valid = true;

    // receives the buffer and saves it
    int dim = i_ub.get_sizeOfPacket() ;      // number of bits received / 8 = bytes received
    //printf("dim %d \n", dim);
    //receivedBufferSize = dim;
    mutex.wait();
    receivedBuffer = i_ub.get_packet(); 
    u32* buf1 = (u32*)receivedBuffer;
    char*  receivedBuffer_char = (char*) receivedBuffer;
    //printf("received buffer \n");
    
    // check for bytes lost. If the first byte is not 0x80 there is an error
    if(*buf1 < 0x80) {
        //printf("wrong packet \n");
        if(VERBOSE) {
            fprintf(readEvents,"wrong packet \n");
            fprintf(readEvents,"----------------------- \n");
        }
        receivedBuffer_char -= 5;
        //printf("wrong packet \n");
        //return;
        mutex.post();
    }

    //printf("received values \n");

    if(VERBOSE) {
        int num_events = dim >> 3 ;
        u32* buf2 = (u32*)receivedBuffer_char;
        //plotting out
        for (int evt = 0; evt < num_events; evt++) {
            unsigned long t     = buf2[2 * evt];
            unsigned long blob  = buf2[2 * evt + 1];
            fprintf(readEvents,"%08X %08X \n",t, blob);        
        }
        fprintf(readEvents,"----------------------- \n");
    }



    // the thrid part of the buffer is free to avoid overflow
    //totDim += dim;
    int overflow = 0;      
    int removeLater=0;
    
    int status = 0;
    
    if(totDim < TH1 && (totDim+dim) > TH1){
       pcRead = converterBuffer+TH2;
       status = 1;
       memcpy(pcBuffer,receivedBuffer,dim);
       status = 2;
       pcBuffer += dim;
       totDim   += dim;
       removeLater = 1; 
    }
    else if(totDim < TH2 && (totDim + dim) > TH2){
        pcRead = converterBuffer;
        status = 3;
        memcpy(pcBuffer,receivedBuffer,dim);
        status = 4;
        pcBuffer += dim;
        totDim   += dim;
        removeLater = 2; 
    }
    else if((totDim + dim) > TH3){
        pcRead   = converterBuffer + TH1;
        overflow = totDim+dim - TH3;
        status = 5;
        memcpy(pcBuffer,receivedBuffer,dim-overflow);
        status = 6;
        //wrap overflown
        memcpy(converterBuffer,receivedBuffer - overflow + dim, overflow);  
        status = 7;      
        pcBuffer = converterBuffer + overflow;
        pcRead = converterBuffer + TH2;
        totDim = overflow;
        removeLater = 3; 
    }
    else { // general case where no boundaries are crossed
    status = 8;
        memcpy(pcBuffer,receivedBuffer,dim);
        status = 9;
        pcBuffer += dim;
        totDim += dim;
    }
    mutex.post();

    //printf("onRead: ended \n");
    //printf("pcBuffer: 0x%x pcRead: 0x%x \n", pcBuffer, pcRead); 
}



/*
// reading out from a circular buffer with 2 entry points
void cFrameConverter::onRead(eventBuffer& i_ub) {
    valid = true;
    //printf("onRead ");
    // receives the buffer and saves it
    int dim = i_ub.get_sizeOfPacket() ;      // number of bits received / 8 = bytes received
    //printf("dim %d \n", dim);
   
    mutex.wait();
    receivedBuffer = i_ub.get_packet();    
    memcpy(pcBuffer,receivedBuffer,dim);
    
    if (totDim < TH1) {
        pcBuffer += dim;
    }
    else if((totDim>=TH1)&&(totDim<TH2)&&(state!=1)){
        //printf("greater than TH1 \n");
        pcBuffer = converterBuffer + TH1; 
        pcRead = converterBuffer + TH2;
        state = 1;
    }
    else if(totDim >= TH2) {
        //printf("greater that TH2 \n");
        pcBuffer = converterBuffer;
        pcRead = converterBuffer + TH1;
        totDim = 0;
        state = 0;
    }
    // the thrid part of the buffer is free to avoid overflow
    totDim += dim;

    mutex.post();
    //printf("onRead: ended \n");
    //printf("pcBuffer: 0x%x pcRead: 0x%x \n", pcBuffer, pcRead); 
}
*/


/*
// reading out from a circular buffer with 3 entry points
void cFrameConverter::onRead(eventBuffer& i_ub) {
    valid = true;
    //printf("onRead ");
    // receives the buffer and saves it
    int dim = i_ub.get_sizeOfPacket() ;      // number of bits received / 8 = bytes received
    //printf("dim %d \n", dim);
   
    mutex.wait();
    receivedBuffer = i_ub.get_packet();

 
    memcpy(pcBuffer,receivedBuffer,dim);
    
    
    pcBuffer += dim;
    totDim += dim;
    
    if((totDim>=TH1)&&(totDim<TH2)&&(state!=1)){
        pcBuffer = converterBuffer + TH1; 
        pcRead = converterBuffer + TH2;
        state = 1;
    }else if((totDim >= TH2)&&(totDim < TH3)&&(state!=2)) {
        pcBuffer = converterBuffer + TH2;
        pcRead   = converterBuffer;       
        state    = 2;
    }
    else if(totDim >= TH3) {
        pcBuffer = converterBuffer;
        pcRead   = converterBuffer + TH1;
        totDim   = 0;
        state    = 0;
    }
    // after the thrid part of the buffer is free to avoid overflow
     

    mutex.post();
    //printf("onRead: ended \n");
    //printf("pcBuffer: 0x%x pcRead: 0x%x \n", pcBuffer, pcRead);
   
}
*/


/*
void cFrameConverter::onRead(eventBuffer& i_ub) {
    // receives the buffer and saves it
    //cout << "C_yarpViewer::onRead(unmaskedbuffer& i_ub)" << endl;
    //start_u = clock();
    //i_ub.get_sizeOfPacket() size of the packet in bits
    unmask_events.unmaskData(i_ub.get_packet(), i_ub.get_sizeOfPacket());
    //start_p = clock();
    //stop = clock();
}
*/

void cFrameConverter::resetTimestamps() {
    //unmask_events.resetTimestamps();
}

void cFrameConverter::getMonoImage(ImageOf<PixelMono>* image, unsigned long minCount, unsigned long maxCount, bool camera){
    assert(image!=0);
    image->resize(retinalSize,retinalSize);
    unsigned char* pImage = image->getRawImage();
    int imagePadding = image->getPadding();
    int imageRowSize = image->getRowSize();
    
    // determining whether the camera is left or right
    int* pBuffer = 0;
    //int* pBuffer = unmask_events.getEventBuffer(camera);
    
    unsigned long* pTime = 0;
    //unsigned long* pTime   = unmask_events.getTimeBuffer(camera);
    
    //printf("timestamp: min %d    max %d  \n", minCount, maxCount);
    //pBuffer += retinalSize * retinalSize - 1;
    for(int r = 0 ; r < retinalSize ; r++){
        for(int c = 0 ; c < retinalSize ; c++) {
            //drawing the retina and the rest of the image separately
            int value = *pBuffer;
            unsigned long timestampactual = *pTime;
            if (((timestampactual * 1.25) > minCount)&&((timestampactual * 1.25) < maxCount)) {   //(timestampactual != lasttimestamp)
                *pImage++ = (unsigned char) 127 + value;
               
            }
            else {
                *pImage++ = (unsigned char) 127;
               
                }
            pBuffer ++;
            pTime ++;
        }
        pImage+=imagePadding;
    }
    //unmask_events.setLastTimestamp(0);
}

unsigned long cFrameConverter::getLastTimeStamp() {
    //return unmask_events.getLastTimestamp();
	return 0;
}

unsigned long cFrameConverter::getLastTimeStampRight() {
    //return unmask_events.getLastTimestampRight();
	return 0;
}

unsigned long cFrameConverter::getEldestTimeStamp() {
    //return unmask_events.getEldestTimeStamp();
	return 0;
}

void cFrameConverter::clearMonoImage() {
    //unmask_events.cleanEventBuffer();
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------

