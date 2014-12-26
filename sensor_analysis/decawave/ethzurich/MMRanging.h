// by Matthias Grob & Manuel Stalder - ETH Zürich - 2015
 
#ifndef MMRANGING_H
#define MMRANGING_H
 
#include "mbed.h"
#include "DW1000.h"
 
#define MMRANGING_TIMEUNIT      1/(128*499.2)               // conversion between LSB of TX and RX timestamps and microseconds
 
class MMRanging {
    public:
        MMRanging(DW1000& DW);
        void callbackRX();
        void callbackTX();
        void requestRanging();
    //private:
        DW1000& dw;
        
        // draft for first test
        bool receiver;
        char message[1021];
        char messageRX[1021];
        uint64_t TX_timestamp;
        uint64_t RX_timestamp;
        int event_i;
        char event[10][20];
        uint64_t eventtimes[10];
        uint8_t counter;
};
 
#endif
