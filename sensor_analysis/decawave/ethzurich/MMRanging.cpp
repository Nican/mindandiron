#include "MMRanging.h"
 
MMRanging::MMRanging(DW1000& DW) : dw(DW){
    MMRanging::dw = dw;
    
    message[0] = '\0';
    messageRX[0] = '\0';
    event_i = 0;
    counter = 0;
    
    dw.setCallbacks(this, &MMRanging::callbackRX, &MMRanging::callbackTX);
    dw.startRX();
}
 
void MMRanging::callbackRX() {
    RX_timestamp = dw.getRXTimestamp();
    dw.receiveString(messageRX);
    if (receiver) {
        message[0] = 'A';                               // acknowledge messages
        for(int i = 0; i < 10; i++)
            message[i+1] = messageRX[i];
        dw.sendString(message);
    }
    eventtimes[event_i] = RX_timestamp - TX_timestamp;                      // TODO: can give some wrong values because of timer reset after 17 seconds
    event[event_i][0] = '!';
    event[event_i][1] = 'R';
    event[event_i][2] = ' ';
    for(int i = 0; i < 10; i++)
        event[event_i][i+3] = messageRX[i];
    if (event_i == 8)
        event_i = 0;
    else
        event_i++;
    dw.startRX();
}
 
void MMRanging::callbackTX() {
    TX_timestamp = dw.getTXTimestamp();
    eventtimes[event_i] = 0;
    event[event_i][0] = '!';
    event[event_i][1] = 'S';
    event[event_i][2] = ' ';
    for(int i = 0; i < 10; i++)
        event[event_i][i+3] = message[i];
    if (event_i == 8)
        event_i = 0;
    else
        event_i++;
}
 
void MMRanging::requestRanging() {
    sprintf(message, "%d", counter);                  // send numbers to acknowledge
    counter++;
    dw.sendString(message);
}
