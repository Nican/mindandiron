// by Matthias Grob & Manuel Stalder - ETH Zürich - 2015
#include "mbed.h"
#include "PC.h"                                     // Serial Port via USB for debugging with Terminal
#include "DW1000.h"                                 // our DW1000 device driver
#include "MMRanging.h"                              // our self developed raning application
 
PC          pc(USBTX, USBRX, 921600);               // USB UART Terminal
DW1000      dw(PA_7, PA_6, PA_5, PB_6, PB_9);       // SPI1 on Nucleo Board (MOSI, MISO, SCLK, CS, IRQ)
MMRanging   r(dw);                                  // Ranging class for getting distances and later positions
 
char message[100] = "";
 
int main() {
    pc.printf("DecaWave 0.2\r\nup and running!\r\n");  
    dw.setEUI(0xFAEDCD01FAEDCD01);                  // basic methods called to check if we have a working SPI connection
    pc.printf("DEVICE_ID register: 0x%X\r\n", dw.getDeviceID());
    pc.printf("EUI register: %016llX\r\n", dw.getEUI());
    pc.printf("Voltage: %f\r\n", dw.getVoltage());
 
    r.receiver = true;
    
    while(1) {
        for(int j = 0; j < 10; j++)
            if(r.event[j][0] == '!') {
                pc.printf("%s Time: %fus\r\n", r.event[j], r.eventtimes[j]*MMRANGING_TIMEUNIT);
                r.event[j][0] = 'X';
            }    
        if (!r.receiver) {
                r.requestRanging();
                wait(1);
        }
    }
}
