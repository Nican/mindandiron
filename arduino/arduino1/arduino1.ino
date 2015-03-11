// Put professional header here
// Include some type of copyright


#include <Herkulex.h>

#include "rc_includes.h"


const int camServoID = 253; //motor ID - verify your ID !
#define TX 8
#define RX 9


int camServoSetpoint = 0;


void setup()
{
    Serial.begin(9600);    // Open serial communications
    setupRC();

    Herkulex.begin(115200, TX, RX);
    Herkulex.reboot(camServoID); //reboot first motor
    delay(500); 
    Herkulex.initialize(); //initialize motors
    delay(200);
    Herkulex.moveOneAngle(camServoID, camServoSetpoint, 1000, LED_GREEN);
    delay(2000);
}

void loop(){
    readComputerCommands(&camServoSetpoint);

    // TODO (Eric): Replace this with light that flashes when running, is on when paused
    controlLight(!getPaused());

    // if (getAuto()) {
    Herkulex.moveOneAngle(camServoID, camServoSetpoint, 1000, LED_BLUE);
    // } else {
    //     Herkulex.moveOneAngle(camServoID, -159, 1000, LED_BLUE);
    // }

    printDataToComputer(Herkulex.getAngle(camServoID), getPaused());
}
