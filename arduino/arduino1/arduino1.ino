// Put professional header here
// Include some type of copyright


#include <Herkulex.h>
#include <TimerOne.h>

#include "rc_includes.h"


const int camServoID = 253; //motor ID - verify your ID !
#define TX 8
#define RX 9

int camServoSetpoint = 0;
int ledState = LOW;


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

    pinMode(LED_PIN_OUT, OUTPUT);
    Timer1.initialize(1000000);  // Timer period in microseconds (max 8.3s)
    Timer1.attachInterrupt(blinkLED); // blinkLED to run every 0.15 seconds
}

void loop(){
    readComputerCommands(&camServoSetpoint);

    if (getAuto()) {
        Herkulex.moveOneAngle(camServoID, camServoSetpoint, 1000, LED_BLUE);
    } else {
        Herkulex.moveOneAngle(camServoID, 0, 1000, LED_BLUE);
    }

    printDataToComputer(Herkulex.getAngle(camServoID), getPaused());
}

// Blinks the LED when paused, leaves it solidly on while running
void blinkLED(void)
{
	if (ledState == LOW) {
		ledState = HIGH;
	} else if (getPaused()) {
		ledState = LOW;
	}
	digitalWrite(LED_PIN_OUT, ledState);
}
