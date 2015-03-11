// Put professional header here
// Include some type of copyright


const int maxCamServoAngle = 159;


// Reads the serial port for a computer command and sets the given int values
void readComputerCommands(int *camServoSetpoint) {
    char incomingByte;
    int tabCounter = 0;
    int camServoCmd = 0;
    while (Serial.available()) {
        incomingByte = Serial.read();
        if (incomingByte == '\t') {
            if (tabCounter == 0) {
                camServoCmd = Serial.parseInt();  // 0 when passed non-int
            }
            tabCounter++;
        }
    }
    if (tabCounter == 8) {
        *camServoSetpoint = boundCamServoCmd(camServoCmd);
    }
}


int boundCamServoCmd(int cmd) {
    // Decided to treat out-of-bound commands as illegitimate to prevent
    // runaways. Could implement this as a clamp function as needed
    if (abs(cmd) > maxCamServoAngle) {
        return 0;
    } else {
        return cmd;
    }
}


// Prints given string to the computer, tab separated
void printDataToComputer(int camServoAngle, int isPaused) {
    Serial.print("CAMSRV\t"); Serial.print(camServoAngle);
    Serial.print("\tPAUSE\t"); Serial.println(isPaused);
}
