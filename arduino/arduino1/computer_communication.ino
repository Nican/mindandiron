// Put professional header here
// Include some type of copyright


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
    if (tabCounter == 2) {
        *camServoSetpoint = boundCamServoCmd(camServoCmd);
    }
}


int boundCamServoCmd(int cmd) {
    // Decided to treat out-of-bound commands as illegitimate to prevent
    // runaways. Could implement this as a clamp function as needed
    if (cmd == 255) {
        return 255;  // Used to trigger the "search for fiducial" sweep
    } else if (abs(cmd) > MAX_CAM_SERVO_ANGLE) {
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
