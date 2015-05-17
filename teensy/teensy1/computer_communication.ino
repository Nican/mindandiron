// Put professional header here
// Include some type of copyright


const float maxWheelSpeed = 0.65;  // meters/second


// Reads the serial port for a computer command and sets the given int values
void readComputerCommands(float *leftVelocitySetpoint,
                          float *rightVelocitySetpoint,
                          int *leftPositionSetpoint,
                          int *rightPositionSetpoint,
                          int *velocityPID,
                          int *collectorAutoCmd,
                          int *sorterAutoSlot) {
    char incomingByte;
    int tabCounter = 0;
    float leftVelocityCmd = 0;
    float rightVelocityCmd = 0;
    int leftPositionCmd = 0;
    int rightPositionCmd = 0;
    int velocityPIDCmd = 1;
    int collectorCmd = 0;
    int sorterCmd = 0;

    // Format
    // \tLVEL\tRVEL\tLPOS\tRPOS\tVEL?\tCOLL\tSORT\tEND
    while (Serial.available()) {
        incomingByte = Serial.read();
        if (incomingByte == '\t') {
            if (tabCounter == 0) {
                leftVelocityCmd = Serial.parseFloat(); // 0 when passed non-float
            } else if (tabCounter == 1) {
                rightVelocityCmd = Serial.parseFloat();
            } else if (tabCounter == 2) {
                leftPositionCmd = Serial.parseInt();
            } else if (tabCounter == 3) {
                rightPositionCmd = Serial.parseInt();
            } else if (tabCounter == 4) {
                velocityPIDCmd = Serial.parseInt();
            } else if (tabCounter == 5) {
                collectorCmd = Serial.parseInt();      // 0 when passed non-int
            } else if (tabCounter == 6) {
                sorterCmd = Serial.parseInt();
            }
            tabCounter++;
        }
    }

    if (tabCounter == 8) {
        *leftVelocitySetpoint = boundVelocity(leftVelocityCmd);
        *rightVelocitySetpoint = boundVelocity(rightVelocityCmd);
        *leftPositionSetpoint = leftPositionCmd;
        *rightPositionSetpoint = rightPositionCmd;
        *velocityPID = velocityPIDCmd;
        *collectorAutoCmd = boundCollectorSignal(collectorCmd);
        *sorterAutoSlot = boundSorterSignal(sorterCmd);
    }
}


float boundVelocity(float cmd) {
    // Decided to treat out-of-bound commands as illegitimate to prevent
    // runaways. Could implement this as a clamp function as needed
    if (abs(cmd) > maxWheelSpeed) {
        return 0;
    } else {
        return cmd;
    }
}


int boundCollectorSignal(int cmd) {
    // Decided to treat out-of-bound commands as illegitimate. Could
    // implement this as a clamp function as needed
    if (abs(cmd) > 1) {
        return 0;
    } else {
        return cmd;
    }
}


int boundSorterSignal(int cmd) {
    // Decided to treat out-of-bound commands as illegitimate. Could
    // implement this as a clamp function as needed
    if (cmd >= 0 && cmd < NUM_SORTER_SLOTS) {
        return cmd;
    } else {
        return 0;
    }
}


// Prints given string to the computer, tab separated
void printDataToComputer(int leftPosition, int rightPosition,
                         int leftVelocity, int rightVelocity,
                         int slot, int aX, int aY, int aZ,
                         int isAuto) {
    // LVEL and RVEL are in ticks/sec
    Serial.print("LPOS\t"); Serial.print(leftPosition);
    Serial.print("\tRPOS\t"); Serial.print(rightPosition);
    Serial.print("\tLVEL\t"); Serial.print(leftVelocity);
    Serial.print("\tRVEL\t"); Serial.print(rightVelocity);
    Serial.print("\tSLOT\t"); Serial.print(slot);
    Serial.print("\tAX\t"); Serial.print(aX);
    Serial.print("\tAY\t"); Serial.print(aY);
    Serial.print("\tAZ\t"); Serial.print(aZ);
    Serial.print("\tAUTO\t"); Serial.println(isAuto);
}
