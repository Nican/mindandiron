// Put professional header here
// Include some type of copyright


const int maxWheelSpeed = 160;


// Reads the serial port for a computer command and sets the given int values
void readComputerCommands(int *leftVelocitySetpoint,
                          int *rightVelocitySetpoint,
                          int *collectorAutoCmd,
                          int *sorterAutoSlot) {
    char incomingByte;
    int tabCounter = 0;
    int leftVelocityCmd = 0;
    int rightVelocityCmd = 0;
    int collectorCmd = 0;
    int sorterCmd = 0;
    while (Serial.available()) {
        incomingByte = Serial.read();
        if (incomingByte == '\t') {
            if (tabCounter == 0) {
                leftVelocityCmd = Serial.parseInt();  // 0 when passed non-int
            } else if (tabCounter == 2) {
                rightVelocityCmd = Serial.parseInt();
            } else if (tabCounter == 4) {
                collectorCmd = Serial.parseInt();
            } else if (tabCounter == 6) {
                sorterCmd = Serial.parseInt();
            }
            tabCounter++;
        }
    }
    if (tabCounter == 8) {
        *leftVelocitySetpoint = boundVelocity(leftVelocityCmd);
        *rightVelocitySetpoint = boundVelocity(rightVelocityCmd);
        *collectorAutoCmd = boundCollectorSignal(collectorCmd);
        *sorterAutoSlot = boundSorterSignal(sorterCmd);
    }
}


int boundVelocity(int cmd) {
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
                         int aX, int aY, int aZ,
                         int isAuto) {
    // LVEL and RVEL are in ticks/sec
    Serial.print("LPOS\t"); Serial.print(leftPosition);
    Serial.print("\tRPOS\t"); Serial.print(rightPosition);
    Serial.print("\tLVEL\t"); Serial.print(leftVelocity);
    Serial.print("\tRVEL\t"); Serial.print(rightVelocity);
    Serial.print("\tAX\t"); Serial.print(aX);
    Serial.print("\tAY\t"); Serial.print(aY);
    Serial.print("\tAZ\t"); Serial.print(aZ);
    Serial.print("\tAUTO\t"); Serial.println(isAuto);
}
