int LED = 13;
int camServoSetpoint = 0;

void setup() {
    Serial.begin(9600);
    pinMode(LED, OUTPUT);
}

void loop() {
    Serial.println("About to read CMD");
    readComputerCommands(&camServoSetpoint);
//    Serial.print("camServoSetpoint: "); Serial.println(camServoSetpoint);
    if (camServoSetpoint > 50) {
        digitalWrite(LED, HIGH);
    } else {
        digitalWrite(LED, LOW);
    }
    
    delay(500);
    digitalWrite(LED, HIGH);
    delay(75);
    digitalWrite(LED, LOW);
    delay(75);
    digitalWrite(LED, HIGH);
    delay(75);
    digitalWrite(LED, LOW);
    delay(75);
}

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
//    Serial.print("tabCounter: "); Serial.println(tabCounter);
    if (tabCounter == 2) {
        *camServoSetpoint = boundCamServoCmd(camServoCmd);
    }
}

int boundCamServoCmd(int cmd) {
    // Decided to treat out-of-bound commands as illegitimate to prevent
    // runaways. Could implement this as a clamp function as needed
    if (cmd == 255) {
        return 255;  // Used to trigger the "search for fiducial" sweep
    } else if (abs(cmd) > 160) {
        return 0;
    } else {
        return cmd;
    }
}
