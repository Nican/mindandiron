// Put professional header here
// Include some type of copyright

void setupRC() {
    pinMode(PAUSE_IN_PIN, INPUT);
    pinMode(AUTO_IN_PIN, INPUT);
    pinMode(LIGHT_OUT_PIN, OUTPUT);
    digitalWrite(LIGHT_OUT_PIN, LOW);
}


int getPaused() {
    return !digitalRead(PAUSE_IN_PIN);  // PAUSE _IN_PIN high when running, low when paused
}


int getAuto() {
    int paused = getPaused();  // PAUSE _IN_PIN high when running, low when paused
    int auto_on = digitalRead(AUTO_IN_PIN);
    return auto_on && !paused;
}


void controlLight(int on) {
    if (on == 0) {
        digitalWrite(LIGHT_OUT_PIN, LOW);
    } else {
        digitalWrite(LIGHT_OUT_PIN, HIGH);
    }
}
