#include <Arduino.h>
#include <math.h>

const int buttonPin = 2;
const unsigned long sampleIntervalMs = 20;
unsigned long lastSampleMs;

void printSyntheticImu(int id, unsigned long timestampMs, float ax, float ay,
                       float az, const float gyro[3]) {
    // IMU,<id>,<millis>,<ax>,<ay>,<az>,<gx>,<gy>,<gz> (g and degrees/s).
    Serial.print("IMU,");
    Serial.print(id);
    Serial.print(',');
    Serial.print(timestampMs);
    Serial.print(',');
    Serial.print(ax, 5);
    Serial.print(',');
    Serial.print(ay, 5);
    Serial.print(',');
    Serial.print(az, 5);
    for (int axis = 0; axis < 3; ++axis) {
        Serial.print(',');
        Serial.print(gyro[axis], 5);
    }
    Serial.println();
}

void streamSyntheticImu(int id, unsigned long now, float direction) {
    // Translate along X, Y, Z, then rotate about X, Y, Z. Each stage
    // starts with one second at rest and ends back at the starting pose.
    const int stage = (now / 5000UL) % 6;
    const int axis = stage % 3;
    const unsigned long stageMs = now % 5000UL;
    float accel[3] = {0, 0, 1};
    float gyro[3] = {0, 0, 0};
    if (stageMs >= 1000UL) {
        const float rate = 2.0f * PI / 4.0f;
        const float phase = rate * ((stageMs - 1000UL) / 1000.0f);
        const float sine = sinf(phase);
        const float cosine = cosf(phase);
        if (stage < 3) {
            // Second derivative of direction * 0.15 * (1 - cos(phase))^2.
            accel[axis] += direction * 0.30f * rate * rate
                           * (sine * sine + (1.0f - cosine) * cosine) / 9.80665f;
        } else {
            const float angle = direction * (PI / 6.0f) * (1.0f - cosine);
            gyro[axis] = direction * 30.0f * rate * sine;
            // Express world +Z gravity compensation in the rotating sensor frame.
            if (axis < 2) {
                accel[1 - axis] = (axis == 0 ? 1.0f : -1.0f) * sinf(angle);
                accel[2] = cosf(angle);
            }
        }
    }
    printSyntheticImu(id, now, accel[0], accel[1], accel[2], gyro);
}

void setup() {
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    lastSampleMs = millis() - sampleIntervalMs;
}

void loop() {
    bool pressed = digitalRead(buttonPin) == LOW;
    digitalWrite(LED_BUILTIN, pressed ? HIGH : LOW);

    const unsigned long now = millis();
    if (now - lastSampleMs >= sampleIntervalMs) {
        lastSampleMs = now;
        Serial.println(pressed ? 1 : 0);
        streamSyntheticImu(1, now, 1.0f);
        streamSyntheticImu(2, now, -1.0f);
    }
}
