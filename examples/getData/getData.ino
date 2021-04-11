#include <xSI02.h>
#include <Wire.h>

xSI02 SI02;

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    SI02.begin();
}

void loop()
{
    SI02.read();
    Serial.println(SI02.Roll());
    Serial.println(SI02.Pitch());
    Serial.println(SI02.GForce());

    Serial.println(SI02.getMX());
    Serial.println(SI02.getMY());
    Serial.println(SI02.getMZ());

    delay(100);
}