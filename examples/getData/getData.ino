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
    Serial.println(SI02.getAX());
    Serial.println(SI02.getAY());
    Serial.println(SI02.getAZ());

    Serial.println(SI02.getMX());
    Serial.println(SI02.getMY());
    Serial.println(SI02.getMZ());

    delay(100);
}