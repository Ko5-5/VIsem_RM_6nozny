#include <Arduino.h>
#include "prox_sensor.h"

void setupProxFront()
{
    Serial.begin(9600);
    pinMode(PROX_TRIG_2, OUTPUT); // Pin, do kt�rego pod��czymy trig jako wyj�cie
    pinMode(PROX_ECHO_2, INPUT);  // a echo, jako wej�cie
}
void setupProxBack()
{
    Serial.begin(9600);
    pinMode(PROX_TRIG_1, OUTPUT); // Pin, do kt�rego pod��czymy trig jako wyj�cie
    pinMode(PROX_ECHO_1, INPUT);  // a echo, jako wej�cie
}
int zmierzOdlegloscFront()
{
    long czas = 0, dystans = 0;

    digitalWrite(PROX_TRIG_2, LOW);
    vTaskDelay(2 / portTICK_PERIOD_MS);
    // delayMicroseconds(2);
    digitalWrite(PROX_TRIG_2, HIGH);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // delayMicroseconds(10);
    digitalWrite(PROX_TRIG_2, LOW);

    czas = pulseIn(PROX_ECHO_2, HIGH);
    dystans = czas / 58;

    return dystans;
}

int zmierzOdlegloscBack()
{
    long czas = 0, dystans = 0;

    digitalWrite(PROX_TRIG_1, LOW);
    vTaskDelay(2 / portTICK_PERIOD_MS);
    // delayMicroseconds(2);
    digitalWrite(PROX_TRIG_1, HIGH);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // delayMicroseconds(10);
    digitalWrite(PROX_TRIG_1, LOW);

    czas = pulseIn(PROX_ECHO_1, HIGH);
    dystans = czas / 58;

    return dystans;
}