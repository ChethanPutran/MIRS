
#include "max6675.h"

int ktcSO = 8;
int ktcCS = 9;
int ktcCLK = 10;

MAX6675 ktc(ktcCLK, ktcCS, ktcSO);

String dataLabel = "Voltage(V)";

float threshold = 0.05;
float celcius, fh, previousCData, previousFData;

void setup()
{
    Serial.begin(9600);
    delay(500);
}

void loop()
{
    celcius = ktc.readCelsius();
    fh = ktc.readFahrenheit();

    if ((celcius <= previousCData - threshold) || (celcius >= previousCData + threshold))
    {
        Serial.print(celcius);
        Serial.print(',');
        Serial.println(fh);
    }
    previousCData = celcius;
    previousFData = fh;

    delay(1000);
    Serial.flush();
}
