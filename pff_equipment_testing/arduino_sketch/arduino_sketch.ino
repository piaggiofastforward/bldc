#define PERIOD          10  // period in ms
#define STRING_POT_PIN  A0
void setup() 
{
  Serial.begin(115200);
}
unsigned long last_sample_time = millis();
void loop ()
{
  unsigned int time_now = millis();
  if ((time_now - last_sample_time) >= PERIOD)
    Serial.println(analogRead(STRING_POT_PIN));
}
