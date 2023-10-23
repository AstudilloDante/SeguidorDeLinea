#include <OrangutanMotors.h>
#include <QTRSensors.h>

OrangutanMotors motors;
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
int val = 0;
const int maximum = 150;  //Valor de motores
float VProporcional = 0.015;
float VDerivativo = 0.2;
float Vintegral = 0.0003;
float Vreductor = 0.96;
unsigned int last_Error = 0;
long integral = 0;

void setup() {
  pinMode(9, OUTPUT);  //led verde
  pinMode(8, OUTPUT);  // led  rojo
  pinMode(10, INPUT_PULLUP);// 
  Serial.begin(9600);//iniciamos
  
}

void loop() {
  digitalWrite(8, HIGH);//luz roja, que se esta calibrando
  for(int recalibracion = 1;recalibracion < 4; recalibracion++)//calibramos unas 3 veces para evitar problemas
  {
    calibracion();
  }
  digitalWrite(8, LOW);
  digitalWrite(9, HIGH);//Encendemos luz verde de que ya se calibro correctamente
  while(val == LOW )    //listo para la carrera
  { 
    ConfiguracionPID();//PID
    val=digitalRead(10);//una ves termine la carrera presionamo para apagar los motores
  }
  motors.setSpeeds(0,0);

}



void ConfiguracionPID()
{
  uint16_t position = qtr.readLineBlack(sensorValues);

  int Error = (int)position - 3500;  // Valor promedio si los dos sensores en medio estan sobre la linea negra es 3500
  
  if (Error > -100 && Error < 100) return 0;// pequeñas variaciones pueden hacer que tambalee

  int derivative = Error - last_Error;
  integral += Error;
  last_Error = Error;

  float power_difference = abs(Error * VProporcional + integral * Vintegral + derivative * VDerivativo);
  if (power_difference>= 250)
    power_difference=250;

  if (power_difference = 0)//
    motors.setSpeeds(250, 250);

//si la pista no tiene giros de 90 grados pueden hacer comentarios los 'if'
  if (position<2100) 
    motors.setSpeeds(0,250);//90 grados a la izquierda
  else if  (position>5200)//
    motors.setSpeeds(250,0);//90 grados a la derecha
//
  if (power_difference < 0)
    motors.setSpeeds(maximum * Vreductor, maximum + power_difference);  
  else
    motors.setSpeeds(maximum + power_difference, maximum * Vreductor);
  
}

void calibracion ()
{
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  
  motors.setSpeeds(0, 0);
}
