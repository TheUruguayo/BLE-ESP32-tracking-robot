#include "my_library.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>



//Display
#define SCREEN_WIDTH 128      // OLED display width, in pixels
#define SCREEN_HEIGHT 64      // OLED display height, in pixels
// Declaration for SSD1306 display connected using software SPI (default case):
#define OLED_MOSI  23
#define OLED_CLK   18
#define OLED_DC    4 //4
#define OLED_CS    5
#define OLED_RESET 17
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

bool mostrarprints = false;

////////////////Variable para los motores/L298N
// Motor B//iz mirando de atras
int motorBpin1 = 22;//27
int motorBpin2 = 26;//26
int enableBpin = 16;//14

// Motor A//der mirando de atras
int motorApin1 = 33;              //Pin1 del driver
int motorApin2 = 32;              //Pin2 del driver
int enableApin = 25;

// Setting PWM properties
const int freq = 30000;
const int pwmChannelA = 0;
const int pwmChannelB = 1;
const int resolution = 8;
int dutyCycle = 140;

///////////////////variables para

int sensorPinIz = 35; //el out del sensor esta en el pin 34
int sensorPinPowerIz = 19; //el vcc del sensor esta en el pin 22
//int sensorPinAde = 13 ;//el out del sensor esta en el pin 35
//int sensorPinPowerAde = 12 ; //el out del sensor esta en el pin 35
int sensorPinDer = 34;//el out del sensor esta en el pin 35
int sensorPinPowerDer  = 21;//el power del sensor esta en el pin 21
int sensorValue = HIGH; //high es no , me parece poco saludable
//sonido
// variables del los HC-sr04
#define echoPin 27 // attach pin D2 Arduino to pin Echo of HC-SR04  19
#define trigPin 14 //attach pin D3 Arduino to pin Trig of HC-SR04
#define echoPin2 12
#define trigPin2 13
long duration = 0; // variable for the duration of sound wave travel
int distance = 0; // variable for the distance measurement
long duration2 = 0;
int distance2 = 0;
//*****************************************bandera global
bool obstacle = false;// evita que se active parar varias veces, indica que hay alguien delante



////////////////////////////////////////////Funciones RELACIONADAS A LOS \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
//**************************************************************************************************************************\\

int leerSensor(int pinsensor, int powerPinSensor) {
  pinMode(powerPinSensor, OUTPUT);//gpio 22 es output
  digitalWrite(powerPinSensor, HIGH); //gpio 22 es high,1,3.3v
  delay(20);// no gusta
  //esperoMillis(20);
  pinMode(pinsensor, INPUT);//este esta conectado al out del sensor
  int salida = digitalRead(pinsensor);
  digitalWrite(powerPinSensor, LOW); //gpio 22 es high,1,3.3v

  return salida;
}
void initSensorSonido() {
  pinMode(trigPin, OUTPUT);   // Sets the trigPin as an OUTPUT
  pinMode(trigPin2, OUTPUT);  // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT);    // Sets the echoPin as an INPUT
  pinMode(echoPin2, INPUT);   // Sets the echoPin as an INPUT
  //Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
  //Serial.println("with Arduino UNO R3");
}
int leerSensorSonido() {
  int salida = HIGH;
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);

  // Calculating the distance
  distance = duration * 0.034 / 2;        // Speed of sound wave divided by 2 (go and back)
  distance2 = duration2 * 0.034 / 2;      // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor

  if ((distance < 22 && distance > 0) || (distance2 < 22 && distance2 > 0))  {
    salida = LOW;
  } else salida = HIGH;

  if (salida == LOW && !obstacle) {     //esto hace que no pares todo el tiempoo
    parar();
    obstacle = true;
  } else if (salida == HIGH) {
    obstacle = false;
  }
  (salida == LOW ) ? digitalWrite(2, HIGH) : digitalWrite(2, LOW);
  imprimo("Distance 1: ", String(distance));
  imprimo("Distance 2: ", String(distance2));
  return salida;
}
void leerSensores(int sensors[]) {
  sensors[0] = leerSensor(sensorPinIz, sensorPinPowerIz);         //izqueirda
  sensors[1] = leerSensor(sensorPinDer, sensorPinPowerDer);       //derecha
  sensors[2] = leerSensorSonido(); //adelante
  //  sensors[3] = leerSensor(sensorPinBack,sensorPinPowerBack);
}
int obstaculo(int sensors[]) {
  /////////////////////////////////Sensores
  leerSensores(sensors);
  int sensorValueIz = sensors[0];
  int sensorValueDer = sensors[1];
  int sensorValueAde = sensors[2];
  int sensorValueBack = HIGH; //esto es para probar nomas
  //int sensorValueDer = sensores[1];
  // int sensorValueAde = sensores[2];;
  //int sensorValueBack = sensores[3];;
  int salida = 0;
  if (sensorValueIz == LOW) {
    salida = 1;
  }
  if (sensorValueDer == LOW) {
    salida = salida + 2;
  }
  if (sensorValueAde == LOW) {
    salida = salida + 7;
  }
  if (sensorValueBack == LOW) {
    salida = salida + 11;
  }
  (salida > 0) ? digitalWrite(2, HIGH) : digitalWrite(2, LOW);
  (salida > 0) ? Serial.println("obstaculo") : Serial.println("");
  switch (salida) {
    case 1: imprimo("!!, !! Izquierda", ""); break;
    case 2: imprimo("!!, !! Derecha", ""); break;
    case 3: imprimo("!!, !! Derecha e Izqueirda", ""); break;
    case 7: imprimo("!!, !! Adelante", ""); break;
    case 8: imprimo("!!, !! Adelante e Izquierda", ""); break;
    case 9: imprimo("!!, !! Adelante y Derecha", ""); break;
    case 10: imprimo("!!, !! Adelante , Izquierda y Derecha", ""); break;
    case 11: imprimo("!!, !! Atras", ""); break;
    case 12: imprimo("!!, !! Atras e Izquierda", ""); break;
    case 13: imprimo("!!, !! Atras e Derecha", ""); break;
    case 14: imprimo("!!, !! Atras , Izquierda y Derecha", ""); break;
    case 18: imprimo("!!, !! Atras y Adelante", ""); break;
    case 19: imprimo("!!, !! Atras , Adelante  Izquierda", ""); break;
    case 20: imprimo("!!, !! Atras , Adelante y Derecha", ""); break;
    case 21: imprimo("ATASCADO COMPLETAMENTE", ""); break;
    default : Serial.println("clear"); break;
  }
  return (salida);
}
//***********************************************FUNCION EXTRAS***************************************************
//****************************************************************************************************************
double distancia(int rssia, int calibrara, int constantea) {
  double dis ;
  double pot = (calibrara - rssia) / (10 * constantea);
  dis = pow(10, dis) - 1;
  return dis;
}
int listaCircular(int rssi[], int largo) {
  int promedio = rssi[0];
  for (int i = 1; i < largo; i++) {
    promedio = rssi[i] + promedio;
  }
}
//////////////////////////
void imprimo(String mensaje1, String mensaje2) {

  if (mostrarprints) {
    Serial.print(mensaje1); Serial.println(mensaje2);
  }
}
void muestroTodo() {
  if (mostrarprints) {
    mostrarprints = false;
  } else mostrarprints = true;

}

///////////////////////////////////
#define pdTICKS_TO_MS( xTicks )   ( ( uint32_t ) ( xTicks ) * 1000 / configTICK_RATE_HZ )
static void esperarMillis(double espero )
{
  int tiempo = 0;
  TickType_t time_start = xTaskGetTickCount();
  while (1) {
    tiempo = pdTICKS_TO_MS(xTaskGetTickCount() - time_start);
    if ( tiempo > espero) {
      Serial.print("Millis transcurridos :"); Serial.println(espero);
      break;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
//***********************************************FUNCION Display**************************************************
//****************************************************************************************************************
void initDisplay(int time) { //inicializa el display
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.drawBitmap(0, 0,  Cara5, 128, 64, WHITE);
  display.display();
  delay(time);
  display.drawBitmap(0, 0,  Cara5, 128, 64, WHITE);
  display.invertDisplay(true);
  delay(time);
  display.invertDisplay(false);
  delay(time);
  display.clearDisplay();
  reloj(0);
}
void testdrawstyle(String valor, int rssi, int calibrar, int constante) {          //muestra valores
  display.clearDisplay();

  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0, 0);            // Start at top-left corner
  //display.println(F("Te voy a encontrar!"));

  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  display.println(valor);

  display.setTextSize(2);             // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  double a = distancia(rssi, calibrar, constante);
  display.print(a);
  display.println(F("m--" ));
  display.print(F("-Rssi->" ));
  display.print(rssi);
  display.print(F("-TX0->" ));
  display.print(calibrar);

  //display.print(F("--TX0" ));


  display.display();
  //delay(1000);
}
///////////
void Display(bool conectado, String valori, int rssii, int calibrari, int constantei) {

  if (conectado) {
    testdrawstyle(valori,  rssii, calibrari, constantei);
  }
  else {
    mostrarImagen(Cara1, 1000, true, "");
    mostrarImagen(Cara2, 1000, true, "");

  }
}
bool buscando=false;
bool estasBuscando(){
  return buscando;
  }
//***********************************************FUNCION IMAGEN***************************************************
//****************************************************************************************************************
void mostrarTexto(String texto) {
  display.setCursor(50, 10);            // Start at top-left corner
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setTextSize(2);               // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.print(texto[0]);display.print(texto[1]);display.print(texto[2]); display.println(F("s" ));

}
void mostrarTextoRssiAnterior(String texto) {
  display.setCursor(60, 30);            // Start at top-left corner
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setTextSize(2);               // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
   display.print(F("Ro:" ));display.print(texto[0]);display.print(texto[1]);display.println(texto[2]);

}
void mostrarTextoRssiMedido(String texto) {
  display.setCursor(60, 50);            // Start at top-left corner
  display.setTextColor(SSD1306_WHITE);  // Draw white text
  display.setTextSize(2);               // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
   display.print(F("Rp:" ));display.print(texto[0]);display.print(texto[1]);display.println(texto[2]);

}
void mostrarImagen(const unsigned char imagen [] PROGMEM, int tiempo, bool full, String texto) {
  display.clearDisplay();
  if (full)  display.drawBitmap(0, 0,  imagen, 128, 64, WHITE);
  else {
    display.drawBitmap(0, 8,  imagen, 48, 48, WHITE);

  }
  mostrarTexto(texto);
  display.display();
  vTaskDelay(tiempo / portTICK_PERIOD_MS);
}

void reloj(double segundos) {
  mostrarImagen(frame0, 37, false, String(segundos));
  mostrarImagen(frame1, 37, false, String(segundos));  mostrarImagen(frame2, 37, false, String(segundos)); mostrarImagen(frame3, 37, false, String(segundos)); mostrarImagen(frame4, 37, false, String(segundos));
  mostrarImagen(frame5, 37, false, String(segundos));  mostrarImagen(frame6, 37, false, String(segundos)); mostrarImagen(frame7, 37, false, String(segundos)); mostrarImagen(frame8, 37, false, String(segundos));
  mostrarImagen(frame9, 37, false, String(segundos)); mostrarImagen(frame10, 37, false, String(segundos)); mostrarImagen(frame11, 37, false, String(segundos)); mostrarImagen(frame12, 37, false, String(segundos));
  mostrarImagen(frame13, 37, false, String(segundos)); mostrarImagen(frame14, 37, false, String(segundos)); mostrarImagen(frame15, 37, false, String(segundos)); mostrarImagen(frame16, 37, false, String(segundos));
  mostrarImagen(frame17, 37, false, String(segundos)); mostrarImagen(frame18, 37, false, String(segundos)); mostrarImagen(frame19, 37, false, String(segundos)); mostrarImagen(frame20, 37, false, String(segundos));
  mostrarImagen(frame21, 37, false, String(segundos)); mostrarImagen(frame22, 37, false, String(segundos)); mostrarImagen(frame23, 37, false, String(segundos)); mostrarImagen(frame24, 37, false, String(segundos));
  mostrarImagen(frame25, 37, false, String(segundos)); mostrarImagen(frame26, 37, false, String(segundos)); mostrarImagen(frame27, 37, false, String(segundos));
  
}
void phone(double segundos) {
    
    mostrarImagen(phone0, 37, false, String(segundos));
    mostrarImagen(phone1, 37, false, String(segundos));  mostrarImagen(phone2, 37, false, String(segundos)); mostrarImagen(phone3, 37, false, String(segundos)); mostrarImagen(phone4, 37, false, String(segundos));
    mostrarImagen(phone5, 37, false, String(segundos));  mostrarImagen(phone6, 37, false, String(segundos)); mostrarImagen(phone7, 37, false, String(segundos)); mostrarImagen(phone8, 37, false, String(segundos));
    mostrarImagen(phone9, 37, false, String(segundos)); mostrarImagen(phone10, 37, false, String(segundos)); mostrarImagen(phone11, 37, false, String(segundos)); mostrarImagen(phone12, 37, false, String(segundos));
    mostrarImagen(phone13, 37, false, String(segundos)); mostrarImagen(phone14, 37, false, String(segundos)); mostrarImagen(phone15, 37, false, String(segundos)); mostrarImagen(phone16, 37, false, String(segundos));
    mostrarImagen(phone17, 37, false, String(segundos)); mostrarImagen(phone18, 37, false, String(segundos)); mostrarImagen(phone19, 37, false, String(segundos)); mostrarImagen(phone20, 37, false, String(segundos));
    mostrarImagen(phone21, 37, false, String(segundos)); mostrarImagen(phone22, 37, false, String(segundos)); mostrarImagen(phone23, 37, false, String(segundos)); mostrarImagen(phone24, 37, false, String(segundos));
    mostrarImagen(phone25, 37, false, String(segundos)); mostrarImagen(phone26, 37, false, String(segundos)); mostrarImagen(phone27, 37, false, String(segundos));
    
}
void calculadora(double segundos) {
 
    mostrarImagen(calculator0, 37, false, String(segundos));
    mostrarImagen(calculator1, 37, false, String(segundos));  mostrarImagen(calculator2, 37, false, String(segundos)); mostrarImagen(calculator3, 37, false, String(segundos)); mostrarImagen(calculator4, 37, false, String(segundos));
    mostrarImagen(calculator5, 37, false, String(segundos));  mostrarImagen(calculator6, 37, false, String(segundos)); mostrarImagen(calculator7, 37, false, String(segundos)); mostrarImagen(calculator8, 37, false, String(segundos));
    mostrarImagen(calculator9, 37, false, String(segundos)); mostrarImagen(calculator10, 37, false, String(segundos)); mostrarImagen(calculator11, 37, false, String(segundos)); mostrarImagen(calculator12, 37, false, String(segundos));
    mostrarImagen(calculator13, 37, false, String(segundos)); mostrarImagen(calculator14, 37, false, String(segundos)); mostrarImagen(calculator15, 37, false, String(segundos)); mostrarImagen(calculator16, 37, false, String(segundos));
    mostrarImagen(calculator17, 37, false, String(segundos)); mostrarImagen(calculator18, 37, false, String(segundos)); mostrarImagen(calculator19, 37, false, String(segundos)); mostrarImagen(calculator20, 37, false, String(segundos));
    mostrarImagen(calculator21, 37, false, String(segundos)); mostrarImagen(calculator22, 37, false, String(segundos)); mostrarImagen(calculator23, 37, false, String(segundos)); mostrarImagen(calculator24, 37, false, String(segundos));
    mostrarImagen(calculator25, 37, false, String(segundos)); mostrarImagen(calculator26, 37, false, String(segundos)); mostrarImagen(calculator27, 37, false, String(segundos));

}
void lupa(double segundos){
  mostrarImagen(lupa0, 37, false, String(segundos));
  mostrarImagen(lupa1, 37, false, String(segundos));  mostrarImagen(lupa2, 37, false, String(segundos)); mostrarImagen(lupa3, 37, false, String(segundos)); mostrarImagen(lupa4, 37, false, String(segundos));
  mostrarImagen(lupa5, 37, false, String(segundos));  mostrarImagen(lupa6, 37, false, String(segundos)); mostrarImagen(lupa7, 37, false, String(segundos)); mostrarImagen(lupa8, 37, false, String(segundos));
  mostrarImagen(lupa9, 37, false, String(segundos)); mostrarImagen(lupa10, 37, false, String(segundos)); mostrarImagen(lupa11, 37, false, String(segundos)); mostrarImagen(lupa12, 37, false, String(segundos));
  mostrarImagen(lupa13, 37, false, String(segundos)); mostrarImagen(lupa14, 37, false, String(segundos)); mostrarImagen(lupa15, 37, false, String(segundos)); mostrarImagen(lupa16, 37, false, String(segundos));
  mostrarImagen(lupa17, 37, false, String(segundos)); mostrarImagen(lupa18, 37, false, String(segundos)); mostrarImagen(lupa19, 37, false, String(segundos)); mostrarImagen(lupa20, 37, false, String(segundos));
  mostrarImagen(lupa21, 37, false, String(segundos)); mostrarImagen(lupa22, 37, false, String(segundos)); mostrarImagen(lupa23, 37, false, String(segundos)); mostrarImagen(lupa24, 37, false, String(segundos));
  mostrarImagen(lupa25, 37, false, String(segundos)); mostrarImagen(lupa26, 37, false, String(segundos)); mostrarImagen(lupa27, 37, false, String(segundos));

  
  
  }
//***********************************************FUNCION Deep Sleep***********************************************
//****************************************************************************************************************
void callbackDormir() { //esto es por si nescesitamos quese hagan cosas antes de ir a dormir
  imprimo(("entro al callback"), "");
}
void Dormir(int threshold) {
  /* Greater the value, more the sensitivity */
  touch_pad_t touchPin;
  touchAttachInterrupt(T3, callbackDormir, threshold) ;

  //Configure Touchpad as wakeup source
  esp_sleep_enable_touchpad_wakeup();
  //Go to sleep now
  Serial.println("A dormir");
  esp_deep_sleep_start();

}


void cuentaOvejas(bool &touch2detected, int threshold) {

  if (touch2detected) {
    touch2detected = false;
    int count = 0;
    bool sleep = false;
    Serial.println("en el cuentaovejas");
    while (touchRead(T3) <= threshold) {
      //Serial.println(touchRead(T3));
      count = count + 1;
      if (count == 5000) {
        imprimo(String(count / 1000), "-");
        delay(5000);//mejor mostrar animacion de 5000s
        Dormir(threshold);
      }
      (count % 1000 == 0) ? Serial.print(count / 1000) : false;
    }

    imprimo(("Toque detectado"), "");
  }
}

//***********************************************FUNCION motor***********************************************
//****************************************************************************************************************

void iniciarMotores() {
  pinMode(motorApin1, OUTPUT);
  pinMode(motorApin2, OUTPUT);
  pinMode(enableApin, OUTPUT);
  pinMode(motorBpin1, OUTPUT);
  pinMode(motorBpin2, OUTPUT);
  pinMode(enableBpin, OUTPUT);
  // configure LED PWM functionalitites
  ledcSetup(pwmChannelA, freq, resolution);
  ledcSetup(pwmChannelB, freq, resolution);
  ledcAttachPin(enableApin, pwmChannelA);
  ledcAttachPin(enableBpin, pwmChannelB);

  // attach the channel to the GPIO to be controlled

  digitalWrite(motorApin1, LOW);
  digitalWrite(motorApin2, LOW);
  digitalWrite(motorBpin1, LOW);
  digitalWrite(motorBpin2, LOW);
}
//***********************************************FUNCION MOVIMIENTO***********************************************
//****************************************************************************************************************
void moverAtras() {
  dutyCycle = 180;
  //if (dutyCycle > 200) dutyCycle = 200;
  Serial.print("Back with duty cycle: "); Serial.println(dutyCycle);
  ledcWrite(pwmChannelA, dutyCycle);
  ledcWrite(pwmChannelB, dutyCycle);
  digitalWrite(motorApin1, LOW);
  digitalWrite(motorApin2, HIGH);
  digitalWrite(motorBpin1, LOW);
  digitalWrite(motorBpin2, HIGH);
  ledcWrite(pwmChannelA, dutyCycle);
  ledcWrite(pwmChannelB, dutyCycle);
}
void parar() {

  digitalWrite(motorApin1, LOW);
  digitalWrite(motorApin2, LOW);
  digitalWrite(motorBpin1, LOW);
  digitalWrite(motorBpin2, LOW);
  Serial.println("Parar: ");
  dutyCycle = 140;
}
int motorcali = 15;
void constMotor(unsigned int a) {
  motorcali = a;
}
void moverAdelante() {
  dutyCycle = 180;
  //if (dutyCycle > 200) dutyCycle = 200;
  if (!obstacle) {
    ledcWrite(pwmChannelA, dutyCycle + motorcali);
    ledcWrite(pwmChannelB, dutyCycle);
    digitalWrite(motorApin1, HIGH);
    digitalWrite(motorApin2, LOW);
    digitalWrite(motorBpin1, HIGH);
    digitalWrite(motorBpin2, LOW);
    ledcWrite(pwmChannelA, dutyCycle);
    ledcWrite(pwmChannelB, dutyCycle);
    //Serial.print("Se mueve adelante: "); Serial.println(dutyCycle+motorcali);
  }
}

int tiempogiro = 550;
void tiempoDeGiro(int tg) {
  tiempogiro = tg;
}
int tiempogiroiz = 550;
void tiempoDeGiroIZ(int tg) {
  tiempogiroiz = tg;
}
void moverIz() {
  dutyCycle = 180;
  //if (dutyCycle > 255) dutyCycle = 255;
  Serial.print("Giro iz "); Serial.println(dutyCycle);
  ledcWrite(pwmChannelA, dutyCycle);
  ledcWrite(pwmChannelB, dutyCycle);
  digitalWrite(motorApin1, LOW);
  digitalWrite(motorApin2, HIGH);
  digitalWrite(motorBpin1, HIGH);
  digitalWrite(motorBpin2, LOW);
  ledcWrite(pwmChannelA, dutyCycle);
  ledcWrite(pwmChannelB, dutyCycle);
  esperarMillis(tiempogiro);

  parar();
}
int constDer = 15;
void cambiarDer(int cons) {

  constDer = cons;
}
void moverDer() {
  dutyCycle = 180;
  //if (dutyCycle > 255) dutyCycle = 255;
  Serial.print("Giro derecha: "); Serial.println(dutyCycle);
  ledcWrite(pwmChannelA, dutyCycle + constDer);
  ledcWrite(pwmChannelB, dutyCycle);
  digitalWrite(motorApin1, HIGH);
  digitalWrite(motorApin2, LOW);
  digitalWrite(motorBpin1, LOW);
  digitalWrite(motorBpin2, HIGH);
  ledcWrite(pwmChannelA, dutyCycle);
  ledcWrite(pwmChannelB, dutyCycle);
  esperarMillis(tiempogiro);
  //esperoMillis(tiempogiro);
  parar();
}
//*********************************FUNCIONES DE BUSQEUDA BUSQUEDA*************************************************
//****************************************************************************************************************
void marcarMapa(int mapa[]) {
  mapa[0] = 1;
}
void girarMapa(int mapa[], int dir) { //if dir ==0 entonces iz
  if (dir == 0) {
    mapa[3] = mapa[2];
    mapa[2] = mapa[1];
    mapa[1] = mapa[0];

  } else {
    mapa[1] = mapa[2];
    mapa[2] = mapa[3];
    mapa[3] = mapa[0];
  }
  mapa[0] = 0;

}

void decidir(int mapa[], int sensores[]) {

  bool giroIz = true;         //libre el sensor
  bool giroDer = true;        //libre el sensor
   bool reseteo=false;
  //chequeo los sensores para ver mis opciones
  if (sensores[0] == 0) { //tenes a alguien a la iz
    giroIz = false;
  }
  else if (sensores[1] == 0) {//tenes a alguien a la der
    giroDer = false;
  }

  ///////////////mapa
  if ((mapa[0] + mapa[1] + mapa[2] + mapa[3]) == 4  ) { //implica que el mapa esta lleno
    reseteo=true;
    mapa[3] = 0;
    mapa[2] = 0;
    mapa[1] = 0;
    mapa[0] = 0;
  }
  if (mapa[1] + mapa[3] == 2)       {//giro 180 y no me importa nada
    Serial.println("giro 180");
    girarMapa(mapa, 0);
    moverIz();
    girarMapa(mapa, 0);
    moverIz();
  }
  else if (mapa[3] == 0 && giroIz) {//  me muevo a la iz
    Serial.println("giro izquierda");
    girarMapa(mapa, 0);
    moverIz();
  }// me muevo der
  else if (mapa[1] == 0 && giroDer) {
    Serial.println("giro derecha");
    girarMapa(mapa, 1);
    moverDer();
  }
if (reseteo) goFoward(1000, sensores,reseteo);
}
/////////////////
bool estoymidiendo=false;
bool estasMidiendo(){
  return estoymidiendo;
  }
int medirRssi( int &rssi, long tiempo,bool &jugar) {
  //unsigned long time;
  //time = millis();
  estoymidiendo=true;
  int aux = 0;
  int count = 0;
  int* arrayrssi = new int[60];
  TickType_t time_start = xTaskGetTickCount(); // toma eltiempo actual
  int trecorrido = 0;
  while ((trecorrido < tiempo) &&jugar) {
    trecorrido = pdTICKS_TO_MS(xTaskGetTickCount() - time_start);
    arrayrssi[count] = rssi;
    count = count + 1;
    aux = (rssi + aux);
    vTaskDelay(150 / portTICK_PERIOD_MS);
  }
  
  aux = aux / count;
  //varianza
  double varianza = 0;
  for (int i = 0; i < count; i++) {
    varianza = varianza + (arrayrssi[i] - aux) * (arrayrssi[i] - aux);
  }
  varianza = varianza / count;
  sqrt(varianza);
  delete[] arrayrssi;
  imprimo(("Varianza: "), String(varianza));
  imprimo(("Cantidad de veces que promedio: "), String(count));
  imprimo(("Promedio de rssi: "), String(aux));
  Serial.print("Promedio de rssi: "); Serial.println( String(aux));
  if (!jugar) return 100;
  if (varianza>10) {    return medirRssi( rssi, tiempo,jugar);    }
  //else
   estoymidiendo=false;
  return aux;
}
/////////////////
bool eq1 = false;
void elegirEq() {
  eq1 = (eq1) ? false : true;
}
double distanciaRssi(int rssimedido) {
  double salida = 0;
  double rssid = abs(-rssimedido);
  if (rssid <= 40) {
    salida = 501; //si ocurre alguna averracion no se cae
  }
  else if (eq1) salida = (-1 / 300) * pow((rssid), 2) - (1 / 2) * rssid - 85 / 6 ;
  else salida = (-0.00179) * pow((rssid), 2) + (0.238) * rssid - (246) / rssid ;
  //salida = pow((rssid / 100 + 0.5), 2) - 1 / rssid - 0.4;s
  //a*(-80)^2-b*(80)+c=4.5,a*(-50)^2-b*(50)+c=2.5,a*(-40)^2-b*(40)+c=0.5
  Serial.print("Se calcula una distancia de:"); Serial.println(salida);
  return salida * 1000;
}
////////////////////
double goFoward(double distancia, int sensores[],bool &jugar) {
  buscando=true;
  leerSensorSonido();
  TickType_t time_start = xTaskGetTickCount(); // toma eltiempo actual
  double recorrido = 0;
  while (!obstacle&& jugar) {
    recorrido = pdTICKS_TO_MS(xTaskGetTickCount() - time_start);        //comparo con el tiempo incial
    leerSensorSonido();                                                 // leo sensor
    moverAdelante();                                                    // me muevo , depende de la bandera obstacle
    if (obstacle) {//tengo alguien delante?
      Serial.print("Obstaculo=Millis transcurridos :"); Serial.println(recorrido);
      buscando=false;
      parar();

      return recorrido;
    }
    if ( recorrido > distancia) {
      Serial.print("Millis transcurridos :"); Serial.println(recorrido);
      break;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  parar();
  leerSensores(sensores);//le todos los sensores
  buscando=false;
  return recorrido;
}
////////////
bool esperarextra = false;
bool esperoExtra() {
  esperarextra = (esperarextra) ? false : true;
}
//**********************************************CEREBRO**********************************************************
//****************************************************************************************************************
int rssicero=0;
int getRssiAnterior(){
  return rssicero;
  }
void setRssiAnterior(int rssip){
  rssicero=rssip;  
   }
void cerebro(bool &bandera, int &rssi, int &rssi2, int &calibrar, int mapa[], int sensores[],bool &jugar) {
  int rssianterior = rssi;
  int rssipromedio =  medirRssi(rssi, 6000,jugar);
  int distancia = distanciaRssi(rssipromedio); //es el tiempo de recorrdia
  while (rssi < calibrar && jugar) {
    imprimo("Mapa antes de arrancar a moverse", "");
    
    distancia = goFoward(distancia, sensores,jugar);  //devuelve cuanto se movio
    if (!jugar)  break;
    
    rssipromedio = medirRssi(rssi, 6000,jugar);
    //setRssiAnterior(rssipromedio);
    if ((rssipromedio < rssianterior) ) {          //empeora, me muevo para atras
      marcarMapa(mapa);
      moverAtras();
      esperarMillis(distancia);                        //te moves lo mismo que avanzaste
      parar();
      if (!jugar) break;
      calculadora(rssipromedio);
      calculadora(rssipromedio);
      decidir(mapa, sensores);
    }
    else if (obstacle) {                           //la señal es mejor pero tengo un obstaculo          moverAtras();

      moverAtras();
      esperarMillis(500);
      parar();
      if (!jugar) break;
      calculadora(rssipromedio);
      calculadora(rssipromedio);
      decidir(mapa, sensores);
    }
    rssianterior = rssipromedio;
    distancia = distanciaRssi(rssipromedio);        //determino cuanto me voy a mover
    if (esperarextra) {
      imprimo(("tiempo extra"), "");
      esperarMillis(5000);
    }
  }
  Serial.println("te encontre y me canse");
  bandera = true;
}
