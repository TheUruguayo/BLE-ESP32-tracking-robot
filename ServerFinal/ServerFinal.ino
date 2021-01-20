#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEClient.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEScan.h>
#include <SPI.h>
#include <Wire.h>
#include "my_library.h"
#define pdTICKS_TO_MS( xTicks )   ( ( uint32_t ) ( xTicks ) * 1000 / configTICK_RATE_HZ )
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define SERVICE2_UUID        "45ecddcf-c316-488d-8558-3222e5cb9b3c"
/////////////////////////Variable Globales relacionadas al ble
int vartiempo=20;
bool deviceConnected = false; //bandera de coneccion
int tiemporefresh = 1000;
int rssi = -48; //rssi medido entre esp32 y usario
int rssi2 = -60;
int calibrar = -48; //distancia minima que detecta el sensor
int constante = 2; //constante de ambiente
String valor = "";
String addres = "";
esp_bd_addr_t peerAddress;// se guarda la address del cliente
esp_bd_addr_t peerAddress2;
esp_bd_addr_t peerAddress3;
int conectados=0;
///creacion de threads multitarea
TaskHandle_t Task1;//LOOP INFINITO EN CORE 0
TaskHandle_t Task2;//LOOP INFINITO EN CORE 1
TaskHandle_t Task3;//LOOP INFINITO EN CORE 0
//
int sensores[] = {HIGH, HIGH, HIGH, HIGH};
int mapa[] = {0, 0, 0, 0};//no tiene por que estar en el main
int LED_BUILTIN = 2;//LED AZUL
bool pica=false;//indica si te econtro
bool startbusqueda=false; //para pedirle al robot que arranque la busqueda

////// Touch pin
int threshold = 40;
bool touch2detected = false;
//////auxiliares
bool fowarden2=false;
bool iz=false;
bool der=false;
int tiempocalibrado=0;
TickType_t calibradostart; 
//fin de juego
bool jugar=true;
//semaforo
SemaphoreHandle_t xMutex;
//***********************************************CALLBACKS********************************************************
//****************************************************************************************************************

class MyCallbacks: public BLECharacteristicCallbacks {//ACA LLEGAN LOS COMANDOS VIA BLE
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      //Serial.print("core del callback");Serial.println(xPortGetCoreID());
      if (value.length() > 0) {
        valor = "";
        String Salida = "";
        for (int i = 0; i < value.length(); i++) {
          // Serial.print(value[i]); // Presenta value.
          valor = valor + value[i];
          if (i > 0) {
            Salida = Salida + value[i];
          }
        }
        //Serial.println("*********"); 
       
        if (valor.charAt(0) == 'r') {                     
          String smapa = "M";
          for (int i = 0; i < 4; i++) {
             smapa = smapa + mapa[i]+"-";          
          }
          rssi = Salida.toInt();
          if (rssi>=-42) jugar=false;
          //Serial.println(rssi);            
          std::string emapa((char*)&smapa, 9);            
          pCharacteristic->setValue(emapa);          
        }
        else if (valor.charAt(0) == '#') {
          
          Serial.println("CALIBRARTIEMPO");         
          tiempocalibrado =  pdTICKS_TO_MS(xTaskGetTickCount() - calibradostart);
          Serial.println(tiempocalibrado);
          pCharacteristic->setValue(tiempocalibrado);
        }       
        else if (valor == "off") { //esto puede ser moverse adelante por ejemplo
          digitalWrite(LED_BUILTIN, LOW);
          Serial.println("LED turned on OFF");
          pCharacteristic->setValue("LED turned on OFF"); // Pone el numero aleatorio
        }
        else if (valor == "show") {                      //esto puede ser moverse adelante por ejemplo
          muestroTodo();
          Serial.println("Se cambio la bandera");
          pCharacteristic->setValue("bandera"); // Pone el numero aleatorio
        }
        else if (valor == "on") {
          digitalWrite(LED_BUILTIN, HIGH);
          Serial.println("LED turned on");
          pCharacteristic->setValue("LED turned on");    // Pone el numero aleatorio
        }
        else if (valor.charAt(0) == 'c') {
          calibrar = Salida.toInt();
          Serial.println("CALIBRADO");
          Serial.println(calibrar);
          Serial.println(Salida);
          calibradostart = xTaskGetTickCount();
          pCharacteristic->setValue("#");
        }       
        else if (valor.charAt(0) == 'd') {
          pCharacteristic->setValue("duriendo");
          Dormir(threshold);
        }
        else if (valor.charAt(0) == 'a') {
          pCharacteristic->setValue("adelante");
          fowarden2= (fowarden2)? false :true;
          
        }
        else if (valor.charAt(0) == 'p') {
          pCharacteristic->setValue("parar");
          parar();
        }
        else if (valor.charAt(0) == 'i') {
          pCharacteristic->setValue("iz");
          iz=true;
          //moverIz();
        }
        else if (valor.charAt(0) == 'x') {
          pCharacteristic->setValue("derecha");
          der=true;
        }
        else if (valor.charAt(0) == 'b') {
          pCharacteristic->setValue("atras");
          moverAtras();
        }
         else if (valor == "s") {
          pCharacteristic->setValue("START");
          startbusqueda= (startbusqueda) ? false:true;
          pica=false;
        }
        else if (valor.charAt(0) == 't') {
          pCharacteristic->setValue("calibrogiro");
          tiempoDeGiro(Salida.toInt());
        }
        else if (valor.charAt(0) == 'v') {
          vartiempo=Salida.toInt();
        }
        else if (valor.charAt(0) == 'k') {
          pCharacteristic->setValue("kalibroadelante");
          constMotor(Salida.toInt());
        }
        else if (valor.charAt(0) == 'z') {
          pCharacteristic->setValue("tiempo para pensar");
          esperoExtra();
        }
         else if (valor.charAt(0) == '!') {
          pCharacteristic->setValue("foward en el 2");
          fowarden2= (fowarden2)? false :true;
        }
        else if (valor.charAt(0) == 'g') {
          pCharacteristic->setValue("giroCambiado");
          cambiarDer(Salida.toInt());
        }
       else if (valor.charAt(0) == 'e') {
          pCharacteristic->setValue("cambio de eq");
          elegirEq();
        }
        
      }
    }
};

class myServerCallback : public BLEServerCallbacks {//ESTO ME DICE SI TENGO ALGUIEN CONECTADO

    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {

      char remoteAddress[18];
      int aux = param->connect.conn_id;
    
      //start sent the update connection parameters to the peer device.
      sprintf(
        remoteAddress,
        "%.2X:%.2X:%.2X:%.2X:%.2X:%.2X",
        param->connect.remote_bda[0],
        param->connect.remote_bda[1],
        param->connect.remote_bda[2],
        param->connect.remote_bda[3],
        param->connect.remote_bda[4],
        param->connect.remote_bda[5]
      );

      ESP_LOGI(LOG_TAG, "myServerCallback onConnect, MAC: %s", remoteAddress);
      Serial.print("onConnect, sos la conexion :");Serial.println(aux);
      //Serial.println(aux);
      conectados=conectados+1;
      if (conectados==1) memcpy(&peerAddress, param->connect.remote_bda, 6);
      else if (conectados==2) memcpy(&peerAddress2, param->connect.remote_bda, 6);
      else if (conectados==3) memcpy(&peerAddress3, param->connect.remote_bda, 6);
      //peerAddress=param->connect.remote_bda;
      //rssi2 = esp_ble_gap_read_rssi(peerAddress);
      
      //Serial.print("rssi durante la conexion"); Serial.println(rssi2);
      //Serial.print("ADRESS durante la conexion"); Serial.println(remoteAddress);
      esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_P9 );
      esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL1, ESP_PWR_LVL_P9 );
      deviceConnected = true;
      
    }

    void onDisconnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {

      char remoteAddress[20];

      sprintf(
        remoteAddress,
        "%.2X:%.2X:%.2X:%.2X:%.2X:%.2X",
        param->disconnect.remote_bda[0],
        param->disconnect.remote_bda[1],
        param->disconnect.remote_bda[2],
        param->disconnect.remote_bda[3],
        param->disconnect.remote_bda[4],
        param->disconnect.remote_bda[5]
      );

      ESP_LOGI(LOG_TAG, "myServerCallback onDisconnect, MAC: %s", remoteAddress);

      deviceConnected = false;
      conectados=conectados-1;
    }
};
//***********************************************EVENTOS**********************************************************
//****************************************************************************************************************
static void my_gap_event_handler(esp_gap_ble_cb_event_t  event, esp_ble_gap_cb_param_t* param) {
  ESP_LOGW(LOG_TAG, "custom gap event handler, GAP event: %d", (uint8_t)event);
  //Serial.print("RSSI status"); Serial.println(param->read_rssi_cmpl.status);
  //Serial.print("RSSI del gap event");
  rssi2 = param->read_rssi_cmpl.rssi;
  
  Serial.print("Rssi medido en el evento:");Serial.println(rssi2);
  //Serial.print("Address en gap "); Serial.println(BLEAddress(param->read_rssi_cmpl.remote_addr).toString().c_str());
  
}
//interrupciones
void gotTouch() { //se detecta interrupcion de tacto
  touch2detected = true;

}

//*************************************************SETUP************************************************************
//******************************************************************************************************************
BLEServer *pServer=NULL;
BLEAdvertising *pAdvertising=NULL;
void setup() {//SE INICIALIZA TODO
  Serial.begin(115200);
  iniciarMotores();
  initDisplay(1000);
  initSensorSonido();
  pinMode (LED_BUILTIN, OUTPUT);//onboard blue led
  ///////////////BLE
  BLEDevice::init("Hide&Seek");// nombre del dispositivo bl
  BLEDevice::setPower(ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_P9 );esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL1, ESP_PWR_LVL_P9 );
  pServer = BLEDevice::createServer(); // Create the BLE Server
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pServer->setCallbacks(new myServerCallback());
  pCharacteristic->setCallbacks(new MyCallbacks());
  BLEDevice::setCustomGapHandler(my_gap_event_handler);
  pCharacteristic->setValue("Iniciado.");
  pCharacteristic->addDescriptor(new BLE2902());
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
  pService->start();
  pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
  ///////////////////////////
  touchAttachInterrupt(T3, gotTouch, 40);//SETEA LAS INTERRUPCION EN EL TOUCH 3
  ///////////////////////////////////////Task,multithreading
  xMutex = xSemaphoreCreateMutex(); 


  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
    Task2code,   /* Task function. */
    "Task2",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    10,           /* priority of the task */
    &Task2,      /* Task handle to keep track of created task */
    1);          /* pin task to core 1 */
  delay(500);

  xTaskCreatePinnedToCore(
    Task3code,   /* Task function. */
    "Task3",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &Task3,      /* Task handle to keep track of created task */
    1);          /* pin task to core 1 */

  delay(500);
}
//***********************************************MAIN LOOPS*******************************************************
//****************************************************************************************************************


void Task2code( void * pvParameters ) {// baja prioridad core 1
  TickType_t tiempodormir=xTaskGetTickCount(); 
  for (;;) { // infinite loop
   
   if(deviceConnected && !pica && startbusqueda && jugar){
      
      mapa[0] = 0; mapa[1] = 0; mapa[2] = 0; mapa[3] = 0;
      cerebro(pica,rssi,rssi2,calibrar,mapa,sensores,jugar);
      
      startbusqueda=false;//cerebro es un loop infinito salir de ahi implica haber cumplido las condiciones
      tiempodormir=xTaskGetTickCount();
      }else
      {  
       Serial.print("core del cerebro");Serial.println(xPortGetCoreID());
       imprimo(("Pica es: "), String(pica));
       if (fowarden2) {
             fowarden2=false;
             goFoward(10000,sensores,jugar); 
             tiempodormir=xTaskGetTickCount();      
        }else if (iz==true){
            moverIz();
            iz=false;
            tiempodormir=xTaskGetTickCount();
        }else if (der==true){
            moverDer();
            der=false;
            tiempodormir=xTaskGetTickCount();
        }
      
       }
       obstaculo(sensores); 
       xSemaphoreTake( xMutex, portMAX_DELAY );   
       if (!(estasBuscando() || estasMidiendo())) reloj(int(pdTICKS_TO_MS(xTaskGetTickCount() - tiempodormir)/1000));
       xSemaphoreGive( xMutex );
       jugar=true;
       
       if (pdTICKS_TO_MS(xTaskGetTickCount() - tiempodormir)>240000) Dormir(threshold);
       
  }
}

void Task3code( void * pvParameters ) {
  for (;;) { // infinite loop
    //Serial.println("estoy al pedo");
    xSemaphoreTake( xMutex, portMAX_DELAY );
    if (estasBuscando()) {lupa(rssi);}
    else if (estasMidiendo()) {phone(rssi);}
    xSemaphoreGive( xMutex );
    cuentaOvejas(touch2detected, threshold);
    vTaskDelay(200 / portTICK_PERIOD_MS); 
  }
}


void loop() {//por default el loop va al core 1
   
}
// core 0 prioridad 3
//void Tasksensores( void * pvParameters ) {
//
//  for (;;) {//WHILE TRUE
//    leerSensores(sensores);
////    if (conectados>0){
////      Serial.println("conexion 1 rssi");
////      esp_ble_gap_read_rssi(peerAddress);
////      delay(vartiempo);
////    }
////    if (conectados>1){
////     Serial.println("conexion 2 rssi");
////      esp_ble_gap_read_rssi(peerAddress2);
////      delay(vartiempo);
////    }
////    if (conectados>2){
////      Serial.println("conexion 3 rssi");
////      esp_ble_gap_read_rssi(peerAddress3);
////      delay(vartiempo);
////    }
//  }
//}
// if (deviceConnected){
//  deviceConnected=false;
//  pAdvertising->start();
