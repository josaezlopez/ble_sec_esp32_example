#include <Arduino.h>
#include <soc/soc.h>
#include <soc/rtc_cntl_periph.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID                "00001815-0000-1000-8000-00805F9B34FB"
#define CONTROL_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

uint32_t BTPin;                       // Pin para asociar dispositivo
BLEServer *pServer;                   // Puntero instancia Servidor
BLEAdvertising *pAdvertising;         // Puntero instancia Advertising

// Devuelve un entero aleatorio entre 111111 a 999999
uint32_t getPIN(){
    uint32_t PIN;
    do{
        PIN = esp_random(); 
    }while(PIN>999999L || PIN<=100000L);
    return PIN;
}

// Callback del servidor
class ServerCallbacks: public BLEServerCallbacks {
    public:

      void onConnect(BLEServer* sever, esp_ble_gatts_cb_param_t* param){
        Serial.println("Conectado");
        esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
      }

      void onDisconnect(BLEServer* server){
        Serial.println("Desconectado");
        server->getAdvertising()->start();
        BTPin = getPIN();
        esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &BTPin, sizeof(uint32_t));
      }

};

// Duncion callback para el control del interruptor
class ControlCallback: public BLECharacteristicCallbacks {
    public:

      void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() > 0) {
            Serial.printf("Valor cambiado... nuevo valor:%c\r\n",rxValue[0]);
            if (rxValue[0] == '1') {
              // Put here what is executed when writing '0'
            }
            else if (rxValue[0] == '0') {
              // Put here what is executed when writing '0'
            }
        }
      }

};

// Evento gap, Solo se capturan los eventos de emparejado y autenticacion completada.
void  gapEvent(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param){
    switch(event){
        case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:
            Serial.printf("Request pairing.\r\n PIN:%u\r\n",param->ble_security.key_notif.passkey);
            break;
        case ESP_GAP_BLE_AUTH_CMPL_EVT:
            if(param->ble_security.auth_cmpl.success == true){
                Serial.println("BLE authentication OK");
                return;
            }
            break;
    }
}

// Inicializa e inicia los advertising.
void initAdvertising(){
    Serial.println("Creating advertising");
    pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);        // Necesario para alguno problemas con iPhone.
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
}


void initBLE(){
    Serial.println("Starting BLE!");
    BLEDevice::init("esp32_ble");               // Nombre del dispositivo
    Serial.println("Initializing device");
    Serial.println("Creating server");
    pServer= BLEDevice::createServer();         // Create el servidor
    Serial.printf("Adding service UUID %s\r\n",SERVICE_UUID);
    BLEService *pService = pServer->createService(SERVICE_UUID); // Nuevo servicio en el servidor.

    Serial.println("Adding name characteristic");
    BLECharacteristic *nameCharacteristic = pService->createCharacteristic(
                                            BLEUUID((uint16_t)0x2A00),
                                            BLECharacteristic::PROPERTY_READ
                                        );
    nameCharacteristic->setValue("ch_esp32");
    Serial.println("Adding  apearance characteristic");
    BLECharacteristic *apearanceCharacteristic = pService->createCharacteristic(
                                            BLEUUID((uint16_t)0x2A01),
                                            BLECharacteristic::PROPERTY_READ
                                        ); 
    uint16_t apearance = 0x04C1;
    apearanceCharacteristic->setValue(apearance);
    BLECharacteristic *controlCharacteristic = pService->createCharacteristic(
                                            CONTROL_CHARACTERISTIC_UUID,
                                            BLECharacteristic::PROPERTY_READ |
                                            BLECharacteristic::PROPERTY_WRITE
                                        );


    // Añadimos una carcteristica para controlar el rele con '0` y `1`
    Serial.println("Adding control characteristic\r\n");
    pService->addCharacteristic(controlCharacteristic);
    controlCharacteristic->setValue("0");
    controlCharacteristic->setCallbacks(new ControlCallback());
    controlCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);  // Solo se puede leer y escribir si el dispositivo está autenticado
    pServer->setCallbacks(new ServerCallbacks);
    BLEDevice::m_customGapHandler = gapEvent;   // Para capturar los eventos GAP
    Serial.println("Starting service");
    pService->start();                          // Inicia el servidor
    initAdvertising();
    // Establezca los parámetros de seguridad iocap, auth_req, tamaño de clave y clave de respuesta de inicio
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;     // Emparejado con el peer device despues de la autenticacion
    esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT;                        // Pone las IO capability a No output/No input
    uint8_t key_size = 16;                                          //Tamaño de la clave, entre 7 y 16
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

    // Pone un pin aleatorio, para el proximo emparejado.
    BTPin = getPIN();    

    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_ENABLE;
    uint8_t oob_support = ESP_BLE_OOB_DISABLE;
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &BTPin, sizeof(uint32_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));

    esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));
    /* Si su dispositivo BLE actúa como esclavo, init_key significa qué tipos de clave del maestro se le deben distribuir.
    y la clave de respuesta significa qué clave puede distribuir al maestro.
    Si su dispositivo BLE actúa como maestro, la clave de respuesta significa qué tipos de clave del esclavo se le deben distribuir.
    y la clave de inicio significa qué clave puede distribuir al esclavo. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
}

// Deshabilita BLE
void DeInitBLE(){
    BLEDevice::deinit();
}

void setup(){
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector 
  Serial.begin(115200);
  initBLE();

}


void loop() {
}