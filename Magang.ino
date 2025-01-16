// Mendefinisikan struct untuk menyimpan data sensor IMU
struct DataIMU {
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  long timestamp;  // Timestamp dari data
};

// Mendefinisikan struct untuk menyimpan data sensor ultrasonik
struct DataSensorUltrasonik {
  long timestamp;  // Timestamp dari data
  float jarak;     // Jarak dalam cm
};

// Mendefinisikan struct session untuk menyimpan informasi session
struct Session {
  int sessionCount = 0;    // Hitung jumlah session
  int dataCounter = 0;     // Hitung jumlah data yang dikirim selama session
  bool isSending = false;  // Flag untuk menandakan apakah Bluetooth sedang mengirim data
};

Session session;  // Instance dari struct Session untuk melacak status session

#include <Wire.h>
#include <Adafruit_MPU6050.h>
Adafruit_MPU6050 mpu;  // Instance sensor MPU6050

#include <ESP32Time.h>
ESP32Time rtc(3600);  // Set zona waktu (GMT+1), ubah jika diperlukan

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define buttonPin 16  // Pin GPIO untuk tombol
#define trigUltrasonikSensorPin 19  // Pin trig sensor ultrasonik
#define echoUltrasonikSensorPin 18  // Pin echo sensor ultrasonik

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;  // Flag untuk melacak status koneksi Bluetooth

// Kelas Callback BLE untuk menangani event koneksi/diskoneksi
class MyCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;  // Set flag saat perangkat terhubung
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;  // Set flag saat perangkat terputus
  }
};

// Fungsi untuk membaca data dari sensor ultrasonik
DataSensorUltrasonik dapatJarakUltrasonik() {
  digitalWrite(trigUltrasonikSensorPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigUltrasonikSensorPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigUltrasonikSensorPin, LOW);

  // Mengukur waktu yang dibutuhkan untuk pulsa ultrasonik kembali
  long duration = pulseIn(echoUltrasonikSensorPin, HIGH);
  float jarak = duration * 0.034 / 2;  // Menghitung jarak dalam cm

  DataSensorUltrasonik data;
  data.timestamp = rtc.getEpoch();  // Menggunakan RTC untuk mendapatkan timestamp
  data.jarak = jarak;               // Menyimpan jarak pada struktur data

  return data;
}

// Setup Bluetooth (BLE), karakteristik dan layanan
void bluetoothSetup() {
  BLEDevice::init("USonic");  // Inisialisasi perangkat BLE dengan nama
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyCallbacks());  // Menetapkan callback untuk event koneksi

  BLEService* pService = pServer->createService(SERVICE_UUID);  // Membuat layanan BLE
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);  // Membuat karakteristik untuk mengirim data

  BLE2902* pBLE2902 = new BLE2902();
  pBLE2902->setNotifications(true);  // Mengaktifkan notifikasi pada karakteristik
  pCharacteristic->addDescriptor(pBLE2902);

  pService->start();                                        // Memulai layanan BLE
  pServer->getAdvertising()->addServiceUUID(SERVICE_UUID);  // Mempromosikan UUID layanan
  pServer->startAdvertising();                              // Memulai iklan BLE untuk memungkinkan koneksi

  Serial.println("Menunggu koneksi klien untuk diberi pemberitahuan...");
}

// Fungsi untuk mengirim data melalui Bluetooth jika perangkat terhubung
void bluetoothConnect(String data) {
  if (deviceConnected) {              // Jika perangkat terhubung
    pCharacteristic->setValue(data);  // Menetapkan nilai karakteristik dengan data
    pCharacteristic->notify();        // Memberi pemberitahuan kepada klien yang terhubung dengan data

    // Menambah jumlah data yang dikirim setiap kali data dikirim

    

    // Menampilkan jumlah session dan penghitung data ke Serial Monitor untuk debugging
    Serial.print("Jumlah Session: ");
    Serial.println(session.sessionCount);
    Serial.print("Data yang dikirim: ");
    Serial.println(data);
  }
}

// Setup untuk sensor MPU6050
void mpu6050Setup() {
  if (!mpu.begin()) {  // Inisialisasi sensor MPU6050
    Serial.println("MPU6050 tidak ditemukan. Pastikan koneksi benar.");
    while (1)
      ;  // Tetap di sini jika sensor tidak ditemukan
  }
  Serial.println("MPU6050 terdeteksi!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);  // Menetapkan rentang akselerometer
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);       // Menetapkan rentang giroskop
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);    // Menetapkan bandwidth filter
}

// Fungsi untuk membaca data dari sensor MPU6050
DataIMU dapatDataIMU() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);  // Mendapatkan data akselerometer, giroskop, dan suhu

  DataIMU data;
  data.accelX = a.acceleration.x;   // Menyimpan data akselerometer pada sumbu X
  data.accelY = a.acceleration.y;   // Menyimpan data akselerometer pada sumbu Y
  data.accelZ = a.acceleration.z;   // Menyimpan data akselerometer pada sumbu Z
  data.gyroX = g.gyro.x;            // Menyimpan data giroskop pada sumbu X
  data.gyroY = g.gyro.y;            // Menyimpan data giroskop pada sumbu Y
  data.gyroZ = g.gyro.z;            // Menyimpan data giroskop pada sumbu Z
  data.timestamp = rtc.getEpoch();  // Menggunakan RTC untuk mendapatkan timestamp

  return data;
}

void setup() {
  Serial.begin(115200);                 // Memulai komunikasi serial dengan baud rate 115200
  rtc.setTime(0, 0, 18, 30, 10, 2024);  // Menetapkan waktu awal (sesuaikan jika perlu)

  mpu6050Setup();  // Inisialisasi sensor MPU6050

  pinMode(trigUltrasonikSensorPin, OUTPUT);  // Menetapkan pin trig sensor ultrasonik sebagai output
  pinMode(echoUltrasonikSensorPin, INPUT);   // Menetapkan pin echo sensor ultrasonik sebagai input

  pinMode(buttonPin, INPUT);  // Menetapkan pin tombol sebagai input dengan pull-up internal

  bluetoothSetup();  // Inisialisasi Bluetooth (BLE)
}

void loop() {
  // Membaca status tombol
  int buttonPressed = digitalRead(buttonPin);  // Tombol aktif jika status LOW

  if (buttonPressed == HIGH) {  // Jika tombol ditekan
    if (!session.isSending) {  // Hanya jika belum dalam sesi pengiriman
      session.isSending = true;  // Tandai bahwa sesi pengiriman dimulai
      session.sessionCount++;    // Tambah penghitung sesi
      session.dataCounter = 0;   // Reset penghitung data dalam sesi
      Serial.print("Session baru dimulai: ");
      Serial.println(session.sessionCount);
    }
    session.dataCounter++;
  } else {
    session.isSending = false;  // Reset flag pengiriman jika tombol tidak ditekan
  }

  if (session.isSending && deviceConnected) {  // Hanya kirim data jika sesi aktif dan BLE terhubung
    // Mengumpulkan data dari sensor (ultrasonik dan IMU)
    DataSensorUltrasonik dataSUS = dapatJarakUltrasonik();
    DataIMU dataIMU = dapatDataIMU();

    String dataYangInginDikirim;  // Menyiapkan data untuk dikirim
    dataYangInginDikirim = String(session.sessionCount) + "/" + String(session.dataCounter) + "/" + String(dataSUS.jarak) + "/" + String(dataIMU.timestamp) + "/" + String(dataIMU.gyroX) + ";" + String(dataIMU.gyroY) + ";" + String(dataIMU.gyroZ) + "/" + String(dataIMU.accelX) + ";" + String(dataIMU.accelY) + ";" + String(dataIMU.accelZ);

    bluetoothConnect(dataYangInginDikirim);  // Mengirim data melalui Bluetooth

    delay(500);  // Delay untuk menghindari pengiriman data terlalu cepat
  } else {
    delay(200);  // Delay untuk menghindari polling tombol terlalu cepat
  } // Delay untuk mencegah flooding output serial dan pengiriman data terlalu cepat
}
