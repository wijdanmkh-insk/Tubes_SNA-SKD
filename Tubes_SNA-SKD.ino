#include <Wire.h>
#include <PID_v1.h>
#include <MPU6500_WE.h>
#include <WiFi.h>
#include <WebServer.h>
#include <QMC5883LCompass.h>

// ============================
// DEFINISI PIN DAN KONSTANTA
// ============================

// Sensor Gy271 Inisiasi
QMC5883LCompass compass;

// Sensor MPU6500 (Menggunakan library mpu9250_WE)
const int csPin = 10;  // Chip Select Pin
bool useSPI = true;    // SPI use flag
MPU6500_WE myMPU6500 = MPU6500_WE(&SPI, csPin, useSPI);

/* Use this one if you want to change the default SPI pins (only for ESP32 / STM32 so far): */
// MPU6500_WE myMPU6500 = MPU6500_WE(&SPI, csPin, mosiPin, misoPin, sckPin, useSPI);

// Sensor LDR di setiap sudut (2x2)
const int LDR_TOP_LEFT_PIN     = A0;
const int LDR_TOP_RIGHT_PIN    = A1;
const int LDR_BOTTOM_LEFT_PIN  = A2;
const int LDR_BOTTOM_RIGHT_PIN = A3;

// Driver L298N untuk Motor DC (rotasi horizontal)
const int GEAR_MOTOR_IN1_PIN = 4;
const int GEAR_MOTOR_IN2_PIN = 5;
const int GEAR_MOTOR_EN_PIN  = 6;   // PWM

// Driver L298N untuk Motor Linear Actuator (tilt/vertical)
const int ACTUATOR_IN1_PIN = 7;
const int ACTUATOR_IN2_PIN = 8;
const int ACTUATOR_EN_PIN  = 9;     // PWM

// ============================
// VARIABEL PID DAN TILT
// ============================
double horizontalError    = 0;
double horizontalSetpoint = 0;  // (bisa diatur ke 0 sebagai referensi pusat)
double horizontalOutput   = 0;

double tiltInput    = 0;
double tiltSetpoint = 0;
double tiltOutput   = 0;
double tiltScaleFactor = 0.03;  // Nilai awal; akan dikalibrasi otomatis

double lastTiltInput = 0;
double lastLDRDiff   = 0;
double ldrVerticalDiff = 0;     // Variabel global untuk selisih LDR vertikal

// Objek PID untuk masing-masing sumbu
PID horizontalPID(&horizontalError, &horizontalOutput, &horizontalSetpoint, 2.0, 5.0, 1.0, DIRECT);
PID tiltPID(&tiltInput, &tiltOutput, &tiltSetpoint, 1.0, 1.0, 0.5, DIRECT);

// ============================
// VARIABEL SISTEM
// ============================
bool isSleepMode = false;                     // Status mode: false = Active, true = Sleep

// ============================
// WEB HANDLING
// ============================
WebServer server(80);
String logBuffer = "";

// ============================
// FUNGSI WEB HANDLING
// ============================

// Fungsi untuk menghasilkan halaman web (form PID + log)
// Bagian PID form ditampilkan dengan CSS fixed agar tetap di tempat saat scroll.
String pageTemplate() {
  String page = "<html><head><title>Solar Tracker Log & PID Tuning</title>";
  page += "<style>";
  page += "body { font-family: Arial, sans-serif; margin: 0; padding: 0; }";
  page += "#pid-form { position: fixed; top: 0; left: 0; width: 100%; background-color: #f2f2f2; padding: 10px; border-bottom: 2px solid #333; z-index: 1000; }";
  page += "#log-container { margin-top: 120px; padding: 10px; }";
  page += "</style></head><body>";
  
  // Fixed PID tuning form
  page += "<div id='pid-form'>";
  page += "<h2>PID Tuning</h2>";
  page += "<form action='/updatePID' method='POST'>";
  page += "Kp: <input type='text' name='Kp' value='" + String(Kp) + "'> &nbsp; ";
  page += "Ki: <input type='text' name='Ki' value='" + String(Ki) + "'> &nbsp; ";
  page += "Kd: <input type='text' name='Kd' value='" + String(Kd) + "'> &nbsp; ";
  page += "<input type='submit' value='Update'>";
  page += "</form>";
  page += "</div>";
  
  // Log container
  page += "<div id='log-container'>";
  page += "<h2>Solar Tracker Log</h2>";
  page += "<p><strong>Status Mode:</strong> " + String(isSleepMode ? "Sleep Mode (Panel Tertutup)" : "Active Mode (Tracking Matahari)") + "</p>";
  page += logBuffer;
  page += "</div>";
  
  page += "</body></html>";
  return page;
}

void handleLog() {
  server.send(200, "text/html", pageTemplate());
}

// Handler untuk memperbarui nilai PID dari form web (metode POST)
void handlePIDUpdate() {
  if (server.hasArg("Kp") && server.hasArg("Ki") && server.hasArg("Kd")) {
    Kp = server.arg("Kp").toDouble();
    Ki = server.arg("Ki").toDouble();
    Kd = server.arg("Kd").toDouble();
  }
  // Redirect kembali ke halaman log
  server.sendHeader("Location", "/log", true);
  server.send(302, "text/plain", "");
}

// ============================
// KONFIGURASI ACCESS POINT
// ============================
const char* ssid     = "SolarTracker_Hotspot";
const char* password = "12345678";

// ============================
// FUNGSI Penyearah ke utara
// ============================
void settoNorth() {
  Serial.println("Penyearah ke titik nol.");
  do{
    compass.read();
    int derajat = compass.getAzimuth();
    Serial.print("Posisi sekarang: ");
    Serial.print(derajat);
    Serial.println();

    //PUH PERGERAKAN MOTORNYA GW GA PAHAM

  }while( 0> derajat <3 );
}

// ============================
// FUNGSI RESET PANEL (Saat Booting/Sleep Mode)
// ============================
void resetPanelPosition() {
  Serial.println("Reset posisi panel...");
  
  // Menutup panel ke keadaan nol menggunakan motor tilt (linear actuator)
  digitalWrite(ACTUATOR_IN1_PIN, LOW);
  digitalWrite(ACTUATOR_IN2_PIN, HIGH);
  analogWrite(ACTUATOR_EN_PIN, 255);
  delay(5000);  // Sesuaikan durasi pergerakan sesuai perangkat keras
  analogWrite(ACTUATOR_EN_PIN, 0);
  Serial.println("Panel ditutup ke posisi nol.");
  
  // Reset rotasi horizontal ke titik awal
  digitalWrite(GEAR_MOTOR_IN1_PIN, LOW);
  digitalWrite(GEAR_MOTOR_IN2_PIN, HIGH);
  analogWrite(GEAR_MOTOR_EN_PIN, 255);
  delay(5000);  // Sesuaikan waktu yang diperlukan untuk kembali ke nol
  analogWrite(GEAR_MOTOR_EN_PIN, 0);
  Serial.println("Rotasi horizontal dikembalikan ke titik nol.");
}

// ============================
// FUNGSI KALIBRASI OTOMATIS tiltScaleFactor
// ============================
void autoCalibrateTiltScaleFactor() {
  double tiltChange = abs(tiltInput - lastTiltInput);
  double ldrChange  = abs(ldrVerticalDiff - lastLDRDiff);

  if (ldrChange > 0) {
    tiltScaleFactor = tiltChange / ldrChange;
    Serial.print("Kalibrasi otomatis! tiltScaleFactor baru: ");
    Serial.println(tiltScaleFactor);
  }

  lastTiltInput = tiltInput;
  lastLDRDiff   = ldrVerticalDiff;
}

// ============================
// SETUP SISTEM
// ============================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  compass.init();
  if(!myMPU6500.init()){
    Serial.println("MPU6500 does not respond");
  }
  else{
    Serial.println("MPU6500 is connected");
  }
  delay(2000);

  // Inisialisasi pin motor
  pinMode(GEAR_MOTOR_IN1_PIN, OUTPUT);
  pinMode(GEAR_MOTOR_IN2_PIN, OUTPUT);
  pinMode(GEAR_MOTOR_EN_PIN, OUTPUT);

  pinMode(ACTUATOR_IN1_PIN, OUTPUT);
  pinMode(ACTUATOR_IN2_PIN, OUTPUT);
  pinMode(ACTUATOR_EN_PIN, OUTPUT);

  // Set PID ke mode AUTOMATIC
  horizontalPID.SetMode(AUTOMATIC);
  tiltPID.SetMode(AUTOMATIC);

  // Mengaktifkan mode Access Point
  WiFi.softAP(ssid, password);
  Serial.println("Access Point aktif. Hubungkan ke SSID: " + String(ssid));
  Serial.println("IP Address: " + WiFi.softAPIP().toString());

  // Setup route untuk web server
  server.on("/log", handleLog);
  server.on("/updatePID", HTTP_POST, handlePIDUpdate);
  server.begin();
  Serial.println("Web server mulai.");

  // Reset posisi panel saat booting
  resetPanelPosition();

  // Inisialisasi MPU6500_WE
  if(!myMPU6500.init()){
    Serial.println("MPU6500 does not respond");
  }
  else{
    Serial.println("MPU6500 is connected");
  }
  
  // Konfigurasi Untuk MPU6500_WE
  myMPU6500.enableGyrDLPF();
  myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
  myMPU6500.setSampleRateDivider(5);
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_500);
  myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
  myMPU6500.enableAccDLPF(true);
  myMPU6500.setAccDLPF(MPU6500_DLPF_3);

  // Penyearah ke utara
  settoNorth();

}

// ============================
// LOOP UTAMA
// ============================
void loop() {
  server.handleClient();

  // Baca sensor LDR
  int ldr_top_left    = analogRead(LDR_TOP_LEFT_PIN);
  int ldr_top_right   = analogRead(LDR_TOP_RIGHT_PIN);
  int ldr_bottom_left = analogRead(LDR_BOTTOM_LEFT_PIN);
  int ldr_bottom_right= analogRead(LDR_BOTTOM_RIGHT_PIN);

  
  // Hitung intensitas cahaya rata-rata
  int totalLight = ldr_top_left + ldr_top_right + ldr_bottom_left + ldr_bottom_right;
  double lightIntensity = totalLight / 4.0;

  // --- MODE SLEEP ---
  // Jika intensitas cahaya rendah (misalnya, malam atau hujan), masuk Sleep Mode dan reset panel (jika belum)
  if (lightIntensity < 200) {   // Nilai threshold, sesuaikan dengan lingkungan
    if (!isSleepMode) {         // Hanya reset satu kali saat memasuki sleep mode
      Serial.println("Light intensity low. Entering Sleep Mode.");
      isSleepMode = true;
      resetPanelPosition();

      


    }
  }

  // --- ACTIVE MODE ---
  else {
    isSleepMode = false;
    
    // Perhitungan error untuk kontrol motor
    double leftAverage = (ldrTopLeft + ldrBottomLeft) / 2.0;
    double rightAverage = (ldrTopRight + ldrBottomRight) / 2.0;
    double horizontalError = leftAverage - rightAverage;
    
    double topAverage = (ldrTopLeft + ldrTopRight) / 2.0;
    double bottomAverage = (ldrBottomLeft + ldrBottomRight) / 2.0;
    double ldrVerticalDiff = topAverage - bottomAverage;
    
    // --- Kontrol Motor Horizontal ---
    // Perhitungan PWM menggunakan faktor proporsional (PID: secara default hanya Kp dipakai di sini;
    // Ki & Kd dapat diintegrasikan sesuai kompleksitas sistem)
    horizontalPID.Compute(); 

    int pwmHorizontal = constrain(int(abs(horizontalOutput)), 0, 255);
    if (horizontalError >= 0) {
      digitalWrite(MOTOR_HORIZONTAL_IN1, HIGH);
      digitalWrite(MOTOR_HORIZONTAL_IN2, LOW);
    } else {
      digitalWrite(MOTOR_HORIZONTAL_IN1, LOW);
      digitalWrite(MOTOR_HORIZONTAL_IN2, HIGH);
    }
    analogWrite(MOTOR_HORIZONTAL_PWM, pwmHorizontal);
    
    double desiredTilt = ldrVerticalDiff * tiltScaleFactor;
    tiltSetpoint = desiredTilt;

    // --- Kontrol Motor Tilt ---
    int pwmTilt = constrain(int(abs(tiltOutput)), 0, 255);
    if (ldrVerticalDiff >= 0) {
      digitalWrite(MOTOR_TILT_IN1, HIGH);
      digitalWrite(MOTOR_TILT_IN2, LOW);
    } else {
      digitalWrite(MOTOR_TILT_IN1, LOW);
      digitalWrite(MOTOR_TILT_IN2, HIGH);
    }
    analogWrite(MOTOR_TILT_PWM, pwmTilt);
  }
  
  // Tambahkan data log (log baru diletakkan di atas buffer agar halaman web tampak terus ter-update)
  logBuffer = "<p>Light Intensity: " + String(lightIntensity) +
              " | Mode: " + String(isSleepMode ? "Sleep Mode" : "Active Mode") +
              " | PID (Kp=" + String(Kp) +
              ", Ki=" + String(Ki) +
              ", Kd=" + String(Kd) + ")</p>" + logBuffer;
  
  // Batasi panjang logBuffer (opsional)
  if (logBuffer.length() > 8000) {
    logBuffer = logBuffer.substring(0, 8000);
  }
  
  delay(500);
}
