#include <Wire.h>
#include <PID_v1.h>
#include <MPU6500_WE.h>
#include <WiFi.h>
#include <WebServer.h>
#include <QMC5883LCompass.h>

// ============================
// DEFINISI PIN DAN KONSTANTA
// ============================

// Inisiasi Sensor Gy271
QMC5883LCompass compass;

// Sensor MPU6500 (Menggunakan library mpu9250_WE)
const int csPin = 10;   // Pin Chip Select
bool useSPI = true;     // Flag penggunaan SPI
MPU6500_WE myMPU6500 = MPU6500_WE(&SPI, csPin, useSPI);

// Sensor LDR di setiap sudut (2x2)
const int LDR_TOP_LEFT_PIN     = A0;
const int LDR_TOP_RIGHT_PIN    = A1;
const int LDR_BOTTOM_LEFT_PIN  = A2;
const int LDR_BOTTOM_RIGHT_PIN = A3;

// Driver L298N untuk Motor DC (rotasi horizontal)
const int GEAR_MOTOR_IN1_PIN = 4;
const int GEAR_MOTOR_IN2_PIN = 5;
const int GEAR_MOTOR_EN_PIN  = 6;   // Pin PWM untuk kecepatan motor horizontal

// Driver L298N untuk Motor Linear Actuator (tilt/vertical)
const int ACTUATOR_IN1_PIN = 7;
const int ACTUATOR_IN2_PIN = 8;
const int ACTUATOR_EN_PIN  = 9;     // Pin PWM untuk kecepatan aktuator kemiringan

// Definisikan PI jika belum didefinisikan oleh Arduino
#ifndef PI
#define PI 3.14159265358979323846
#endif

// Batasan Rotasi Horizontal (Azimuth) untuk mencegah kabel terlilit
// QMC5883LCompass.getAzimuth() mengembalikan nilai dari -180 hingga 180 derajat.
// Batasan ini akan menjaga panel dalam rentang aman, misalnya +/- 170 derajat dari Utara.
const int MIN_AZIMUTH_LIMIT = -170; // Batas minimum (misalnya, 170 derajat Barat)
const int MAX_AZIMUTH_LIMIT = 170;  // Batas maksimum (misalnya, 170 derajat Timur)

// Konstanta untuk Reset Posisi
const double RESET_TILT_ANGLE = 0.0; // Sudut kemiringan target untuk reset (misalnya, 0.0 untuk datar)
                                     // Sesuaikan nilai ini dengan posisi "nol" atau "tertutup" panel Anda.
const int RESET_HORIZ_AZIMUTH = MIN_AZIMUTH_LIMIT; // Azimuth target untuk reset horizontal
                                                  // Ini akan menggerakkan panel ke batas kiri (MIN_AZIMUTH_LIMIT) sebagai posisi "rumah"

// ============================
// VARIABEL PID DAN TILT
// ============================
double horizontalError    = 0;
double horizontalSetpoint = 0;  // Setpoint untuk error horizontal (biasanya 0 untuk seimbang)
double horizontalOutput   = 0;  // Output PWM untuk motor horizontal

double tiltInput    = 0;        // Input aktual dari sensor (sudut kemiringan)
double tiltSetpoint = 0;        // Setpoint kemiringan yang diinginkan
double tiltOutput   = 0;        // Output PWM untuk aktuator kemiringan
double tiltScaleFactor = 0.03;  // Nilai awal; akan dikalibrasi otomatis untuk mengkonversi perbedaan LDR ke sudut kemiringan

double lastTiltInput = 0;       // Nilai tiltInput sebelumnya untuk kalibrasi
double lastLDRDiff   = 0;       // Nilai ldrVerticalDiff sebelumnya untuk kalibrasi
double ldrVerticalDiff = 0;     // Variabel global untuk selisih LDR vertikal (atas - bawah)

// Parameter penyetelan PID global
double Kp_horizontal = 2.0;
double Ki_horizontal = 5.0;
double Kd_horizontal = 1.0;

double Kp_tilt = 1.0;
double Ki_tilt = 1.0;
double Kd_tilt = 0.5;

// Objek PID untuk masing-masing sumbu
// Parameter: *Input, *Output, *Setpoint, Kp, Ki, Kd, Direction
PID horizontalPID(&horizontalError, &horizontalOutput, &horizontalSetpoint, Kp_horizontal, Ki_horizontal, Kd_horizontal, DIRECT);
PID tiltPID(&tiltInput, &tiltOutput, &tiltSetpoint, Kp_tilt, Ki_tilt, Kd_tilt, DIRECT);

// ============================
// VARIABEL SISTEM
// ============================
bool isSleepMode = false;                     // Status mode sistem: false = Aktif, true = Tidur
const int LIGHT_INTENSITY_THRESHOLD = 200; // Ambang batas intensitas cahaya untuk masuk mode tidur

// Variabel untuk Skenario A (Kontrol Sederhana)
const int SIMPLE_PWM_SPEED = 150; // Kecepatan PWM konstan untuk kontrol sederhana
const int SIMPLE_HORIZ_THRESHOLD = 50; // Ambang batas error LDR horizontal untuk kontrol sederhana
const int SIMPLE_TILT_THRESHOLD = 50;  // Ambang batas error LDR vertikal untuk kontrol sederhana

// Enum untuk mode kontrol
enum ControlMode {
    MODE_PID_CONTROL,      // Skenario B: Menggunakan PID
    MODE_SIMPLE_CONTROL    // Skenario A: Menggunakan algoritma sederhana
};

ControlMode currentControlMode = MODE_PID_CONTROL; // Mode kontrol default saat booting

// ============================
// PENANGANAN WEB
// ============================
WebServer server(80);
String logBuffer = ""; // Buffer untuk menyimpan log yang akan ditampilkan di halaman web

// ============================
// FUNGSI PENANGANAN WEB
// ============================

// Fungsi untuk menghasilkan halaman web (form PID + log)
// Bagian form PID ditampilkan dengan CSS fixed agar tetap di tempat saat menggulir.
String pageTemplate() {
    String page = "<html><head><title>Log & Penyetelan Pelacak Surya</title>";
    page += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"; // Penting untuk responsivitas mobile
    page += "<style>";
    page += "body { font-family: Arial, sans-serif; margin: 0; padding: 0; background-color: #e0e0e0; }";
    page += "#header { position: fixed; top: 0; left: 0; width: 100%; background-color: #333; color: white; padding: 10px; border-bottom: 2px solid #555; z-index: 1000; text-align: center; }";
    page += "#pid-form { background-color: #f2f2f2; padding: 10px; border-bottom: 2px solid #ccc; z-index: 999; margin-top: 60px; }"; // Margin-top disesuaikan dengan tinggi header
    page += "#mode-select { background-color: #e6e6e6; padding: 10px; border-bottom: 2px solid #bbb; z-index: 998; }";
    page += "#log-container { margin-top: 10px; padding: 10px; background-color: #fff; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }";
    page += "h1, h2, h3 { color: #333; margin-top: 0; }";
    page += "p { margin: 5px 0; }";
    page += "input[type='text'], input[type='submit'] { padding: 8px; border-radius: 4px; border: 1px solid #ccc; }";
    page += "input[type='submit'] { background-color: #4CAF50; color: white; cursor: pointer; }";
    page += "input[type='submit']:hover { background-color: #45a049; }";
    page += ".radio-group label { margin-right: 15px; font-weight: bold; }";
    page += "</style></head><body>";

    // Header Global
    page += "<div id='header'><h1>Pemantauan Pelacak Surya</h1></div>";

    // Formulir penyetelan PID
    page += "<div id='pid-form'>";
    page += "<h2>Penyetelan PID</h2>";
    page += "<form action='/updatePID' method='POST'>";
    page += "<h3>PID Horizontal:</h3> ";
    page += "Kp: <input type='text' name='Kp_h' value='" + String(Kp_horizontal) + "'> &nbsp; ";
    page += "Ki: <input type='text' name='Ki_h' value='" + String(Ki_horizontal) + "'> &nbsp; ";
    page += "Kd: <input type='text' name='Kd_h' value='" + String(Kd_horizontal) + "'> &nbsp; ";
    page += "<br><br>";
    page += "<h3>PID Kemiringan:</h3> ";
    page += "Kp: <input type='text' name='Kp_t' value='" + String(Kp_tilt) + "'> &nbsp; ";
    page += "Ki: <input type='text' name='Ki_t' value='" + String(Ki_tilt) + "'> &nbsp; ";
    page += "Kd: <input type='text' name='Kd_t' value='" + String(Kd_tilt) + "'> &nbsp; ";
    page += "<input type='submit' value='Perbarui PID'>";
    page += "</form>";
    page += "</div>";

    // Pemilihan Mode Kontrol
    page += "<div id='mode-select'>";
    page += "<h2>Pilih Mode Kontrol</h2>";
    page += "<form action='/selectMode' method='POST' class='radio-group'>";
    page += "<label><input type='radio' name='mode' value='pid' " + (currentControlMode == MODE_PID_CONTROL ? "checked" : "") + "> Skenario B (Kontrol PID)</label>";
    page += "<label><input type='radio' name='mode' value='simple' " + (currentControlMode == MODE_SIMPLE_CONTROL ? "checked" : "") + "> Skenario A (Kontrol Sederhana)</label>";
    page += "<input type='submit' value='Pilih Mode'>";
    page += "</form>";
    page += "</div>";
    
    // Kontainer log
    page += "<div id='log-container'>";
    page += "<h2>Log Pelacak Surya</h2>";
    page += "<p><strong>Status Mode Sistem:</strong> " + String(isSleepMode ? "Mode Tidur (Panel Tertutup)" : "Mode Aktif (Pelacakan Matahari)") + "</p>";
    page += "<p><strong>Mode Kontrol Aktif:</strong> " + (currentControlMode == MODE_PID_CONTROL ? "Skenario B (PID)" : "Skenario A (Sederhana)") + "</p>";
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
    // Perbarui parameter PID Horizontal jika ada di request
    if (server.hasArg("Kp_h") && server.hasArg("Ki_h") && server.hasArg("Kd_h")) {
        Kp_horizontal = server.arg("Kp_h").toDouble();
        Ki_horizontal = server.arg("Ki_h").toDouble();
        Kd_horizontal = server.arg("Kd_h").toDouble();
        horizontalPID.SetTunings(Kp_horizontal, Ki_horizontal, Kd_horizontal);
        Serial.print("PID Horizontal diperbarui: Kp="); Serial.print(Kp_horizontal);
        Serial.print(", Ki="); Serial.print(Ki_horizontal);
        Serial.print(", Kd="); Serial.println(Kd_horizontal);
    }
    // Perbarui parameter PID Kemiringan jika ada di request
    if (server.hasArg("Kp_t") && server.hasArg("Ki_t") && server.hasArg("Kd_t")) {
        Kp_tilt = server.arg("Kp_t").toDouble();
        Ki_tilt = server.arg("Ki_t").toDouble();
        Kd_tilt = server.arg("Kd_t").toDouble();
        tiltPID.SetTunings(Kp_tilt, Ki_tilt, Kd_tilt);
        Serial.print("PID Kemiringan diperbarui: Kp="); Serial.print(Kp_tilt);
        Serial.print(", Ki="); Serial.print(Ki_tilt);
        Serial.print(", Kd="); Serial.println(Kd_tilt);
    }
    server.sendHeader("Location", "/log", true);
    server.send(302, "text/plain", "");
}

// Handler untuk memilih mode kontrol (Skenario A atau B)
void handleModeSelect() {
    if (server.hasArg("mode")) {
        String mode = server.arg("mode");
        if (mode == "pid") {
            currentControlMode = MODE_PID_CONTROL;
            Serial.println("Mode kontrol diubah ke: Skenario B (PID)");
        } else if (mode == "simple") {
            currentControlMode = MODE_SIMPLE_CONTROL;
            Serial.println("Mode kontrol diubah ke: Skenario A (Sederhana)");
        }
        // PENTING: Atur ulang posisi panel setelah mengganti skenario untuk pengujian yang konsisten
        resetPanelPosition();
        settoNorth(); // Juga sesuaikan kembali ke Utara setelah reset
    }
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
    Serial.println("Menyesuaikan ke titik nol (Utara).");
    long startTime = millis();
    const long timeout = 30000; // Batas waktu 30 detik untuk penyelarasan

    do {
        compass.read();
        int derajat = compass.getAzimuth(); // Dapatkan azimuth dalam derajat (-180 hingga 180)
        Serial.print("Posisi sekarang: ");
        Serial.print(derajat);
        Serial.println(" derajat");

        bool moved = false; // Flag untuk memeriksa apakah motor bergerak

        // Logika untuk bergerak menuju Utara (0 derajat) sambil menghormati batasan rotasi
        if (derajat > 5) { // Jika menghadap ke Timur secara signifikan (> 5 derajat), putar berlawanan arah jarum jam (kiri)
            // Pastikan tidak melewati batas bawah saat bergerak ke kiri
            if (derajat > MIN_AZIMUTH_LIMIT) {
                digitalWrite(GEAR_MOTOR_IN1_PIN, LOW);
                digitalWrite(GEAR_MOTOR_IN2_PIN, HIGH);
                analogWrite(GEAR_MOTOR_EN_PIN, SIMPLE_PWM_SPEED);
                Serial.println("Memutar ke kiri...");
                moved = true;
            } else { // Sudah di atau melewati batas bawah, hentikan
                digitalWrite(GEAR_MOTOR_IN1_PIN, LOW);
                digitalWrite(GEAR_MOTOR_IN2_PIN, LOW);
                analogWrite(GEAR_MOTOR_EN_PIN, 0);
                Serial.println("Terhenti di batas bawah saat menyesuaikan ke Utara.");
            }
        } else if (derajat < -5) { // Jika menghadap ke Barat secara signifikan (< -5 derajat), putar searah jarum jam (kanan)
            // Pastikan tidak melewati batas atas saat bergerak ke kanan
            if (derajat < MAX_AZIMUTH_LIMIT) {
                digitalWrite(GEAR_MOTOR_IN1_PIN, HIGH);
                digitalWrite(GEAR_MOTOR_IN2_PIN, LOW);
                analogWrite(GEAR_MOTOR_EN_PIN, SIMPLE_PWM_SPEED);
                Serial.println("Memutar ke kanan...");
                moved = true;
            } else { // Sudah di atau melewati batas atas, hentikan
                digitalWrite(GEAR_MOTOR_IN1_PIN, LOW);
                digitalWrite(GEAR_MOTOR_IN2_PIN, LOW);
                analogWrite(GEAR_MOTOR_EN_PIN, 0);
                Serial.println("Terhenti di batas atas saat menyesuaikan ke Utara.");
            }
        } else { // Dalam rentang yang dapat diterima (-5 hingga +5 derajat)
            analogWrite(GEAR_MOTOR_EN_PIN, 0); // Hentikan motor
            digitalWrite(GEAR_MOTOR_IN1_PIN, LOW);
            digitalWrite(GEAR_MOTOR_IN2_PIN, LOW);
            Serial.println("Berhasil menyelaraskan ke Utara.");
            break; // Keluar dari loop jika sudah sejajar
        }

        if (!moved) { // Jika tidak ada pergerakan yang diizinkan karena batasan, hentikan upaya penyelarasan
            Serial.println("Tidak dapat menyelaraskan ke Utara karena batasan rotasi.");
            break;
        }

        delay(100); // Penundaan kecil untuk memungkinkan motor bergerak dan kompas memperbarui

        if (millis() - startTime > timeout) {
            Serial.println("Batas waktu habis: Tidak dapat menyelaraskan ke Utara.");
            analogWrite(GEAR_MOTOR_EN_PIN, 0); // Hentikan motor saat batas waktu habis
            digitalWrite(GEAR_MOTOR_IN1_PIN, LOW);
            digitalWrite(GEAR_MOTOR_IN2_PIN, LOW);
            break;
        }

    } while (true); // Loop tanpa henti hingga sejajar atau batas waktu habis
}

// ============================
// FUNGSI RESET PANEL (Saat Booting/Mode Tidur)
// ============================
void resetPanelPosition() {
    Serial.println("Mengatur ulang posisi panel...");

    // Menggerakkan kemiringan ke posisi nol (misalnya, datar horizontal) menggunakan aktuator linier dan MPU6500
    // Definisikan RESET_TILT_ANGLE sebagai sudut kemiringan saat panel dalam posisi "nol" atau "tertutup"
    // Misalnya, 0.0 untuk datar, atau -45.0 jika itu adalah posisi tertutup secara fisik.
    Serial.print("Menggerakkan kemiringan ke posisi ");
    Serial.print(RESET_TILT_ANGLE);
    Serial.println(" derajat...");
    long tilt_timeout = millis() + 10000; // Batas waktu 10 detik untuk reset tilt

    // Menggunakan kontrol proporsional sederhana untuk mencapai RESET_TILT_ANGLE
    // Loop akan berlanjut sampai error kurang dari 2.0 derajat atau timeout
    while (abs(tiltInput - RESET_TILT_ANGLE) > 2.0 && millis() < tilt_timeout) {
        // Dapatkan data dari MPU6500 untuk pengukuran kemiringan aktual
        xyzFloat gValue = myMPU6500.getGValues();
        tiltInput = atan2(gValue.y, sqrt(gValue.x * gValue.x + gValue.z * gValue.z)) * 180.0 / PI;

        double tilt_error = tiltInput - RESET_TILT_ANGLE;
        // Hitung PWM berdasarkan error. Kp sederhana=5, min PWM 50 untuk memastikan motor bergerak
        int pwm = constrain(int(abs(tilt_error) * 5), 50, 255);

        if (tilt_error > 2.0) { // Jika panel miring ke atas dari target, miringkan ke bawah
            digitalWrite(ACTUATOR_IN1_PIN, HIGH);
            digitalWrite(ACTUATOR_IN2_PIN, LOW);
        } else if (tilt_error < -2.0) { // Jika panel miring ke bawah dari target, miringkan ke atas
            digitalWrite(ACTUATOR_IN1_PIN, LOW);
            digitalWrite(ACTUATOR_IN2_PIN, HIGH);
        } else { // Sudah dekat target
            pwm = 0; // Hentikan PWM jika sudah dekat target
        }
        analogWrite(ACTUATOR_EN_PIN, pwm);
        delay(50); // Penundaan kecil untuk pembacaan sensor dan pergerakan motor
    }
    analogWrite(ACTUATOR_EN_PIN, 0); // Hentikan motor
    digitalWrite(ACTUATOR_IN1_PIN, LOW); // Pastikan pin IN1 dan IN2 LOW untuk menghentikan motor sepenuhnya
    digitalWrite(ACTUATOR_IN2_PIN, LOW);
    Serial.println("Panel dimiringkan ke posisi reset.");

    // Mengatur ulang rotasi horizontal ke titik awal yang diketahui (misalnya, batas kiri atau RESET_HORIZ_AZIMUTH)
    Serial.print("Menggerakkan horizontal ke posisi ");
    Serial.print(RESET_HORIZ_AZIMUTH);
    Serial.println(" derajat...");
    long horiz_timeout = millis() + 10000; // Batas waktu 10 detik

    // Baca kompas sebelum loop untuk mendapatkan posisi awal
    compass.read();
    int currentAzimuth = compass.getAzimuth();

    // Menggunakan kontrol sederhana untuk mencapai RESET_HORIZ_AZIMUTH
    // Loop akan berlanjut sampai error kurang dari 5 derajat atau timeout
    while (abs(currentAzimuth - RESET_HORIZ_AZIMUTH) > 5 && millis() < horiz_timeout) {
        compass.read(); // Baca kompas di setiap iterasi
        currentAzimuth = compass.getAzimuth();

        if (currentAzimuth > RESET_HORIZ_AZIMUTH) { // Jika di kanan dari target, putar kiri
            digitalWrite(GEAR_MOTOR_IN1_PIN, LOW);
            digitalWrite(GEAR_MOTOR_IN2_PIN, HIGH);
        } else if (currentAzimuth < RESET_HORIZ_AZIMUTH) { // Jika di kiri dari target, putar kanan
            digitalWrite(GEAR_MOTOR_IN1_PIN, HIGH);
            digitalWrite(GEAR_MOTOR_IN2_PIN, LOW);
        }
        analogWrite(GEAR_MOTOR_EN_PIN, SIMPLE_PWM_SPEED); // Gunakan kecepatan konstan untuk reset
        delay(50);
    }
    analogWrite(GEAR_MOTOR_EN_PIN, 0); // Hentikan motor
    digitalWrite(GEAR_MOTOR_IN1_PIN, LOW);
    digitalWrite(GEAR_MOTOR_IN2_PIN, LOW);
    Serial.println("Rotasi horizontal kembali ke posisi reset.");
}

// ============================
// FUNGSI KALIBRASI OTOMATIS tiltScaleFactor
// ============================
void autoCalibrateTiltScaleFactor() {
    // Hanya coba kalibrasi jika ada perubahan signifikan untuk menghindari pembagian dengan nol atau noise
    // Memeriksa apakah ada perubahan yang cukup besar pada perbedaan LDR dan input kemiringan
    if (abs(ldrVerticalDiff - lastLDRDiff) > 5 && abs(tiltInput - lastTiltInput) > 0.1) {
        // Hitung faktor skala baru
        tiltScaleFactor = abs(tiltInput - lastTiltInput) / abs(ldrVerticalDiff - lastLDRDiff);
        Serial.print("Kalibrasi otomatis! tiltScaleFactor baru: ");
        Serial.println(tiltScaleFactor);
    }

    // Perbarui nilai terakhir untuk iterasi berikutnya
    lastTiltInput = tiltInput;
    lastLDRDiff   = ldrVerticalDiff;
}

// ============================
// SETUP SISTEM
// ============================
void setup() {
    Serial.begin(115200); // Inisialisasi komunikasi serial
    Wire.begin();         // Inisialisasi komunikasi I2C
    compass.init();       // Inisialisasi kompas QMC5883L

    // Inisialisasi MPU6500
    if (!myMPU6500.init()) {
        Serial.println("MPU6500 tidak merespons! Silakan periksa kabel.");
        while (1); // Hentikan eksekusi jika MPU6500 tidak ditemukan, karena ini sensor kritis
    } else {
        Serial.println("MPU6500 terhubung.");
    }
    delay(2000); // Beri waktu sensor untuk stabil setelah inisialisasi

    // Inisialisasi pin motor sebagai OUTPUT
    pinMode(GEAR_MOTOR_IN1_PIN, OUTPUT);
    pinMode(GEAR_MOTOR_IN2_PIN, OUTPUT);
    pinMode(GEAR_MOTOR_EN_PIN, OUTPUT);

    pinMode(ACTUATOR_IN1_PIN, OUTPUT);
    pinMode(ACTUATOR_IN2_PIN, OUTPUT);
    pinMode(ACTUATOR_EN_PIN, OUTPUT);

    // Set PID ke mode AUTOMATIC agar mulai menghitung output
    horizontalPID.SetMode(AUTOMATIC);
    tiltPID.SetMode(AUTOMATIC);

    // Mengaktifkan mode Access Point (Hotspot) pada ESP32
    WiFi.softAP(ssid, password);
    Serial.println("Access Point aktif. Hubungkan ke SSID: " + String(ssid));
    Serial.println("Alamat IP: " + WiFi.softAPIP().toString());

    // Setup rute untuk server web
    server.on("/log", handleLog); // Rute untuk menampilkan log dan form PID
    server.on("/updatePID", HTTP_POST, handlePIDUpdate); // Rute untuk memperbarui parameter PID
    server.on("/selectMode", HTTP_POST, handleModeSelect); // Rute baru untuk memilih mode kontrol
    server.begin(); // Memulai server web
    Serial.println("Server web dimulai.");

    // Konfigurasi Untuk MPU6500_WE
    myMPU6500.enableGyrDLPF();      // Aktifkan Digital Low Pass Filter untuk Gyro
    myMPU6500.setGyrDLPF(MPU6500_DLPF_6); // Atur filter Gyro
    myMPU6500.setSampleRateDivider(5);    // Atur pembagi sample rate
    myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_500); // Atur rentang Gyro
    myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);   // Atur rentang Akselerometer
    myMPU6500.enableAccDLPF(true);  // Aktifkan Digital Low Pass Filter untuk Akselerometer
    myMPU6500.setAccDLPF(MPU6500_DLPF_3); // Atur filter Akselerometer

    // Reset posisi panel ke posisi awal saat booting
    resetPanelPosition();

    // Sesuaikan panel ke arah Utara menggunakan kompas
    settoNorth();
}

// ============================
// LOOP UTAMA
// ============================
void loop() {
    server.handleClient(); // Tangani permintaan klien web

    // Baca sensor LDR dari pin analog
    int ldr_top_left     = analogRead(LDR_TOP_LEFT_PIN);
    int ldr_top_right    = analogRead(LDR_TOP_RIGHT_PIN);
    int ldr_bottom_left  = analogRead(LDR_BOTTOM_LEFT_PIN);
    int ldr_bottom_right = analogRead(LDR_BOTTOM_RIGHT_PIN);

    // Hitung intensitas cahaya rata-rata dari keempat LDR
    int totalLight = ldr_top_left + ldr_top_right + ldr_bottom_left + ldr_bottom_right;
    double lightIntensity = totalLight / 4.0;

    // --- MODE TIDUR (berdasarkan intensitas cahaya keseluruhan) ---
    if (lightIntensity < LIGHT_INTENSITY_THRESHOLD) {
        if (!isSleepMode) { // Hanya atur ulang sekali saat memasuki mode tidur
            Serial.println("Intensitas cahaya rendah. Memasuki Mode Tidur.");
            isSleepMode = true;
            resetPanelPosition(); // Panggil fungsi reset posisi panel
        }
        // Dalam mode tidur, sistem tidak melakukan pelacakan.
        // Anda bisa menambahkan logika deep sleep di sini untuk menghemat daya.
        // Motor juga harus dihentikan sepenuhnya di mode tidur.
        digitalWrite(GEAR_MOTOR_IN1_PIN, LOW);
        digitalWrite(GEAR_MOTOR_IN2_PIN, LOW);
        analogWrite(GEAR_MOTOR_EN_PIN, 0);
        digitalWrite(ACTUATOR_IN1_PIN, LOW);
        digitalWrite(ACTUATOR_IN2_PIN, LOW);
        analogWrite(ACTUATOR_EN_PIN, 0);

    }
    // --- MODE AKTIF (pelacakan matahari) ---
    else {
        isSleepMode = false; // Set mode ke aktif

        // Hitung error untuk kontrol motor horizontal
        double leftAverage = (ldr_top_left + ldr_bottom_left) / 2.0;
        double rightAverage = (ldr_top_right + ldr_bottom_right) / 2.0;
        horizontalError = leftAverage - rightAverage; // Perbarui variabel global horizontalError

        // Hitung error untuk kontrol motor kemiringan (vertikal)
        double topAverage = (ldr_top_left + ldr_top_right) / 2.0;
        double bottomAverage = (ldr_bottom_left + ldr_bottom_right) / 2.0;
        ldrVerticalDiff = topAverage - bottomAverage; // Perbarui variabel global ldrVerticalDiff

        // Dapatkan data dari MPU6500 untuk pengukuran kemiringan aktual
        xyzFloat gValue = myMPU6500.getGValues(); // Dapatkan nilai akselerometer (x, y, z)
        // Hitung sudut pitch dari data akselerometer.
        tiltInput = atan2(gValue.y, sqrt(gValue.x * gValue.x + gValue.z * gValue.z)) * 180.0 / PI; // Konversi radian ke derajat
        
        // Dapatkan azimuth dari kompas untuk log (perlu dipanggil di sini untuk mendapatkan nilai terbaru)
        compass.read();
        int currentAzimuth = compass.getAzimuth(); // Azimuth saat ini (-180 hingga 180)

        // ===================================
        // LOGIKA KONTROL BERDASARKAN MODE YANG DIPILIH
        // ===================================
        if (currentControlMode == MODE_PID_CONTROL) {
            // --- Skenario B: Kontrol PID ---
            Serial.println("Mode: Kontrol PID");

            // Kontrol Motor Horizontal dengan PID
            horizontalPID.Compute(); // Hitung output PID untuk kontrol horizontal
            int pwmHorizontal = constrain(int(abs(horizontalOutput)), 0, 255); // Batasi nilai PWM antara 0-255

            // Terapkan batasan rotasi horizontal
            if (horizontalOutput > 0) { // Jika output positif (putar searah jarum jam / kanan)
                if (currentAzimuth < MAX_AZIMUTH_LIMIT) { // Izinkan putar kanan jika belum mencapai batas atas
                    digitalWrite(GEAR_MOTOR_IN1_PIN, HIGH);
                    digitalWrite(GEAR_MOTOR_IN2_PIN, LOW);
                } else { // Sudah di atau melewati batas atas, hentikan motor
                    digitalWrite(GEAR_MOTOR_IN1_PIN, LOW);
                    digitalWrite(GEAR_MOTOR_IN2_PIN, LOW);
                    pwmHorizontal = 0; // Pastikan PWM nol
                    Serial.println("Batas rotasi horizontal (kanan) tercapai!");
                }
            } else if (horizontalOutput < 0) { // Jika output negatif (putar berlawanan arah jarum jam / kiri)
                if (currentAzimuth > MIN_AZIMUTH_LIMIT) { // Izinkan putar kiri jika belum mencapai batas bawah
                    digitalWrite(GEAR_MOTOR_IN1_PIN, LOW);
                    digitalWrite(GEAR_MOTOR_IN2_PIN, HIGH);
                } else { // Sudah di atau melewati batas bawah, hentikan motor
                    digitalWrite(GEAR_MOTOR_IN1_PIN, LOW);
                    digitalWrite(GEAR_MOTOR_IN2_PIN, LOW);
                    pwmHorizontal = 0; // Pastikan PWM nol
                    Serial.println("Batas rotasi horizontal (kiri) tercapai!");
                }
            } else { // Jika output nol, hentikan motor
                digitalWrite(GEAR_MOTOR_IN1_PIN, LOW);
                digitalWrite(GEAR_MOTOR_IN2_PIN, LOW);
                pwmHorizontal = 0; // Pastikan PWM nol
            }
            analogWrite(GEAR_MOTOR_EN_PIN, pwmHorizontal); // Terapkan PWM ke pin enable motor

            // Kontrol Motor Kemiringan dengan PID
            tiltSetpoint = ldrVerticalDiff * tiltScaleFactor; // Setpoint kemiringan berdasarkan LDR
            tiltPID.Compute(); // Hitung output PID untuk kontrol kemiringan
            int pwmTilt = constrain(int(abs(tiltOutput)), 0, 255); // Batasi nilai PWM antara 0-255
            if (tiltOutput > 0) { // Jika output positif, miringkan satu arah
                digitalWrite(ACTUATOR_IN1_PIN, HIGH);
                digitalWrite(ACTUATOR_IN2_PIN, LOW);
            } else if (tiltOutput < 0) { // Jika output negatif, miringkan arah lain
                digitalWrite(ACTUATOR_IN1_PIN, LOW);
                digitalWrite(ACTUATOR_IN2_PIN, HIGH);
            } else { // Jika output nol, hentikan motor
                digitalWrite(ACTUATOR_IN1_PIN, LOW);
                digitalWrite(ACTUATOR_IN2_PIN, LOW);
            }
            analogWrite(ACTUATOR_EN_PIN, pwmTilt); // Terapkan PWM ke pin enable aktuator

            // Auto-kalibrasi hanya relevan untuk PID
            autoCalibrateTiltScaleFactor();

        } else if (currentControlMode == MODE_SIMPLE_CONTROL) {
            // --- Skenario A: Kontrol Sederhana (tanpa PID) ---
            Serial.println("Mode: Kontrol Sederhana");

            int pwmHorizontalSimple = SIMPLE_PWM_SPEED; // Kecepatan PWM untuk kontrol sederhana

            // Kontrol Motor Horizontal Sederhana (On/Off berdasarkan ambang batas)
            if (horizontalError > SIMPLE_HORIZ_THRESHOLD) { // Terlalu banyak cahaya di kiri, putar kanan
                // Terapkan batasan rotasi horizontal
                if (currentAzimuth < MAX_AZIMUTH_LIMIT) { // Izinkan putar kanan jika belum mencapai batas atas
                    digitalWrite(GEAR_MOTOR_IN1_PIN, HIGH);
                    digitalWrite(GEAR_MOTOR_IN2_PIN, LOW);
                } else { // Sudah di atau melewati batas atas, hentikan motor
                    digitalWrite(GEAR_MOTOR_IN1_PIN, LOW);
                    digitalWrite(GEAR_MOTOR_IN2_PIN, LOW);
                    pwmHorizontalSimple = 0;
                    Serial.println("Batas rotasi horizontal (kanan) tercapai! (Sederhana)");
                }
            } else if (horizontalError < -SIMPLE_HORIZ_THRESHOLD) { // Terlalu banyak cahaya di kanan, putar kiri
                // Terapkan batasan rotasi horizontal
                if (currentAzimuth > MIN_AZIMUTH_LIMIT) { // Izinkan putar kiri jika belum mencapai batas bawah
                    digitalWrite(GEAR_MOTOR_IN1_PIN, LOW);
                    digitalWrite(GEAR_MOTOR_IN2_PIN, HIGH);
                } else { // Sudah di atau melewati batas bawah, hentikan motor
                    digitalWrite(GEAR_MOTOR_IN1_PIN, LOW);
                    digitalWrite(GEAR_MOTOR_IN2_PIN, LOW);
                    pwmHorizontalSimple = 0;
                    Serial.println("Batas rotasi horizontal (kiri) tercapai! (Sederhana)");
                }
            } else { // Dalam rentang toleransi, hentikan
                digitalWrite(GEAR_MOTOR_IN1_PIN, LOW);
                digitalWrite(GEAR_MOTOR_IN2_PIN, LOW);
                pwmHorizontalSimple = 0;
            }
            analogWrite(GEAR_MOTOR_EN_PIN, pwmHorizontalSimple);

            // Kontrol Motor Kemiringan Sederhana (On/Off berdasarkan ambang batas)
            if (ldrVerticalDiff > SIMPLE_TILT_THRESHOLD) { // Terlalu banyak cahaya di atas, miringkan ke bawah
                digitalWrite(ACTUATOR_IN1_PIN, HIGH);
                digitalWrite(ACTUATOR_IN2_PIN, LOW);
                analogWrite(ACTUATOR_EN_PIN, SIMPLE_PWM_SPEED);
            } else if (ldrVerticalDiff < -SIMPLE_TILT_THRESHOLD) { // Terlalu banyak cahaya di bawah, miringkan ke atas
                digitalWrite(ACTUATOR_IN1_PIN, LOW);
                digitalWrite(ACTUATOR_IN2_PIN, HIGH);
                analogWrite(ACTUATOR_EN_PIN, SIMPLE_PWM_SPEED);
            } else { // Dalam rentang toleransi, hentikan
                digitalWrite(ACTUATOR_IN1_PIN, LOW);
                digitalWrite(ACTUATOR_IN2_PIN, LOW);
                analogWrite(ACTUATOR_EN_PIN, 0);
            }
            // Auto-kalibrasi tidak relevan untuk kontrol sederhana
        }

        // Tambahkan data log ke buffer untuk ditampilkan di halaman web
        String currentModeString = (currentControlMode == MODE_PID_CONTROL ? "Skenario B (PID)" : "Skenario A (Sederhana)");
        logBuffer = "<p><strong>Mode Kontrol:</strong> " + currentModeString +
                    " | Cahaya: " + String(lightIntensity, 2) +
                    " | Azimuth: " + String(currentAzimuth) + "°" +
                    " | Error Horiz.: " + String(horizontalError, 2) +
                    " | Output Horiz.: " + String(horizontalOutput, 2) +
                    " | Input Kemiringan (IMU): " + String(tiltInput, 2) + "°" +
                    " | Setpoint Kemiringan: " + String(tiltSetpoint, 2) + "°" +
                    " | Output Kemiringan: " + String(tiltOutput, 2) +
                    " | Selisih Vertikal LDR: " + String(ldrVerticalDiff, 2) +
                    " | LDR (TL:" + ldr_top_left + " TR:" + ldr_top_right + " BL:" + ldr_bottom_left + " BR:" + ldr_bottom_right + ")" +
                    " | MPU-G (X:" + String(gValue.x, 2) + " Y:" + String(gValue.y, 2) + " Z:" + String(gValue.z, 2) + ")" +
                    "</p>" + logBuffer;

    } // Akhir dari blok MODE AKTIF

    // Batasi panjang logBuffer agar tidak terlalu besar di memori (opsional)
    if (logBuffer.length() > 8000) {
        logBuffer = logBuffer.substring(0, 8000);
    }

    delay(500); // Penundaan antara setiap iterasi loop (sesuaikan sesuai kebutuhan responsivitas)
}
