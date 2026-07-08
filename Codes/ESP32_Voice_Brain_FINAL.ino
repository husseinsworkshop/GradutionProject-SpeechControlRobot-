#include <WiFi.h>
#include <WebServer.h>
#include <WiFiManager.h>

WebServer server(80);

// ==================================================
// أقطاب التوصيل (UART و MP3)
// ==================================================
#define RXD2 16
#define TXD2 17
#define MP3_RX 4
#define MP3_TX 5

// ==================================================
// متغيرات حالة الروبوت والمهام
// ==================================================
enum Location { LOC_EMERGENCY = 0, LOC_RECEPTION = 1, LOC_PHARMACY = 2, LOC_ON_THE_WAY = 3 };
Location currentLocation = LOC_EMERGENCY;

bool isMoving = false;
bool hasMedicine = false;
bool     obstacleWarningActive = false;
unsigned long lastObstaclePlay = 0;
int currentMissionOwner = 0; // 0=None, 1=Receptionist, 2=Nurse
int requestedMedicine = 0;   // 0=None, 1=Paracetamol, 2=Ponstan

String lastCommandKey = "";
unsigned long lastCommandMillis = 0;

// ==================================================
// مشغل الصوت (DFPlayer Mini)
// ==================================================
void sendMP3Command(uint8_t command, uint8_t para_h, uint8_t para_l) {
  uint8_t buf[10] = {0x7E, 0xFF, 0x06, command, 0x00, para_h, para_l, 0x00, 0x00, 0xEF};
  uint16_t checksum = 0 - (buf[1] + buf[2] + buf[3] + buf[4] + buf[5] + buf[6]);
  buf[7] = (uint8_t)(checksum >> 8); 
  buf[8] = (uint8_t)(checksum & 0xFF); 
  for (uint8_t i = 0; i < 10; i++) Serial1.write(buf[i]);
  delay(50);
}

void playTrack(uint8_t track) {
  sendMP3Command(0x0F, 1, track);
  Serial.printf("[AUDIO] Playing Track: %03d\n", track);
}

// ==================================================
// إرسال الأوامر (Flags) للروبوت
// ==================================================
void sendUARTAction(uint8_t flagId) {
  uint8_t frame[] = {0xAA, 0x20, flagId, 0xBB};
  Serial2.write(frame, 4);
  delay(20);
  Serial2.write(frame, 4); // إرسال مرتين لضمان الوصول
  Serial2.flush();
  Serial.printf("[UART SEND] Sent Action Flag: %03d\n", flagId);
}

// ==================================================
// الكلمات المفتاحية المبسطة
// ==================================================
const String recepCallWords[]    = { "استقبال", "ريسبشن" };
const String pharmWords[]        = { "صيدليه", "صيدلية" };
const String emergWords[]        = { "طوارئ", "مرضى" };

const String nurseBringWords[]   = { "جيب", "احضر", "أحضر", "اجلب", "أجلب", "جيبلي" };
const String nurseParaWords[]    = { "باراسيتامول", "براسيتول", "براسيتمول" };
const String nursePonstanWords[] = { "بونستان", "بون ستان" };
const String nurseGiveWords[]    = { "انطيني", "اعطيني","اعطني" ,"أعطني","أعطيني" };

const String pharmTakeWords[]    = { "خذ", "اخذ", "خذه" ,"امسك,"};
const String pharmGraspWords[]   = { };

const String whereAmIWords[]     = { "وين", "اين", "أين" };

// ==================================================
// دوال معالجة النصوص والمطابقة
// ==================================================
String normalizeText(String text) {
  text.toLowerCase();
  text.replace("أ", "ا"); text.replace("إ", "ا"); text.replace("آ", "ا");
  text.replace("ى", "ي"); text.replace("ة", "ه");
  text.trim(); return text;
}

bool containsWord(const String &text, const String keywords[], int count) {
  for (int i = 0; i < count; i++) { if (text.indexOf(keywords[i]) != -1) return true; }
  return false;
}

// ==================================================
// المحرك الرئيسي لمعالجة السيناريوهات
// ==================================================
void processVoiceCommand(String role, String text) {
  role = normalizeText(role);
  text = normalizeText(text);
  
  // 1) فلتر لمنع التكرار (Anti-Spam)
  String key = role + "|" + text;
  if (key == lastCommandKey && (millis() - lastCommandMillis < 3000)) return; 
  lastCommandKey = key; lastCommandMillis = millis();

  Serial.printf("\n>>> [ROLE]: %s | [CMD]: %s\n", role.c_str(), text.c_str());

  // 2) سؤال "أين أنت؟" (متاح للجميع)
  if (containsWord(text, whereAmIWords, 3)) {
    if (isMoving) { playTrack(21); }
    else if (currentLocation == LOC_RECEPTION) { playTrack(18); }
    else if (currentLocation == LOC_PHARMACY)  { playTrack(19); }
    else if (currentLocation == LOC_EMERGENCY) { playTrack(20); }
    return;
  }

  // 3) منع إعطاء أوامر أثناء الحركة (باستثناء أين أنت)
  if (isMoving) {
    playTrack(8); // الحالة 8: أنا مشغول بمهمة حالية
    return;
  }

  // ------------------------------------------------
  // 1) سكربتات الريسبشن
  // ------------------------------------------------
  if (role == "receptionist") {
    if (containsWord(text, recepCallWords, 2)) {
      if (currentLocation == LOC_RECEPTION) { 
        playTrack(2); // الحالة 2: هو أصلاً في الاستقبال
      } else {
        playTrack(1); sendUARTAction(1); // الحالة 1: 001 = MOVE_TO_RECEPTION
        isMoving = true; currentMissionOwner = 1; currentLocation = LOC_ON_THE_WAY;
      }
    } 
    else if (containsWord(text, pharmWords, 2)) {
      playTrack(3); sendUARTAction(2); // الحالة 3: 002 = MOVE_TO_PHARMACY
      isMoving = true; currentMissionOwner = 1; currentLocation = LOC_ON_THE_WAY;
    } 
    else if (containsWord(text, emergWords, 2)) {
      playTrack(3); sendUARTAction(3); // الحالة 4: 003 = MOVE_TO_EMERGENCY
      isMoving = true; currentMissionOwner = 1; currentLocation = LOC_ON_THE_WAY;
    }
    else { playTrack(23); } // الأمر غير مفهوم
  }
  
  // ------------------------------------------------
  // 2) سكربتات الممرض
  // ------------------------------------------------
  else if (role == "nurse") {
    if (containsWord(text, nurseBringWords, 6)) {
      if (containsWord(text, nurseParaWords, 3)) {
        playTrack(9); sendUARTAction(2); // الحالة 9: 002 = MOVE_TO_PHARMACY
        isMoving = true; currentMissionOwner = 2; requestedMedicine = 1; currentLocation = LOC_ON_THE_WAY;
      } 
      else if (containsWord(text, nursePonstanWords, 2)) {
        playTrack(10); sendUARTAction(2); // الحالة 10: 002 = MOVE_TO_PHARMACY
        isMoving = true; currentMissionOwner = 2; requestedMedicine = 2; currentLocation = LOC_ON_THE_WAY;
      } 
      else {
        playTrack(11); // لم يحدد اسم الدواء "أي دواء تريد أن أجلب؟"
      }
    }
    else if (containsWord(text, nurseGiveWords, 3)) {
      if (hasMedicine) {
        playTrack(14); sendUARTAction(20); // الحالة 14: تفضل الدواء (020 = RELEASE_ITEM)
        hasMedicine = false; requestedMedicine = 0;
      } else {
        playTrack(22); // الحالة 22: هذا الأمر غير متاح (لأنه لا يملك دواء)
      }
    }
    else { playTrack(23); } // غير مفهوم
  }
  
  // ------------------------------------------------
  // 3) سكربتات الصيدلي
  // ------------------------------------------------
  else if (role == "pharmacist") {
    if (containsWord(text, pharmTakeWords, 3)) {
      playTrack(16); sendUARTAction(11); // الحالة 16: 011 = ARM_READY_TO_TAKE
    } 
    else if (containsWord(text, pharmGraspWords, 3)) {
      // احتياط: في حال أراد الصيدلي الأمر اليدوي 
      playTrack(17); sendUARTAction(12); // الحالة 17: 012 = ARM_GRASP_AND_RETURN
      hasMedicine = true; isMoving = true; currentMissionOwner = 0; currentLocation = LOC_ON_THE_WAY;
    }
    else { playTrack(23); } // الأمر غير مفهوم
  }
  
  else {
    playTrack(22); // الحالة 22: هذا الأمر غير متاح لهذا المستخدم
  }
}

// ==================================================
// خوادم الويب (Web HTTP Handlers)
// ==================================================
void sendCORSHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "*");
}

void handleCommand() {
  sendCORSHeaders();
  if (!server.hasArg("text")) { server.send(400, "text/plain", "Missing text"); return; }
  String role = server.hasArg("role") ? server.arg("role") : "nurse"; 
  processVoiceCommand(role, server.arg("text")); 
  server.send(200, "text/plain", "OK");
}

// ==================================================
// الإعدادات الأولية Setup
// ==================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // To Rover ESP32
  Serial1.begin(9600, SERIAL_8N1, MP3_RX, MP3_TX); // To DFPlayer Mini
  delay(500);

  sendMP3Command(0x09, 0x00, 0x02); delay(500); // Select SD Card
  sendMP3Command(0x06, 0x00, 30);   // Volume 30

  // اتصال الواي فاي المحلي باستخدام WiFiManager
  WiFiManager wm;
  Serial.println("Starting WiFiManager...");
  
  // إذا لم يجد شبكة محفوظة، سيقوم ببث شبكة باسم "Voice_ESP32_AP" لتتصل بها بهاتفك وتدخل الرمز السري لشبكتك المنزلية
  if (wm.autoConnect("Voice_ESP32_AP", "12345678")) {
    Serial.println("\nWiFi Connected!");
    Serial.print("IP Address for Web Commands: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to WiFi.");
  }

  server.on("/command", HTTP_GET, handleCommand);
  server.on("/status", HTTP_GET, [](){ sendCORSHeaders(); server.send(200, "text/plain", "connected"); });
  server.on("/command", HTTP_OPTIONS, [](){ sendCORSHeaders(); server.send(204); });
  server.begin();
  
  Serial.println("Voice Brain System Ready.");
}

// ==================================================
// حلقة التكرار الأساسية (استقبال الـ Flags من الروبوت)
// ==================================================
void loop() {
  server.handleClient();

  // --- استقبال الـ Flags من الروبوت عبر UART ---
  static enum { WAIT_START, WAIT_TYPE, WAIT_CODE, WAIT_END } uartState = WAIT_START;
  static uint8_t rxType = 0, rxCode = 0;

  while (Serial2.available() > 0) {
    uint8_t c = Serial2.read();
    switch(uartState) {
      case WAIT_START: if (c == 0xAA) uartState = WAIT_TYPE; break;
      case WAIT_TYPE: 
        if (c == 0x20) { rxType = c; uartState = WAIT_CODE; } // 0x20 تعني استقبال Action / Report
        else uartState = WAIT_START; 
        break;
      case WAIT_CODE: rxCode = c; uartState = WAIT_END; break;
      case WAIT_END:
        if (c == 0xBB) {
          Serial.printf("<<< [UART RECEIVED] Report From Rover: %03d\n", rxCode);

          if (rxCode == 13) { 
            // 013 = الروبوت وصل إلى الريسبشن
            currentLocation = LOC_RECEPTION; isMoving = false;
            playTrack(2); // الحالة 2: "إلى أين تريد أن تذهب؟"
          } 
          else if (rxCode == 14) { 
            // 014 = الروبوت وصل إلى الصيدلية
            currentLocation = LOC_PHARMACY; isMoving = false;
            if (currentMissionOwner == 1) { 
              playTrack(5); // الحالة 5: استدعاء من الريسبشن: "وصلت إلى وجهتك"
            } 
            else if (currentMissionOwner == 2) { 
              // استدعاء من الممرض: يطلب الدواء المحدد ويجهز الذراع تلقائياً
              if (requestedMedicine == 1) playTrack(12); // الحالة 12
              else if (requestedMedicine == 2) playTrack(13); // الحالة 13
              sendUARTAction(11); // إرسال أمر فتح الذراع (011 = ARM_READY_TO_TAKE)
            }
          } 
          else if (rxCode == 15) {
            // 015 = الروبوت وصل إلى الطوارئ
            currentLocation = LOC_EMERGENCY; isMoving = false;
            playTrack(5); // الحالة 5: "وصلت إلى وجهتك"
            currentMissionOwner = 0; // الروبوت جاهز لمهمة جديدة
          } 
          else if (rxCode == 12) {
            // 012 = الروبوت التقط الدواء بحساس الـ IR وهو في طريق العودة (الحالة 17)
            hasMedicine = true; isMoving = true; currentLocation = LOC_ON_THE_WAY;
            playTrack(17); // "تم استلام الدواء والعودة إلى غرفة الطوارئ"
          }
          else if (rxCode == 30) {
  // Obstacle detected — play warning track 030
  obstacleWarningActive = true;
  lastObstaclePlay = 0; // force immediate play on next loop check
}
else if (rxCode == 31) {
  // Obstacle cleared
  obstacleWarningActive = false;
  Serial.println("[OBSTACLE] Cleared.");
}
        }
        uartState = WAIT_START;
        break;
    }
  }
  // Obstacle warning — repeat track 030 every 5 seconds
if (obstacleWarningActive) {
  if (millis() - lastObstaclePlay >= 5000) {
    lastObstaclePlay = millis();
    playTrack(30);
    Serial.println("[OBSTACLE] Playing warning track 030");
  }
}
}