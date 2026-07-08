#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WebSocketsServer.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoOTA.h> // <-- ADDED: OTA Library

// ===================== WIFI AP ========================
const char* ssid     = "Rover_Command_Center";
const char* password = "12345678";

// ===================== UART (Voice ESP32) ==============
#define RXD2 16
#define TXD2 17

#define UART_START        0xAA
#define UART_END          0xBB
#define UART_TYPE_ACTION  0x20

// ===================== LINE SENSORS ===================
const int sensorPins[] = { 36, 39, 34, 35, 33 };
const int numSensors   = 5;

// ===================== HALL EFFECT (station magnet) ===
#define HALL_PIN 23
bool hallActiveLow = true; // Configurable from web interface

// ===================== IR GRIPPER SENSOR ==============
#define IR_GRIPPER_PIN 4

// ===================== DC MOTORS ======================
const int ENA = 14, IN1 = 27, IN2 = 18;
const int IN3 = 25, IN4 = 19, ENB = 13;
const int freq = 30000, resolution = 8;
const int channelL = 0; 
const int channelR = 1; 

// ===================== ULTRASONIC =====================
#define TRIG_PIN 32
#define ECHO_PIN 26
float stopDistance = 15.0;
bool radarEnabled  = true;
bool obstacleDetected = false;
unsigned long lastPingTime = 0;
int currentDistance = 999;
bool          obstaclePlaying    = false;
unsigned long lastObstacleReport = 0;
// ===================== SERVO (PCA9685) ================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 60
#define SERVOMIN  150
#define SERVOMAX  600

#define SRV_SHOULDER_L 0
#define SRV_SHOULDER_R 1
#define SRV_ELBOW      2
#define SRV_WRIST_V    3
#define SRV_WRIST_ROT  5   
#define SRV_GRIPPER    4   
#define SRV_BASE       6

// ===================== PID STATE ======================
float Kp = 10.0, Ki = 0.0, Kd = 0.0;
int   baseSpeed = 150;
float maxRpmL = 150.0, maxRpmR = 150.0, motorKp = 1.0;
bool  useEncoders = false, useAccel = true, useCurve = true;
float accelRate = 10.0, curveSlow = 0.05;
float currentLeftPWM = 0, currentRightPWM = 0;
int   sensorDebounce = 20;

unsigned long sensorLowStart[5] = {0,0,0,0,0};
bool  isCalibrating[5]          = {false,false,false,false,false};
int   minValues[]  = {4095,4095,4095,4095,4095};
int   maxValues[]  = {0,0,0,0,0};
int   thresholds[] = {2000,2000,2000,2000,2000};

bool  motorsEnabled = false;
float error=0, P=0, I=0, D=0, previousError=0, pidValue=0;
int   rawAnalogValues[5], digitizedValues[5];

// ===================== ENCODERS =======================
const int ENC_L = 5;  
const int ENC_R = 2;  
volatile unsigned long lastTimeL=0, lastTimeR=0, deltaL=0, deltaR=0;
float rpmL=0, rpmR=0;

void IRAM_ATTR isrLeft()  { unsigned long n=micros(); if(n-lastTimeL>20000){deltaL=n-lastTimeL;lastTimeL=n;} }
void IRAM_ATTR isrRight() { unsigned long n=micros(); if(n-lastTimeR>20000){deltaR=n-lastTimeR;lastTimeR=n;} }

// ===================== SERVO STATE ====================
int   targetAngles[7]      = {180,180,180,38,0,100,90};
float currentFloatAngles[7]= {180,180,180,38,0,100,90};
float servoRates[7]        = {2,2,2,2,2,2,2};
bool  anglesChanged        = false;
unsigned long lastAnglesSave=0, lastServoUpdate=0;

// ===================== MACRO PROFILES =================
#define MAX_STEPS 50
int recordedSteps[MAX_STEPS][7];
int stepCount = 0;
bool isPlaying  = false;
int  playIndex  = 0;
unsigned long lastPlayTime = 0;
int  playDelay  = 1000;

// ===================== NAVIGATION STATE MACHINE =======
#define STATION_EMERGENCY  0
#define STATION_RECEPTION  1
#define STATION_PHARMACY   2
#define STATION_NONE      -1

int currentStation = STATION_EMERGENCY;
int targetStation  = STATION_NONE;

bool inStationZone = false;
unsigned long lastStationTime = 0;

// Delay for station passing
bool stationStopPending = false;
unsigned long stationStopTime = 0;
int stationPauseTime = 1000; // Configurable from web interface

// Auto Catch/Return Delays
bool irGripperArmed = false;
bool autoReturnPending = false;
bool autoReturnDelaying = false;
unsigned long autoReturnTimer = 0;

// ===================== SERVERS ========================
WebSocketsServer webSocket = WebSocketsServer(81);
WebServer server(80);
Preferences preferences;

// ===================== HTML DASHBOARD =================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body{font-family:Arial;text-align:center;margin:10px;background-color:#eee;}
    .card{background:white;padding:20px;border-radius:10px;box-shadow:0 4px 8px rgba(0,0,0,.1);max-width:500px;margin:auto;}
    .btn{padding:15px 10px;border:none;cursor:pointer;border-radius:5px;font-size:16px;flex:1;margin:5px 0;color:white;font-weight:bold;}
    .btn-green{background-color:#4CAF50;} .btn-red{background-color:#f44336;}
    .btn-blue{background-color:#2196F3;} .btn-gray{background-color:#607D8B;}
    .btn-warn{background-color:#ff9800;} .btn-purple{background-color:#9C27B0;}
    .btn-teal{background-color:#009688;}
    .sensor-row{display:flex;align-items:center;justify-content:space-between;margin:12px 0;}
    .val-text{width:45px;font-weight:bold;text-align:left;font-size:14px;}
    .bar-bg{flex-grow:1;height:16px;background-color:#ddd;margin:0 10px;position:relative;border-radius:8px;}
    .bar{height:100%;background-color:#2196F3;width:0%;border-radius:8px;transition:width .1s;}
    .marker{position:absolute;top:-4px;width:3px;height:24px;background-color:red;z-index:5;}
    .digi-box{width:30px;height:30px;border-radius:5px;background-color:#ccc;color:white;font-weight:bold;line-height:30px;margin-right:10px;text-align:center;}
    .digi-1{background-color:#ff9800;box-shadow:0 0 8px #ff9800;} .digi-0{background-color:#2196F3;}
    .btn-cal{padding:8px 10px;font-size:12px;cursor:pointer;border-radius:4px;border:none;background-color:#607D8B;color:white;width:55px;font-weight:bold;flex:none;}
    .btn-cal.active{background-color:#ff9800;}
    .rpm-display{background:#333;color:#0f0;padding:15px;border-radius:8px;font-family:monospace;font-size:18px;margin:10px 0;display:flex;justify-content:space-around;}
    .obs-alert{background:#f44336;color:white;font-weight:bold;animation:pulse 1s infinite;}
    @keyframes pulse{0%{opacity:1;}50%{opacity:.5;}100%{opacity:1;}}
    input[type="text"],input[type="number"]{width:60px;text-align:center;padding:5px;font-size:14px;}
    input[type="range"]{width:100%;height:25px;margin-bottom:10px;accent-color:#2196F3;}
    input[type="checkbox"]{transform:scale(1.3);margin-right:5px;}
    .flex-row{display:flex;justify-content:center;gap:10px;margin-bottom:10px;flex-wrap:wrap;}
    .button-group{display:flex;gap:10px;flex-wrap:wrap;justify-content:center;margin-bottom:10px;}
    .profile-group{display:flex;gap:5px;justify-content:space-between;margin-bottom:10px;flex-wrap:wrap;}
    .profile-btn{padding:8px;border:none;color:white;border-radius:4px;font-weight:bold;cursor:pointer;flex:1;min-width:18%;font-size:12px;}
    hr{border:0;border-top:1px solid #ccc;margin:20px 0;} h3{color:#333;}
    .slider-row{display:flex;justify-content:space-between;font-size:14px;font-weight:bold;margin-top:10px;}
    #connStatus{padding:8px;border-radius:5px;font-weight:bold;margin-bottom:15px;color:white;background-color:#f44336;}
    #navStatus{background:#1a237e;color:#fff;padding:8px;border-radius:5px;margin-bottom:10px;font-weight:bold;font-size:18px;}
    .reset-home-btn{width:100%;padding:14px;background:linear-gradient(135deg,#b71c1c,#e53935);border:none;color:white;border-radius:8px;font-size:15px;font-weight:bold;cursor:pointer;margin-bottom:10px;letter-spacing:1px;}
    .reset-home-btn:hover{opacity:0.88;}
    .magnet-box { display:flex; align-items:center; justify-content:space-between; background: #e3f2fd; padding: 12px; border-radius: 8px; margin: 12px 0; border: 2px solid #2196F3; }
  </style>
  <script>
    let socket; let lastSrvTime=0; var calState=[false,false,false,false,false];
    function toggleMotors(){fetch("/toggle");}
    function toggleEncoders(){fetch("/toggleEncoders");}
    function toggleRadar(){fetch("/toggleRadar");}
    function resetHome(){fetch("/resetHome").then(()=>alert("Station Reset to Emergency (0)"));}
    function toggleCalibrate(s){
      calState[s]=!calState[s];
      var b=document.getElementById("btnCal"+s),st=calState[s]?1:0;
      if(calState[s]){b.innerText="SAVE";b.className="btn-cal active";}
      else{b.innerText="CAL";b.className="btn-cal";}
      fetch("/calibrate?sensor="+s+"&state="+st);
    }
    function saveSettings(){
      var kp=document.getElementById("kp").value,ki=document.getElementById("ki").value,kd=document.getElementById("kd").value;
      var spd=document.getElementById("speed").value,ml=document.getElementById("maxRpmL").value,mr=document.getElementById("maxRpmR").value;
      var mkp=document.getElementById("mkp").value,acc=document.getElementById("accel").value,curv=document.getElementById("curve").value;
      var deb=document.getElementById("deb").value;
      var pTime=document.getElementById("pauseTime").value;
      var uAcc=document.getElementById("chkAcc").checked?1:0,uCurv=document.getElementById("chkCurv").checked?1:0;
      var hLow=document.getElementById("chkHall").checked?1:0;
      var sr0=document.getElementById("sr0").value,sr2=document.getElementById("sr2").value,sr3=document.getElementById("sr3").value;
      var sr4=document.getElementById("sr4").value,sr5=document.getElementById("sr5").value,sr6=document.getElementById("sr6").value;
      fetch(`/save?kp=${kp}&ki=${ki}&kd=${kd}&speed=${spd}&ml=${ml}&mr=${mr}&mkp=${mkp}&acc=${acc}&curv=${curv}&deb=${deb}&pause=${pTime}&uacc=${uAcc}&ucurv=${uCurv}&hlow=${hLow}&sr0=${sr0}&sr2=${sr2}&sr3=${sr3}&sr4=${sr4}&sr5=${sr5}&sr6=${sr6}`)
        .then(()=>alert("Settings Saved Successfully!"));
    }
    function requestStatus(){
      fetch("/status").then(r=>r.json()).then(data=>{
        for(var i=0;i<5;i++){
          document.getElementById("v"+i).innerText=data.raw[i];
          document.getElementById("b"+i).style.width=((data.raw[i]/4095)*100)+"%";
          document.getElementById("m"+i).style.left=((data.t[i]/4095)*100)+"%";
          var d=document.getElementById("d"+i);
          d.innerText=data.digi[i];
          if(data.digi[i]==1){d.className="digi-box digi-1";document.getElementById("b"+i).style.backgroundColor="#ff9800";}
          else{d.className="digi-box digi-0";document.getElementById("b"+i).style.backgroundColor="#2196F3";}
        }
        document.getElementById("rpmL").innerText=data.rpmL.toFixed(1);
        document.getElementById("rpmR").innerText=data.rpmR.toFixed(1);
        
        // MAGNET STATUS UPDATE
        if(data.hall==1){
          document.getElementById("hallStateBox").innerText="MAGNET";
          document.getElementById("hallStateBox").className="digi-box digi-1";
        } else {
          document.getElementById("hallStateBox").innerText="NONE";
          document.getElementById("hallStateBox").className="digi-box digi-0";
        }

        var ds=document.getElementById("distDisplay"),rb=document.getElementById("rBtn");
        if(data.rE){rb.innerText="RADAR: ON";rb.className="btn btn-blue";
          if(data.obs){ds.innerText="OBSTACLE: "+data.dist+" cm";document.getElementById("rpmBar").className="rpm-display obs-alert";}
          else{ds.innerText="Clear: "+data.dist+" cm";document.getElementById("rpmBar").className="rpm-display";}
        }else{rb.innerText="RADAR: OFF";rb.className="btn btn-gray";ds.innerText="Radar: OFF";document.getElementById("rpmBar").className="rpm-display";}
        var mb=document.getElementById("mBtn");
        if(data.m){mb.innerText="STOP MOTORS";mb.className="btn btn-red";}
        else{mb.innerText="START MOTORS";mb.className="btn btn-green";}
        var eb=document.getElementById("eBtn");
        if(data.useE){eb.innerText="ENCODERS: ON";eb.className="btn btn-blue";}
        else{eb.innerText="ENCODERS: OFF";eb.className="btn btn-gray";}
        if(data.nav) document.getElementById("navStatus").innerText="NAV: "+data.nav;
        setTimeout(requestStatus,200);
      }).catch(e=>setTimeout(requestStatus,1000));
    }
    function initWS(){
      socket=new WebSocket('ws://'+window.location.hostname+':81/');
      socket.onopen=function(){
        document.getElementById("connStatus").innerText="WebSockets Connected \u2714";
        document.getElementById("connStatus").style.backgroundColor="#4CAF50";
        socket.send("GET_STATE");
      };
      socket.onmessage=function(event){
        let msg=event.data;
        if(msg.startsWith("S:")){let p=msg.split(":");if(document.getElementById("s"+p[1])){document.getElementById("s"+p[1]).value=p[2];document.getElementById("sv"+p[1]).innerText=p[2];}}
        else if(msg.startsWith("STEPS:")){document.getElementById("stepTxt").innerText="Recorded Steps: "+msg.split(":")[1];}
        else if(msg.startsWith("MSG:")){alert(msg.substring(4));}
      };
      socket.onclose=function(){
        document.getElementById("connStatus").innerText="Disconnected!";
        document.getElementById("connStatus").style.backgroundColor="#f44336";
        setTimeout(initWS,2000);
      };
    }
    function srv(id,val){document.getElementById("sv"+id).innerText=val;let n=Date.now();if(n-lastSrvTime>30){socket.send("S:"+id+":"+val);lastSrvTime=n;}}
    function resetArm(){let a={6:90,0:180,2:180,3:38,4:0,5:100};for(let id in a){document.getElementById("s"+id).value=a[id];document.getElementById("sv"+id).innerText=a[id];}socket.send("RESET");}
    function cmd(a){socket.send(a);}
    window.onload=function(){setTimeout(requestStatus,500);initWS();};
  </script>
</head>
<body><div class="card">
  <div id="connStatus">Connecting...</div>
  <div id="navStatus">NAV: IDLE</div>
  <button class="reset-home-btn" onclick="resetHome()">&#8962; RESET STATION TO EMERGENCY (0)</button>
  <h2>Rover Control</h2>
  <div class="button-group">
    <button id="mBtn" class="btn btn-green" onclick="toggleMotors()">START MOTORS</button>
    <button id="eBtn" class="btn btn-gray"  onclick="toggleEncoders()">ENCODERS: OFF</button>
  </div>
  <div class="rpm-display" id="rpmBar" style="flex-direction:column;align-items:center;gap:5px;">
    <div style="display:flex;justify-content:space-between;width:100%;align-items:center;">
      <button id="rBtn" class="btn btn-blue" style="flex:none;width:90px;padding:5px;font-size:12px;margin:0;" onclick="toggleRadar()">RADAR: ON</button>
      <div id="distDisplay" style="font-size:20px;font-weight:bold;">Radar: --</div>
      <div style="width:90px;"></div>
    </div>
    <div style="display:flex;width:100%;justify-content:space-around;margin-top:5px;">
      <div>Left RPM: <span id="rpmL">0.0</span></div>
      <div>Right RPM: <span id="rpmR">0.0</span></div>
    </div>
  </div>
  <hr>
  <h3>Sensor Diagnostics</h3>
  <div id="sensors">
    <div class="sensor-row"><div id="d0" class="digi-box">0</div><span class="val-text" id="v0">0</span><div class="bar-bg"><div id="b0" class="bar"></div><div id="m0" class="marker"></div></div><button id="btnCal0" class="btn-cal" onclick="toggleCalibrate(0)">CAL</button></div>
    <div class="sensor-row"><div id="d1" class="digi-box">0</div><span class="val-text" id="v1">0</span><div class="bar-bg"><div id="b1" class="bar"></div><div id="m1" class="marker"></div></div><button id="btnCal1" class="btn-cal" onclick="toggleCalibrate(1)">CAL</button></div>
    <div class="sensor-row"><div id="d2" class="digi-box">0</div><span class="val-text" id="v2">0</span><div class="bar-bg"><div id="b2" class="bar"></div><div id="m2" class="marker"></div></div><button id="btnCal2" class="btn-cal" onclick="toggleCalibrate(2)">CAL</button></div>
    <div class="sensor-row"><div id="d3" class="digi-box">0</div><span class="val-text" id="v3">0</span><div class="bar-bg"><div id="b3" class="bar"></div><div id="m3" class="marker"></div></div><button id="btnCal3" class="btn-cal" onclick="toggleCalibrate(3)">CAL</button></div>
    <div class="sensor-row"><div id="d4" class="digi-box">0</div><span class="val-text" id="v4">0</span><div class="bar-bg"><div id="b4" class="bar"></div><div id="m4" class="marker"></div></div><button id="btnCal4" class="btn-cal" onclick="toggleCalibrate(4)">CAL</button></div>
    
    <!-- HIGHLY VISIBLE MAGNET STATE BOX -->
    <div class="magnet-box">
      <span class="val-text" style="width:140px; color:#1565C0; font-size:16px;">Magnet State:</span>
      <div id="hallStateBox" class="digi-box" style="width:90px; font-size:14px; margin:0;">NONE</div>
    </div>
    <div style="text-align: right; margin-bottom: 15px;">
      <label style="font-weight:bold; font-size:14px;"><input type="checkbox" id="chkHall" %CHK_HALL%> Active LOW (Invert Logic)</label>
    </div>
  </div>
  <hr>
  <h3>Advanced & PID Parameters</h3>
  <div class="flex-row">
    <div>Max RPM L:<br><input type="number" id="maxRpmL" value="%MAX_L%"></div>
    <div>Max RPM R:<br><input type="number" id="maxRpmR" value="%MAX_R%"></div>
    <div>Motor Kp:<br><input type="number" id="mkp" value="%MKp%"></div>
  </div>
  <div class="flex-row">
    <div>Line Kp:<br><input type="number" id="kp" value="%Kp%"></div>
    <div>Line Ki:<br><input type="number" id="ki" value="%Ki%"></div>
    <div>Line Kd:<br><input type="number" id="kd" value="%Kd%"></div>
    <div>Base Spd:<br><input type="number" id="speed" value="%SPEED%"></div>
  </div>
  <div class="flex-row">
    <div><label><input type="checkbox" id="chkAcc" %CHK_ACC%> Accel Rate:</label><br><input type="number" id="accel" value="%ACC%"></div>
    <div><label><input type="checkbox" id="chkCurv" %CHK_CURV%> Curve Slow:</label><br><input type="number" id="curve" value="%CURV%"></div>
    <div>Debounce (ms):<br><input type="number" id="deb" value="%DEB%"></div>
    
    <!-- HIGHLY VISIBLE PAUSE TIMER BOX -->
    <div style="background: #fff3e0; padding: 5px; border-radius: 5px; border: 1px solid #2196F3;">
       <span style="font-weight:bold; color:#d32f2f;">Station Pause Timer (ms):</span><br>
       <input type="number" id="pauseTime" value="%PAUSE%" style="width:80px; font-weight:bold;">
    </div>
  </div>
  <hr style="border-top:3px solid #333;">
  <h2>Robotic Arm Control</h2>
  <div class="button-group">
    <button class="btn btn-blue" onclick="cmd('REC')">+ Save Step</button>
    <button class="btn btn-red"  onclick="cmd('CLEAR')">Clear Macro</button>
  </div>
  <div class="rpm-display" style="font-size:16px;padding:10px;background:#222;margin-bottom:5px;" id="stepTxt">Recorded Steps: 0</div>
  
  <div style="background:#333;padding:10px;border-radius:8px;margin-bottom:15px;">
    <h4 style="margin:0 0 10px 0;color:#fff;">Macro Profiles</h4>
    <div class="profile-group">
      <button class="profile-btn btn-purple" onclick="cmd('SAVE_A')">Save A</button>
      <button class="profile-btn btn-purple" onclick="cmd('SAVE_B')">Save B</button>
      <button class="profile-btn btn-purple" onclick="cmd('SAVE_C')">Save C</button>
      <button class="profile-btn btn-purple" onclick="cmd('SAVE_D')">Save D</button>
      <button class="profile-btn btn-purple" onclick="cmd('SAVE_E')">Save E</button>
    </div>
    <div class="profile-group">
      <button class="profile-btn btn-teal" onclick="cmd('LOAD_A')">Load A</button>
      <button class="profile-btn btn-teal" onclick="cmd('LOAD_B')">Load B</button>
      <button class="profile-btn btn-teal" onclick="cmd('LOAD_C')">Load C</button>
      <button class="profile-btn btn-teal" onclick="cmd('LOAD_D')">Load D</button>
      <button class="profile-btn btn-teal" onclick="cmd('LOAD_E')">Load E</button>
    </div>
    <div class="profile-group">
      <button class="profile-btn btn-green" onclick="cmd('PLAY_A')">&#9654; Play A</button>
      <button class="profile-btn btn-green" onclick="cmd('PLAY_B')">&#9654; Play B</button>
      <button class="profile-btn btn-green" onclick="cmd('PLAY_C')">&#9654; Play C</button>
      <button class="profile-btn btn-green" onclick="cmd('PLAY_D')">&#9654; Play D</button>
      <button class="profile-btn btn-green" onclick="cmd('PLAY_E')">&#9654; Play E</button>
    </div>
  </div>

  <div style="background:#e3f2fd;padding:10px;border-radius:8px;margin-bottom:15px;">
    <h4 style="margin:0 0 10px 0;color:#1565C0;">Servo Slew Speeds</h4>
    <div class="slider-row" style="color:#1565C0;"><span>Base</span><span id="sv_sr6">%SR6%</span></div>
    <input type="range" min="0.05" max="10" step="0.05" id="sr6" value="%SR6%" oninput="document.getElementById('sv_sr6').innerText=this.value" onchange="socket.send('SR:6:'+this.value)">
    <div class="slider-row" style="color:#1565C0;"><span>Shoulder</span><span id="sv_sr0">%SR0%</span></div>
    <input type="range" min="0.05" max="10" step="0.05" id="sr0" value="%SR0%" oninput="document.getElementById('sv_sr0').innerText=this.value" onchange="socket.send('SR:0:'+this.value)">
    <div class="slider-row" style="color:#1565C0;"><span>Elbow</span><span id="sv_sr2">%SR2%</span></div>
    <input type="range" min="0.05" max="10" step="0.05" id="sr2" value="%SR2%" oninput="document.getElementById('sv_sr2').innerText=this.value" onchange="socket.send('SR:2:'+this.value)">
    <div class="slider-row" style="color:#1565C0;"><span>Wrist V</span><span id="sv_sr3">%SR3%</span></div>
    <input type="range" min="0.05" max="10" step="0.05" id="sr3" value="%SR3%" oninput="document.getElementById('sv_sr3').innerText=this.value" onchange="socket.send('SR:3:'+this.value)">
    <div class="slider-row" style="color:#1565C0;"><span>Wrist Rot</span><span id="sv_sr5">%SR5%</span></div>
<input type="range" min="0.05" max="10" step="0.05" id="sr5" value="%SR5%" oninput="document.getElementById('sv_sr5').innerText=this.value" onchange="socket.send('SR:5:'+this.value)">
<div class="slider-row" style="color:#1565C0;"><span>Gripper</span><span id="sv_sr4">%SR4%</span></div>
<input type="range" min="0.05" max="10" step="0.05" id="sr4" value="%SR4%" oninput="document.getElementById('sv_sr4').innerText=this.value" onchange="socket.send('SR:4:'+this.value)">
  </div>
  <div class="slider-row"><span>Base (Ch6)</span><span id="sv6">90</span></div>
  <input type="range" min="0" max="180" id="s6" oninput="srv(6,this.value)">
  <div class="slider-row"><span>Shoulder (Ch0+1)</span><span id="sv0">180</span></div>
  <input type="range" min="0" max="180" id="s0" oninput="srv(0,this.value)">
  <div class="slider-row"><span>Elbow (Ch2)</span><span id="sv2">180</span></div>
  <input type="range" min="0" max="180" id="s2" oninput="srv(2,this.value)">
  <div class="slider-row"><span>Wrist Vertical (Ch3)</span><span id="sv3">38</span></div>
  <input type="range" min="0" max="180" id="s3" oninput="srv(3,this.value)">
<div class="slider-row"><span>Wrist Rotate</span><span id="sv5">100</span></div>
<input type="range" min="0" max="180" id="s5" oninput="srv(5,this.value)">
<div class="slider-row"><span>Gripper</span><span id="sv4">0</span></div>
<input type="range" min="0" max="68" id="s4" oninput="srv(4,this.value)">
  <button class="btn btn-gray" style="width:100%" onclick="resetArm()">&#8635; Reset Arm</button>
  <button class="btn btn-green" style="width:100%;margin-top:15px;padding:20px;" onclick="saveSettings()">SAVE ALL SETTINGS</button>
</div></body></html>
)rawliteral";

// ===================== HARDWARE ========================
void setMotorSpeed(int speedLeft, int speedRight) {
  if (obstacleDetected) { speedLeft = 0; speedRight = 0; }
  if (speedLeft >= 0)  { digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); ledcWrite(channelL, speedLeft); }
  else                 { digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);  ledcWrite(channelL, abs(speedLeft)); }
  if (speedRight >= 0) { digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); ledcWrite(channelR, speedRight); }
  else                 { digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);  ledcWrite(channelR, abs(speedRight)); }
}

void writeServoHardware(int id, float floatAngle) {
  int angle = round(floatAngle);
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  if (id == SRV_SHOULDER_L) {
    pwm.setPWM(SRV_SHOULDER_L, 0, pulse);
    pwm.setPWM(SRV_SHOULDER_R, 0, map(180-angle,0,180,SERVOMIN,SERVOMAX));
  } else { pwm.setPWM(id, 0, pulse); }
}

void setTargetServo(int id, int angle) {
  if (angle < 0) angle = 0;
  if (id == SRV_GRIPPER && angle > 68) angle = 68;
  else if (angle > 180) angle = 180;
  targetAngles[id] = angle; anglesChanged = true; lastAnglesSave = millis();
}

// ===================== MACRO PROFILES ==================
void saveProfile(const char* name, int client) {
  preferences.begin(name, false);
  preferences.putInt("count", stepCount);
  for(int i=0; i<stepCount; i++)
    for(int s=0; s<7; s++) {
      String key = "s"+String(i)+"_"+String(s);
      preferences.putInt(key.c_str(), recordedSteps[i][s]);
    }
  preferences.end();
  if (client >= 0) webSocket.sendTXT(client, "MSG:Profile "+String(name)+" Saved!");
}

void loadProfile(const char* name, int client) {
  preferences.begin(name, true);
  int cnt = preferences.getInt("count", 0);
  if (cnt > 0) {
    isPlaying = false; stepCount = cnt;
    for(int i=0; i<stepCount; i++)
      for(int s=0; s<7; s++) {
        String key = "s"+String(i)+"_"+String(s);
        int def = (s == SRV_GRIPPER) ? 34 : ((s == SRV_WRIST_ROT) ? 100 : 90);
        recordedSteps[i][s] = preferences.getInt(key.c_str(), def);
        if (s == SRV_GRIPPER && recordedSteps[i][s] > 68) recordedSteps[i][s] = 68;
      }
    webSocket.broadcastTXT("STEPS:"+String(stepCount));
    if (client >= 0) webSocket.sendTXT(client, "MSG:Profile "+String(name)+" Loaded!");
  } else {
    if (client >= 0) webSocket.sendTXT(client, "MSG:Profile "+String(name)+" is empty.");
  }
  preferences.end();
}

// ===================== ARM SEQUENCE HELPERS ============
void playProfileA() {
  Serial.println("[ARM] Profile A → CATCH");
  loadProfile("ProfA", -1);
  if (stepCount > 0) { isPlaying = true; playIndex = 0; lastPlayTime = 0; }
}

void playProfileB() {
  Serial.println("[ARM] Profile B → RELEASE");
  loadProfile("ProfB", -1);
  if (stepCount > 0) { isPlaying = true; playIndex = 0; lastPlayTime = 0; }
}

// ===================== NAVIGATION ======================
const char* stationName(int s) {
  if (s == STATION_EMERGENCY) return "EMERGENCY (0)";
  if (s == STATION_RECEPTION) return "RECEPTION (1)";
  if (s == STATION_PHARMACY)  return "PHARMACY (2)";
  return "UNKNOWN";
}

void sendReportToVoice(uint8_t reportId) {
  uint8_t frame[4] = { UART_START, UART_TYPE_ACTION, reportId, UART_END };
  Serial2.write(frame, 4);
  Serial2.flush();
  Serial.printf("[UART TX] Sent Report: %03d\n", reportId);
}

// ===================== WEBSOCKET HANDLER ===============
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_TEXT) {
    String msg = (char*)payload;
    if (msg.startsWith("S:")) {
      int f=msg.indexOf(':'), s=msg.indexOf(':',f+1);
      int id=msg.substring(f+1,s).toInt(), ang=msg.substring(s+1).toInt();
      isPlaying = false; setTargetServo(id, ang);
    } else if (msg.startsWith("SR:")) {
      int f=msg.indexOf(':'), s=msg.indexOf(':',f+1);
      servoRates[msg.substring(f+1,s).toInt()] = msg.substring(s+1).toFloat();
    } else if (msg == "RESET") {
      isPlaying=false;
      setTargetServo(SRV_BASE,90);    setTargetServo(SRV_SHOULDER_L,180);
      setTargetServo(SRV_ELBOW,180);  setTargetServo(SRV_WRIST_V,38);
      setTargetServo(SRV_WRIST_ROT,100); setTargetServo(SRV_GRIPPER,0);
    } else if (msg == "REC") {
      if (stepCount < MAX_STEPS) { for(int i=0;i<7;i++) recordedSteps[stepCount][i]=targetAngles[i]; stepCount++; webSocket.broadcastTXT("STEPS:"+String(stepCount)); }
      else webSocket.sendTXT(num,"MSG:Max steps (50) reached");
    } else if (msg == "CLEAR") { isPlaying=false; stepCount=0; webSocket.broadcastTXT("STEPS:0"); }
    
    // Profiles A to E
    else if (msg=="SAVE_A") saveProfile("ProfA",num);
    else if (msg=="SAVE_B") saveProfile("ProfB",num);
    else if (msg=="SAVE_C") saveProfile("ProfC",num);
    else if (msg=="SAVE_D") saveProfile("ProfD",num);
    else if (msg=="SAVE_E") saveProfile("ProfE",num);
    
    else if (msg=="LOAD_A") loadProfile("ProfA",num);
    else if (msg=="LOAD_B") loadProfile("ProfB",num);
    else if (msg=="LOAD_C") loadProfile("ProfC",num);
    else if (msg=="LOAD_D") loadProfile("ProfD",num);
    else if (msg=="LOAD_E") loadProfile("ProfE",num);
    
    else if (msg=="PLAY_A"){ loadProfile("ProfA",num); if(stepCount>0){isPlaying=true;playIndex=0;lastPlayTime=0;} }
    else if (msg=="PLAY_B"){ loadProfile("ProfB",num); if(stepCount>0){isPlaying=true;playIndex=0;lastPlayTime=0;} }
    else if (msg=="PLAY_C"){ loadProfile("ProfC",num); if(stepCount>0){isPlaying=true;playIndex=0;lastPlayTime=0;} }
    else if (msg=="PLAY_D"){ loadProfile("ProfD",num); if(stepCount>0){isPlaying=true;playIndex=0;lastPlayTime=0;} }
    else if (msg=="PLAY_E"){ loadProfile("ProfE",num); if(stepCount>0){isPlaying=true;playIndex=0;lastPlayTime=0;} }
    
    else if (msg=="GET_STATE") {
      for(int i=0;i<=6;i++) { if(i==1)continue; webSocket.sendTXT(num,"S:"+String(i)+":"+String(targetAngles[i])); }
      webSocket.sendTXT(num,"STEPS:"+String(stepCount));
    }
  }
}

// ===================== HTTP HANDLERS ===================
void handleRoot() {
  String html = index_html;
  html.replace("%Kp%",   String(Kp));     html.replace("%Ki%",    String(Ki));
  html.replace("%Kd%",   String(Kd));     html.replace("%SPEED%", String(baseSpeed));
  html.replace("%MAX_L%",String(maxRpmL));html.replace("%MAX_R%", String(maxRpmR));
  html.replace("%MKp%",  String(motorKp));html.replace("%ACC%",   String(accelRate));
  html.replace("%CURV%", String(curveSlow)); html.replace("%DEB%",String(sensorDebounce));
  html.replace("%PAUSE%", String(stationPauseTime));
  html.replace("%CHK_ACC%", useAccel?"checked":"");
  html.replace("%CHK_CURV%",useCurve?"checked":"");
  html.replace("%CHK_HALL%",hallActiveLow?"checked":"");
  html.replace("%SR6%",String(servoRates[SRV_BASE]));
  html.replace("%SR0%",String(servoRates[SRV_SHOULDER_L]));
  html.replace("%SR2%",String(servoRates[SRV_ELBOW]));
  html.replace("%SR3%",String(servoRates[SRV_WRIST_V]));
  html.replace("%SR4%",String(servoRates[SRV_WRIST_ROT]));
  html.replace("%SR5%",String(servoRates[SRV_GRIPPER]));
  server.send(200,"text/html",html);
}

void handleToggle() {
  motorsEnabled = !motorsEnabled;
  if (!motorsEnabled) { currentLeftPWM=0; currentRightPWM=0; setMotorSpeed(0,0); }
  server.send(200,"text/plain",motorsEnabled?"ON":"OFF");
}
void handleToggleEncoders() {
  useEncoders=!useEncoders;
  preferences.begin("robot-config",false); preferences.putBool("useE",useEncoders); preferences.end();
  server.send(200,"text/plain",useEncoders?"ON":"OFF");
}
void handleToggleRadar() {
  radarEnabled=!radarEnabled;
  if(!radarEnabled) obstacleDetected=false;
  preferences.begin("robot-config",false); preferences.putBool("radarE",radarEnabled); preferences.end();
  server.send(200,"text/plain",radarEnabled?"ON":"OFF");
}
void handleCalibrate() {
  if(!server.hasArg("sensor")||!server.hasArg("state")) return;
  int s=server.arg("sensor").toInt(), state=server.arg("state").toInt();
  if(state==1){isCalibrating[s]=true;minValues[s]=4095;maxValues[s]=0;}
  else { isCalibrating[s]=false; thresholds[s]=(minValues[s]+maxValues[s])/2;
    preferences.begin("robot-config",false); preferences.putInt(("t"+String(s)).c_str(),thresholds[s]); preferences.end(); }
  server.send(200,"text/plain","OK");
}
void handleResetHome() {
  currentStation = STATION_EMERGENCY;
  preferences.begin("nav-state", false);
  preferences.putInt("station", currentStation);
  preferences.end();
  Serial.println("[RESET] Station manually reset to EMERGENCY (0)");
  server.send(200,"text/plain","OK");
}
void handleStatus() {
  String json; json.reserve(600);
  json += "{\"raw\":["; for(int i=0;i<5;i++){json+=String(rawAnalogValues[i]);if(i<4)json+=",";}
  json += "],\"digi\":[";  for(int i=0;i<5;i++){json+=String(digitizedValues[i]);if(i<4)json+=",";}
  json += "],\"t\":[";     for(int i=0;i<5;i++){json+=String(thresholds[i]);if(i<4)json+=",";}
  
  bool hallTriggered = hallActiveLow ? (digitalRead(HALL_PIN) == LOW) : (digitalRead(HALL_PIN) == HIGH);

  json += "],\"rpmL\":"; json+=String(rpmL); json+=",\"rpmR\":"; json+=String(rpmR);
  json += ",\"useE\":";  json+=(useEncoders?"true":"false");
  json += ",\"m\":";     json+=(motorsEnabled?"true":"false");
  json += ",\"rE\":";    json+=(radarEnabled?"true":"false");
  json += ",\"obs\":";   json+=(obstacleDetected?"true":"false");
  json += ",\"hall\":";  json+=(hallTriggered?"1":"0");
  json += ",\"dist\":";  json+=String(currentDistance);
  json += ",\"nav\":\""; json+=stationName(currentStation); json+="\"}";
  server.send(200,"application/json",json);
}
void handleSave() {
  if(server.hasArg("kp"))   Kp=server.arg("kp").toFloat();
  if(server.hasArg("ki"))   Ki=server.arg("ki").toFloat();
  if(server.hasArg("kd"))   Kd=server.arg("kd").toFloat();
  if(server.hasArg("speed"))baseSpeed=server.arg("speed").toInt();
  if(server.hasArg("ml"))   maxRpmL=server.arg("ml").toFloat();
  if(server.hasArg("mr"))   maxRpmR=server.arg("mr").toFloat();
  if(server.hasArg("mkp"))  motorKp=server.arg("mkp").toFloat();
  if(server.hasArg("acc"))  accelRate=server.arg("acc").toFloat();
  if(server.hasArg("curv")) curveSlow=server.arg("curv").toFloat();
  if(server.hasArg("deb"))  sensorDebounce=server.arg("deb").toInt();
  if(server.hasArg("pause")) stationPauseTime=server.arg("pause").toInt();
  if(server.hasArg("uacc")) useAccel=server.arg("uacc").toInt()==1;
  if(server.hasArg("ucurv"))useCurve=server.arg("ucurv").toInt()==1;
  if(server.hasArg("hlow"))  hallActiveLow=server.arg("hlow").toInt()==1;
  if(server.hasArg("sr0"))  servoRates[SRV_SHOULDER_L]=server.arg("sr0").toFloat();
  if(server.hasArg("sr2"))  servoRates[SRV_ELBOW]=server.arg("sr2").toFloat();
  if(server.hasArg("sr3"))  servoRates[SRV_WRIST_V]=server.arg("sr3").toFloat();
  if(server.hasArg("sr4"))  servoRates[SRV_WRIST_ROT]=server.arg("sr4").toFloat();
  if(server.hasArg("sr5"))  servoRates[SRV_GRIPPER]=server.arg("sr5").toFloat();
  if(server.hasArg("sr6"))  servoRates[SRV_BASE]=server.arg("sr6").toFloat();

  preferences.begin("robot-config",false);
  preferences.putFloat("kp",Kp); preferences.putFloat("ki",Ki); preferences.putFloat("kd",Kd);
  preferences.putInt("speed",baseSpeed); preferences.putFloat("ml",maxRpmL); preferences.putFloat("mr",maxRpmR);
  preferences.putFloat("mkp",motorKp);   preferences.putFloat("acc",accelRate); preferences.putFloat("curv",curveSlow);
  preferences.putInt("deb",sensorDebounce); preferences.putInt("pause",stationPauseTime);
  preferences.putBool("uacc",useAccel); preferences.putBool("ucurv",useCurve); preferences.putBool("hlow",hallActiveLow);
  preferences.putFloat("sr0",servoRates[SRV_SHOULDER_L]); preferences.putFloat("sr2",servoRates[SRV_ELBOW]);
  preferences.putFloat("sr3",servoRates[SRV_WRIST_V]);    preferences.putFloat("sr4",servoRates[SRV_WRIST_ROT]);
  preferences.putFloat("sr5",servoRates[SRV_GRIPPER]);  preferences.putFloat("sr6",servoRates[SRV_BASE]);
  for(int i=0;i<7;i++) preferences.putInt(("sa"+String(i)).c_str(),targetAngles[i]);
  anglesChanged=false; preferences.end();
  server.send(200,"text/plain","OK");
}

// ===================== SETUP ===========================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n[BOOT] Rover ESP32 starting...");

  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  // Sensors
  for(int i=0;i<numSensors;i++) pinMode(sensorPins[i], INPUT);
  pinMode(HALL_PIN,       INPUT_PULLUP); // Note: we still set pullup, but logic depends on hallActiveLow
  pinMode(IR_GRIPPER_PIN, INPUT_PULLUP); // LOW = object detected
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);

  // Motors
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT); pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  ledcSetup(channelL, freq, resolution);
  ledcAttachPin(ENA, channelL);
  ledcSetup(channelR, freq, resolution);
  ledcAttachPin(ENB, channelR);
  setMotorSpeed(0, 0);

  // Encoders
  pinMode(ENC_L, INPUT_PULLDOWN); pinMode(ENC_R, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(ENC_L), isrLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), isrRight, RISING);

  // Servo driver
  Wire.begin(); pwm.begin();
  pwm.setOscillatorFrequency(27000000); pwm.setPWMFreq(SERVO_FREQ); delay(10);

  // Load preferences
  preferences.begin("robot-config", true);
  radarEnabled  = preferences.getBool("radarE", true);
  Kp=preferences.getFloat("kp",10.0); Ki=preferences.getFloat("ki",0.0); Kd=preferences.getFloat("kd",0.0);
  baseSpeed=preferences.getInt("speed",150);
  maxRpmL=preferences.getFloat("ml",150.0); maxRpmR=preferences.getFloat("mr",150.0);
  motorKp=preferences.getFloat("mkp",1.0); accelRate=preferences.getFloat("acc",10.0);
  curveSlow=preferences.getFloat("curv",0.05); sensorDebounce=preferences.getInt("deb",20);
  stationPauseTime=preferences.getInt("pause",1000);
  useEncoders=preferences.getBool("useE",false); useAccel=preferences.getBool("uacc",true);
  useCurve=preferences.getBool("ucurv",true); hallActiveLow=preferences.getBool("hlow",true);
  for(int i=0;i<5;i++) thresholds[i]=preferences.getInt(("t"+String(i)).c_str(),2000);
  servoRates[SRV_SHOULDER_L]=preferences.getFloat("sr0",2.0);
  servoRates[SRV_ELBOW]=preferences.getFloat("sr2",2.0);
  servoRates[SRV_WRIST_V]=preferences.getFloat("sr3",2.0);
  servoRates[SRV_WRIST_ROT]=preferences.getFloat("sr4",2.0);
  servoRates[SRV_GRIPPER]=preferences.getFloat("sr5",2.0);
  servoRates[SRV_BASE]=preferences.getFloat("sr6",2.0);
  for(int i=0;i<7;i++){
    int def=90;
    if(i==SRV_SHOULDER_L||i==SRV_SHOULDER_R) def=180;
    else if(i==SRV_ELBOW) def=180;
    else if(i==SRV_WRIST_V) def=38;
    else if(i==SRV_WRIST_ROT) def=100;
    else if(i==SRV_GRIPPER) def=0;
    targetAngles[i]=preferences.getInt(("sa"+String(i)).c_str(),def);
    if(i==SRV_GRIPPER&&targetAngles[i]>68) targetAngles[i]=68;
    currentFloatAngles[i]=targetAngles[i];
  }
  preferences.end();

  // Load Station
  preferences.begin("nav-state", true);
  currentStation = preferences.getInt("station", STATION_EMERGENCY);
  preferences.end();

  // Apply servo positions
  for(int i=0;i<=6;i++) { if(i==1) continue; writeServoHardware(i, currentFloatAngles[i]); }

  // WiFi AP
  WiFi.mode(WIFI_AP);
  if (WiFi.softAP(ssid, password))
    Serial.printf("[WiFi] AP: %s  IP: %s\n", ssid, WiFi.softAPIP().toString().c_str());

  // --- OTA SETUP ---
  ArduinoOTA.setHostname("Rover-ESP32");
  ArduinoOTA.onStart([]() {
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() { Serial.println("\nEnd"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("OTA Ready");

  server.on("/",               handleRoot);
  server.on("/toggle",         handleToggle);
  server.on("/toggleEncoders", handleToggleEncoders);
  server.on("/toggleRadar",    handleToggleRadar);
  server.on("/calibrate",      handleCalibrate);
  server.on("/status",         handleStatus);
  server.on("/save",           handleSave);
  server.on("/resetHome",      handleResetHome);
  server.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  Serial.println("[BOOT] Complete. Ready.");
}

// ===================== LOOP ============================
void loop() {
  ArduinoOTA.handle(); // <-- ADDED: OTA Handler
  webSocket.loop();
  server.handleClient();

  // ── 1. UART FRAME PARSER ──────────────────────────────
  static enum { WAIT_START, WAIT_TYPE, WAIT_CODE, WAIT_END } uartSt = WAIT_START;
  static uint8_t uartType = 0, uartCode = 0;

  int bytesRead = 0;
  while (Serial2.available() > 0 && bytesRead < 20) {
    uint8_t c = Serial2.read(); bytesRead++;
    switch(uartSt) {
      case WAIT_START: if (c == UART_START) uartSt = WAIT_TYPE; break;
      case WAIT_TYPE:
        if (c == UART_TYPE_ACTION) { uartType = c; uartSt = WAIT_CODE; }
        else uartSt = WAIT_START;
        break;
      case WAIT_CODE: uartCode = c; uartSt = WAIT_END; break;
      case WAIT_END:
        if (c == UART_END) {
          Serial.printf("<<< [UART RX] Action Code: %03d\n", uartCode);
          
          // Emergency Stops / Overrides
          if (uartCode == 21 || uartCode == 17) { 
             stationStopPending = false; motorsEnabled = false; setMotorSpeed(0, 0); 
          }
          else if (uartCode == 1) { targetStation = STATION_RECEPTION; motorsEnabled = true; }
          else if (uartCode == 2) { targetStation = STATION_PHARMACY; motorsEnabled = true; }
          else if (uartCode == 3) { targetStation = STATION_EMERGENCY; motorsEnabled = true; }
          else if (uartCode == 11) { irGripperArmed = true; Serial.println("ARM ARMED (Waiting for IR Object)"); }
          else if (uartCode == 12) { irGripperArmed = false; playProfileA(); autoReturnPending = true; } // Manual Grasp
          else if (uartCode == 20) { playProfileB(); }
        }
        uartSt = WAIT_START;
        break;
    }
  }

  // ── 2. IR GRIPPER AUTO-CATCH (With 1.5s delay before driving) ──
  // Step 1: Detect object and start Catch profile immediately
  if (irGripperArmed && !isPlaying && !motorsEnabled && currentStation == STATION_PHARMACY) {
    if (digitalRead(IR_GRIPPER_PIN) == LOW) { // Object inside gripper
      irGripperArmed = false;
      Serial.println("[IR] Object detected! Auto-Catching...");
      sendReportToVoice(12); // Send instantly: Voice ESP32 plays track 17 "Grasped" immediately!
      playProfileA();
      autoReturnPending = true;
    }
  }

  // Step 2: Wait for profile A to finish, then start 1.5s delay
  if (autoReturnPending && !isPlaying) {
    autoReturnPending = false;
    autoReturnDelaying = true;
    autoReturnTimer = millis();
    Serial.println("[ARM] Catch Finished. Waiting 1.5s before moving...");
  }

  // Step 3: Wait 1.5 seconds, then move
  if (autoReturnDelaying && (millis() - autoReturnTimer >= 1500)) {
    autoReturnDelaying = false;
    Serial.println("[AUTO-RETURN] Time's up! Going to Emergency");
    targetStation = STATION_EMERGENCY;
    motorsEnabled = true;
  }

  // ── 3. STATION DETECTION (With Configurable Pass-through Delay) ────────
  if (motorsEnabled && !isPlaying) {
    int sumLine = 0;
    long weightedSum = 0;
    int weights[] = {-2000, -1000, 0, 1000, 2000};
    
    for(int i=0;i<numSensors;i++) {
      int raw = analogRead(sensorPins[i]);
      rawAnalogValues[i] = raw;
      if (raw < thresholds[i]) { // Black Line
        digitizedValues[i] = 1; sumLine++; weightedSum += weights[i];
      } else { digitizedValues[i] = 0; }
    }

    bool hallNow = hallActiveLow ? (digitalRead(HALL_PIN) == LOW) : (digitalRead(HALL_PIN) == HIGH);
    bool atMarker =  hallNow;

    // Trigger detection only if we aren't already waiting to stop!
    if (atMarker && !inStationZone && !stationStopPending) {
      if (millis() - lastStationTime > 3000) { // Deep debounce (3 seconds)
        inStationZone = true;
        lastStationTime = millis();
        currentStation = (currentStation + 1) % 3;
        Serial.printf("[STATION CROSSED] Current: %s\n", stationName(currentStation));
        
        preferences.begin("nav-state", false);
        preferences.putInt("station", currentStation);
        preferences.end();

        if (currentStation == targetStation) {
          // DO NOT STOP YET! Start the configurable pass-through timer.
          stationStopPending = true;
          stationStopTime = millis();
          Serial.printf("[TARGET HIT] Waiting %d ms to clear the line before stopping...\n", stationPauseTime);
        }
      }
    } else if (!atMarker) {
      inStationZone = false;
    }

    // Process the dynamic stop delay
    if (stationStopPending && (millis() - stationStopTime >= stationPauseTime)) {
      stationStopPending = false;
      motorsEnabled = false;
      setMotorSpeed(0,0);
      targetStation = STATION_NONE;
      Serial.println("[ARRIVED] Delay passed. Motors Stopped!");
      
      uint8_t rep = 0;
      if (currentStation == STATION_RECEPTION) rep = 13;
      else if (currentStation == STATION_PHARMACY) rep = 14;
      else if (currentStation == STATION_EMERGENCY) rep = 15;
      sendReportToVoice(rep);
    }

    // PID Processing (Continues even during the delay!)
    if (sumLine > 0) error = (float)weightedSum / sumLine;
    P = error; I += error; D = error - previousError;
    pidValue = Kp*P + Ki*I + Kd*D; previousError = error;

    int dynamicBase = baseSpeed;
    if(useCurve){ dynamicBase=baseSpeed-(abs(error)*curveSlow); if(dynamicBase<0) dynamicBase=0; }
    
    float scaleL=1.0, scaleR=1.0;
    if(maxRpmL>0 && maxRpmR>0){ if(maxRpmL<maxRpmR) scaleR=maxRpmL/maxRpmR; else scaleL=maxRpmR/maxRpmL; }
    
    int lT=(dynamicBase-pidValue)*scaleL, rT=(dynamicBase+pidValue)*scaleR;
    int lC=lT, rC=rT;
    
    if(useEncoders){
      float eL=((abs(lT)/255.0)*maxRpmL)-rpmL, eR=((abs(rT)/255.0)*maxRpmR)-rpmR;
      if(lT>0) lC+=(eL*motorKp); else if(lT<0) lC-=(eL*motorKp);
      if(rT>0) rC+=(eR*motorKp); else if(rT<0) rC-=(eR*motorKp);
    }
    
    if(useAccel){
      if(lC>currentLeftPWM)  currentLeftPWM +=min(accelRate,(float)lC-currentLeftPWM);
      else if(lC<currentLeftPWM) currentLeftPWM -=min(accelRate,currentLeftPWM-(float)lC);
      if(rC>currentRightPWM) currentRightPWM+=min(accelRate,(float)rC-currentRightPWM);
      else if(rC<currentRightPWM)currentRightPWM-=min(accelRate,currentRightPWM-(float)rC);
    } else { currentLeftPWM=lC; currentRightPWM=rC; }
    
    int fL=constrain((int)currentLeftPWM,-255,255);
    int fR=constrain((int)currentRightPWM,-255,255);
    int minT=130;
    if(fL>0&&fL<minT) fL=minT; if(fL<0&&fL>-minT) fL=-minT;
    if(fR>0&&fR<minT) fR=minT; if(fR<0&&fR>-minT) fR=-minT;
    
    setMotorSpeed(fL,fR);

  } else {
    // Stopped
    setMotorSpeed(0,0);
    // Still read sensors for UI telemetry
    for(int i=0;i<numSensors;i++) {
      rawAnalogValues[i] = analogRead(sensorPins[i]);
      digitizedValues[i] = (rawAnalogValues[i] < thresholds[i]) ? 1 : 0;
    }
  }

 // ── 4. ULTRASONIC ─────────────────────────────────────
  if (radarEnabled && millis()-lastPingTime > 50) {
    digitalWrite(TRIG_PIN, LOW); delayMicroseconds(5);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(20);
    digitalWrite(TRIG_PIN, LOW);
    long dur = pulseIn(ECHO_PIN, HIGH, 30000);
    currentDistance = (dur == 0) ? 999 : dur * 0.034 / 2;
    bool wasObstacle = obstacleDetected;
    obstacleDetected = (currentDistance > 0 && currentDistance <= stopDistance);

    // Only report when motors are running (no point warning when parked)
    if (motorsEnabled) {
      if (obstacleDetected && !wasObstacle) {
        // Obstacle just appeared — send immediately
        sendReportToVoice(30);
        lastObstacleReport = millis();
        obstaclePlaying = true;
      } else if (obstacleDetected && obstaclePlaying) {
        // Still blocked — repeat every 5 seconds
        if (millis() - lastObstacleReport >= 5000) {
          sendReportToVoice(30);
          lastObstacleReport = millis();
        }
      } else if (!obstacleDetected && wasObstacle) {
        // Obstacle just cleared
        obstaclePlaying = false;
        sendReportToVoice(31);
      }
    } else {
      // Motors stopped — reset obstacle state silently
      if (obstaclePlaying) {
        obstaclePlaying = false;
        sendReportToVoice(31);
      }
    }
    lastPingTime = millis();
  } else if (!radarEnabled) {
    obstacleDetected = false;
    currentDistance = 999;
    if (obstaclePlaying) {
      obstaclePlaying = false;
      sendReportToVoice(31);
    }
  }

  // ── 5. SERVO SMOOTH INTERPOLATION ────────────────────
  if (millis()-lastServoUpdate > 15) {
    for(int i=0;i<=6;i++) {
      if(i==1) continue;
      float diff = targetAngles[i] - currentFloatAngles[i];
      if(abs(diff) > 0.1) {
        float rate = servoRates[i];
        currentFloatAngles[i] += (diff>0) ? min(rate,diff) : max(-rate,diff);
        writeServoHardware(i, currentFloatAngles[i]);
      }
    }
    lastServoUpdate = millis();
  }

  // ── 6. MACRO PLAYBACK ─────────────────────────────────
  if (isPlaying && millis()-lastPlayTime > playDelay) {
    setTargetServo(SRV_BASE,          recordedSteps[playIndex][SRV_BASE]);
    setTargetServo(SRV_SHOULDER_L,    recordedSteps[playIndex][SRV_SHOULDER_L]);
    setTargetServo(SRV_ELBOW,         recordedSteps[playIndex][SRV_ELBOW]);
    setTargetServo(SRV_WRIST_V,       recordedSteps[playIndex][SRV_WRIST_V]);
    setTargetServo(SRV_WRIST_ROT,     recordedSteps[playIndex][SRV_WRIST_ROT]);
    setTargetServo(SRV_GRIPPER,       recordedSteps[playIndex][SRV_GRIPPER]);
    for(int i=0;i<=6;i++) { if(i==1) continue; webSocket.broadcastTXT("S:"+String(i)+":"+String(recordedSteps[playIndex][i])); }
    playIndex++;
    if (playIndex >= stepCount) isPlaying = false;
    lastPlayTime = millis();
  }

  // ── 7. AUTO-SAVE PREFERENCES ─────────────────────────
  if (anglesChanged && millis()-lastAnglesSave > 5000) {
    preferences.begin("robot-config",false);
    for(int i=0;i<7;i++) preferences.putInt(("sa"+String(i)).c_str(),targetAngles[i]);
    preferences.end(); anglesChanged=false;
  }

  // ── 8. RPM CALCULATION ───────────────────────────────
  if (micros()-lastTimeL > 1000000) rpmL=0; else if(deltaL>0) rpmL=60000000.0/deltaL;
  if (micros()-lastTimeR > 1000000) rpmR=0; else if(deltaR>0) rpmR=60000000.0/deltaR;
}