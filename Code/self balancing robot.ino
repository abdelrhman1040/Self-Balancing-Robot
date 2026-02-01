#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <AccelStepper.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// --- Network Settings ---
const char* ssid = "aaaa";
const char* password = "22334455";

// --- System Objects ---
Preferences preferences;
WebServer server(80);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29); 

// --- Pin Definitions ---
#define STEP_PIN_1   14   
#define DIR_PIN_1    12    
#define STEP_PIN_2   26   
#define DIR_PIN_2    27   

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);

// --- PID Control Variables ---
volatile float Kp = 383, Ki = 5.3, Kd = 198.5;
volatile float setPoint = -1.4;
float deadband = 0.5;

// --- Drift Control ---
volatile float Kp_s = 0.0008; 
volatile float Ki_s = 0.0001; 

// --- Profile Management ---
int activeProfile = 1;

// --- Movement Variables ---
volatile float moveTilt = 0;   
volatile float turnSpeed = 0;  

// --- Variables ---
volatile float displayAngle = 0;
float speedOutput = 0, sumSpeedError = 0;
float lastBalanceError = 0, sumBalanceError = 0;
float desiredAngle = 0;

// --- DEBUG / GRAPH VARIABLES ---
volatile float dbg_bP = 0, dbg_bI = 0, dbg_bD = 0;
volatile float dbg_sP = 0, dbg_sI = 0;

const float FALL_LIMIT = 45.0; 
const float MAX_HW_SPEED = 12000; 
const float MAX_TILT = 5.0; 
bool sensorConnected = false;

TaskHandle_t Task1; 

// --- Web Dashboard ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
   <meta name="viewport" content="width=device-width, initial-scale=1">
   <title>self balancing robot </title>
   <style>
      body { font-family: 'Segoe UI', sans-serif; background: #000; color: #eee; margin:0; padding:5px; text-align:center; }
      h2 { color: #f39c12; margin: 5px 0; font-size: 18px; }
      
      .container { display: flex; flex-wrap: wrap; justify-content: center; gap: 10px; }
      .card { background: #1e1e1e; padding: 10px; border-radius: 8px; box-shadow: 0 4px 10px rgba(0,0,0,0.5); width: 100%; max-width: 450px; }
      
      .live-vals { display: flex; justify-content: space-around; margin-bottom: 5px; }
      .live-num { font-size: 20px; font-weight: bold; color: #00bcd4; }
      .live-lbl { font-size: 10px; color: #aaa; }
      .lat-num { font-size: 18px; font-weight: bold; color: #e91e63; }

      /* ROBOT VISUALIZER STYLING */
      .vis-container { position: relative; width: 100%; height: 250px; background: #000; border-radius: 10px; border: 1px solid #333; margin-bottom: 10px; overflow: hidden; }
      canvas#cvsRobot { width: 100%; height: 100%; }

      /* Profile Styling */
      .prof-section { background: #222; padding: 10px; border-radius: 5px; margin-bottom: 5px; border: 1px solid #444; }
      .prof-row { display: flex; gap: 5px; margin-bottom: 5px; justify-content: center; }
      .p-btn { flex: 1; padding: 8px; background: #333; border: 1px solid #555; color: #fff; cursor: pointer; border-radius: 4px; font-size: 14px; }
      .p-active { background: #00bcd4; color: #000; font-weight: bold; border-color: #eee; }
      
      .action-btn { width: 48%; padding: 10px; font-weight: bold; cursor: pointer; border: none; border-radius: 5px; color: white; }
      .btn-save { background: #4CAF50; } 
      .btn-load { background: #FF9800; } 
      .btn-save:active, .btn-load:active { transform: scale(0.98); opacity: 0.8; }

      .graph-card { background: #000; border: 1px solid #333; border-radius: 8px; padding: 5px; width: 100%; max-width: 600px; }
      canvas#cvsMaster { width: 100%; height: 300px; display: block; }
      
      .legend { font-size: 11px; margin-top: 5px; display: flex; flex-wrap: wrap; justify-content: center; gap: 8px; }
      .legend label { cursor: pointer; display: flex; align-items: center; gap: 3px; user-select: none; }
      input[type=checkbox] { accent-color: #555; width: 15px; height: 15px; cursor: pointer; }

      input[type=range] { width: 100%; cursor: pointer; margin: 8px 0; }
      .row { display: flex; justify-content: space-between; font-size: 13px; color: #ccc; }
      .btn { background: #2196F3; border: none; color: white; padding: 5px; width: 50px; height: 50px; font-size: 20px; border-radius: 50%; margin: 3px; cursor: pointer; }
      .btn-stop { background: #f44336; width: 70px; height: 70px; border-radius: 15px; }
   </style>
</head>
<body>

   <h2>ESP32 Robot Visualizer</h2>

   <div class="container">
      <div class="card">
          
          <div class="vis-container">
              <canvas id="cvsRobot"></canvas>
          </div>

          <div class="prof-section">
             <div class="prof-row">
                <button id="p1" class="p-btn p-active" onclick="selProf(1)">P1</button>
                <button id="p2" class="p-btn" onclick="selProf(2)">P2</button>
                <button id="p3" class="p-btn" onclick="selProf(3)">P3</button>
             </div>
             <div class="prof-row">
                <button class="action-btn btn-save" onclick="doSave()">SAVE</button>
                <button class="action-btn btn-load" onclick="doLoad()">LOAD</button>
             </div>
          </div>

          <div class="live-vals">
             <div><span id="txtAng" class="live-num">0.0</span><br><span class="live-lbl">Angle</span></div>
             <div><span id="txtSpd" class="live-num">0.00</span><br><span class="live-lbl">m/s</span></div>
             <div><span id="txtLat" class="lat-num">0 ms</span><br><span class="live-lbl">Ping</span></div>
          </div>
      </div>
   </div>

   <div class="container" style="margin-top: 15px;">
      <div class="card">
         <h3>Balance PID</h3>
         <div class="row"><span>Kp</span><span id="v_kp">%KP%</span></div>
         <input id="i_kp" type="range" min="0" max="1200" step="1" value="%KP%" oninput="snd('bp',this.value,'v_kp')">
         <div class="row"><span>Ki</span><span id="v_ki">%KI%</span></div>
         <input id="i_ki" type="range" min="0" max="50" step="0.1" value="%KI%" oninput="snd('bi',this.value,'v_ki')">
         <div class="row"><span>Kd</span><span id="v_kd">%KD%</span></div>
         <input id="i_kd" type="range" min="0" max="500" step="0.5" value="%KD%" oninput="snd('bd',this.value,'v_kd')">
         <div class="row"><span>SetPoint</span><span id="v_sp">%SP%</span></div>
         <input id="i_sp" type="range" min="-10" max="10" step="0.1" value="%SP%" oninput="snd('sp',this.value,'v_sp')">
      </div>

      <div class="card">
         <h3>Speed PID</h3>
         <div class="row"><span>Kp_s</span><span id="v_kps">%KPS%</span></div>
         <input id="i_kps" type="range" min="0" max="0.01" step="0.00001" value="%KPS%" oninput="snd('sp_p',this.value,'v_kps')">
         <div class="row"><span>Ki_s</span><span id="v_kis">%KIS%</span></div>
         <input id="i_kis" type="range" min="0" max="0.001" step="0.00001" value="%KIS%" oninput="snd('sp_i',this.value,'v_kis')">
      </div>

      <div class="card">
         <div><button class="btn" onmousedown="mv('F')" onmouseup="mv('S')" ontouchstart="mv('F')" ontouchend="mv('S')">&#8593;</button></div>
         <div>
            <button class="btn" onmousedown="mv('L')" onmouseup="mv('S')" ontouchstart="mv('L')" ontouchend="mv('S')">&#8592;</button>
            <button class="btn btn-stop" onclick="mv('S')">STOP</button>
            <button class="btn" onmousedown="mv('R')" onmouseup="mv('S')" ontouchstart="mv('R')" ontouchend="mv('S')">&#8594;</button>
         </div>
         <div><button class="btn" onmousedown="mv('B')" onmouseup="mv('S')" ontouchstart="mv('B')" ontouchend="mv('S')">&#8595;</button></div>
      </div>
   </div>

   <div class="container" style="margin-top: 10px;">
      <div class="graph-card">
         <div style="color:#aaa; border-bottom:1px solid #333; margin-bottom:5px;">History Graph</div>
         <canvas id="cvsMaster"></canvas>
         <div class="legend">
            <label style="color:white;"><input type="checkbox" id="cb_ang" checked>Angle</label>
            <label style="color:cyan;"><input type="checkbox" id="cb_spd" checked>Speed</label>
            <label style="color:#ffcc00;"><input type="checkbox" id="cb_bp">Bal-P</label>
            <label style="color:#ff00ff;"><input type="checkbox" id="cb_bi">Bal-I</label>
            <label style="color:#00ff00;"><input type="checkbox" id="cb_bd">Bal-D</label>
         </div>
      </div>
   </div>

<script>
   function id(n){return document.getElementById(n);}
   function snd(k,v,l){ id(l).innerText=v; fetch("/set?k="+k+"&v="+v).catch(e=>console.log(e)); }
   function mv(d){ fetch("/move?d="+d).catch(e=>console.log(e)); }

   // Global Wheel Position (accumulates rotation)
   var wheelPos = 0;

   // --- Robot Visualizer Logic ---
   function drawRobotVisualizer(angle, speedSteps) {
       var cvs = id('cvsRobot');
       var ctx = cvs.getContext('2d');
       var w = cvs.width = cvs.clientWidth;
       var h = cvs.height = cvs.clientHeight;
       
       // Center Coordinates (Pivot at bottom center)
       var cx = w / 2;
       var cy = h - 40; 

       // --- UPDATE WHEEL POSITION ---
       // speedSteps is steps/sec. 3200 steps = 1 rev (2PI rads).
       // Update interval is approx 100ms (0.1s).
       // Delta Angle = (speed / 3200) * 2PI * 0.1
       // We use 0.000196 rads per step-interval approx.
       var deltaRad = (speedSteps / 3200.0) * 2 * Math.PI * 0.1;
       wheelPos += deltaRad;

       ctx.clearRect(0,0,w,h);

       // 1. Draw Faint Background Arcs
       var rOuter = (h - 60); 
       var rInner = rOuter - 60;

       ctx.beginPath();
       ctx.arc(cx, cy, rOuter, Math.PI, 0); 
       ctx.strokeStyle = "#333"; ctx.lineWidth = 1; ctx.stroke();
       
       ctx.beginPath();
       ctx.arc(cx, cy, rInner, Math.PI, 0); 
       ctx.strokeStyle = "#333"; ctx.lineWidth = 1; ctx.stroke();

       // Vertical Zero Line
       ctx.beginPath();
       ctx.moveTo(cx, cy - rOuter - 10);
       ctx.lineTo(cx, cy);
       ctx.strokeStyle = "#ff9800"; ctx.lineWidth = 2; ctx.stroke();

       // 2. Draw Robot Body (Tilting)
       ctx.save();
       ctx.translate(cx, cy);
       ctx.rotate(angle * Math.PI / 180);

       // Body Outline
       ctx.strokeStyle = "white";
       ctx.lineWidth = 4;
       ctx.lineJoin = "round";
       ctx.strokeRect(-25, -120, 50, 120);

       // 3. Draw Wheel (Spinning)
       // We create a nested rotation for the wheel spokes
       ctx.save(); 
       // Translate is already at (0,0) relative to body pivot, which is correct for the wheel hub
       ctx.rotate(wheelPos); // Rotate spokes by accumulated speed

       // Wheel Circle (Static relative to hub, but we draw it here)
       ctx.fillStyle = "black"; 
       ctx.beginPath();
       ctx.arc(0, 0, 28, 0, Math.PI*2);
       ctx.fill(); 
       
       ctx.strokeStyle = "white";
       ctx.lineWidth = 3;
       ctx.stroke();

       // Spokes (5 lines, spinning)
       ctx.lineWidth = 2;
       ctx.beginPath();
       for(let i=0; i<5; i++) {
           let rad = (i * 2 * Math.PI / 5) - (Math.PI/2); 
           ctx.moveTo(0,0);
           ctx.lineTo(28 * Math.cos(rad), 28 * Math.sin(rad));
       }
       ctx.stroke();

       ctx.restore(); // Restore from Wheel Rotation

       // Center Dot (Static on hub)
       ctx.fillStyle = "white";
       ctx.beginPath();
       ctx.arc(0, 0, 4, 0, Math.PI*2);
       ctx.fill();

       ctx.restore(); // Restore from Body Rotation

       // 4. Floor Line
       ctx.strokeStyle = "white";
       ctx.lineWidth = 2;
       ctx.beginPath();
       ctx.moveTo(0, cy + 30); 
       ctx.lineTo(w, cy + 30);
       ctx.stroke();
       
       // 5. Angle Text (Color Logic)
       if(Math.abs(angle) <= 2.0) {
           ctx.fillStyle = "#00ff00"; 
       } else {
           ctx.fillStyle = "#ff0000"; 
       }
       
       ctx.font = "bold 24px Segoe UI";
       ctx.textAlign = "center";
       ctx.fillText(angle.toFixed(1) + "°", cx, h - 5);
   }

   // --- Profile Logic ---
   var currentSlot = 1;
   function selProf(p) {
       currentSlot = p;
       for(let i=1; i<=3; i++) id('p'+i).className = "p-btn";
       id('p'+p).className = "p-btn p-active";
   }
   function doSave() {
       if(confirm("Save to P" + currentSlot + "?")) fetch("/save?p=" + currentSlot);
   }
   function doLoad() {
       if(confirm("Load P" + currentSlot + "?")) {
           fetch("/load?p=" + currentSlot).then(r=>r.json()).then(d=>{
                 id('i_kp').value = d.kp; id('v_kp').innerText = d.kp;
                 id('i_ki').value = d.ki; id('v_ki').innerText = d.ki;
                 id('i_kd').value = d.kd; id('v_kd').innerText = d.kd;
                 id('i_sp').value = d.sp; id('v_sp').innerText = d.sp;
                 id('i_kps').value = d.kps; id('v_kps').innerText = d.kps;
                 id('i_kis').value = d.kis; id('v_kis').innerText = d.kis;
             });
       }
   }

   const LEN = 100;
   var dAng = new Array(LEN).fill(0);
   var dSpd = new Array(LEN).fill(0);
   var dBP = new Array(LEN).fill(0);
   var dBI = new Array(LEN).fill(0);
   var dBD = new Array(LEN).fill(0);

   function drawGraph(cid, datasets) {
      var c = id(cid); var ctx = c.getContext('2d');
      var w = c.width = c.clientWidth; var h = c.height = c.clientHeight;
      ctx.clearRect(0,0,w,h);
      ctx.strokeStyle = '#333'; ctx.beginPath(); ctx.moveTo(0, h/2); ctx.lineTo(w, h/2); ctx.stroke();

      datasets.forEach(ds => {
            if(id(ds.id).checked) {
                  ctx.strokeStyle = ds.color; ctx.lineWidth = ds.width || 1.5; ctx.beginPath();
                  for(var i=0; i<LEN; i++){
                        var val = ds.data[i];
                        var scale = ds.scale || 1.0;
                        var y = h/2 - (val * (h/2)/scale);
                        if(i==0) ctx.moveTo(0, y); else ctx.lineTo(i*(w/LEN), y);
                  }
                  ctx.stroke();
            }
      });
   }

   setInterval(function(){
      var startTime = Date.now();
      fetch("/data").then(r=>r.json()).then(d=>{
         var latency = Date.now() - startTime; 
         id("txtLat").innerText = latency + " ms";

         var mps = d.s * 0.000102; 
         
         // UPDATE ROBOT VISUALIZER (Angle + Speed)
         drawRobotVisualizer(d.a, d.s);
         
         // UPDATE LIVE TEXT
         id("txtAng").innerText = d.a.toFixed(2);
         id("txtSpd").innerText = mps.toFixed(2);

         dAng.push(d.a); dAng.shift();
         dSpd.push(d.s); dSpd.shift();
         dBP.push(d.bP); dBP.shift();
         dBI.push(d.bI); dBI.shift();
         dBD.push(d.bD); dBD.shift();

         drawGraph('cvsMaster', [
               {id:'cb_ang', data:dAng, color:'white',   scale:45, width:2},     
               {id:'cb_spd', data:dSpd, color:'cyan',    scale:12000},           
               {id:'cb_bp',  data:dBP,  color:'#ffcc00', scale:8000},            
               {id:'cb_bi',  data:dBI,  color:'#ff00ff', scale:8000},
               {id:'cb_bd',  data:dBD,  color:'#00ff00', scale:8000}
         ]);

      }).catch(e=>{
         id("txtLat").innerText = "OFF";
      });
   }, 100); 
</script>
</body>
</html>
)rawliteral";


// --- MAIN CONTROL LOOP ---
void MotorTaskCode( void * pvParameters ) {
   unsigned long lastTime = 0;
   
   for(;;) { 
      stepper1.runSpeed();
      stepper2.runSpeed();
      
      unsigned long now = millis();
      if (now - lastTime >= 10) { 
         
         if(sensorConnected) {
            sensors_event_t event; bno.getEvent(&event);
            displayAngle = event.orientation.z;
            if (displayAngle > 180) displayAngle -= 360;
         }

         if (abs(setPoint - displayAngle) > FALL_LIMIT) {
            speedOutput = 0; sumBalanceError = 0; sumSpeedError = 0;
            dbg_bP=0; dbg_bI=0; dbg_bD=0;
            stepper1.setSpeed(0); stepper2.setSpeed(0);
         } else {
            
            float term_sP = speedOutput * Kp_s;
            float term_sI = sumSpeedError * Ki_s;
            dbg_sP = term_sP; dbg_sI = term_sI;

            float driftCorrection = term_sP + term_sI;
            driftCorrection = constrain(driftCorrection, -MAX_TILT, MAX_TILT);
            
            desiredAngle = setPoint + driftCorrection + moveTilt; 

            sumSpeedError += speedOutput;
            sumSpeedError = constrain(sumSpeedError, -200000, 200000);

            // Balance PID
            float balanceError = desiredAngle - displayAngle;
            float rawOutput = 0;

            if (abs(balanceError) < deadband) {
               rawOutput = 0; sumBalanceError = 0;
               dbg_bP=0; dbg_bI=0; dbg_bD=0;
            } 
            else {
               sumBalanceError += balanceError;
               if ((balanceError > 0 && lastBalanceError < 0) || (balanceError < 0 && lastBalanceError > 0)) sumBalanceError = 0;
               sumBalanceError = constrain(sumBalanceError, -8000, 8000);

               float term_bP = Kp * balanceError; 
               float term_bI = Ki * sumBalanceError; 
               float term_bD = Kd * (balanceError - lastBalanceError);
               
               dbg_bP = term_bP; dbg_bI = term_bI; dbg_bD = term_bD;
               
               rawOutput = term_bP + term_bI + term_bD;
            }

            // Accel Limit
            float max_change = 300.0; 
            float change = rawOutput - speedOutput;
            if (change > max_change) change = max_change;
            if (change < -max_change) change = -max_change;
            speedOutput += change;

            lastBalanceError = balanceError;

            // Apply
            float s1 = speedOutput + turnSpeed;
            float s2 = -speedOutput + turnSpeed;
            s1 = constrain(s1, -MAX_HW_SPEED, MAX_HW_SPEED);
            s2 = constrain(s2, -MAX_HW_SPEED, MAX_HW_SPEED);
            stepper1.setSpeed(s1); stepper2.setSpeed(s2);
         }
         lastTime = now;
      }
   } 
}

// --- WEB HANDLERS ---
void handleRoot() {
   String s = index_html;
   s.replace("%KP%", String(Kp)); s.replace("%KI%", String(Ki)); s.replace("%KD%", String(Kd));
   s.replace("%SP%", String(setPoint)); s.replace("%KPS%", String(Kp_s, 5)); s.replace("%KIS%", String(Ki_s, 5));
   server.send(200, "text/html", s);
}

void handleSet() {
   if(server.hasArg("k") && server.hasArg("v")) {
      String key = server.arg("k"); float val = server.arg("v").toFloat();
      if(key=="bp") Kp=val; else if(key=="bi") Ki=val; else if(key=="bd") Kd=val;
      else if(key=="sp") setPoint=val; else if(key=="sp_p") Kp_s=val; else if(key=="sp_i") Ki_s=val;
   }
   server.send(200, "text/plain", "OK");
}

void handleSave() {
    if(server.hasArg("p")) {
        int p = server.arg("p").toInt();
        String s = "_" + String(p);
        preferences.putFloat(("kp" + s).c_str(), Kp); 
        preferences.putFloat(("ki" + s).c_str(), Ki); 
        preferences.putFloat(("kd" + s).c_str(), Kd);
        preferences.putFloat(("sp" + s).c_str(), setPoint);
        preferences.putFloat(("kps" + s).c_str(), Kp_s); 
        preferences.putFloat(("kis" + s).c_str(), Ki_s);
        server.send(200, "text/plain", "Saved");
    } else server.send(400, "text/plain", "Err");
}

void handleLoad() {
    if(server.hasArg("p")) {
        int p = server.arg("p").toInt();
        String s = "_" + String(p);
        
        Kp = preferences.getFloat(("kp" + s).c_str(), 420); 
        Ki = preferences.getFloat(("ki" + s).c_str(), 2.0); 
        Kd = preferences.getFloat(("kd" + s).c_str(), 200);
        setPoint = preferences.getFloat(("sp" + s).c_str(), 0);
        Kp_s = preferences.getFloat(("kps" + s).c_str(), 0.001); 
        Ki_s = preferences.getFloat(("kis" + s).c_str(), 0.0001);

        String json = "{";
        json += "\"kp\":" + String(Kp) + ",";
        json += "\"ki\":" + String(Ki) + ",";
        json += "\"kd\":" + String(Kd) + ",";
        json += "\"sp\":" + String(setPoint) + ",";
        json += "\"kps\":" + String(Kp_s, 5) + ",";
        json += "\"kis\":" + String(Ki_s, 5);
        json += "}";
        server.send(200, "application/json", json);
    } else server.send(400, "text/plain", "Err");
}

void handleMove() {
    if(server.hasArg("d")) {
        String d = server.arg("d");
        moveTilt = 0; turnSpeed = 0;
        if(d == "F") moveTilt = -1.5;   
        else if(d == "B") moveTilt = 1.5; 
        else if(d == "L") turnSpeed = -800; 
        else if(d == "R") turnSpeed = 800;  
    }
    server.send(200, "text/plain", "OK");
}

void handleData() {
   String json = "{";
   json += "\"a\":" + String(displayAngle) + ",";
   json += "\"s\":" + String(speedOutput) + ",";
   json += "\"bP\":" + String(dbg_bP) + ",";
   json += "\"bI\":" + String(dbg_bI) + ",";
   json += "\"bD\":" + String(dbg_bD) + ",";
   json += "\"sP\":" + String(dbg_sP) + ",";
   json += "\"sI\":" + String(dbg_sI);
   json += "}";
   server.send(200, "application/json", json);
}

void setup() {
   WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
   Serial.begin(115200);
   Wire.begin(); Wire.setClock(400000); 

   preferences.begin("robot_config", false);
   
   String s = "_1";
   Kp = preferences.getFloat(("kp" + s).c_str(), 420); 
   Ki = preferences.getFloat(("ki" + s).c_str(), 2.0); 
   Kd = preferences.getFloat(("kd" + s).c_str(), 200);
   setPoint = preferences.getFloat(("sp" + s).c_str(), 0);
   Kp_s = preferences.getFloat(("kps" + s).c_str(), 0.001); 
   Ki_s = preferences.getFloat(("kis" + s).c_str(), 0.0001);

   if (!bno.begin()) sensorConnected = false; 
   else { sensorConnected = true; bno.setExtCrystalUse(true); }

   stepper1.setMaxSpeed(MAX_HW_SPEED); stepper2.setMaxSpeed(MAX_HW_SPEED);
   
   xTaskCreatePinnedToCore(MotorTaskCode, "MotorTask", 10000, NULL, 1, &Task1, 1);

   WiFi.mode(WIFI_STA); WiFi.begin(ssid, password);
   server.on("/", handleRoot); 
   server.on("/set", handleSet); 
   server.on("/data", handleData);
   server.on("/move", handleMove); 
   server.on("/save", handleSave);
   server.on("/load", handleLoad);
   server.begin(); 
}

void loop() {
   if (WiFi.status() == WL_CONNECTED) server.handleClient();
   delay(2);
}