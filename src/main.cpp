#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

// =========================================================================
// KONFIGURASI HARDWARE & JARINGAN
// =========================================================================
#define SDA_PIN 21
#define SCL_PIN 22
#define DSP_RESET_PIN 19
#define ADAU1701_I2C_ADDR 0x34

const char* ssid = "freeDSP_Control"; 
const char* password = "24082007";            

Preferences preferences;
WebServer server(80);

// =========================================================================
// === MAPPING ALAMAT REGISTER ===
// =========================================================================
const uint16_t ADDR_MUTE[4] = {9, 8, 10, 11};                     // Mute 1 - 4
const uint16_t ADDR_GAIN_OUT[4] = {136, 137, 138, 139};           // Master Gain (Single) 1 - 4
const uint16_t ADDR_DELAY[4] = {132, 133, 134, 135};              // Delay (Samples) 1 - 4
const uint16_t ADDR_LIMITER_THRESH[4] = {151, 166, 181, 196};     // Limiter Threshold 1 - 4

// Gen Filter (1st Low & 2nd High)
const uint16_t ADDR_GEN_FILTER_LOW[4] = {12, 17, 22, 27};   
const uint16_t ADDR_GEN_FILTER_HIGH[4] = {15, 20, 25, 30};  

// Mixer 2x (Input A & B ke Output 1-4)
const uint16_t ADDR_MIXER[4][2] = {
  {0, 4}, 
  {1, 5}, 
  {2, 6}, 
  {3, 7}  
};

// Alamat Koefisien Parametric EQ [Channel 0-3][Band 0-4]
const uint16_t ADDR_EQ_BAND[4][5] = {
  {32, 37, 42, 47, 52},   
  {57, 62, 67, 72, 77},   
  {82, 87, 92, 97, 102}, 
  {107, 112, 117, 122, 127}  
};

// =========================================================================
// STRUKTUR DATA PRESET
// =========================================================================
struct ParamEQ_Band {
  float freq;
  float qFactor;
  float gain;
};

struct DSP_Settings {
  float mixerInput[4][2];      
  bool mute[4];                
  int delaySamples[4];         
  float gainOutput[4];         
  float limiterThreshold[4];
  float genFilterLowFreq[4];
  float genFilterHighFreq[4];
  ParamEQ_Band eqBand[4][5];    
};

DSP_Settings currentSettings;

// =========================================================================
// FUNGSI KOMUNIKASI I2C & MATEMATIKA DSP
// =========================================================================
void writeDSP(uint16_t address, const uint8_t *data, size_t length) {
  if (address == 0) return;
  Wire.beginTransmission(ADAU1701_I2C_ADDR);
  Wire.write((uint8_t)(address >> 8));
  Wire.write((uint8_t)(address & 0xFF));
  for (size_t i = 0; i < length; i++) {
    Wire.write(data[i]);
  }
  Wire.endTransmission();
}

void floatTo523(float value, uint8_t* buffer) {
  int32_t val = value * (1 << 23); 
  buffer[0] = (val >> 24) & 0xFF;
  buffer[1] = (val >> 16) & 0xFF;
  buffer[2] = (val >> 8) & 0xFF;
  buffer[3] = val & 0xFF;
}

void setGainDSP(uint16_t address, float dbValue) {
  if (address == 0) return;
  float linear = pow(10.0, dbValue / 20.0);
  uint8_t data[4];
  floatTo523(linear, data);
  writeDSP(address, data, 4);
}

void setMuteDSP(uint16_t address, bool isMuted) {
  if (address == 0) return;
  uint8_t data[4] = {0, 0, 0, 0};
  if (!isMuted) data[1] = 0x80; 
  writeDSP(address, data, 4);
}

void setDelayDSP(uint16_t address, int samples) {
  if (address == 0) return;
  uint8_t data[4];
  data[0] = (samples >> 24) & 0xFF;
  data[1] = (samples >> 16) & 0xFF;
  data[2] = (samples >> 8) & 0xFF;
  data[3] = samples & 0xFF;
  writeDSP(address, data, 4);
}

void calculateAndSendFirstOrderFilter(uint16_t baseAddress, float cutoffFreq, bool isHighPass, float fs = 48000.0) {
    if (baseAddress == 0) return;
    if (cutoffFreq < 10.0) cutoffFreq = 10.0;
    if (cutoffFreq > 20000.0) cutoffFreq = 20000.0;

    float x = exp(-2.0 * M_PI * cutoffFreq / fs);
    float b0, b1, a1;

    if (isHighPass) {
        b0 =  (1.0 + x) / 2.0;
        b1 = -(1.0 + x) / 2.0;
        a1 = -x;
    } else { 
        b0 =  1.0 - x;
        b1 =  0.0;
        a1 = -x;
    }

    float coefficients[3] = {b0, b1, a1};
    for(int i = 0; i < 3; i++) {
        int32_t val = coefficients[i] * (1 << 23);
        uint8_t buf[4];
        buf[0] = (val >> 24) & 0xFF;
        buf[1] = (val >> 16) & 0xFF;
        buf[2] = (val >> 8) & 0xFF;
        buf[3] = val & 0xFF;
        writeDSP(baseAddress + i, buf, 4);
    }
}

void calculateAndSendPeakingEQ(uint16_t baseAddress, float centerFreq, float qFactor, float gainDB, float fs = 48000.0) {
    if (baseAddress == 0) return;
    if (centerFreq < 20.0) centerFreq = 20.0;
    if (centerFreq > 24000.0) centerFreq = 24000.0;

    // Batasi Q-Factor antara 0.5 sampai 15
    if (qFactor < 0.5) qFactor = 0.5;
    if (qFactor > 15.0) qFactor = 15.0;

    float A = pow(10.0, gainDB / 40.0);
    float w0 = 2.0 * M_PI * centerFreq / fs;
    float alpha = sin(w0) / (2.0 * qFactor);
    float cosW0 = cos(w0);

    float b0 =  1.0 + alpha * A;
    float b1 = -2.0 * cosW0;
    float b2 =  1.0 - alpha * A;
    float a0 =  1.0 + alpha / A;
    float a1 = -2.0 * cosW0;
    float a2 =  1.0 - alpha / A;

    b0 /= a0; b1 /= a0; b2 /= a0;
    a1 /= a0; a2 /= a0;

    float coefficients[5] = {b0, b1, b2, a1, a2};
    for(int i = 0; i < 5; i++) {
        int32_t val = coefficients[i] * (1 << 23);
        uint8_t buf[4];
        buf[0] = (val >> 24) & 0xFF;
        buf[1] = (val >> 16) & 0xFF;
        buf[2] = (val >> 8) & 0xFF;
        buf[3] = val & 0xFF;
        writeDSP(baseAddress + i, buf, 4);
    }
}

// =========================================================================
// SISTEM PRESET (NVS)
// =========================================================================
void applyAllSettings() {
  for(int i = 0; i < 4; i++) {
    setMuteDSP(ADDR_MUTE[i], currentSettings.mute[i]);
    setGainDSP(ADDR_GAIN_OUT[i], currentSettings.gainOutput[i]);
    setDelayDSP(ADDR_DELAY[i], currentSettings.delaySamples[i]);
    setGainDSP(ADDR_LIMITER_THRESH[i], currentSettings.limiterThreshold[i]);
    setGainDSP(ADDR_MIXER[i][0], currentSettings.mixerInput[i][0]);
    setGainDSP(ADDR_MIXER[i][1], currentSettings.mixerInput[i][1]);
    calculateAndSendFirstOrderFilter(ADDR_GEN_FILTER_LOW[i], currentSettings.genFilterLowFreq[i], false);
    calculateAndSendFirstOrderFilter(ADDR_GEN_FILTER_HIGH[i], currentSettings.genFilterHighFreq[i], true);
    
    for(int b = 0; b < 5; b++) {
      calculateAndSendPeakingEQ(ADDR_EQ_BAND[i][b], currentSettings.eqBand[i][b].freq, currentSettings.eqBand[i][b].qFactor, currentSettings.eqBand[i][b].gain);
    }
  }
}

void savePreset(uint8_t slot) {
  String key = "preset_" + String(slot);
  preferences.begin("dsp_data", false); 
  preferences.putBytes(key.c_str(), &currentSettings, sizeof(DSP_Settings));
  preferences.end();
}

void loadPreset(uint8_t slot) {
  String key = "preset_" + String(slot);
  preferences.begin("dsp_data", true);
  if (preferences.getBytesLength(key.c_str()) == sizeof(DSP_Settings)) {
    preferences.getBytes(key.c_str(), &currentSettings, sizeof(DSP_Settings));
    applyAllSettings();
  }
  preferences.end();
}

// =========================================================================
// HALAMAN WEB UI (HTML + CSS + Canvas Grafik Perbaikan Skala dB & Hz)
// =========================================================================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>freeDSP Control Panel</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 0; padding: 15px; background: #1a1a1a; color: #fff; }
    .card { background: #2c2c2c; padding: 15px; margin-bottom: 15px; border-radius: 8px; box-shadow: 0 4px 6px rgba(0,0,0,0.3); }
    h2, h3 { color: #00d2ff; margin-top: 0; }
    .row { display: flex; flex-direction: column; margin-bottom: 10px; }
    .inline-row { display: flex; gap: 10px; align-items: center; margin-bottom: 8px; }
    .input-group { display: flex; gap: 5px; margin-top: 4px; }
    button { padding: 10px; background: #00d2ff; border: none; font-weight: bold; cursor: pointer; border-radius: 5px; color: #000; flex: 1; }
    button:active { background: #0098b8; }
    input[type=range] { width: 100%; accent-color: #00d2ff; }
    input[type=number] { padding: 8px; width: 100%; background: #444; border: 1px solid #555; color: #fff; border-radius: 4px; }
    .btn-set { background: #ff9800; color: #000; flex: 0 0 80px; font-weight: bold; }
    .eq-box { background: #222; padding: 10px; margin-bottom: 8px; border-radius: 5px; }
    .preset-box { display: flex; gap: 5px; }
    .canvas-wrap { position: relative; margin-top: 10px; background: #111; border-radius: 5px; padding: 5px; }
    canvas { width: 100%; height: 180px; display: block; }
  </style>
  <script>
    function sendAPI(endpoint, params) {
      fetch(`${endpoint}?${params}`).catch(err => console.error(err));
    }
    function updateVal(endpoint, ch, val, spanId, unit) {
      document.getElementById(spanId).innerText = val + " " + unit;
      sendAPI(endpoint, `ch=${ch}&val=${val}`);
    }
    function updateInputVal(endpoint, ch, inputId, spanId, unit) {
      let val = document.getElementById(inputId).value;
      document.getElementById(spanId).innerText = val + " " + unit;
      sendAPI(endpoint, `ch=${ch}&val=${val}`);
      drawGraph(ch);
    }
    function updateEQ(ch, band) {
      let qInput = document.getElementById(`eq_q_${ch}_${band}`);
      let qVal = parseFloat(qInput.value);
      // Validasi Q Factor otomatis di sisi web (Min 0.5, Max 15)
      if(qVal < 0.5) { qVal = 0.5; qInput.value = 0.5; }
      if(qVal > 15.0) { qVal = 15.0; qInput.value = 15.0; }

      let freq = document.getElementById(`eq_f_${ch}_${band}`).value;
      let gain = document.getElementById(`eq_g_${ch}_${band}`).value;
      sendAPI('/set_eq', `ch=${ch}&band=${band}&freq=${freq}&q=${qVal}&gain=${gain}`);
      drawGraph(ch);
    }
    function updateMute(ch, isMuted) {
      sendAPI('/set_mute', `ch=${ch}&val=${isMuted ? 1 : 0}`);
    }
    function presetAct(action, slot) {
      sendAPI(`/${action}_preset`, `slot=${slot}`);
      if(action == 'load') setTimeout(() => location.reload(), 600);
    }

    // Fungsi Render Grafik dengan Skala Tanda +, -, dan Hz Lebih Rapat di atas 100Hz
    function drawGraph(ch) {
      let canvas = document.getElementById(`graph_${ch}`);
      if(!canvas || !canvas.getContext) return;
      let ctx = canvas.getContext('2d');
      let w = canvas.width;
      let h = canvas.height;

      ctx.clearRect(0, 0, w, h);

      // Skala dB (Kiri) dengan tanda +, 0, dan -
      ctx.fillStyle = '#888';
      ctx.font = '9px Arial';
      ctx.strokeStyle = '#333';
      ctx.lineWidth = 1;

      let dbLevels = [15, 0, -15, -30];
      dbLevels.forEach(db => {
        let y = ((15 - db) / 45) * h;
        ctx.beginPath();
        ctx.moveTo(38, y); ctx.lineTo(w, y);
        ctx.stroke();
        let label = db > 0 ? "+" + db + "dB" : db + "dB";
        ctx.fillText(label, 2, y + 3);
      });

      // Skala Frekuensi Hz (Bawah) - Lebih rapat di atas 100Hz (Kelipatan 100, 500, 1k, dst)
      let freqMarkers = [10, 50, 100, 200, 500, 1000, 5000, 10000, 20000];
      freqMarkers.forEach(f => {
        let px = 38 + ((Math.log10(f / 10) / Math.log10(20000 / 10)) * (w - 38));
        ctx.strokeStyle = '#222';
        ctx.beginPath();
        ctx.moveTo(px, 0); ctx.lineTo(px, h - 15);
        ctx.stroke();
        
        ctx.fillStyle = '#777';
        let label = f >= 1000 ? (f/1000) + "k" : f + "Hz";
        ctx.fillText(label, px - 10, h - 3);
      });

      let lowF = parseFloat(document.getElementById(`numL_${ch}`).value) || 1000;
      let highF = parseFloat(document.getElementById(`numH_${ch}`).value) || 50;

      // Plot Kurva Respon Audio
      ctx.strokeStyle = '#00d2ff';
      ctx.lineWidth = 2;
      ctx.beginPath();

      for (let px = 38; px < w; px++) {
        let freq = 10 * Math.pow(20000 / 10, (px - 38) / (w - 38));
        let db = 0;

        if (freq > lowF) db -= (freq - lowF) / 400;
        if (freq < highF) db -= (highF - freq) / 15;

        for(let b=0; b<5; b++) {
          let eqF = parseFloat(document.getElementById(`eq_f_${ch}_${b}`).value) || 1000;
          let eqG = parseFloat(document.getElementById(`eq_g_${ch}_${b}`).value) || 0;
          let eqQ = parseFloat(document.getElementById(`eq_q_${ch}_${b}`).value) || 1.0;
          let diff = Math.log10(freq / eqF);
          db += eqG * Math.exp(-(diff * diff) * eqQ * 4);
        }

        let py = ((15 - db) / 45) * h;
        if (py < 0) py = 0;
        if (py > h) py = h;

        if (px === 38) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      ctx.stroke();
    }

    window.onload = function() {
      for(let i=0; i<4; i++) {
        let canvas = document.getElementById(`graph_${i}`);
        if(canvas) {
          canvas.width = canvas.offsetWidth;
          canvas.height = canvas.offsetHeight;
          drawGraph(i);
        }
      }
    };
  </script>
</head>
<body>
  <h2>freeDSP Controller</h2>
  
  <div class="card">
    <h3>Preset Management</h3>
    <div class="preset-box">
      <button onclick="presetAct('load', 1)">Load P1</button>
      <button onclick="presetAct('save', 1)">Save P1</button>
      <button onclick="presetAct('load', 2)">Load P2</button>
      <button onclick="presetAct('save', 2)">Save P2</button>
    </div>
  </div>

  <script>
    for(let i=0; i<4; i++) {
      document.write(`
        <div class="card">
          <h3>Output Channel ${i+1}</h3>
          
          <div class="inline-row">
            <label><b>Mute:</b></label>
            <input type="checkbox" onchange="updateMute(${i}, this.checked)">
          </div>

          <div class="row">
            <label>Gain Output (<span id="gain_${i}">0</span> dB):</label>
            <input type="range" min="-80" max="15" step="1" value="0" onchange="updateVal('/set_gain', ${i}, this.value, 'gain_${i}', 'dB')">
          </div>

          <div class="row">
            <label>Mixer Input A (<span id="mixA_${i}">0</span> dB):</label>
            <input type="range" min="-30" max="12" step="1" value="0" onchange="updateVal('/set_mixA', ${i}, this.value, 'mixA_${i}', 'dB')">
          </div>

          <div class="row">
            <label>Mixer Input B (<span id="mixB_${i}">0</span> dB):</label>
            <input type="range" min="-30" max="12" step="1" value="0" onchange="updateVal('/set_mixB', ${i}, this.value, 'mixB_${i}', 'dB')">
          </div>

          <div class="row">
            <label>Gen Filter 1st (Low Freq: <span id="filtL_${i}">1000</span> Hz):</label>
            <div class="input-group">
              <input type="number" id="numL_${i}" min="10" max="20000" value="1000">
              <button class="btn-set" onclick="updateInputVal('/set_filter_low', ${i}, 'numL_${i}', 'filtL_${i}', 'Hz')">Set</button>
            </div>
          </div>

          <div class="row">
            <label>Gen Filter 2nd (High Freq: <span id="filtH_${i}">50</span> Hz):</label>
            <div class="input-group">
              <input type="number" id="numH_${i}" min="10" max="20000" value="50">
              <button class="btn-set" onclick="updateInputVal('/set_filter_high', ${i}, 'numH_${i}', 'filtH_${i}', 'Hz')">Set</button>
            </div>
          </div>

          <!-- Canvas Grafik dengan Skala dB dan Hz Diperbaiki -->
          <div class="canvas-wrap">
            <canvas id="graph_${i}"></canvas>
          </div>

          <div class="row" style="margin-top:10px;">
            <label>Delay (<span id="del_${i}">0</span> samples):</label>
            <input type="range" min="0" max="100" step="1" value="0" onchange="updateVal('/set_delay', ${i}, this.value, 'del_${i}', 'samples')">
          </div>

          <div class="row">
            <label>Limiter Threshold (<span id="lim_${i}">0</span> dB):</label>
            <input type="range" min="-40" max="0" step="1" value="0" onchange="updateVal('/set_limiter', ${i}, this.value, 'lim_${i}', 'dB')">
          </div>

          <h4 style="color:#00d2ff; margin-bottom:5px;">Parametric EQ (Band 1 - 5)</h4>
`);
      for(let b=0; b<5; b++) {
        document.write(`
          <div class="eq-box">
            <label><b>Band ${b+1}</b></label>
            <div class="row"><small>Freq (Hz):</small><input type="number" id="eq_f_${i}_${b}" value="1000" onchange="updateEQ(${i}, ${b})"></div>
            <div class="row"><small>Q Factor (0.5 - 15):</small><input type="number" step="0.1" min="0.5" max="15" id="eq_q_${i}_${b}" value="1.0" onchange="updateEQ(${i}, ${b})"></div>
            <div class="row"><small>Gain (dB):</small><input type="number" step="0.5" id="eq_g_${i}_${b}" value="0" onchange="updateEQ(${i}, ${b})"></div>
          </div>
        `);
      }
      document.write(`</div>`);
    }
  </script>
</body>
</html>
)rawliteral";

// =========================================================================
// ENDPOINT WEB API (ROUTING)
// =========================================================================
void setupRouting() {
  server.on("/", HTTP_GET, [](){ server.send(200, "text/html", index_html); });

  server.on("/set_gain", HTTP_GET, [](){
    int ch = server.arg("ch").toInt(); float val = server.arg("val").toFloat();
    currentSettings.gainOutput[ch] = val; setGainDSP(ADDR_GAIN_OUT[ch], val);
    server.send(200, "text/plain", "OK");
  });

  server.on("/set_mixA", HTTP_GET, [](){
    int ch = server.arg("ch").toInt(); float val = server.arg("val").toFloat();
    currentSettings.mixerInput[ch][0] = val; setGainDSP(ADDR_MIXER[ch][0], val);
    server.send(200, "text/plain", "OK");
  });

  server.on("/set_mixB", HTTP_GET, [](){
    int ch = server.arg("ch").toInt(); float val = server.arg("val").toFloat();
    currentSettings.mixerInput[ch][1] = val; setGainDSP(ADDR_MIXER[ch][1], val);
    server.send(200, "text/plain", "OK");
  });

  server.on("/set_mute", HTTP_GET, [](){
    int ch = server.arg("ch").toInt(); bool val = server.arg("val").toInt() == 1;
    currentSettings.mute[ch] = val; setMuteDSP(ADDR_MUTE[ch], val);
    server.send(200, "text/plain", "OK");
  });

  server.on("/set_delay", HTTP_GET, [](){
    int ch = server.arg("ch").toInt(); int val = server.arg("val").toInt();
    currentSettings.delaySamples[ch] = val; setDelayDSP(ADDR_DELAY[ch], val);
    server.send(200, "text/plain", "OK");
  });

  server.on("/set_limiter", HTTP_GET, [](){
    int ch = server.arg("ch").toInt(); float val = server.arg("val").toFloat();
    currentSettings.limiterThreshold[ch] = val; setGainDSP(ADDR_LIMITER_THRESH[ch], val);
    server.send(200, "text/plain", "OK");
  });

  server.on("/set_filter_low", HTTP_GET, [](){
    int ch = server.arg("ch").toInt(); float val = server.arg("val").toFloat();
    currentSettings.genFilterLowFreq[ch] = val; calculateAndSendFirstOrderFilter(ADDR_GEN_FILTER_LOW[ch], val, false);
    server.send(200, "text/plain", "OK");
  });

  server.on("/set_filter_high", HTTP_GET, [](){
    int ch = server.arg("ch").toInt(); float val = server.arg("val").toFloat();
    currentSettings.genFilterHighFreq[ch] = val; calculateAndSendFirstOrderFilter(ADDR_GEN_FILTER_HIGH[ch], val, true);
    server.send(200, "text/plain", "OK");
  });

  server.on("/set_eq", HTTP_GET, [](){
    int ch = server.arg("ch").toInt();
    int band = server.arg("band").toInt();
    float freq = server.arg("freq").toFloat();
    float q = server.arg("q").toFloat();
    float gain = server.arg("gain").toFloat();

    // Validasi rentang Q-Factor di ESP32
    if (q < 0.5) q = 0.5;
    if (q > 15.0) q = 15.0;

    if(ch >= 0 && ch < 4 && band >= 0 && band < 5) {
      currentSettings.eqBand[ch][band].freq = freq;
      currentSettings.eqBand[ch][band].qFactor = q;
      currentSettings.eqBand[ch][band].gain = gain;
      calculateAndSendPeakingEQ(ADDR_EQ_BAND[ch][band], freq, q, gain);
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/save_preset", HTTP_GET, [](){ savePreset(server.arg("slot").toInt()); server.send(200, "text/plain", "Saved"); });
  server.on("/load_preset", HTTP_GET, [](){ loadPreset(server.arg("slot").toInt()); server.send(200, "text/plain", "Loaded"); });
}

// =========================================================================
// SETUP & LOOP UTAMA
// =========================================================================
void setup() {
  Serial.begin(115200);
  
  pinMode(DSP_RESET_PIN, OUTPUT);
  digitalWrite(DSP_RESET_PIN, LOW);
  delay(20);
  digitalWrite(DSP_RESET_PIN, HIGH);
  delay(1500); 

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  WiFi.softAP(ssid, password);
  Serial.print("Akses Web UI di IP: ");
  Serial.println(WiFi.softAPIP()); 

  loadPreset(1); 

  setupRouting();
  server.begin();
}

void loop() {
  server.handleClient();
}