// Embedded single-file web UI for PropaneScale
/**
 * @file web_root.h
 * @author Gerald Manweiler
 * 
 * @brief Defines the HTML content for the root page served by the ESP32 web server.
 * 
 * @details This file contains a raw string literal with the HTML, CSS, and JavaScript for the web interface of the PropaneScale project. The page displays telemetry data and provides buttons to trigger tare and calibration actions via the WiFi API.
 * 
 * @version 0.1
 * @date 2024-06-01
 * @copyright Copyright (c) 2024 Gerald Manweiler
 */

#pragma once

// Root HTML served at '/'
static const char ROOT_PAGE[] = R"rawliteral(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>PropaneScale</title>
  <style>
    body{font-family:Arial,Helvetica,sans-serif;margin:12px}
    .card{border:1px solid #ccc;padding:12px;border-radius:6px;max-width:520px}
    button{margin:6px}
    pre{background:#f6f6f6;padding:8px}
  </style>
</head>
<body>
  <h1>PropaneScale</h1>
  <div class="card">
    <div><strong>Connection:</strong> ESP32 AP - PropaneScale (no password)</div>
    <div id="status">Loading telemetry...</div>
    <pre id="telemetry">{}</pre>
    <div>
      <button id="btnTare">Tare</button>
      <button id="btnCal">Calibrate (prompt)</button>
      <button id="btnSave">Save Calibration</button>
    </div>
  </div>

  <script>
    async function fetchTelemetry(){
      try{
        const r = await fetch('/api/telemetry');
        if(!r.ok){ document.getElementById('status').textContent = 'No telemetry'; return; }
        const j = await r.json();
        document.getElementById('telemetry').textContent = JSON.stringify(j, null, 2);
        document.getElementById('status').textContent = 'OK';
      }catch(e){ document.getElementById('status').textContent = 'Error'; }
    }

    async function postAction(path, qs){
      try{
        const url = qs ? path + '?' + qs : path;
        const r = await fetch(url, {method:'POST'});
        return r.ok;
      }catch(e){return false}
    }

    document.getElementById('btnTare').addEventListener('click', async ()=>{
      const ok = await postAction('/api/tare');
      alert(ok ? 'Tare enqueued' : 'Request failed');
    });

    document.getElementById('btnCal').addEventListener('click', async ()=>{
      const w = prompt('Known weight (lbs):', '26.0');
      if(w !== null){
        const ok = await postAction('/api/calibrate', 'weight=' + encodeURIComponent(w));
        alert(ok ? 'Calibration enqueued' : 'Request failed');
      }
    });

    document.getElementById('btnSave').addEventListener('click', async ()=>{
      const ok = await postAction('/api/save');
      alert(ok ? 'Save requested' : 'Request failed');
    });

    // poll every second
    fetchTelemetry();
    setInterval(fetchTelemetry, 1000);
  </script>
</body>
</html>
)rawliteral";
