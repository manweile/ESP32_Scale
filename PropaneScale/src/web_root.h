/**
 * @file web_root.h
 * @author Gerald Manweiler
 *
 * @brief Defines the HTML content for the root page served by the ESP32 web server.
 *
 * @details This file contains a raw string literal with the HTML, CSS, and JavaScript for the web interface of the PropaneScale project.
 * The page displays telemetry data and provides buttons to trigger tare and calibration actions via the WiFi API.
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
    <div><strong>Connection:</strong> ESP32 AP - PropaneScale</div>
    <div id="status">Loading telemetry...</div>
    <!-- Only show connection and status lines; diagnostic fields remain in JSON telemetry -->
    <div id="initEeprom" style="display:none;margin-top:6px;color:#333"></div>
    <div id="initCal" style="display:none;margin-top:6px;color:#333"></div>
    <div id="levelPrompt" style="margin-top:8px;color:#333;font-weight:600;"></div>
    <div id="startupPrompt" style="margin-top:8px;color:#0066aa;font-weight:700;"></div>
    <pre id="startupReport" style="background:#fff8e1;padding:8px;border:1px solid #ffd54f"></pre>
    <pre id="telemetry">{}</pre>
    <div>
      <button id="btnLevel">Level Read</button>
      <button id="btnLevelCancel" style="display:none;margin-left:6px">Cancel</button>
    </div>
  </div>

  <script>
    // persistent lines for level-read UX
    let levelLines = [];

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

    document.getElementById('btnLevel').addEventListener('click', async ()=>{
      const ok = await postAction('/api/level');
      if(!ok){ alert('Request failed'); return; }

      // initialize persistent prompt lines and poll status until IDLE
      levelLines = [
        'Waiting for tank placement...',
        'Load placement timeout: 15 seconds.',
        'Click cancel to end level read'
      ];
      document.getElementById('levelPrompt').innerHTML = levelLines.map(l=>'<div>'+l+'</div>').join('');
      document.getElementById('status').textContent = 'Level read started';
      document.getElementById('btnLevelCancel').style.display = 'inline-block';
      const poll = setInterval(async ()=>{
        try{
          const r = await fetch('/api/level/status');
          if(!r.ok) return;
            const j = await r.json();
            // update status line
            document.getElementById('status').textContent = j.state + (j.prompt ? ' - ' + j.prompt : '');

            // accumulate and display UX lines until OK pressed
            if (j.state === 'WAIT_LOAD') {
              // keep initial lines
            } else if (j.state === 'SETTLING') {
              const settling = 'Tank detected. Settling for 5 seconds before final read...';
              if (!levelLines.includes(settling)) {
                levelLines.push(settling);
                document.getElementById('levelPrompt').innerHTML = levelLines.map(l=>'<div>'+l+'</div>').join('');
              }
            } else if (j.state === 'READING') {
              const reading = 'Reading tank weight...';
              if (!levelLines.includes(reading)) {
                levelLines.push(reading);
                document.getElementById('levelPrompt').innerHTML = levelLines.map(l=>'<div>'+l+'</div>').join('');
              }
            }

                  if(j.report) {
                    // show persistent modal with OK button instead of alert
                    clearInterval(poll);
                    document.getElementById('btnLevelCancel').style.display = 'none';
                    document.getElementById('reportText').textContent = j.report;
                    // mark current report type so OK posts to the correct ack endpoint
                    window.__currentReportType = 'level';
                    document.getElementById('reportModal').style.display = 'block';
                  } else if(j.state === 'IDLE') {
              // no report yet but idle
              clearInterval(poll);
              document.getElementById('btnLevelCancel').style.display = 'none';
              fetchTelemetry();
            }
        }catch(e){ /* ignore */ }
      }, 1000);
    });

      document.getElementById('btnLevelCancel').addEventListener('click', async ()=>{
        const ok = await postAction('/api/level/cancel');
        if (ok) {
          document.getElementById('status').textContent = 'Cancelled';
          document.getElementById('btnLevelCancel').style.display = 'none';
        } else {
          alert('Cancel request failed');
        }
      });

      // Startup tare polling
      async function pollStartupStatus(){
        try{
          const r = await fetch('/api/startup/status');
          if(!r.ok) return;
          const j = await r.json();
          // update startup prompt and report
          document.getElementById('startupPrompt').textContent = j.prompt ? j.prompt : '';
          document.getElementById('startupReport').textContent = j.report ? j.report : '';
            // If a startup report exists, show the shared modal and set ack target
            if (j.report) {
              document.getElementById('reportText').textContent = j.report;
              window.__currentReportType = 'startup';
              document.getElementById('reportModal').style.display = 'block';
            }
        }catch(e){ /* ignore */ }
      }

      // Poll the application-level status endpoint for init diagnostics and fields
      async function pollAppStatus(){
        try{
          const r = await fetch('/api/app/status');
          if(!r.ok) return;
          const j = await r.json();
          // startupReport may be null
          document.getElementById('startupReport').textContent = j.startupReport ? j.startupReport : '';
          // Keep diagnostic fields only inside the JSON block; do not surface them as separate lines
          // If the app-level startupReport contains content, show modal for ack
          if (j.startupReport) {
            document.getElementById('reportText').textContent = j.startupReport;
            window.__currentReportType = 'startup';
            document.getElementById('reportModal').style.display = 'block';
          }
        }catch(e){ /* ignore */ }
      }

      // poll startup status once a second
      setInterval(pollStartupStatus, 1000);
      pollStartupStatus();

      // poll app status (init diagnostics) once a second
      setInterval(pollAppStatus, 1000);
      pollAppStatus();

    // poll every second
    fetchTelemetry();
    setInterval(fetchTelemetry, 1000);
  </script>
  <style>
    /* simple modal for report */
    #reportModal{display:none;position:fixed;left:0;top:0;width:100%;height:100%;background:rgba(0,0,0,0.4);align-items:center;justify-content:center}
    #reportModal .box{background:#fff;padding:12px;border-radius:6px;max-width:520px;margin:auto}
    #reportModal pre{white-space:pre-wrap}
  </style>
  <div id="reportModal"><div class="box"><h3>Level Read Result</h3><pre id="reportText"></pre><div style="text-align:right"><button id="reportOk">OK</button></div></div></div>
  <script>
    document.getElementById('reportOk').addEventListener('click', async ()=>{
      // Acknowledge the report server-side so subsequent polls don't return stale data
      const reportType = window.__currentReportType || 'level';
      const path = reportType === 'startup' ? '/api/startup/ack' : '/api/level/ack';
      const ok = await postAction(path);
      if (!ok) {
        alert('Failed to acknowledge report');
        return;
      }
      document.getElementById('reportModal').style.display = 'none';
      // clear persistent level prompt lines when user acknowledges
      levelLines = [];
      document.getElementById('levelPrompt').textContent = '';
      // clear startup prompt/report display
      document.getElementById('startupPrompt').textContent = '';
      document.getElementById('startupReport').textContent = '';
      fetchTelemetry();
    });
  </script>
</body>
</html>
)rawliteral";
