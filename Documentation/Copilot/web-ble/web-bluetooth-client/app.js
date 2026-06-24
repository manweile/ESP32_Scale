// app.js — scaffold for Web Bluetooth client
// NOTE: This file contains stubs and helpers only; the BLE connect/subscribe
// logic is intentionally left as TODO and must be implemented when ready.

(function(){
  // UUIDs from firmware spec
  const SERVICE_UUID = '0000feed-0000-1000-8000-00805f9b34fb';
  const CHAR_WEIGHT = '0000be01-0000-1000-8000-00805f9b34fb';
  const CHAR_TARE = '0000be02-0000-1000-8000-00805f9b34fb';
  const CHAR_CALIBRATE = '0000be03-0000-1000-8000-00805f9b34fb';
  const CHAR_RAW = '0000be04-0000-1000-8000-00805f9b34fb';
  const CHAR_SAMPLING = '0000be05-0000-1000-8000-00805f9b34fb';
  const CHAR_BATTERY = '0000be06-0000-1000-8000-00805f9b34fb';

  // UI refs
  const $ = id => document.getElementById(id);
  const deviceNameEl = $('device-name');
  const connectBtn = $('connect-btn');
  const disconnectBtn = $('disconnect-btn');
  const weightDisplay = $('weight-display');
  const tareBtn = $('tare-btn');
  const calibrateBtn = $('calibrate-btn');
  const samplingInput = $('sampling-input');
  const setSamplingBtn = $('set-sampling-btn');
  const logEl = $('log');

  let device = null;
  let server = null;
  let characteristics = {};

  function log(...args){
    const line = `[${new Date().toISOString()}] ${args.join(' ')}\n`;
    logEl.textContent += line;
    logEl.scrollTop = logEl.scrollHeight;
    console.log(...args);
  }

  // Parsing helper (from docs) — safe to use when notifications arrive
  function parseWeightNotification(buffer){
    const dv = buffer instanceof DataView ? buffer : new DataView(buffer);
    const version = dv.getUint8(0);
    // Firmware sends IEEE-754 float32 (little-endian) representing pounds
    const weight_lbs = dv.getFloat32(1, true);
    const seq = dv.getUint16(5, true);
    return {version, weight_lbs, seq};
  }

  // UI enable/disable helpers
  function setConnectedState(connected){
    deviceNameEl.textContent = connected && device ? (device.name || 'ESP32') : 'Not connected';
    connectBtn.disabled = connected;
    disconnectBtn.disabled = !connected;
    tareBtn.disabled = !connected;
    calibrateBtn.disabled = !connected;
    setSamplingBtn.disabled = !connected;
  }

  // TODO: implement BLE connection flow. The steps are:
  // 1) navigator.bluetooth.requestDevice({ filters: [{ services: [SERVICE_UUID] }] })
  // 2) device.gatt.connect()
  // 3) getPrimaryService(SERVICE_UUID) and getCharacteristic(...) for needed chars
  // 4) startNotifications() for weight/raw/battery and set oncharacteristicvaluechanged handlers
  // 5) wire UI buttons to writes on tare/calibrate/sampling characteristics
  // The actual calls and error handling are intentionally left as TODO.
  async function connect(){
    if(!('bluetooth' in navigator)){
      log('Web Bluetooth not available in this browser.');
      alert('Web Bluetooth API not available. Use Chrome/Edge on supported platforms.');
      return;
    }

    log('TODO: implement navigator.bluetooth.requestDevice and connect flow');
    // Example (commented):
    // const d = await navigator.bluetooth.requestDevice({ filters: [{ services: [SERVICE_UUID] }] });
    // device = d;
    // server = await device.gatt.connect();
    // const svc = await server.getPrimaryService(SERVICE_UUID);
    // characteristics.weight = await svc.getCharacteristic(CHAR_WEIGHT);
    // await characteristics.weight.startNotifications();
    // characteristics.weight.addEventListener('characteristicvaluechanged', e => {
    //   const parsed = parseWeightNotification(e.target.value.buffer);
    //   onWeight(parsed);
    // });
    setConnectedState(true);
  }

  function disconnect(){
    if(device && device.gatt && device.gatt.connected){
      try{ device.gatt.disconnect(); }catch(e){ console.warn(e); }
    }
    device = null; server = null; characteristics = {};
    setConnectedState(false);
    log('Disconnected');
  }

  // Handler when a parsed weight arrives
  function onWeight({version, weight_lbs, seq}){
    const lbs = weight_lbs.toFixed(2);
    weightDisplay.textContent = `${lbs} lb`;
    log(`weight v${version} seq=${seq} ${lbs} lb`);
  }

  // Wire UI
  function initUI(){
    connectBtn.addEventListener('click', connect);
    disconnectBtn.addEventListener('click', disconnect);
    tareBtn.addEventListener('click', ()=>{
      log('TODO: write TARE command to characteristic');
    });
    calibrateBtn.addEventListener('click', ()=>{
      log('TODO: open calibrate dialog and write CALIBRATE command');
    });
    setSamplingBtn.addEventListener('click', ()=>{
      log('TODO: write sampling interval to sampling characteristic: ', samplingInput.value);
    });
  }

  // Initialize on load
  window.addEventListener('load', ()=>{
    initUI();
    setConnectedState(false);
    log('Scaffold loaded — BLE logic not implemented.');
  });

  // Expose helpers for testing (optional)
  window.PropaneScaleClient = { parseWeightNotification };

})();
