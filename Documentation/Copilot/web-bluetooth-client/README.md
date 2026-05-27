# Web Bluetooth Client (scaffold)

Minimal scaffold for a Web Bluetooth client to talk to the PropaneScale ESP32.

Purpose
- Provide a tiny single-file SPA skeleton that demonstrates the UI layout and where BLE logic should live.
- Not implemented: this is a scaffold only; connection and full feature logic are left as TODOs.

Run (local dev)
- Serve the folder over `localhost` (HTTPS required for Web Bluetooth in many browsers). For quick testing you can run a local server on the development machine and open the page there.

Example (Python 3):
```bash
python -m http.server 8000
# then open http://localhost:8000 in a browser
```

Files
- `index.html` — UI skeleton
- `app.js` — client-side JS stubs and helpers
- `styles.css` — minimal styling

Next steps
- Implement BLE connect/disconnect, subscribe to notifications, and wire controls to write characteristics.
- Add reconnection/backoff and permission/error handling.
