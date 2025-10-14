// ==UserScript==
// @name         GoSweetSpot Auto-Fill + Measure (Next Available Row)
// @namespace    http://tampermonkey.net/
// @version      V-05/10/25 20:45
// @description  Trigger measurement via MQTT and auto-fill the next available row in GoSweetSpot
// @match        https://nzc.gosweetspot.com/ship
// @grant        none
// ==/UserScript==

(function () {
  'use strict';

  console.log("[GSS] Script running...");

  // ---------- Config ----------
  const JSON_URL = "https://nzc.redlite.nz:8443/";  // your JSON endpoint
  const POLL_TIMEOUT_MS = 7000;                     // max wait for fresh data after CAPTURE
  const POLL_INTERVAL_MS = 400;

  // MQTT over WebSocket (set to your broker's WS/WSS endpoint)
  const MQTT_WS_URL = "wss://nzc.redlite.nz:9001";  // e.g., mosquitto websockets
  const MQTT_TOPIC_CMD = "measure/cmd";
  const MQTT_PAYLOAD = "CAPTURE";
  const MQTT_JS_URL = "https://unpkg.com/mqtt/dist/mqtt.min.js";

  // ---------- UI creation ----------
  function createFixedFetchButton() {
    const id = "gss-fetch";
    if (document.getElementById(id)) return;

    const btn = document.createElement("button");
    btn.id = id;
    btn.innerText = "Fetch Data";
    Object.assign(btn.style, {
      position: "fixed",
      top: "10px",
      right: "10px",
      zIndex: "1000",
      padding: "10px 14px",
      fontSize: "14px",
      background: "#007bff",
      color: "#fff",
      border: "1px solid #0056b3",
      borderRadius: "5px",
      cursor: "pointer",
      boxShadow: "0 2px 5px rgba(0,0,0,0.3)"
    });
    // FIX: use block bodies so arrow function doesn't return an assignment
    btn.addEventListener("mouseenter", () => { btn.style.background = "#0056b3"; });
    btn.addEventListener("mouseleave", () => { btn.style.background = "#007bff"; });
    btn.addEventListener("click", fetchData);
    document.body.appendChild(btn);
  }

  // Insert MEASURE button into the target panel heading
  async function createMeasureButtonInPanel() {
    const id = "gss-measure";
    if (document.getElementById(id)) return;

    const target = await waitForPanelHeading();
    if (!target) {
      console.warn("[GSS] Panel heading not found; falling back to fixed-position MEASURE button.");
      createFallbackMeasureButton();
      return;
    }

    const btn = document.createElement("button");
    btn.id = id;
    btn.type = "button";
    btn.innerText = "Measure";
    // Use existing site styles if present
    btn.className = "btn btn-primary btn-sm";
    Object.assign(btn.style, {
      marginLeft: "8px",
      float: "right"
    });
    btn.addEventListener("click", measureThenFetch);

    try {
      target.appendChild(btn);
    } catch {
      const wrap = document.createElement("div");
      wrap.style.display = "flex";
      wrap.style.alignItems = "center";
      wrap.style.justifyContent = "space-between";
      while (target.firstChild) wrap.appendChild(target.firstChild);
      target.appendChild(wrap);
      wrap.appendChild(btn);
    }

    console.log("[GSS] MEASURE button inserted into panel heading.");
  }

  function createFallbackMeasureButton() {
    const id = "gss-measure";
    if (document.getElementById(id)) return;
    const btn = document.createElement("button");
    btn.id = id;
    btn.innerText = "Measure";
    Object.assign(btn.style, {
      position: "fixed",
      top: "10px",
      right: "120px",
      zIndex: "1000",
      padding: "10px 14px",
      fontSize: "14px",
      background: "#28a745",
      color: "#fff",
      border: "1px solid #1e7e34",
      borderRadius: "5px",
      cursor: "pointer",
      boxShadow: "0 2px 5px rgba(0,0,0,0.3)"
    });
    // FIX: same ESLint-safe handlers here
    btn.addEventListener("mouseenter", () => { btn.style.background = "#218838"; });
    btn.addEventListener("mouseleave", () => { btn.style.background = "#28a745"; });
    btn.addEventListener("click", measureThenFetch);
    document.body.appendChild(btn);
  }

  // Wait for the specific panel heading you provided
  function waitForPanelHeading(timeoutMs = 8000) {
    const cssSel = "#accordionpackages > div > div.panel-heading";
    const xSel = '/html/body/div[1]/div[2]/div/div[5]/div[2]/div[2]/div/div[1]';

    return new Promise(resolve => {
      const t0 = Date.now();

      const tryFind = () => {
        let el = document.querySelector(cssSel);
        if (!el) {
          try {
            const res = document.evaluate(xSel, document, null, XPathResult.FIRST_ORDERED_NODE_TYPE, null);
            el = res.singleNodeValue;
          } catch {/* ignore */}
        }
        if (el) return resolve(el);
        if (Date.now() - t0 > timeoutMs) return resolve(null);
        setTimeout(tryFind, 150);
      };

      tryFind();
    });
  }

  // ---------- MQTT (browser) ----------
  let mqttClient = null;
  let mqttReady = false;
  let mqttLoading = false;

  function loadMqttJs() {
    if (window.mqtt) return Promise.resolve();
    if (mqttLoading) {
      return new Promise((res, rej) => {
        const iv = setInterval(() => { if (window.mqtt) { clearInterval(iv); res(); } }, 100);
        setTimeout(() => rej(new Error("mqtt.js load timeout")), 6000);
      });
    }
    mqttLoading = true;
    return new Promise((resolve, reject) => {
      const s = document.createElement('script');
      s.src = "https://unpkg.com/mqtt/dist/mqtt.min.js";
      s.onload = () => resolve();
      s.onerror = () => reject(new Error("Failed to load mqtt.min.js"));
      document.head.appendChild(s);
    });
  }

  async function ensureMqtt() {
    if (mqttReady && mqttClient) return mqttClient;
    await loadMqttJs();

    return new Promise((resolve, reject) => {
      try {
        new URL(MQTT_WS_URL);
        mqttClient = window.mqtt.connect(MQTT_WS_URL, {
          clientId: "vm-gss-" + Math.random().toString(36).slice(2, 8),
          clean: true,
          connectTimeout: 5000,
          reconnectPeriod: 2000
        });
        mqttClient.on('connect', () => { console.log("[GSS][MQTT] connected"); mqttReady = true; resolve(mqttClient); });
        mqttClient.on('error', (e) => console.error("[GSS][MQTT] error:", e?.message || e));
        mqttClient.on('reconnect', () => console.log("[GSS][MQTT] reconnecting..."));
        mqttClient.on('close', () => { console.log("[GSS][MQTT] closed"); mqttReady = false; });
      } catch (e) { reject(e); }
    });
  }

  async function publishCapture() {
    const client = await ensureMqtt();
    return new Promise((resolve, reject) => {
      client.publish(MQTT_TOPIC_CMD, MQTT_PAYLOAD, { qos: 0, retain: false }, (err) => {
        if (err) { console.error("[GSS][MQTT] publish failed:", err); reject(err); }
        else { console.log("[GSS][MQTT] CAPTURE sent"); resolve(); }
      });
    });
  }

  // ---------- Fetch + Fill ----------
  async function fetchData() {
    console.log("[GSS][Fetch] Requesting JSON data...");
    try {
      const response = await fetch(JSON_URL, { method: "GET", mode: "cors", headers: { "Accept": "application/json" } });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      const data = await response.json();
      console.log("[GSS][Fetch] JSON:", data);

      if (!isValidPayload(data)) {
        console.error("[GSS][Fetch] Invalid JSON format:", data);
        alert("Invalid data received. Check your API response.");
        return;
      }
      autoFillNextAvailableRow(data);
    } catch (err) {
      console.error("[GSS][Fetch] Error:", err);
      alert("Failed to fetch data. If self-signed, open the JSON URL once and accept the cert.");
    }
  }

  async function measureThenFetch() {
    try {
      await publishCapture();

      const started = Date.now();
      let tries = 0;
      while (Date.now() - started < POLL_TIMEOUT_MS) {
        tries++;
        try {
          const resp = await fetch(JSON_URL, { method: "GET", mode: "cors", headers: { "Accept": "application/json" }, cache: "no-store" });
          if (resp.ok) {
            const data = await resp.json();
            if (isValidPayload(data)) {
              console.log(`[GSS][Measure] Got data after ${tries} poll(s):`, data);
              autoFillNextAvailableRow(data);
              return;
            }
          }
        } catch (_) { /* ignore transient poll errors */ }
        await sleep(POLL_INTERVAL_MS);
      }
      alert("Timed out waiting for measurement JSON.");
    } catch (err) {
      console.error("[GSS][Measure] Failed:", err);
      alert("Could not trigger measurement. Check MQTT WS URL and broker.");
    }
  }

  function isValidPayload(data) {
    if (typeof data !== "object" || data == null) return false;
    const okNum = v => v !== null && v !== "" && isFinite(+v);
    return okNum(data.length) && okNum(data.width) && okNum(data.height);
  }

  function sleep(ms) { return new Promise(res => setTimeout(res, ms)); }

  function autoFillNextAvailableRow(data) {
    const nextRowNumber = findNextEmptyRow();
    if (nextRowNumber === -1) {
      alert("No available rows left to fill.");
      return;
    }
    console.log(`[GSS][Form] Filling row ${nextRowNumber}`, data);

    function setValue(selector, value) {
      const input = document.querySelector(`input[type="text"][data-stock-dim="${selector}"][data-row="${nextRowNumber}"]`);
      if (input) {
        input.value = value;
        input.dispatchEvent(new Event('input', { bubbles: true }));
        console.log(`[GSS][Form] Set ${selector} row ${nextRowNumber} -> ${value}`);
      } else {
        console.warn(`[GSS][Form] Field ${selector} not found in row ${nextRowNumber}`);
      }
    }

    setValue("length", data.length);
    setValue("width",  data.width);
    setValue("height", data.height);
    setValue("kg",     data.weight ?? 0);
    console.log(`[GSS][Form] Row ${nextRowNumber} filled.`);
  }

  function findNextEmptyRow() {
    const rows = document.querySelectorAll('tr[data-row]');
    for (const row of rows) {
      const rowNum = row.getAttribute("data-row");
      const lengthField = document.querySelector(`input[type="text"][data-stock-dim="length"][data-row="${rowNum}"]`);
      if (lengthField && !lengthField.value.trim()) {
        console.log(`[GSS][Form] Empty row: ${rowNum}`);
        return rowNum;
      }
    }
    console.warn("[GSS][Form] No empty row found.");
    return -1;
  }

  // Init
  createFixedFetchButton();      // keep the Fetch button as a floating fallback
  createMeasureButtonInPanel();  // put MEASURE in your requested panel heading
})();
