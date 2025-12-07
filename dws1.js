// ==UserScript==
// @name         GoSweetSpot Auto-Fill + Measure (Next Available Row)
// @namespace    http://tampermonkey.net/
// @version      V-22/11/25 21:10-WS
// @description  Trigger measurement via MeasurePi and auto-fill the next available row in GoSweetSpot
// @match        https://nzc.gosweetspot.com/ship
// @grant        none
// ==/UserScript==

(function () {
  'use strict';

  console.log("[GSS] Script running (WebSocket mode)...");

  // ---------- Config ----------
  const API_BASE = "https://nzc.redlite.nz:9001/";        // MeasurePi Flask base
  const WS_URL   = "wss://nzc.redlite.nz:9001/ws";        // MeasurePi WebSocket

  const JSON_URL          = API_BASE + "json";            // returns { current: {...}, history: [...] }
  const CAPTURE_URL       = API_BASE + "api/capture";     // triggers CAP to UNO via MQTT
  const MANUAL_WEIGHT_URL = API_BASE + "api/manual_weight";

  const POLL_TIMEOUT_MS   = 7000;   // max wait for fresh data after CAPTURE
  const POLL_INTERVAL_MS  = 400;    // HTTP poll interval (fallback if WS fails)

  // ---------- Manual weight helpers ----------
  const promptForManualWeight = async (message = "Scale offline. Enter weight in kg:") => {
    let lastValue = "";
    while (true) {
      const response = window.prompt(message, lastValue);
      if (response === null) {
        return null;
      }

      const parsed = Number.parseFloat(response);
      if (!Number.isFinite(parsed) || parsed < 0) {
        alert("Please enter a valid non-negative number for weight (kg).");
        lastValue = response;
        continue;
      }

      return Math.round(parsed * 1000) / 1000;
    }
  };

  async function submitManualWeight(weightKg) {
    const body = JSON.stringify({ weight: weightKg });
    const response = await fetch(MANUAL_WEIGHT_URL, {
      method: "POST",
      mode: "cors",
      headers: { "Content-Type": "application/json" },
      body
    });

    if (!response.ok) {
      const errorText = await response.text().catch(() => "");
      throw new Error(`Manual weight API error (${response.status}): ${errorText}`.trim());
    }

    return response.json().catch(() => ({ status: "accepted" }));
  }

  const needsManualWeight = (data) => {
    if (!data || typeof data !== "object") return false;
    if (data.manual_weight_required === true) return true;
    if (data.scale_present === false) {
      const w = Number.parseFloat(data.weight ?? data.weight_gross ?? data.weight_net);
      if (!Number.isFinite(w)) return true;
    }
    const weightValue = Number.parseFloat(data.weight ?? data.weight_gross ?? data.weight_net);
    return !Number.isFinite(weightValue);
  };

  async function ensureManualWeight(data) {
    if (!needsManualWeight(data)) {
      return data;
    }

    const manualWeight = await promptForManualWeight();
    if (manualWeight === null) {
      throw new Error("Manual weight entry cancelled by user");
    }

    try {
      await submitManualWeight(manualWeight);
    } catch (err) {
      console.warn("[GSS][ManualWeight] Submission failed", err);
    }

    const normalized = Number.parseFloat(manualWeight.toFixed(3));
    data.weight = normalized;
    data.weight_net = normalized;
    data.weight_gross = normalized;
    data.manual_weight = true;
    data.manual_weight_required = false;
    return data;
  }

  // ---------- WebSocket: keep latest measurement in memory ----------
  let ws = null;
  let wsConnected = false;
  let latestMeasurement = null;
  let latestMeasurementTimestamp = 0;  // seconds since epoch (server timestamp if available)

  function connectWebSocket() {
    if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) {
      return;
    }

    console.log("[MeasureWS] Connecting to", WS_URL);
    ws = new WebSocket(WS_URL);

    ws.onopen = () => {
      wsConnected = true;
      console.log("[MeasureWS] connected");
    };

    ws.onmessage = (event) => {
      try {
        const msg = JSON.parse(event.data);
        if (msg && msg.type === "current_measurement" && msg.data) {
          const d = normalizeMeasurement(msg.data) ?? msg.data;
          if (!isValidPayload(d)) return;
          const ts = typeof msg.timestamp === "number"
            ? msg.timestamp
            : (typeof d.timestamp === "number" ? d.timestamp : (Date.now() / 1000));

          latestMeasurement = d;
          latestMeasurementTimestamp = ts;
          // console.log("[MeasureWS] measurement:", d);
        }
      } catch (e) {
        console.error("[MeasureWS] parse error:", e, event.data);
      }
    };

    ws.onclose = () => {
      wsConnected = false;
      console.log("[MeasureWS] closed, retrying in 2s");
      setTimeout(connectWebSocket, 2000);
    };

    ws.onerror = (err) => {
      console.error("[MeasureWS] error", err);
      // let onclose handle reconnect
    };
  }

  async function waitForNewMeasurement(prevTimestamp, timeoutMs) {
    const start = Date.now();
    while (Date.now() - start < timeoutMs) {
      if (latestMeasurement && latestMeasurementTimestamp) {
        if (!prevTimestamp || latestMeasurementTimestamp > prevTimestamp) {
          return { ...latestMeasurement }; // shallow copy
        }
      }
      await sleep(200);
    }
    return null;
  }

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
    btn.addEventListener("mouseenter", () => { btn.style.background = "#0056b3"; });
    btn.addEventListener("mouseleave", () => { btn.style.background = "#007bff"; });
    btn.addEventListener("click", fetchData);
    document.body.appendChild(btn);
  }

  function injectMeasureButtonIntoRow(row) {
    const rowNum = row?.getAttribute?.("data-row");
    if (!rowNum) return;
    if (row.querySelector('.gss-measure-row-btn')) return;

    let cell = row.querySelector('td:last-child');
    if (!cell) {
      cell = document.createElement('td');
      row.appendChild(cell);
    }

    const btn = document.createElement("button");
    btn.type = "button";
    btn.innerText = "Measure";
    btn.className = "btn btn-primary btn-sm gss-measure-row-btn";
    Object.assign(btn.style, {
      marginLeft: "8px",
      float: "right"
    });
    btn.addEventListener("click", () => measureThenFetch(rowNum));

    cell.appendChild(btn);
  }

  function createMeasureButtonsForRows() {
    const rows = document.querySelectorAll('tr[data-row]');
    rows.forEach(injectMeasureButtonIntoRow);

    const observer = new MutationObserver(mutations => {
      for (const mutation of mutations) {
        for (const node of mutation.addedNodes) {
          if (!(node instanceof HTMLElement)) continue;
          if (node.matches('tr[data-row]')) {
            injectMeasureButtonIntoRow(node);
          } else {
            node.querySelectorAll?.('tr[data-row]').forEach(injectMeasureButtonIntoRow);
          }
        }
      }
    });

    observer.observe(document.body, { childList: true, subtree: true });
  }

  // ---------- Payload helpers ----------
  function extractMeasurementFromPayload(payload) {
    if (!payload || typeof payload !== "object") return null;
    if (payload.current && typeof payload.current === "object") {
      return payload.current;
    }
    return payload;
  }

  function normalizeMeasurement(data) {
    if (typeof data !== "object" || data == null) return null;

    const dimensionVal = Number.parseFloat(
      data.dimension ?? data.dimension_cm ?? data.dim
    );

    const pickVal = (...keys) => {
      for (const k of keys) {
        const v = Number.parseFloat(data[k]);
        if (Number.isFinite(v)) return v;
      }
      if (Number.isFinite(dimensionVal)) return dimensionVal;
      return null;
    };

    const normalized = {
      height: pickVal("height", "height_box", "height_cm"),
      width: pickVal("width", "width_box", "width_cm"),
      length: pickVal("length", "length_box", "length_cm"),
    };

    const weightVal = pickVal("weight", "weight_gross", "weight_net");
    if (Number.isFinite(weightVal)) {
      normalized.weight = weightVal;
    }

    return normalized;
  }

  function isValidPayload(data) {
    if (typeof data !== "object" || data == null) return false;
    const okNum = v => v !== null && v !== "" && isFinite(+v);
    return okNum(data.length) && okNum(data.width) && okNum(data.height);
  }

  function sleep(ms) { return new Promise(res => setTimeout(res, ms)); }

  // ---------- Fetch + Fill ----------
  async function fetchData(targetRowNumber = null) {
    console.log("[GSS][Fetch] Requesting data...");

    try {
      let data = null;

      // Prefer last WebSocket measurement if available
      if (latestMeasurement && isValidPayload(latestMeasurement)) {
        data = { ...latestMeasurement };
        console.log("[GSS][Fetch] Using WS measurement:", data);
      } else {
        const response = await fetch(JSON_URL, {
          method: "GET",
          mode: "cors",
          headers: { "Accept": "application/json" }
        });
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        const payload = await response.json();
        const m = normalizeMeasurement(extractMeasurementFromPayload(payload));
        if (!isValidPayload(m)) {
          console.error("[GSS][Fetch] Invalid JSON format:", payload);
          alert("Invalid data received. Check your API response.");
          return;
        }
        data = m;
        console.log("[GSS][Fetch] HTTP measurement:", data);
      }

      try {
        await ensureManualWeight(data);
      } catch (manualErr) {
        console.warn("[GSS][ManualWeight] Cancelled or failed", manualErr);
        alert("Manual weight entry is required to continue.");
        return;
      }
      autoFillRow(data, targetRowNumber);
    } catch (err) {
      console.error("[GSS][Fetch] Error:", err);
      alert("Failed to fetch data. If using self-signed cert, open the API URL once and accept the certificate.");
    }
  }

  async function measureThenFetch(targetRowNumber = null) {
    console.log("[GSS][Measure] Triggering capture...");
    const prevTs = latestMeasurementTimestamp || 0;

    try {
      // Trigger capture via MeasurePi API (which publishes CAP to MQTT)
      const capResp = await fetch(CAPTURE_URL, {
        method: "POST",
        mode: "cors",
        headers: { "Accept": "application/json" }
      });

      if (!capResp.ok) {
        const text = await capResp.text().catch(() => "");
        throw new Error(`Capture API error (${capResp.status}): ${text}`);
      }

      const capJson = await capResp.json().catch(() => ({}));
      console.log("[GSS][Measure] Capture response:", capJson);

      // Try WebSocket-first: wait for a *new* measurement
      let data = null;
      if (wsConnected) {
        data = await waitForNewMeasurement(prevTs, POLL_TIMEOUT_MS);
        if (data && isValidPayload(data)) {
          console.log("[GSS][Measure] Got WS measurement:", data);
        }
      }

      // Fallback: HTTP polling on /json
      if (!data || !isValidPayload(data)) {
        console.log("[GSS][Measure] WS wait failed or invalid; falling back to HTTP poll.");
        const started = Date.now();
        let tries = 0;
        while (Date.now() - started < POLL_TIMEOUT_MS) {
          tries++;
          try {
            const resp = await fetch(JSON_URL, {
              method: "GET",
              mode: "cors",
              headers: { "Accept": "application/json" },
              cache: "no-store"
            });
            if (resp.ok) {
              const payload = await resp.json();
              const m = normalizeMeasurement(extractMeasurementFromPayload(payload));
              if (isValidPayload(m)) {
                data = m;
                console.log(`[GSS][Measure] Got HTTP data after ${tries} poll(s):`, data);
                break;
              }
            }
          } catch {
            // ignore transient errors
          }
          await sleep(POLL_INTERVAL_MS);
        }
      }

      if (!data || !isValidPayload(data)) {
        alert("Timed out waiting for measurement data.");
        return;
      }

      try {
        await ensureManualWeight(data);
      } catch (manualErr) {
        console.warn("[GSS][ManualWeight] Cancelled or failed", manualErr);
        alert("Manual weight entry is required to continue.");
        return;
      }

      autoFillRow(data, targetRowNumber);
    } catch (err) {
      console.error("[GSS][Measure] Failed:", err);
      alert("Could not trigger measurement. Check MeasurePi API and WebSocket connectivity.");
    }
  }

  function autoFillRow(data, targetRowNumber = null) {
    const nextRowNumber = targetRowNumber ?? findNextEmptyRow();
    if (nextRowNumber === -1 || nextRowNumber == null) {
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

  // ---------- Init ----------
  connectWebSocket();
  createFixedFetchButton();      // floating Fetch button
  createMeasureButtonsForRows(); // MEASURE button on each data row
})();
