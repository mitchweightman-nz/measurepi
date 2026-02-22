# Standard Operating Procedure: Measurement Rounding for Height, Width, and Length

## Purpose
Ensure dimensional measurements (height, width, length) are rounded consistently before being surfaced in API responses and LCD output.

## Applicable Systems
- MeasurePi Flask service (`MeasurePi.py`).
- Incoming MQTT payloads on the `measure/data` topic containing `height`, `width`, and `length` fields.

## Rounding Rules
- Default rules: `ceil` for **height**, **width**, and **length** (`DEFAULT_ROUNDING_SETTINGS`).
- Permitted rule values per dimension:
  - `"ceil"` → round up using `math.ceil`.
  - `"floor"` → round down using `math.floor`.
  - `"none"` → no rounding; use the raw float value.
  - A digit string `"0"`–`"10"` → round to that many decimal places using Python `round`.
- Safety fallback: if a rule is invalid or processing fails, the value is rounded to **1 decimal place** and a log entry is printed.
- Only `height`, `width`, and `length` are rounded by this procedure; other fields (e.g., weight) have their own rounding rules.

## Procedure
1. **Check current rules**
   - Send `GET /api/settings` and confirm the `rounding_settings` values for each dimension.

2. **Update rounding rules (if needed)**
   - Send `POST /api/settings` with JSON containing any of `height`, `width`, `length` using one of the permitted rule values.
   - Example payloads:
     - `{ "height": "ceil", "width": "1", "length": "floor" }`
     - `{ "height": "none" }` (leave other dimensions unchanged)
   - Valid keys outside `height`, `width`, `length` are ignored; invalid values are rejected with a log message.

3. **Apply rounding during measurement handling**
   - Incoming MQTT measurements are stored, then `_apply_rounding` runs when measurements are prepared for JSON responses and LCD display.
   - Rounding is applied per the current `rounding_settings` for each dimension.

4. **Verify results**
   - Call `GET /json` and confirm the `current` payload reflects the expected rounded values for `height`, `width`, and `length`.
   - If an LCD is connected, verify the screen shows the rounded values used in the `H:`, `W:`, and `L:` fields.

5. **Error handling**
   - If rounding raises an error (e.g., non-numeric input), the value is rounded to 1 decimal place and a warning is logged.
   - Ensure MQTT payloads provide numeric values for `height`, `width`, and `length` to avoid fallback handling.

## References
- Rounding logic: `_apply_rounding` and `rounding_settings` in `MeasurePi.py`.
- Settings API: `/api/settings` handler in `MeasurePi.py`.
- Measurement presentation: `/json` response composition and LCD text generation in `MeasurePi.py`.
