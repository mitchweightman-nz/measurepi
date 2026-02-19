# Project Agent Guide

This guide applies to the entire `measurepi` repository unless a more specific `AGENTS.md` appears in a subdirectory.

## Expectations

- Keep changes focused and clearly scoped.
- Use concise commit messages that summarize what changed and why.
- Prefer clear, self-documenting code; add brief comments when logic is non-obvious.
- Do **not** wrap import statements in `try/except` blocks.

## Repository Layout

- `uno_q_app/sketch/`: Arduino UNO Q microcontroller sketch (`sketch.ino`) and sketch metadata.
- `uno_q_app/python/`: Linux-side services (`bridge.py`, `mqtt.py`, `dashboard.py`) and Flask template files.
- `uno_q_app/app.yaml`: Arduino App manifest and default environment values.
- `legacy/`: historical Raspberry Pi and UNO R4 assets kept for reference only.

## Python and JavaScript Style

- Favor consistent formatting (Black-compatible Python, Prettier-friendly JavaScript).
- Use descriptive names for variables and functions.
- Keep functions small and handle edge cases explicitly.
- Validate user or external input before use.

## Testing

- Run relevant tests or linters when modifying executable code.
- For documentation-only updates, tests are optional.
- Record the commands and results in the change summary.
- If no automated tests exist, perform a quick manual check and describe what you verified.

## Documentation

- Update README when behavior, defaults, ports, or environment variable names change.
- Keep UNO Q deployment instructions aligned with `uno_q_app/app.yaml` and Python runtime behavior.
