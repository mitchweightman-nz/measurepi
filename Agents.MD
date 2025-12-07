# Project Agent Guide

This file provides guidance for collaborators working within the `measurepi` repository. These instructions apply to the entire project directory unless another `AGENTS.md` is added in a subdirectory.

## Expectations
- Keep changes focused and clearly scoped. Include concise commit messages that summarize what changed and why.
- Prefer clear, self-documenting code over clever shortcuts. Add short comments when logic may be non-obvious.
- Avoid wrapping import statements in `try/except` blocks.

## Python and JavaScript style
- Favor consistent formatting; Black-compatible Python and Prettier-friendly JavaScript are preferred defaults.
- Use descriptive names for variables and functions. Keep functions small when possible.
- Validate user or external inputs before use and handle edge cases explicitly.

## Testing
- Run any relevant tests or linters when you touch executable code. Note the commands and results in your change summary.
- If no automated tests exist, perform a quick manual check and describe what you verified.

## Documentation
- Update comments or README-style files when behavior or usage changes.
- Keep instructions up to date so future contributors can follow along easily.
