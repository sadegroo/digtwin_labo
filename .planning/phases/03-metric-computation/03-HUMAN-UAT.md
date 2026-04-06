---
status: partial
phase: 03-metric-computation
source: [03-VERIFICATION.md]
started: 2026-04-06T10:08:00Z
updated: 2026-04-06T10:08:00Z
---

## Current Test

[awaiting human testing]

## Tests

### 1. End-to-end metric accuracy with real data
expected: Run the scoring script against a real .mldatx file and confirm the diagnostic line prints correctly — "Metrics: swingup=YES t=X.XXs SMAPE=Y.Y% participation=YES" — with values consistent with manual inspection of the overlay plot
result: [pending]

### 2. SMAPE window mode selectability (ROADMAP SC5)
expected: Confirm whether the D-09 hybrid policy (max(5s, time_to_first_90deg)) satisfies the "configurable windows" goal, or whether explicit 'fixed'/'angle'/'swingup' switching is needed. CONTEXT.md D-09 locked this decision before implementation.
result: [pending]

## Summary

total: 2
passed: 0
issues: 0
pending: 2
skipped: 0
blocked: 0

## Gaps
