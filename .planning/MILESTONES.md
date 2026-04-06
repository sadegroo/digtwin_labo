# Milestones

## v1.0 Swingup Competition Scoring Script (Shipped: 2026-04-06)

**Phases completed:** 5 phases, 8 plans, 8 tasks

**Key accomplishments:**

- Interactive file-by-file `.mldatx` loading with team assignment, SimMode run discrimination, and session state accumulation
- Keyword-scored `listdlg` signal selection, preview plots, and `interp1`-based time alignment to t=0 at first non-zero command
- Conv-based sustained-hold swingup detection, angular SMAPE with wrapping guards, and D-09 hybrid window policy
- Dense-ranking stepper + absolute-band BLDC scoring with per-team diagnostics and split leaderboard display
- CSV/xlsx export with timestamped filenames and grouped bar chart score breakdown
- Single-file architecture: 1,379 lines, 25 local functions in `scripts/score_competition.m`

**Known gaps:**

- METR-05: SMAPE window configurability superseded by D-09 hybrid policy — awaiting instructor confirmation

**Stats:** 5 days (2026-04-02 to 2026-04-06), 65 commits

---
