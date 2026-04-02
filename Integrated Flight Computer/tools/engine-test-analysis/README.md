# Engine Test Analysis

This tool processes logs created by:

- `0:/Integrated Flight Computer/tests/engine_test.ks`

It extracts spool-up/spool-down step metrics and builds a basic feed-forward model.

## Usage

Run against the latest log in `Integrated Flight Computer/engine test logs`:

```powershell
python "Integrated Flight Computer/tools/engine-test-analysis/analyze_engine_test.py"
```

Run against a specific log:

```powershell
python "Integrated Flight Computer/tools/engine-test-analysis/analyze_engine_test.py" `
  --log "Integrated Flight Computer/engine test logs/engine_test_log_00000001.csv"
```

Also print JSON model to terminal:

```powershell
python "Integrated Flight Computer/tools/engine-test-analysis/analyze_engine_test.py" --print-json
```

## Outputs

Given `engine_test_log_00000001.csv`, the script writes:

- `engine_test_log_00000001_model.json`
- `engine_test_log_00000001_summary.txt`

## Model form

The generated model is:

- `thrust_ss(u)` from a fitted steady-state map (quadratic if enough points, else linear)
- directional time constants from median step response times:
  - `tau_up` (for increasing thrust)
  - `tau_down` (for decreasing thrust)

Use:

`thrust_dot = (thrust_ss(u) - thrust_now) / tau_dir`

where `tau_dir` picks `tau_up` or `tau_down` based on whether the commanded steady-state thrust is above or below current thrust.
