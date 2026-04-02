# Engine Test README

This guide covers how to run the engine characterization workflow and analyze results with:

- Python CLI script
- Jupyter notebook

---

## 1) Run an Engine Test in kOS

### Option A: Bootloader (recommended)

Set your kOS boot file to:

`boot/engine_test_boot.ks`

That bootloader will:

- open the kOS terminal automatically
- run `0:/Integrated Flight Computer/tests/engine_test.ks`

### Option B: Manual run from kOS terminal

```ks
RUNPATH("0:/Integrated Flight Computer/tests/engine_test.ks").
```

Logs are written to:

`0:/Integrated Flight Computer/engine test logs/engine_test_log_XXXXXXXX.csv`

Metadata now also records detected TweakScale values for the engine and intake parts
when a TweakScale module is present.

### Manual geometry metadata (recommended for scale studies)

If TweakScale values resolve as `-1`, set these constants near the top of
`Integrated Flight Computer/tests/engine_test.ks` before each run:

- `ET_ENGINE_DIAMETER_M_MANUAL`
- `ET_INTAKE_DIAMETER_M_MANUAL`
- `ET_SCALE_FACTOR_MANUAL`

These are logged in CSV metadata as:

- `engine_diameter_m_manual`
- `intake_diameter_m_manual`
- `scale_factor_manual`

### Phase progression behavior

The test script no longer advances phases on fixed per-phase timers.

Each phase now:

1. runs for at least `ET_PHASE_MIN_HOLD_S` (default 10 s), then
2. waits until steady-state checks pass for:
   - thrust
   - fuel flow
   - prop requirement met (when that module field is available)
3. keeps those checks stable for `ET_STEADY_HOLD_S` (default 4 s) before advancing

There is also a safety timeout (`ET_PHASE_MAX_WAIT_S`) to prevent endless waiting.

---

## 2) Analyze with Python (CLI)

From this repo root (`Ships/Script`), run:

```powershell
python "Integrated Flight Computer/tools/engine-test-analysis/analyze_engine_test.py"
```

To analyze a specific log:

```powershell
python "Integrated Flight Computer/tools/engine-test-analysis/analyze_engine_test.py" `
  --log "Integrated Flight Computer/engine test logs/engine_test_log_00000003.csv"
```

Outputs (next to the source CSV):

- `*_summary.txt`
- `*_model.json`

---

## 2b) Build a TweakScale-Agnostic Model (multi-run)

Use the multi-run fitter with explicit diameters:

```powershell
python "Integrated Flight Computer/tools/engine-test-analysis/build_scale_agnostic_model.py" `
  --run "Integrated Flight Computer/engine test logs/engine_test_log_00000001.csv" 1.25 1.25 `
  --run "Integrated Flight Computer/engine test logs/engine_test_log_00000002.csv" 0.75 0.75
```

Output:

- `Integrated Flight Computer/engine test logs/engine_scale_agnostic_model.json`

Model form:

- steady thrust map at reference size: `T_ref(u)`
- scale law: `T(u, D) = T_ref(u) * (D / D_ref)^alpha_thrust`
- spool laws: `tau_up/down(D) = tau_ref_up/down * (D / D_ref)^beta_up/down`

---

## 3) Analyze with Jupyter Notebook

Notebook file:

`Integrated Flight Computer/tools/engine-test-analysis/analyze_engine_test.ipynb`

### Start Jupyter

From repo root:

```powershell
jupyter lab
```

or:

```powershell
jupyter notebook
```

Open:

`Integrated Flight Computer/tools/engine-test-analysis/analyze_engine_test.ipynb`

### In the notebook

1. Set `LOG_PATH` to your target CSV (or enable auto-pick latest).
2. Run all cells.
3. Review:
   - key telemetry plots
   - model vs measured thrust plot
   - residual plot
   - steady-state map plot
   - normalized step-response overlays
4. Check the **Steady-state reach check per phase** table.
5. Use the **Multi-Run Pipeline (Single Notebook Entry Point)** section at the end to process multiple logs and build a scale-agnostic model in one run.
   Only edit `PIPELINE_RUNS` and execute that cell.

---

## 4) Steady-State Validation

The notebook includes a per-phase steady-state check using tail-window metrics:

- thrust tail slope
- thrust tail standard deviation
- fuel-flow tail slope

Control flags in the notebook:

- `ENFORCE_STEADY_STATE = False` (set `True` to hard-fail if any phase is non-steady)
- `STEADY_*` threshold constants (tune as needed)

---

## 5) Python/Jupyter Dependencies

Minimum Python packages for analysis and plotting:

- `numpy`
- `pandas`
- `matplotlib`

For notebook execution:

- `jupyter` (or `notebook`)

Example install:

```powershell
python -m pip install numpy pandas matplotlib jupyter
```

---

## 6) Common Issues

### `matplotlib` import error in notebook

Install matplotlib, then restart kernel:

```powershell
python -m pip install matplotlib
```

### Log shows non-steady phases

- increase phase hold durations in `Integrated Flight Computer/tests/engine_test.ks`
- rerun and re-check the steady-state table

### Missing module-derived channels (`-1` values)

That means the module field name was not resolved for this engine/intake module. Core vessel channels are still usable; module field probing can be expanded per part/module.

For scale experiments, this is why manual geometry metadata fields were added.
