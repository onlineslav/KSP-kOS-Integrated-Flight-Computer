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
