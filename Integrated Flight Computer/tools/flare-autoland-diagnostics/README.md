# Flare and Autoland Diagnostics Notebook

This Jupyter notebook helps visualize and diagnose autoland and flare performance from Integrated Flight Computer log files.

## How to Use

1.  **Open the notebook:**
    Run `jupyter notebook` in your terminal from the project root, and then navigate to `Integrated Flight Computer/tools/flare-autoland-diagnostics/` and open `flare_autoland_diagnostics.ipynb`.

2.  **Set the log file path:**
    In the second code cell, change the `log_file_path` variable to the absolute path of the log file you want to analyze.

    ```python
    log_file_path = r'f:\Kerbal Space Program\Ships\Script\Integrated Flight Computer\logs\ifc_log_00000127.csv'
    ```

3.  **Run the cells:**
    Run all the cells in the notebook to generate the plots.

## Plots

The notebook generates the following plots:

*   **Flight Profile:** Altitude, Vertical Speed, and Airspeed vs. Time.
*   **Pitch, AoA and Throttle:** Pitch, Angle of Attack, and Throttle commands vs. Time.
*   **ILS Tracking Performance:** Localizer and Glideslope deviation vs. Time.
*   **Flare Details:** A detailed view of the flare phase, including FPA commands, target vertical speed, and flare fraction.

Phase changes are marked with vertical dashed lines on the plots.
