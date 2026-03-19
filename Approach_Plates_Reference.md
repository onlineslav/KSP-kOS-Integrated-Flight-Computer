# IFC Approach Plates Reference

Source of truth: `Integrated Flight Computer/nav/nav_beacons.ks`.

## ILS Beacon Data

| ILS ID | Runway | Threshold Lat | Threshold Lng | Elevation (m MSL) | Final Heading (deg) | GS Angle (deg) |
|---|---|---:|---:|---:|---:|---:|
| `KSC_ILS_09` | KSC 09 | -0.04877658 | -74.70026463 | 70 | 90.4 | 3.0 |
| `KSC_ILS_27` | KSC 27 | -0.05005861 | -74.51751840 | 70 | 270.4 | 3.0 |
| `ISL_ILS_09` | ISL 09 | -1.517254 | -71.9515 | 134 | 89.0 | 3.0 |
| `ISL_ILS_27` | ISL 27 | -1.516002 | -71.8566 | 134 | 269.0 | 3.0 |
| `DAF_ILS_36` | DAF 36 | -6.5825 | -144.0405556 | 822 | 0.4 | 3.0 |
| `DAF_ILS_18` | DAF 18 | -6.4666667 | -144.0388889 | 822 | 180.4 | 3.0 |

## Fix/Waypoint Beacon Data

Notes:
- All `IAF`/`FAF` fix coordinates are generated from runway threshold + bearing + distance using `GEO_DESTINATION(...)`.
- KSC FAF altitude is computed as `70 + ROUND(8000 * TAN(3.0), 0)` (about `489 m`).
- Island FAF altitude is computed as `134 + ROUND(8000 * TAN(3.0), 0)` (about `553 m`).
- Desert FAF altitude is computed as `822 + ROUND(15000 * TAN(3.0), 0)` (about `1608 m`).

### KSC RWY 09 Fixes

| Fix ID | Type | Definition | Stored Alt (m MSL) |
|---|---|---|---:|
| `KSC_IAF_09_60` | IAF | 60 km from RWY09 threshold on inbound course extension (bearing 270) | 3000 |
| `KSC_IAF_09_30` | IAF | 30 km from RWY09 threshold on inbound course extension (bearing 270) | 1500 |
| `KSC_FAF_09` | FAF | 8 km from RWY09 threshold on inbound course extension (bearing 270) | `ksc09_faf_alt` (about 489) |

### KSC RWY 27 Fixes

| Fix ID | Type | Definition | Stored Alt (m MSL) |
|---|---|---|---:|
| `KSC_IAF_27_60` | IAF | 60 km from RWY27 threshold on inbound course extension (bearing 90) | 3000 |
| `KSC_IAF_27_30` | IAF | 30 km from RWY27 threshold on inbound course extension (bearing 90) | 1500 |
| `KSC_FAF_27` | FAF | 8 km from RWY27 threshold on inbound course extension (bearing 90) | `ksc27_faf_alt` (about 489) |

### Island RWY 09 Fixes

| Fix ID | Type | Definition | Stored Alt (m MSL) |
|---|---|---|---:|
| `ISL_IAF_09_30` | IAF | 30 km from ISL RWY09 threshold on inbound course extension (bearing 270) | 1500 |
| `ISL_FAF_09` | FAF | 8 km from ISL RWY09 threshold on inbound course extension (bearing 270) | `isl_faf_alt` (about 553) |

### Island RWY 27 Fixes

| Fix ID | Type | Definition | Stored Alt (m MSL) |
|---|---|---|---:|
| `ISL_IAF_27_30` | IAF | 30 km from ISL RWY27 threshold on inbound course extension (bearing 90) | 1500 |
| `ISL_FAF_27` | FAF | 8 km from ISL RWY27 threshold on inbound course extension (bearing 90) | `isl_faf_alt` (about 553) |

### Desert RWY 36 Fixes

| Fix ID | Type | Definition | Stored Alt (m MSL) |
|---|---|---|---:|
| `DAF_IAF_36_60` | IAF | 60 km from DAF RWY36 threshold on inbound course extension (bearing 180) | 3000 |
| `DAF_IAF_36_30` | IAF | 30 km from DAF RWY36 threshold on inbound course extension (bearing 180) | 1500 |
| `DAF_FAF_36` | FAF | 15 km from DAF RWY36 threshold on inbound course extension (bearing 180) | `daf_faf_alt` (about 1608) |

### Desert RWY 18 Fixes

| Fix ID | Type | Definition | Stored Alt (m MSL) |
|---|---|---|---:|
| `DAF_IAF_18_60` | IAF | 60 km from DAF RWY18 threshold on inbound course extension (bearing 0) | 3000 |
| `DAF_IAF_18_30` | IAF | 30 km from DAF RWY18 threshold on inbound course extension (bearing 0) | 1500 |
| `DAF_FAF_18` | FAF | 15 km from DAF RWY18 threshold on inbound course extension (bearing 0) | `daf_faf_alt` (about 1608) |

## Plate Definitions

### Plate Registry IDs

| Registry ID | Plate Name |
|---|---|
| `PLATE_KSC_ILS09` | KSC ILS RWY 09 |
| `PLATE_KSC_ILS27` | KSC ILS RWY 27 |
| `PLATE_KSC_ILS09_SHORT` | KSC ILS RWY 09 (short) |
| `PLATE_KSC_ILS27_SHORT` | KSC ILS RWY 27 (short) |
| `PLATE_ISL_ILS09` | ISL ILS RWY IS09 |
| `PLATE_ISL_ILS27` | ISL ILS RWY IS27 |
| `PLATE_DAF_ILS36` | DAF ILS RWY 36 |
| `PLATE_DAF_ILS36_SHORT` | DAF ILS RWY 36 (short) |
| `PLATE_DAF_ILS18` | DAF ILS RWY 18 |
| `PLATE_DAF_ILS18_SHORT` | DAF ILS RWY 18 (short) |

### Full Plate Data

| Plate Var | Name | ILS ID | Vapp (m/s) | Fix Sequence (in order) | `alt_at` Targets (m MSL) |
|---|---|---|---:|---|---|
| `PLATE_KSC_ILS09` | KSC ILS RWY 09 | `KSC_ILS_09` | 75 | `KSC_IAF_09_60 -> KSC_IAF_09_30 -> KSC_FAF_09` | `3000, 1500, ksc09_faf_alt` |
| `PLATE_KSC_ILS27` | KSC ILS RWY 27 | `KSC_ILS_27` | 75 | `KSC_IAF_27_60 -> KSC_IAF_27_30 -> KSC_FAF_27` | `3000, 1500, ksc27_faf_alt` |
| `PLATE_KSC_ILS09_SHORT` | KSC ILS RWY 09 (short) | `KSC_ILS_09` | 75 | `KSC_IAF_09_30 -> KSC_FAF_09` | `1500, ksc09_faf_alt` |
| `PLATE_KSC_ILS27_SHORT` | KSC ILS RWY 27 (short) | `KSC_ILS_27` | 75 | `KSC_IAF_27_30 -> KSC_FAF_27` | `1500, ksc27_faf_alt` |
| `PLATE_ISL_ILS09` | ISL ILS RWY IS09 | `ISL_ILS_09` | 70 | `ISL_IAF_09_30 -> ISL_FAF_09` | `1500, isl_faf_alt` |
| `PLATE_ISL_ILS27` | ISL ILS RWY IS27 | `ISL_ILS_27` | 70 | `ISL_IAF_27_30 -> ISL_FAF_27` | `1500, isl_faf_alt` |
| `PLATE_DAF_ILS36` | DAF ILS RWY 36 | `DAF_ILS_36` | 70 | `DAF_IAF_36_60 -> DAF_IAF_36_30 -> DAF_FAF_36` | `3000, 1500, daf_faf_alt` |
| `PLATE_DAF_ILS36_SHORT` | DAF ILS RWY 36 (short) | `DAF_ILS_36` | 70 | `DAF_IAF_36_30 -> DAF_FAF_36` | `1500, daf_faf_alt` |
| `PLATE_DAF_ILS18` | DAF ILS RWY 18 | `DAF_ILS_18` | 70 | `DAF_IAF_18_60 -> DAF_IAF_18_30 -> DAF_FAF_18` | `3000, 1500, daf_faf_alt` |
| `PLATE_DAF_ILS18_SHORT` | DAF ILS RWY 18 (short) | `DAF_ILS_18` | 70 | `DAF_IAF_18_30 -> DAF_FAF_18` | `1500, daf_faf_alt` |

## Runtime Selection Logic

- `GET_PLATE_FOR_RUNWAY("09", FALSE)` -> `PLATE_KSC_ILS09`
- `GET_PLATE_FOR_RUNWAY("09", TRUE)` -> `PLATE_KSC_ILS09_SHORT`
- `GET_PLATE_FOR_RUNWAY("27", FALSE)` -> `PLATE_KSC_ILS27`
- `GET_PLATE_FOR_RUNWAY("27", TRUE)` -> `PLATE_KSC_ILS27_SHORT`
- `GET_PLATE_FOR_RUNWAY("36", FALSE)` -> `PLATE_DAF_ILS36`
- `GET_PLATE_FOR_RUNWAY("36", TRUE)` -> `PLATE_DAF_ILS36_SHORT`
- `GET_PLATE_FOR_RUNWAY("18", FALSE)` -> `PLATE_DAF_ILS18`
- `GET_PLATE_FOR_RUNWAY("18", TRUE)` -> `PLATE_DAF_ILS18_SHORT`
