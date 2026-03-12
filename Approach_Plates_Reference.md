# IFC Approach Plates Reference

Source of truth: `Integrated Flight Computer/nav/nav_beacons.ks`.

## ILS Beacon Data

| ILS ID | Runway | Threshold Lat | Threshold Lng | Elevation (m MSL) | Final Heading (deg) | GS Angle (deg) |
|---|---|---:|---:|---:|---:|---:|
| `KSC_ILS_09` | KSC 09 | -0.04877658 | -74.70026463 | 70 | 90.4 | 3.0 |
| `KSC_ILS_27` | KSC 27 | -0.05005861 | -74.51751840 | 70 | 270.4 | 3.0 |
| `ISL_ILS_09` | ISL 09 | -1.517254 | -71.9515 | 134 | 89.0 | 3.0 |
| `ISL_ILS_27` | ISL 27 | -1.516002 | -71.8566 | 134 | 269.0 | 3.0 |

## Fix/Waypoint Beacon Data

Notes:
- All `IAF`/`FAF` fix coordinates are generated from runway threshold + bearing + distance using `GEO_DESTINATION(...)`.
- KSC FAF altitude is computed as `70 + ROUND(8000 * TAN(3.0), 0)` (about `489 m`).
- Island FAF altitude is computed as `134 + ROUND(8000 * TAN(3.0), 0)` (about `553 m`).

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

### Full Plate Data

| Plate Var | Name | ILS ID | Vapp (m/s) | Fix Sequence (in order) | `alt_at` Targets (m MSL) |
|---|---|---|---:|---|---|
| `PLATE_KSC_ILS09` | KSC ILS RWY 09 | `KSC_ILS_09` | 75 | `KSC_IAF_09_60 -> KSC_IAF_09_30 -> KSC_FAF_09` | `3000, 1500, ksc09_faf_alt` |
| `PLATE_KSC_ILS27` | KSC ILS RWY 27 | `KSC_ILS_27` | 75 | `KSC_IAF_27_60 -> KSC_IAF_27_30 -> KSC_FAF_27` | `3000, 1500, ksc27_faf_alt` |
| `PLATE_KSC_ILS09_SHORT` | KSC ILS RWY 09 (short) | `KSC_ILS_09` | 75 | `KSC_IAF_09_30 -> KSC_FAF_09` | `1500, ksc09_faf_alt` |
| `PLATE_KSC_ILS27_SHORT` | KSC ILS RWY 27 (short) | `KSC_ILS_27` | 75 | `KSC_IAF_27_30 -> KSC_FAF_27` | `1500, ksc27_faf_alt` |
| `PLATE_ISL_ILS09` | ISL ILS RWY IS09 | `ISL_ILS_09` | 70 | `ISL_IAF_09_30 -> ISL_FAF_09` | `1500, isl_faf_alt` |
| `PLATE_ISL_ILS27` | ISL ILS RWY IS27 | `ISL_ILS_27` | 70 | `ISL_IAF_27_30 -> ISL_FAF_27` | `1500, isl_faf_alt` |

## Runtime Selection Logic

- `GET_PLATE_FOR_RUNWAY("09", FALSE)` -> `PLATE_KSC_ILS09`
- `GET_PLATE_FOR_RUNWAY("09", TRUE)` -> `PLATE_KSC_ILS09_SHORT`
- `GET_PLATE_FOR_RUNWAY("27", FALSE)` -> `PLATE_KSC_ILS27`
- `GET_PLATE_FOR_RUNWAY("27", TRUE)` -> `PLATE_KSC_ILS27_SHORT`

