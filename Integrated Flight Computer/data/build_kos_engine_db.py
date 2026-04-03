"""Build a kOS READJSON-compatible engine database from plain JSON.

Input:  engine_database.json (plain JSON for tooling/editing)
Output: engine_database_kos.json (kOS typed JSON for READJSON)
"""

from __future__ import annotations

import json
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
SRC = THIS_DIR / "engine_database.json"
DST = THIS_DIR / "engine_database_kos.json"

T_STRING = "kOS.Safe.Encapsulation.StringValue"
T_INT = "kOS.Safe.Encapsulation.ScalarIntValue"
T_DOUBLE = "kOS.Safe.Encapsulation.ScalarDoubleValue"
T_LIST = "kOS.Safe.Encapsulation.ListValue"
T_LEXICON = "kOS.Safe.Encapsulation.Lexicon"


def to_kos_typed(value):
    if isinstance(value, dict):
        entries = []
        for key, val in value.items():
            entries.append(to_kos_typed(str(key)))
            entries.append(to_kos_typed(val))
        return {"entries": entries, "$type": T_LEXICON}

    if isinstance(value, list):
        return {"items": [to_kos_typed(v) for v in value], "$type": T_LIST}

    if isinstance(value, str):
        return {"value": value, "$type": T_STRING}

    if isinstance(value, bool):
        # This DB currently uses numeric flags, but convert bools safely if introduced.
        return {"value": 1 if value else 0, "$type": T_INT}

    if isinstance(value, int):
        return {"value": value, "$type": T_INT}

    if isinstance(value, float):
        return {"value": value, "$type": T_DOUBLE}

    if value is None:
        raise TypeError("Null values are not supported for kOS DB serialization.")

    raise TypeError(f"Unsupported value type: {type(value)!r}")


def main() -> None:
    raw = json.loads(SRC.read_text(encoding="utf-8"))
    typed = to_kos_typed(raw)
    DST.write_text(json.dumps(typed, indent=2), encoding="utf-8")
    print(f"Wrote {DST} ({DST.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
