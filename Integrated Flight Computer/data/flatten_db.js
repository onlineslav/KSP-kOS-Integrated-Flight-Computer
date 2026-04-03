// flatten_db.js
// Flattens multimode engine entries into sibling top-level keys.
//
// Input:  engine_database.json  (modes nested under each engine)
// Output: engine_database.json  (overwritten with flat structure)
//
// For each multimode engine "foo" with modes {"Dry": {...}, "Wet": {...}}:
//   - Parent "foo" keeps: displayName, defaultDiameter, multimode:1,
//     modeKeys:[...], plus maxThrust/vel_at_zero/atm_at_one copied from
//     the first mode (for thrust-ratio scale inference at init).
//   - New siblings: "foo__Dry" and "foo__Wet" with all mode data.
//
// Run: node flatten_db.js   (from the data/ directory, or adjust paths)

const fs   = require("fs");
const path = require("path");

const dbPath = path.join(__dirname, "engine_database.json");
const db     = JSON.parse(fs.readFileSync(dbPath, "utf8"));

const engines    = db.engines;
const flatEngines = {};

for (const [partName, entry] of Object.entries(engines)) {
    if (!entry.multimode || entry.modes === 0) {
        // Single-mode: copy as-is, drop the sentinel modes:0 field
        const { modes, ...rest } = entry;
        flatEngines[partName] = rest;
        continue;
    }

    // Multimode: entry.modes is an object { "Dry": {...}, "Wet": {...} }
    const modeKeys = Object.keys(entry.modes);
    const firstMode = entry.modes[modeKeys[0]];

    // Build the parent stub (no curves, no propellant ratios)
    flatEngines[partName] = {
        displayName:     entry.displayName,
        defaultDiameter: entry.defaultDiameter,
        multimode:       1,
        modeKeys:        modeKeys,
        // Copy primary-mode scale-inference fields so _EM_DETECT_SCALE works
        // against the top-level entry at EM_INIT time.
        maxThrust:       firstMode.maxThrust,
        vel_at_zero:     firstMode.vel_at_zero,
        atm_at_one:      firstMode.atm_at_one
    };

    // Flatten each mode into a sibling entry
    for (const [modeName, modeData] of Object.entries(entry.modes)) {
        flatEngines[`${partName}__${modeName}`] = modeData;
    }
}

const out = {
    engines: flatEngines,
    intakes: db.intakes
};

const json = JSON.stringify(out, null, 2);
fs.writeFileSync(dbPath, json, "utf8");
console.log(`Done. Wrote ${json.length} bytes to ${dbPath}`);

// Report engine keys written
const keys = Object.keys(out.engines);
console.log(`Engine entries: ${keys.length}`);
keys.forEach(k => console.log("  " + k));
