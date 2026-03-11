@LAZYGLOBAL OFF.

// ============================================================
// ifc_ui.ks  -  Integrated Flight Computer
// Low-level screen engine.  All display code calls these
// primitives; nothing else writes PRINT AT() directly.
// ============================================================

// ----------------------------
// String primitives
// ----------------------------

// Repeat character c exactly n times.
FUNCTION STR_REPEAT {
  PARAMETER c, n.
  LOCAL s IS "".
  LOCAL i IS 0.
  UNTIL i >= n { SET s TO s + c. SET i TO i + 1. }
  RETURN s.
}

// Pad or truncate s to exactly w characters (right-pad with spaces).
FUNCTION STR_PAD {
  PARAMETER s, w.
  UNTIL s:LENGTH >= w { SET s TO s + " ". }
  IF s:LENGTH > w { RETURN s:SUBSTRING(0, w). }
  RETURN s.
}

// Right-justify s in a field of width w (left-pad with spaces).
FUNCTION STR_RJUST {
  PARAMETER s, w.
  UNTIL s:LENGTH >= w { SET s TO " " + s. }
  IF s:LENGTH > w { RETURN s:SUBSTRING(s:LENGTH - w, w). }
  RETURN s.
}

// Format a number with dec decimal places.
FUNCTION UI_FMT {
  PARAMETER val, dec.
  RETURN "" + ROUND(val, dec).
}

// Format seconds as "MM:SS".
FUNCTION UI_FORMAT_TIME {
  PARAMETER secs.
  LOCAL s IS MAX(ROUND(secs, 0), 0).
  LOCAL m IS FLOOR(s / 60).
  LOCAL sc IS MOD(s, 60).
  LOCAL mm IS STR_RJUST("" + m,  2):REPLACE(" ", "0").
  LOCAL ss IS STR_RJUST("" + sc, 2):REPLACE(" ", "0").
  RETURN mm + ":" + ss.
}

// Format flap detent as "▐▐ APPR" (bar + label).
FUNCTION UI_FORMAT_FLP {
  PARAMETER detent.
  LOCAL names IS LIST("UP", "CLMB", "APPR", "LAND").
  LOCAL bar IS "".
  LOCAL i IS 0.
  UNTIL i >= detent { SET bar TO bar + "▐". SET i TO i + 1. }
  LOCAL name IS "det" + detent.
  IF detent < names:LENGTH { SET name TO names[detent]. }
  RETURN bar + " " + name.
}

// ----------------------------
// Screen write
// ----------------------------

// Print text at row, padded/truncated to UI_W chars.
FUNCTION UI_P {
  PARAMETER text, row.
  PRINT STR_PAD(text, UI_W) AT(0, row).
}

// Print a thin separator (─) across full width.
FUNCTION UI_SEP {
  PARAMETER row.
  PRINT STR_REPEAT("─", UI_W) AT(0, row).
}

// Print a thick separator (═) across full width.
FUNCTION UI_THICK_SEP {
  PARAMETER row.
  PRINT STR_REPEAT("═", UI_W) AT(0, row).
}

// Clear a single row (fill with spaces).
FUNCTION UI_CLR {
  PARAMETER row.
  PRINT STR_REPEAT(" ", UI_W) AT(0, row).
}

// ----------------------------
// Bar renderers
// ----------------------------

// Fill bar: left portion filled, right portion empty.
// frac: 0..1. Returns a string of exactly w chars.
FUNCTION UI_BAR_FILL {
  PARAMETER frac, w, fill_char, empty_char.
  LOCAL pos IS ROUND(CLAMP(frac, 0, 1) * w).
  LOCAL s IS "".
  LOCAL i IS 0.
  UNTIL i >= w {
    IF i < pos { SET s TO s + fill_char. }
    ELSE        { SET s TO s + empty_char. }
    SET i TO i + 1.
  }
  RETURN s.
}

// Center-anchored deviation bar with end caps.
// dev: deflection value.  max_dev: value at full deflection.
// w:   inner width (not counting ◄ ► end caps).
// Returns a string of exactly w+2 chars.
FUNCTION UI_BAR_DEV {
  PARAMETER dev, max_dev, w.
  LOCAL ctr IS FLOOR(w / 2).
  LOCAL frac IS CLAMP(dev / MAX(ABS(max_dev), 0.001), -1, 1).
  LOCAL marker IS CLAMP(ROUND(ctr + frac * ctr), 0, w - 1).
  LOCAL s IS "◄".
  LOCAL i IS 0.
  UNTIL i >= w {
    IF      i = marker AND i = ctr { SET s TO s + "╋". }
    ELSE IF i = marker             { SET s TO s + "█". }
    ELSE IF i = ctr                { SET s TO s + "╋". }
    ELSE                           { SET s TO s + "━". }
    SET i TO i + 1.
  }
  RETURN s + "►".
}

// ----------------------------
// Init
// ----------------------------

// Call once at startup.  Reads terminal width, clears screen,
// draws all static chrome separators.
FUNCTION UI_INIT {
  SET UI_W TO MAX(TERMINAL:WIDTH, 42).
  CLEARSCREEN.
  UI_THICK_SEP(0).
  // row 1: header (written by DISPLAY_HEADER)
  UI_THICK_SEP(2).
  // row 3: breadcrumb (written by DISPLAY_BREADCRUMB)
  UI_SEP(4).
  // rows 5-10: primary zone
  UI_SEP(11).
  // rows 12-14: secondary/debug zone
  UI_SEP(15).
  // row 16: alert bar
  UI_SEP(17).
  // row 18: logger status
  UI_SEP(19).
  // row 20: key hints
  UI_THICK_SEP(21).
}
