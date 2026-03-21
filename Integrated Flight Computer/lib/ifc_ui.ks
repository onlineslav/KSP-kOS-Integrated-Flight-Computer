@LAZYGLOBAL OFF.

// ============================================================
// ifc_ui.ks  -  Integrated Flight Computer
// Low-level screen engine. ASCII-only to avoid terminal charset issues.
// ============================================================

// Pre-built space strings " ", "  ", ... up to UI_W.
// Populated in UI_INIT. Index k holds a string of k spaces.
GLOBAL _UI_SPACES IS LIST("").

FUNCTION STR_REPEAT {
  PARAMETER c, n.
  // Fast path: spaces use the pre-built lookup table (P6a).
  IF c = " " AND n >= 0 AND n < _UI_SPACES:LENGTH {
    RETURN _UI_SPACES[n].
  }
  LOCAL s IS "".
  LOCAL i IS 0.
  UNTIL i >= n {
    SET s TO s + c.
    SET i TO i + 1.
  }
  RETURN s.
}

FUNCTION STR_PAD {
  PARAMETER s, w.
  UNTIL s:LENGTH >= w { SET s TO s + " ". }
  IF s:LENGTH > w { RETURN s:SUBSTRING(0, w). }
  RETURN s.
}

FUNCTION STR_RJUST {
  PARAMETER s, w.
  UNTIL s:LENGTH >= w { SET s TO " " + s. }
  IF s:LENGTH > w { RETURN s:SUBSTRING(s:LENGTH - w, w). }
  RETURN s.
}

FUNCTION UI_FMT {
  PARAMETER val, dec.
  RETURN "" + ROUND(val, dec).
}

FUNCTION UI_FORMAT_TIME {
  PARAMETER secs.
  LOCAL s IS MAX(ROUND(secs, 0), 0).
  LOCAL m IS FLOOR(s / 60).
  LOCAL sc IS MOD(s, 60).
  LOCAL mm IS STR_RJUST("" + m,  2):REPLACE(" ", "0").
  LOCAL ss IS STR_RJUST("" + sc, 2):REPLACE(" ", "0").
  RETURN mm + ":" + ss.
}

FUNCTION UI_FORMAT_FLP {
  PARAMETER detent.
  LOCAL names IS LIST("UP", "CLMB", "APPR", "LAND").
  LOCAL bar IS "".
  LOCAL i IS 0.
  UNTIL i >= detent {
    SET bar TO bar + "#".
    SET i TO i + 1.
  }
  LOCAL name IS "DET" + detent.
  IF detent < names:LENGTH { SET name TO names[detent]. }
  RETURN bar + " " + name.
}

FUNCTION UI_P {
  PARAMETER text, row.
  PRINT STR_PAD(text, UI_W) AT(0, row).
}

FUNCTION UI_SEP {
  PARAMETER row.
  PRINT STR_REPEAT("-", UI_W) AT(0, row).
}

FUNCTION UI_THICK_SEP {
  PARAMETER row.
  PRINT STR_REPEAT("=", UI_W) AT(0, row).
}

FUNCTION UI_CLR {
  PARAMETER row.
  PRINT STR_REPEAT(" ", UI_W) AT(0, row).
}

FUNCTION UI_BAR_FILL {
  PARAMETER frac, w, fill_char, empty_char.
  LOCAL pos IS ROUND(CLAMP(frac, 0, 1) * w).
  LOCAL s IS "".
  LOCAL i IS 0.
  UNTIL i >= w {
    IF i < pos { SET s TO s + fill_char. }
    ELSE       { SET s TO s + empty_char. }
    SET i TO i + 1.
  }
  RETURN s.
}

FUNCTION UI_BAR_DEV {
  PARAMETER dev, max_dev, w.
  LOCAL ctr IS FLOOR(w / 2).
  LOCAL frac IS CLAMP(dev / MAX(ABS(max_dev), 0.001), -1, 1).
  LOCAL marker IS CLAMP(ROUND(ctr + frac * ctr), 0, w - 1).
  LOCAL s IS "<".
  LOCAL i IS 0.
  UNTIL i >= w {
    IF      i = marker AND i = ctr { SET s TO s + "|". }
    ELSE IF i = marker             { SET s TO s + "*". }
    ELSE IF i = ctr                { SET s TO s + "|". }
    ELSE                           { SET s TO s + "-". }
    SET i TO i + 1.
  }
  RETURN s + ">".
}

FUNCTION UI_INIT {
  SET UI_W TO MAX(TERMINAL:WIDTH, 42).
  LOCAL term_h IS MAX(TERMINAL:HEIGHT, 22).

  // Build space-string lookup table up to terminal width (P6a).
  _UI_SPACES:CLEAR().
  LOCAL _sp IS "".
  LOCAL _si IS 0.
  UNTIL _si > UI_W {
    _UI_SPACES:ADD(_sp).
    SET _sp TO _sp + " ".
    SET _si TO _si + 1.
  }

  // Dynamic row layout: scale to terminal height while keeping
  // the original 22-row geometry as the minimum.
  SET UI_HDR_ROW   TO 1.
  SET UI_CRUMB_ROW TO 3.
  SET UI_PRI_TOP   TO 5.
  SET UI_PRI_BOT   TO term_h - 12.
  SET UI_SEC_TOP   TO term_h - 10.
  SET UI_SEC_BOT   TO term_h - 8.
  SET UI_ALERT_ROW TO term_h - 6.
  SET UI_LOG_ROW   TO term_h - 4.
  SET UI_HINT_ROW  TO term_h - 2.

  CLEARSCREEN.
  UI_THICK_SEP(0).
  UI_THICK_SEP(2).
  UI_SEP(4).
  UI_SEP(UI_PRI_BOT + 1).
  UI_SEP(UI_SEC_BOT + 1).
  UI_SEP(UI_ALERT_ROW + 1).
  UI_SEP(UI_LOG_ROW + 1).
  UI_THICK_SEP(UI_HINT_ROW + 1).
}
