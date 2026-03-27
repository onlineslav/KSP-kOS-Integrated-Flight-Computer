@LAZYGLOBAL OFF.


// Multi-CPU guard:
// Only one processor should own terminal/UI startup to avoid state thrash.
// Tune PRIMARY_CORE_HINTS per craft as needed.
// Any matching token will designate this CPU as primary.
LOCAL PRIMARY_CORE_HINTS IS LIST("OPT0", "COCKPIT").
LOCAL core_name_raw IS ("" + CORE:PART):TOUPPER.
LOCAL core_name IS core_name_raw:REPLACE(" ", ""):REPLACE("-", ""):REPLACE("_", "").
LOCAL is_primary_core IS FALSE.
LOCAL hint_i IS 0.
UNTIL hint_i >= PRIMARY_CORE_HINTS:LENGTH OR is_primary_core {
  LOCAL hint_name IS ("" + PRIMARY_CORE_HINTS[hint_i]):TOUPPER:REPLACE(" ", ""):REPLACE("-", ""):REPLACE("_", "").
  IF hint_name = "" OR core_name:FIND(hint_name) >= 0 {
    SET is_primary_core TO TRUE.
  }
  SET hint_i TO hint_i + 1.
}

IF is_primary_core {
  core:part:getmodule("kOSProcessor"):doevent("Open Terminal").

  SET TERMINAL:CHARHEIGHT TO 18.
  PRINT "+-----------------------------------------------+".
  PRINT "|                  IFC v1.0                     |".
  PRINT "|             TEST FLIGHT PROGRAM               |".
  PRINT "+-----------------------------------------------+".
  PRINT "".
  PRINT "IFC boot on core: " + CORE:PART.

  SAS OFF.
  RCS OFF.

  // Keep local volume (1:) in sync with archive (0:) for boot scripts only.
  // Do not mirror large IFC scripts to 1: because local CPU storage is small.
  LOCAL sync_local_scripts IS TRUE.

  FUNCTION _SYNC_ONE {
    PARAMETER src_path, dst_path.
    IF EXISTS(src_path) {
      COPYPATH(src_path, dst_path).
      PRINT "IFC test sync: " + src_path + " -> " + dst_path.
    } ELSE {
      PRINT "IFC test sync: missing source " + src_path.
    }
  }

  IF sync_local_scripts {
    IF NOT EXISTS("1:/boot") { CREATEDIR("1:/boot"). }

    _SYNC_ONE("0:/boot/ifc_testflight_bootloader.ks", "1:/boot/ifc_testflight_bootloader.ks").
  }

  GLOBAL TESTS_REL_DIR IS "Integrated Flight Computer/tests".
  GLOBAL TESTS_ABS_DIR IS "0:/Integrated Flight Computer/tests/".
  GLOBAL TEST_KEYS IS LIST(
    "1","2","3","4","5","6","7","8","9",
    "A","B","C","D","E","F","G","H","I","J","K","L","M",
    "N","O","P","R","S","T","U","V","W","X","Y","Z"
  ).

  FUNCTION _GET_TEST_SCRIPTS {
    LOCAL out IS LIST().
    IF NOT VOLUME(0):EXISTS(TESTS_REL_DIR) { RETURN out. }
    LOCAL dir_item IS VOLUME(0):OPEN(TESTS_REL_DIR).
    IF dir_item:ISFILE { RETURN out. }

    LOCAL dir_lex IS dir_item:LEXICON.
    LOCAL i IS 0.
    UNTIL i >= dir_lex:KEYS:LENGTH {
      LOCAL key IS dir_lex:KEYS[i].
      LOCAL item IS dir_lex[key].
      IF item:ISFILE AND item:EXTENSION:TOLOWER = "ks" {
        out:ADD(TESTS_ABS_DIR + item:NAME).
      }
      SET i TO i + 1.
    }

    RETURN out.
  }

  FUNCTION _PROMPT_TEST_SELECTION {
    PARAMETER scripts.

    IF scripts:LENGTH = 0 { RETURN "". }

    LOCAL shown IS MIN(scripts:LENGTH, TEST_KEYS:LENGTH).
    IF scripts:LENGTH > TEST_KEYS:LENGTH {
      PRINT "IFC test boot: showing first " + shown + " scripts only.".
    }

    PRINT "".
    PRINT "Available test scripts:".
    LOCAL i IS 0.
    UNTIL i >= shown {
      LOCAL display_name IS scripts[i]:REPLACE(TESTS_ABS_DIR, "").
      PRINT "  [" + TEST_KEYS[i] + "] " + display_name.
      SET i TO i + 1.
    }

    PRINT "".
    PRINT "Press selection key, or Q to abort.".
    TERMINAL:INPUT:CLEAR().

    UNTIL FALSE {
      LOCAL ch IS TERMINAL:INPUT:GETCHAR().
      IF ch = "q" OR ch = "Q" {
        RETURN "".
      }

      LOCAL k IS 0.
      UNTIL k >= shown {
        IF ch:TOUPPER = TEST_KEYS[k] {
          RETURN scripts[k].
        }
        SET k TO k + 1.
      }
    }
  }

  LOCAL tests IS _GET_TEST_SCRIPTS().
  LOCAL test_script IS _PROMPT_TEST_SELECTION(tests).

  IF test_script = "" {
    PRINT "IFC test boot: no test selected.".
  } ELSE IF EXISTS(test_script) {
    PRINT "IFC test boot: loading " + test_script.
    RUNPATH(test_script).
  } ELSE {
    PRINT "IFC test boot: selected script does not exist.".
    PRINT "  missing: " + test_script.
  }
} ELSE {
  PRINT "IFC test boot: skipped on non-primary core.".
  PRINT "  core: " + core_name_raw.
  PRINT "  hints: " + PRIMARY_CORE_HINTS.
}
