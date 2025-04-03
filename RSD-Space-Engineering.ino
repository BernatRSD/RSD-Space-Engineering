/*********************************************************
 *  Select which system to build by uncommenting one of  *
 *  the defines below.                                   *
 *                                                       *
 *  Board for CANSAT_PRIMARY_SYSTEM and GROUND_STATION:  *
 *  Adafruit Feather ESP32-S3 Reverse TFT                *
 *                                                       *
 *  Board for CANSAT_SECONDARY_SYSTEM:                   *
 *  Adafruit Feather RP2040 RFM                          *
 * *******************************************************/

#define CANSAT_PRIMARY_SYSTEM
// #define CANSAT_SECONDARY_SYSTEM
// #define GROUND_STATION


/***********************************************************
 *  TESTING & DEBUGGING                                    *
 *  Uncomment the define below for debugging over Serial.  *
 *  COMMENT OUT for real flight!                           *
 ***********************************************************/

// #define RSD_DEBUG
