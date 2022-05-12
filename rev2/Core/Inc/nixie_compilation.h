/**
 * @file nixie_compilation.h
 * @brief Header file containing macros related to conditional compilation.
 */


#ifndef NIXIE_COMPILATION_H
#define NIXIE_COMPILATION_H

/** Macro indicating the system is using the HV5623 converter. */
#define NIXIE_DISPLAY_OUTPUT_HV5623 (0U)
/** Macro indicating the system is using the HV5523 converter. */
#define NIXIE_DISPLAY_OUTPUT_HV5523 (1U)

/** Macro indicating the custom characteristics in the GATT server should be stored as big endian.*/
#define NIXIE_GATT_SERVER_BIG_ENDIAN (0)
/** Macro indicating the custom characteristics in the GATT server should be stored as little endian.*/
#define NIXIE_GATT_SERVER_LITTLE_ENDIAN (1)

/** Compilation flag for presence of EEPROM on the device. */
#define NIXIE_EEPROM_PRESENT (1)
/** Compilation flag for indicating whether the SW should initialize the data in EEPROM. */
#define NIXIE_INIT_EEPROM (0)
/** Compilation flag for forcing backup domain reset. */
#define NIXIE_FORCE_BACKUP_DOMAIN_RESET (0)
/** Display output HV type indicator. This value will be used to initialize either the EEPROM or the application, if there is no EEPROM in the device. */
#define NIXIE_DISPLAY_OUTPUT_HV_TYPE (NIXIE_DISPLAY_OUTPUT_HV5623)
/** 
 * Compilation flag indicating whether the GATT server stores custom characteristic in big- or little-endian.
 * To keep the design simple, the endianness is reused on the EEPROM. As such, changing the endianness might warrant EEPROM reinitialization.
 */
#define NIXIE_GATT_SERVER_ENDIANNESS (NIXIE_GATT_SERVER_BIG_ENDIAN)

// Macro value checks.
#if (NIXIE_EEPROM_PRESENT != 0) && (NIXIE_EEPROM_PRESENT != 1)
#error "Invalid NIXIE_EEPROM_PRESENT value."
#endif
#if (NIXIE_INIT_EEPROM != 0) && (NIXIE_INIT_EEPROM != 1)
#error "Invalid NIXIE_INIT_EEPROM value."
#endif
#if (NIXIE_FORCE_BACKUP_DOMAIN_RESET != 0) && (NIXIE_FORCE_BACKUP_DOMAIN_RESET != 1)
#error "Invalid NIXIE_FORCE_BACKUP_DOMAIN_RESET value."
#endif
#if (NIXIE_DISPLAY_OUTPUT_HV_TYPE != NIXIE_DISPLAY_OUTPUT_HV5623) && (NIXIE_DISPLAY_OUTPUT_HV_TYPE != NIXIE_DISPLAY_OUTPUT_HV5523)
#error "Invalid NIXIE_DISPLAY_OUTPUT_HV_TYPE value."
#endif
#if (NIXIE_GATT_SERVER_ENDIANNESS != NIXIE_GATT_SERVER_BIG_ENDIAN) && (NIXIE_GATT_SERVER_ENDIANNESS != NIXIE_GATT_SERVER_LITTLE_ENDIAN)
#error "Invalid NIXIE_GATT_SERVER_ENDIANNESS value."
#endif

#endif //NIXIE_COMPILATION_H