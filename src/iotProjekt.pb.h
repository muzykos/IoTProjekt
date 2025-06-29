/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.9.1 */

#ifndef PB_IOT_IOTPROJEKT_PB_H_INCLUDED
#define PB_IOT_IOTPROJEKT_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
/* Typ czujnika do celow informacyjnych */
typedef enum _iot_SensorType {
    iot_SensorType_UNKNOWN = 0,
    iot_SensorType_BME680 = 1,
    iot_SensorType_HCSR04 = 2
} iot_SensorType;

/* Struct definitions */
/* Dane z czujnika BME680 */
typedef struct _iot_BME680Data {
    float temperature; /* w °C */
    float pressure; /* w hPa */
    float humidity; /* w % */
} iot_BME680Data;

/* Dane z czujnika HC-SR04 */
typedef struct _iot_HCSR04Data {
    float distance_cm; /* odleglosc w centymetrach */
} iot_HCSR04Data;

/* Wiadomos ogolna wysylana przez ESP32 */
typedef struct _iot_IoTMessage {
    pb_callback_t device_id; /* unikalny identyfikator urzadzenia */
    uint64_t timestamp; /* znacznik czasu (np. UNIX time) */
    iot_SensorType sensor_type; /* ktory czujnik wysyla  dane */
    pb_size_t which_payload;
    union {
        iot_BME680Data bme680_data;
        iot_HCSR04Data hcsr04_data;
    } payload;
} iot_IoTMessage;


#ifdef __cplusplus
extern "C" {
#endif

/* Helper constants for enums */
#define _iot_SensorType_MIN iot_SensorType_UNKNOWN
#define _iot_SensorType_MAX iot_SensorType_HCSR04
#define _iot_SensorType_ARRAYSIZE ((iot_SensorType)(iot_SensorType_HCSR04+1))

#define iot_IoTMessage_sensor_type_ENUMTYPE iot_SensorType




/* Initializer values for message structs */
#define iot_IoTMessage_init_default              {{{NULL}, NULL}, 0, _iot_SensorType_MIN, 0, {iot_BME680Data_init_default}}
#define iot_BME680Data_init_default              {0, 0, 0}
#define iot_HCSR04Data_init_default              {0}
#define iot_IoTMessage_init_zero                 {{{NULL}, NULL}, 0, _iot_SensorType_MIN, 0, {iot_BME680Data_init_zero}}
#define iot_BME680Data_init_zero                 {0, 0, 0}
#define iot_HCSR04Data_init_zero                 {0}

/* Field tags (for use in manual encoding/decoding) */
#define iot_BME680Data_temperature_tag           1
#define iot_BME680Data_pressure_tag              2
#define iot_BME680Data_humidity_tag              3
#define iot_HCSR04Data_distance_cm_tag           1
#define iot_IoTMessage_device_id_tag             1
#define iot_IoTMessage_timestamp_tag             2
#define iot_IoTMessage_sensor_type_tag           3
#define iot_IoTMessage_bme680_data_tag           4
#define iot_IoTMessage_hcsr04_data_tag           5

/* Struct field encoding specification for nanopb */
#define iot_IoTMessage_FIELDLIST(X, a) \
X(a, CALLBACK, SINGULAR, STRING,   device_id,         1) \
X(a, STATIC,   SINGULAR, UINT64,   timestamp,         2) \
X(a, STATIC,   SINGULAR, UENUM,    sensor_type,       3) \
X(a, STATIC,   ONEOF,    MESSAGE,  (payload,bme680_data,payload.bme680_data),   4) \
X(a, STATIC,   ONEOF,    MESSAGE,  (payload,hcsr04_data,payload.hcsr04_data),   5)
#define iot_IoTMessage_CALLBACK pb_default_field_callback
#define iot_IoTMessage_DEFAULT NULL
#define iot_IoTMessage_payload_bme680_data_MSGTYPE iot_BME680Data
#define iot_IoTMessage_payload_hcsr04_data_MSGTYPE iot_HCSR04Data

#define iot_BME680Data_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FLOAT,    temperature,       1) \
X(a, STATIC,   SINGULAR, FLOAT,    pressure,          2) \
X(a, STATIC,   SINGULAR, FLOAT,    humidity,          3)
#define iot_BME680Data_CALLBACK NULL
#define iot_BME680Data_DEFAULT NULL

#define iot_HCSR04Data_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FLOAT,    distance_cm,       1)
#define iot_HCSR04Data_CALLBACK NULL
#define iot_HCSR04Data_DEFAULT NULL

extern const pb_msgdesc_t iot_IoTMessage_msg;
extern const pb_msgdesc_t iot_BME680Data_msg;
extern const pb_msgdesc_t iot_HCSR04Data_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define iot_IoTMessage_fields &iot_IoTMessage_msg
#define iot_BME680Data_fields &iot_BME680Data_msg
#define iot_HCSR04Data_fields &iot_HCSR04Data_msg

/* Maximum encoded size of messages (where known) */
/* iot_IoTMessage_size depends on runtime parameters */
#define IOT_IOTPROJEKT_PB_H_MAX_SIZE             iot_BME680Data_size
#define iot_BME680Data_size                      15
#define iot_HCSR04Data_size                      5

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
