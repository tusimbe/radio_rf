#ifndef __TELEMETRY_H__
#define __TELEMETRY_H__


#define ToDeg(x) (x*57.2957795131)	// *180/pi

typedef struct _telemetry_sys_status_s
{
    uint16_t voltage_battery;
    int16_t  current_battery;
    uint16_t  battery_remaining;
} TELEMETRY_SYS_STATUS;

typedef struct _telemetry_raw_imu_s
{
    int16_t xacc; ///< X acceleration (raw)
    int16_t yacc; ///< Y acceleration (raw)
    int16_t zacc; ///< Z acceleration (raw)
    int16_t xgyro; ///< Angular speed around X axis (raw)
    int16_t ygyro; ///< Angular speed around Y axis (raw)
    int16_t zgyro; ///< Angular speed around Z axis (raw)
    int16_t xmag; ///< X Magnetic field (raw)
    int16_t ymag; ///< Y Magnetic field (raw)
    int16_t zmag; ///< Z Magnetic field (raw)
} TELEMETRY_RAW_IMU;

typedef struct _telemetry_gps_raw_s
{
    int32_t lat; ///< Latitude in 1E7 degrees
    int32_t lon; ///< Longitude in 1E7 degrees
    int32_t alt; ///< Altitude in 1E3 meters (millimeters) above MSL
    uint8_t fix_type; ///< 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will 
                      ///< not use the value of this field unless it is at least two, 
                      ///< so always correctly fill in the fix.
    uint8_t satellites_visible; ///< Number of satellites visible. If unknown, set to 255
} TELEMETRY_GPS_RAW;

typedef struct _telemetry_attitude_s
{
    float roll; ///< Roll angle (rad, -pi..+pi)
    float pitch; ///< Pitch angle (rad, -pi..+pi)
    float yaw; ///< Yaw angle (rad, -pi..+pi)
} TELEMETRY_ATTITUDE;

typedef struct _telemetry_data_s
{
    TELEMETRY_RAW_IMU raw_imu;
    TELEMETRY_SYS_STATUS sys_status;
    TELEMETRY_GPS_RAW gps_raw;
    TELEMETRY_ATTITUDE attitude;
} TELEMETRY_DATA;

typedef struct _telemetry_stream_s
{
    uint8_t  stream_id;
    uint16_t freq;
} TELEMETRY_STREAM;

int32_t telemetry_init(void);

uint8_t telemetry_push_attitude
(
    void *buf
);

uint8_t telemetry_push_volt_cur
(
    void *buf
);
#endif
