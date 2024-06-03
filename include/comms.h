/*
    comms.h
    
    Initializes & configures MavLink radio link
    Maintains heartbeat & continuous connection
    
    @authors Orfeas Magoulas
*/
#ifndef COMMS_H
#define COMMS_H

#include <mavlink.h>
#include <sensors.h>
#include <vehicle.h>
#include <nav.h>

#define MAVLINK_UPDATE_RATE_HZ 10
#define PRINT_IMU
#define PRINT_GNSS

class Comms
{
private:
    Sensors* sensors;
    Vehicle* vehicle;
    Nav* nav;
    int timestamp;
public:
    bfs::MavLink<5, 10> mavlink;
    Comms(Sensors* sensors, Vehicle* vehicle, Nav* nav) : sensors(sensors), vehicle(vehicle), nav(nav) {}
    void init();
    void run();
};

/*
    Define vehicle info
    Set data stream periods
    Initialize aircraft modes & states
    Initialize sensor states
    Begin MavLink connection
*/
inline void Comms::init()
{
    mavlink.hardware_serial(&Serial4);
    mavlink.aircraft_type(bfs::HELICOPTER);

    // Data stream rates
    mavlink.raw_sens_stream_period_ms(1000 / MAVLINK_UPDATE_RATE_HZ);
    mavlink.ext_status_stream_period_ms(1000 / MAVLINK_UPDATE_RATE_HZ);
    mavlink.rc_chan_stream_period_ms(1000 / MAVLINK_UPDATE_RATE_HZ);
    mavlink.pos_stream_period_ms(1000 / MAVLINK_UPDATE_RATE_HZ);
    mavlink.extra1_stream_period_ms(1000 / MAVLINK_UPDATE_RATE_HZ);
    mavlink.extra2_stream_period_ms(1000 / MAVLINK_UPDATE_RATE_HZ);
    mavlink.extra3_stream_period_ms(1000 / MAVLINK_UPDATE_RATE_HZ);

    // Initial aircraft mode & states
    mavlink.aircraft_mode(bfs::AUTO);
    mavlink.aircraft_state(bfs::ACTIVE);
    mavlink.throttle_enabled(true);

    // Sensors
    mavlink.gyro_installed(true);
    mavlink.accel_installed(true);
    mavlink.mag_installed(true);
    mavlink.gnss_installed(true);

    // Start MavLink connection
    mavlink.Begin(57600);
    mavlink.Update();

    mavlink.SendStatusText(bfs::Severity::INFO, "MavLink Initialized");
}

/*
    Update IMU & GNSS health data
    Update navigational data
    Update MavLink
*/
inline void Comms::run()
{
    if ((millis() - timestamp) < (1000 / MAVLINK_UPDATE_RATE_HZ)) {
        return;
    }
    timestamp = millis();

    // Update health data
    mavlink.gyro_healthy(true);
    mavlink.mag_healthy(true);
    mavlink.accel_healthy(true);

    // Update navigational heading, pitch, roll
    mavlink.nav_hdg_rad(-nav->heading * DEG_TO_RAD);
    mavlink.nav_pitch_rad(nav->pitch * DEG_TO_RAD);
    mavlink.nav_roll_rad(-nav->roll * DEG_TO_RAD);

    // Update GNSS health data
    mavlink.gnss_healthy(true);
    mavlink.gnss_fix(nav->gnss_fix_type);
    mavlink.gnss_num_sats(nav->gnss_satellites);
    mavlink.gnss_healthy(true);

    // Update MavLink navigation lat, lon, alt
    mavlink.nav_lat_rad(nav->lat * PI / 180);
    mavlink.nav_lon_rad(nav->lon * PI / 180);
    mavlink.nav_alt_msl_m(nav->alt);

    mavlink.Update();

    #if defined(PRINT_IMU)
    Serial.print("Orientation: ");
    Serial.print(nav->heading);
    Serial.print(", ");
    Serial.print(nav->pitch);
    Serial.print(", ");
    Serial.println(nav->roll);

    Serial.print("Quaternion: ");
    Serial.print(nav->qw, 4);
    Serial.print(", ");
    Serial.print(nav->qx, 4);
    Serial.print(", ");
    Serial.print(nav->qy, 4);
    Serial.print(", ");
    Serial.println(nav->qz, 4);
    #endif

    #if defined(PRINT_GNSS)
    Serial.print(F("Fix Type: "));
    Serial.println(nav->gnss_fix_type);

    Serial.print(F("Number of satellites: "));
    Serial.println(nav->gnss_satellites);  

    Serial.print(F("Lat: "));
    Serial.print(nav->lat, 7);
    Serial.print(F(" Long: "));
    Serial.print(nav->lon, 7);
    Serial.print(F(" (degrees)"));
    Serial.print(F(" Alt: "));
    Serial.print(nav->alt);
    Serial.print(F(" (meters)"));
    Serial.println();
    #endif
}

#endif