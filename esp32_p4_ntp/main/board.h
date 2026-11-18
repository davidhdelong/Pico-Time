#pragma once
/* Waveshare ESP32-P4-ETH / ESP32-P4-POE-ETH schematic, not the C6 monitor.
 * These expansion pins avoid RMII, flash, console, USB and boot straps. */
#define P4_GPS_RX 5
#define P4_GPS_TX 4
#define P4_GPS_PPS 6
#define P4_ETH_PHY_RESET 51
#define P4_ETH_MDC 31
#define P4_ETH_MDIO 52
#define P4_TIMER_HZ 10000000u
#define P4_TICK_NS 100u
