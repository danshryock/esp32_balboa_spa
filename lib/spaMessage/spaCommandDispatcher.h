#ifndef SPA_COMMAND_DISPATCHER_H
#define SPA_COMMAND_DISPATCHER_H

#include <Arduino.h>
#include <ArduinoJson.h>

#include "../../src/config.h"

/** Once per boot: set spa panel clock from gateway NTP when drift exceeds threshold. Existing installs default off. */
#ifndef AUTO_SYNC_PANEL_CLOCK
#define AUTO_SYNC_PANEL_CLOCK 0
#endif

#ifndef AUTO_SYNC_PANEL_CLOCK_THRESHOLD_MIN
#define AUTO_SYNC_PANEL_CLOCK_THRESHOLD_MIN 2
#endif

/** Light "next show": gap between the off and on toggles. Multicolour lights (e.g. Balboa MoodEFX) step to the
 *  next show only when power returns within ~1 s. On a BP501 400-700 ms was 15/15 reliable; 200-300 ms sometimes
 *  had the second toggle dropped by the controller. */
#ifndef LIGHT_SHOW_TOGGLE_GAP_MS
#define LIGHT_SHOW_TOGGLE_GAP_MS 450
#endif

/** Light "next show": only judge the result on status frames received at least this long after the second toggle. */
#ifndef LIGHT_SHOW_VERIFY_SETTLE_MS
#define LIGHT_SHOW_VERIFY_SETTLE_MS 1000
#endif

/** Light "next show": give up verifying (no retry) if no settled status frame arrives within this time. */
#ifndef LIGHT_SHOW_VERIFY_TIMEOUT_MS
#define LIGHT_SHOW_VERIFY_TIMEOUT_MS 5000
#endif

struct SpaFilterSettingsData;

enum SpaCommandSource
{
  SPA_COMMAND_SOURCE_UNKNOWN = 0,
  SPA_COMMAND_SOURCE_WEB = 1,
  SPA_COMMAND_SOURCE_MQTT = 2,
  SPA_COMMAND_SOURCE_AUTO = 3,
};

enum SpaCommandResultCode
{
  SPA_COMMAND_ACCEPTED = 0,
  SPA_COMMAND_NOT_READY = 1,
  SPA_COMMAND_INVALID_ARGUMENT = 2,
};

struct SpaCommandResult
{
  bool accepted;
  SpaCommandResultCode code;
  const char *reason;
};

struct SpaFilterCycleSettings
{
  uint8_t filt1Hour;
  uint8_t filt1Minute;
  uint8_t filt1DurHour;
  uint8_t filt1DurMinute;
  bool filt2Enable;
  uint8_t filt2Hour;
  uint8_t filt2Minute;
  uint8_t filt2DurHour;
  uint8_t filt2DurMinute;
};

bool spaCanAcceptCommands();
bool spaHasFreshStatus();
/** Balboa 0x20 limits for current `spaStatusData.tempScale` + `tempRange` (see protocol.md). */
void spaProtocolActiveSetpointBand(float &minBand, float &maxBand);
SpaCommandResult spaSendToggleCommand(uint8_t itemCode, SpaCommandSource source = SPA_COMMAND_SOURCE_UNKNOWN);
int spaToggleCountForButtonRequest(uint8_t itemCode, bool requestHasState, bool desiredOn);
int spaPumpToggleCountForSpeed(uint8_t pumpId, uint8_t desiredSpeed);
SpaCommandResult spaSendButtonForBinaryState(uint8_t itemCode, bool desiredOn, SpaCommandSource source = SPA_COMMAND_SOURCE_UNKNOWN);
SpaCommandResult spaSendButtonForPumpSpeed(uint8_t pumpId, uint8_t desiredSpeed, SpaCommandSource source = SPA_COMMAND_SOURCE_UNKNOWN);
SpaCommandResult spaSetHeatingMode(bool ready, SpaCommandSource source = SPA_COMMAND_SOURCE_UNKNOWN);
SpaCommandResult spaSetTempRange(bool high, SpaCommandSource source = SPA_COMMAND_SOURCE_UNKNOWN);
SpaCommandResult spaSetTargetTemperature(float targetTemperature, SpaCommandSource source = SPA_COMMAND_SOURCE_UNKNOWN);
/** Balboa `0x27` set temperature scale (`0x00` Fahrenheit, `0x01` Celsius). */
SpaCommandResult spaSetTemperatureScale(bool celsius, SpaCommandSource source = SPA_COMMAND_SOURCE_UNKNOWN);
/** Balboa `0x27` panel maintenance reminders (`0x00` off, `0x01` on). */
SpaCommandResult spaSetPanelReminders(bool enabled, SpaCommandSource source = SPA_COMMAND_SOURCE_UNKNOWN);
/** Balboa `0x21` set panel 12h/24h display bit while preserving current panel time value. */
SpaCommandResult spaSetSpaPanelClockFormat(bool use24Hour, SpaCommandSource source = SPA_COMMAND_SOURCE_UNKNOWN);
/** Balboa `0x21` set panel clock (hour 0–23, minute 0–59). High bit of hour follows current `spaStatusData.clockMode` (24h vs 12h display). */
SpaCommandResult spaSetSpaPanelClockTime(uint8_t hour24, uint8_t minute, SpaCommandSource source = SPA_COMMAND_SOURCE_UNKNOWN);
/** Balboa `0x21` set panel clock and 12h/24h display bit in one frame. */
SpaCommandResult spaSetSpaPanelClockTimeEx(uint8_t hour24, uint8_t minute, bool use24Hour, SpaCommandSource source = SPA_COMMAND_SOURCE_UNKNOWN);
/** Balboa `0x23` set filter 1/2 daily cycle schedules (8-byte payload). */
SpaCommandResult spaSetFilterCycles(const SpaFilterCycleSettings &settings, SpaCommandSource source = SPA_COMMAND_SOURCE_UNKNOWN);
/** Clear filter 2 start/duration when disabled (wire + readback normalization). */
void spaNormalizeFilter2Schedule(SpaFilterCycleSettings &settings);
void spaNormalizeFilter2Cache(SpaFilterSettingsData &cached);
bool spaValidateFilterCycleSettings(const SpaFilterCycleSettings &settings, const char **outReason = nullptr);
bool spaParseFilterCyclePayload(const uint8_t *eightBytes, SpaFilterCycleSettings &out);
bool spaFilterCycleSettingsEqual(const SpaFilterCycleSettings &a, const SpaFilterSettingsData &cached);
/** Map live filter cache into dispatcher settings. Returns false if filter read not ready. */
bool spaFilterSettingsFromCache(SpaFilterCycleSettings &out);
/** Parse `H:MM` / `HH:MM` (minutes always two digits). */
bool spaParseFilterTimeHm(const String &payload, uint8_t &hour, uint8_t &minute);
/** Parse filter JSON; merge omitted fields from cache when `mergeFromCache` is true. */
bool spaParseFilterCycleJson(JsonObjectConst doc, SpaFilterCycleSettings &out, bool mergeFromCache, const char **errReason);
/** Granular MQTT path under `filter/…` (e.g. `filter/filter1/start`). */
bool spaApplyFilterGranularMqtt(const String &subPath, const String &payload, SpaFilterCycleSettings &out, const char **errReason);
/** True when live status `filterMode` reports filter cycle 1 active. */
bool spaFilter1Running();
/** True when live status `filterMode` reports filter cycle 2 active. */
bool spaFilter2Running();
SpaCommandResult spaSendToggleDiagnostic(
    uint8_t itemCode,
    bool useWifiDestination,
    bool includeZeroPad,
    SpaCommandSource source = SPA_COMMAND_SOURCE_UNKNOWN,
    String *outFrameHex = nullptr);
SpaCommandResult spaSendToggleOnNextCtsDiagnostic(
    uint8_t itemCode,
    bool useWifiDestination,
    bool includeZeroPad,
    SpaCommandSource source = SPA_COMMAND_SOURCE_UNKNOWN,
    String *outFrameHex = nullptr,
    uint32_t *outArmCount = nullptr);

/** Non-blocking once-per-boot panel clock sync when `AUTO_SYNC_PANEL_CLOCK` is set (tub builds). */
void spaPanelClockAutoSyncTick();

/** Step light 1 or 2 to its next show. Off: one toggle turns it on (the light resumes its last show).
 *  On: toggle off, then on after LIGHT_SHOW_TOGGLE_GAP_MS; spaLightShowTick() sends the second toggle and
 *  re-sends once if settled status still shows the light off. */
SpaCommandResult spaAdvanceLightShow(uint8_t light, SpaCommandSource source = SPA_COMMAND_SOURCE_UNKNOWN);
void spaLightShowTick();
/** `GET /api/diagnostics` — `panelClockAutoSync` object (LOCAL_CLIENT builds only). */
void spaPanelClockAutoSyncAppendToJson(JsonObject root);

#endif
