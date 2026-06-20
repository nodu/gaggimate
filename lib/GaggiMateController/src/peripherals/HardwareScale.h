#ifndef HARDWARESCALE_H
#define HARDWARESCALE_H

#include <Arduino.h>
#include <functional>

constexpr int SCALE_READ_INTERVAL_MS = 100;

using scale_reading_callback_t = std::function<void(float weight, float w1, float w2)>;
using scale_factor_callback_t = std::function<void(float scaleFactor)>;

class HardwareScale {
  public:
    HardwareScale(uint8_t data_pin1, uint8_t data_pin2, uint8_t clock_pin, const scale_reading_callback_t &reading_callback,
                  const scale_factor_callback_t &config_callback);
    ~HardwareScale() = default;

    struct RawReading {
        long value1;
        long value2;
    };

    void setup();
    void loop();
    void setScaleFactor(float scale_factor);
    void calibrate(float calibrationWeight);
    bool isReady();
    bool isAvailable() const { return is_initialized; }
    void tare();

  private:
    void doTare();
    void doCalibrate();

    bool is_initialized = false;
    bool _cell1_present = false;
    bool _cell2_present = false;
    uint8_t _data_pin1;
    uint8_t _data_pin2;
    uint8_t _clock_pin;
    float _weight = 0.0f;
    float _scale_factor = 1.0f;
    float _offset1 = 0.0f;
    float _offset2 = 0.0f;
    volatile bool _tare_requested = false;
    volatile bool _calibrate_requested = false;
    float _calibrate_weight = 0.0f;
    int _skip_readings = 0;
    scale_reading_callback_t _reading_callback;
    scale_factor_callback_t _configuration_callback;
    xTaskHandle taskHandle = nullptr;

    const char *LOG_TAG = "HardwareScale";
    static void loopTask(void *arg);

    bool waitForReady();
    RawReading readRaw();
    float rawToWeight(long raw, float offset) const;
    float smooth(float previous, float current);
};

#endif // HARDWARESCALE_H
