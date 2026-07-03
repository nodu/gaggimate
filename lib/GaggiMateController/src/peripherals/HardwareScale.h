#ifndef HARDWARESCALE_H
#define HARDWARESCALE_H

#include <Arduino.h>
#include <atomic>
#include <freertos/queue.h>
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
    bool doTare();
    void doCalibrate(float calibrationWeight);

    enum class ScaleCommandType { Tare, Calibrate, SetScaleFactor };

    struct ScaleCommand {
        ScaleCommandType type;
        float value;
    };

    struct RawSampleStats {
        float average1 = 0.0f;
        float average2 = 0.0f;
        long range1 = 0;
        long range2 = 0;
        int reads = 0;
    };

    bool is_initialized = false;
    bool _cell1_present = false;
    bool _cell2_present = false;
    uint8_t _data_pin1;
    uint8_t _data_pin2;
    uint8_t _clock_pin;
    float _weight = 0.0f;
    float _scale_factor = 0.0f;
    float _offset1 = 0.0f;
    float _offset2 = 0.0f;
    int _skip_readings = 0;
    int _read_timeout_count = 0;
    bool _has_weight = false;
    std::atomic<bool> _tare_pending{false};
    scale_reading_callback_t _reading_callback;
    scale_factor_callback_t _configuration_callback;
    xTaskHandle taskHandle = nullptr;
    QueueHandle_t _command_queue = nullptr;

    const char *LOG_TAG = "HardwareScale";
    static void loopTask(void *arg);

    bool enqueueCommand(ScaleCommandType type, float value = 0.0f);
    bool processNextCommand();
    bool hasPendingCommand() const;
    bool waitForReady(unsigned long timeoutMs);
    RawReading readRaw();
    bool collectStableRawSamples(RawSampleStats &stats, int sampleCount, int discardCount, const char *operation);
    long getStableRawRange() const;
    bool isValidScaleFactor(float scale_factor) const;
    float rawToWeight(float raw, float offset) const;
    float smooth(float previous, float current);
};

#endif // HARDWARESCALE_H
