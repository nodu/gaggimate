#include "HardwareScale.h"
#include <Arduino.h>
#include <cmath>
#include <limits>

namespace {
constexpr float MAX_SCALE_GRAMS = 1500.0f;
constexpr unsigned long MAX_WAIT_READ_MS = 250;
constexpr unsigned long MAX_STARTUP_WAIT_MS = 1200;
constexpr int COMMAND_QUEUE_LENGTH = 4;
constexpr int TARE_SAMPLE_COUNT = 3;
constexpr int TARE_DISCARD_COUNT = 1;
constexpr int CALIBRATION_SAMPLE_COUNT = 16;
constexpr int CALIBRATION_DISCARD_COUNT = 3;
constexpr int MAX_READ_TIMEOUTS_BEFORE_ZERO = 5;
constexpr float MIN_CALIBRATION_WEIGHT_GRAMS = 1.0f;
constexpr float MIN_CALIBRATION_RAW_DELTA = 100.0f;
constexpr float MIN_SCALE_FACTOR_MAGNITUDE = 1.0f;
constexpr float MAX_SCALE_FACTOR_MAGNITUDE = 1000000.0f;
constexpr float LARGE_WEIGHT_CHANGE_GRAMS = 2.0f;
constexpr float FAST_SMOOTHING_ALPHA = 0.65f;
constexpr float STABLE_SMOOTHING_ALPHA = 0.35f;
} // namespace

HardwareScale::HardwareScale(uint8_t data_pin1, uint8_t data_pin2, uint8_t clock_pin,
                             const scale_reading_callback_t &reading_callback, const scale_factor_callback_t &config_callback)
    : _data_pin1(data_pin1), _data_pin2(data_pin2), _clock_pin(clock_pin), _reading_callback(reading_callback),
      _configuration_callback(config_callback) {}

void HardwareScale::setup() {
    _command_queue = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(ScaleCommand));
    if (_command_queue == nullptr) {
        ESP_LOGE(LOG_TAG, "Failed to create hardware scale command queue");
        is_initialized = false;
        return;
    }

    pinMode(_data_pin1, INPUT);
    pinMode(_data_pin2, INPUT);
    pinMode(_clock_pin, OUTPUT);
    // Power-cycle HX711s: hold clock HIGH >60us to enter power-down, then LOW to wake
    digitalWrite(_clock_pin, HIGH);
    delay(1);
    digitalWrite(_clock_pin, LOW);
    delay(1);
    // Second reset cycle to ensure clean state
    digitalWrite(_clock_pin, HIGH);
    delay(1);
    digitalWrite(_clock_pin, LOW);
    ESP_LOGI(LOG_TAG, "Initializing hardware scale on DATA1: GPIO%d, DATA2: GPIO%d, CLOCK: GPIO%d", _data_pin1, _data_pin2,
             _clock_pin);

    // HX711 needs ~400ms to wake from power-down after SCK goes LOW
    delay(1000);

    // Detect which cells are present by checking which data pins go LOW
    long start = millis();
    while ((!_cell1_present || !_cell2_present) && (millis() - start) < MAX_STARTUP_WAIT_MS) {
        if (!_cell1_present && digitalRead(_data_pin1) == LOW)
            _cell1_present = true;
        if (!_cell2_present && digitalRead(_data_pin2) == LOW)
            _cell2_present = true;
        delay(10);
    }

    if (!_cell1_present && !_cell2_present) {
        ESP_LOGE(LOG_TAG, "No HX711 cells detected after %ld ms, aborting", millis() - start);
        is_initialized = false;
        return;
    }
    ESP_LOGI(LOG_TAG, "Detected cells: cell1=%s, cell2=%s", _cell1_present ? "yes" : "no", _cell2_present ? "yes" : "no");

    // Throwaway read to sync, then wait for next conversion
    readRaw();
    long start2 = millis();
    while (!isReady() && (millis() - start2) < 2000) {
        delay(1);
    }
    if (!isReady()) {
        ESP_LOGE(LOG_TAG, "HX711 not ready after first read, aborting");
        is_initialized = false;
        return;
    }

    doTare();
    is_initialized = true;
    ESP_LOGI(LOG_TAG, "Hardware scale initialized successfully");

    _configuration_callback(_scale_factor);

    xTaskCreate(loopTask, "HardwareScale::loop", configMINIMAL_STACK_SIZE * 4, this, 1, &taskHandle);
}

// Only wait for cells that were detected at init time.
// This ensures we never read a non-present cell (which would return garbage).
bool HardwareScale::isReady() {
    if (_cell1_present && digitalRead(_data_pin1) != LOW)
        return false;
    if (_cell2_present && digitalRead(_data_pin2) != LOW)
        return false;
    return true;
}

HardwareScale::RawReading HardwareScale::readRaw() {
    unsigned long value1 = 0;
    unsigned long value2 = 0;

    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    taskENTER_CRITICAL(&mux);

    // Read 24 bits
    for (int8_t i = 23; i >= 0; i--) {
        digitalWrite(_clock_pin, HIGH);
        delayMicroseconds(20);
        value1 |= (digitalRead(_data_pin1) << i);
        value2 |= (digitalRead(_data_pin2) << i);
        digitalWrite(_clock_pin, LOW);
        delayMicroseconds(20);
    }

    // Set gain for next reading (1 extra pulse for gain 128)
    digitalWrite(_clock_pin, HIGH);
    delayMicroseconds(20);
    digitalWrite(_clock_pin, LOW);
    delayMicroseconds(20);

    taskEXIT_CRITICAL(&mux);

    // Convert to signed 24-bit
    if (value1 & 0x800000) {
        value1 |= 0xFF000000;
    }

    if (value2 & 0x800000) {
        value2 |= 0xFF000000;
    }

    return {static_cast<long>(value1), static_cast<long>(value2)};
}

float HardwareScale::rawToWeight(float raw, float offset) const {
    return std::clamp((raw - offset) / _scale_factor, -MAX_SCALE_GRAMS, MAX_SCALE_GRAMS);
}

float HardwareScale::smooth(float previous, float current) {
    const float delta = std::fabs(current - previous);
    const float alpha = delta > LARGE_WEIGHT_CHANGE_GRAMS ? FAST_SMOOTHING_ALPHA : STABLE_SMOOTHING_ALPHA;
    return std::clamp((alpha * current) + ((1.0f - alpha) * previous), -MAX_SCALE_GRAMS, MAX_SCALE_GRAMS);
}

void HardwareScale::loop() {
    // Handle tare/calibrate requests on this task to avoid race conditions
    if (processNextCommand()) {
        return;
    }

    if (_tare_pending.load()) {
        return;
    }

    if (!waitForReady(MAX_WAIT_READ_MS)) {
        _read_timeout_count++;
        ESP_LOGW(LOG_TAG, "HX711 not ready for normal read (%d consecutive timeouts)", _read_timeout_count);
        if (_read_timeout_count >= MAX_READ_TIMEOUTS_BEFORE_ZERO) {
            _reading_callback(0.0f, 0.0f, 0.0f);
        }
        return;
    }
    _read_timeout_count = 0;

    auto raw = readRaw();

    // Discard if a request arrived during the read
    if (hasPendingCommand()) {
        return;
    }

    if (_skip_readings > 0) {
        _skip_readings--;
        return;
    }

    if (!isValidScaleFactor(_scale_factor)) {
        _reading_callback(0.0f, 0.0f, 0.0f);
        return;
    }

    float v1 = _cell1_present ? rawToWeight(static_cast<float>(raw.value1), _offset1) : 0.0f;
    float v2 = _cell2_present ? rawToWeight(static_cast<float>(raw.value2), _offset2) : 0.0f;
    float combined = v1 + v2;
    _weight = _has_weight ? smooth(_weight, combined) : std::clamp(combined, -MAX_SCALE_GRAMS, MAX_SCALE_GRAMS);
    _has_weight = true;
    ESP_LOGD(LOG_TAG, "v1: %.2f, v2: %.2f, Smoothed: %.2f", v1, v2, _weight);

    // Final check — don't send if tare/calibrate was requested
    if (hasPendingCommand()) {
        return;
    }

    _reading_callback(_weight, v1, v2);
}

void HardwareScale::setScaleFactor(float scale_factor) {
    if (!enqueueCommand(ScaleCommandType::SetScaleFactor, scale_factor)) {
        ESP_LOGW(LOG_TAG, "Failed to queue scale factor update");
    }
}

void HardwareScale::tare() {
    _tare_pending.store(true);
    if (!enqueueCommand(ScaleCommandType::Tare)) {
        _tare_pending.store(false);
        ESP_LOGW(LOG_TAG, "Failed to queue tare request");
    }
}

void HardwareScale::calibrate(float calibrationWeight) {
    if (!enqueueCommand(ScaleCommandType::Calibrate, calibrationWeight)) {
        ESP_LOGW(LOG_TAG, "Failed to queue calibration request");
    }
}

bool HardwareScale::enqueueCommand(ScaleCommandType type, float value) {
    if (_command_queue == nullptr) {
        return false;
    }
    const ScaleCommand command{.type = type, .value = value};
    return xQueueSend(_command_queue, &command, 0) == pdTRUE;
}

bool HardwareScale::processNextCommand() {
    if (_command_queue == nullptr) {
        return false;
    }

    ScaleCommand command;
    if (xQueueReceive(_command_queue, &command, 0) != pdTRUE) {
        return false;
    }

    switch (command.type) {
    case ScaleCommandType::Tare:
        _tare_pending.store(true);
        doTare();
        _tare_pending.store(false);
        break;
    case ScaleCommandType::Calibrate:
        doCalibrate(command.value);
        break;
    case ScaleCommandType::SetScaleFactor:
        if (!isValidScaleFactor(command.value)) {
            ESP_LOGW(LOG_TAG, "Rejected invalid scale factor: %.3f", command.value);
            break;
        }
        _scale_factor = command.value;
        _has_weight = false;
        ESP_LOGI(LOG_TAG, "Set scale factor: %.3f", _scale_factor);
        break;
    }

    return true;
}

bool HardwareScale::hasPendingCommand() const { return _command_queue != nullptr && uxQueueMessagesWaiting(_command_queue) > 0; }

bool HardwareScale::waitForReady(unsigned long timeoutMs) {
    long start = millis();
    while (!isReady() && (millis() - start) < timeoutMs) {
        delay(10);
    }
    return isReady();
}

bool HardwareScale::collectStableRawSamples(RawSampleStats &stats, int sampleCount, int discardCount, const char *operation) {
    for (int i = 0; i < discardCount; i++) {
        if (!waitForReady(MAX_WAIT_READ_MS)) {
            ESP_LOGW(LOG_TAG, "HX711 not ready for %s discard read %d", operation, i);
            return false;
        }
        readRaw();
    }

    long sum1 = 0;
    long sum2 = 0;
    long min1 = std::numeric_limits<long>::max();
    long max1 = std::numeric_limits<long>::min();
    long min2 = std::numeric_limits<long>::max();
    long max2 = std::numeric_limits<long>::min();
    int reads = 0;

    for (int i = 0; i < sampleCount; i++) {
        if (!waitForReady(MAX_WAIT_READ_MS)) {
            ESP_LOGW(LOG_TAG, "HX711 not ready for %s read %d", operation, i);
            continue;
        }
        auto raw = readRaw();
        if (_cell1_present) {
            sum1 += raw.value1;
            min1 = std::min(min1, raw.value1);
            max1 = std::max(max1, raw.value1);
        }
        if (_cell2_present) {
            sum2 += raw.value2;
            min2 = std::min(min2, raw.value2);
            max2 = std::max(max2, raw.value2);
        }
        reads++;
    }

    if (reads < sampleCount / 2) {
        ESP_LOGE(LOG_TAG, "%s failed: only %d/%d successful reads", operation, reads, sampleCount);
        return false;
    }

    stats.average1 = _cell1_present ? static_cast<float>(sum1) / reads : 0.0f;
    stats.average2 = _cell2_present ? static_cast<float>(sum2) / reads : 0.0f;
    stats.range1 = _cell1_present ? max1 - min1 : 0;
    stats.range2 = _cell2_present ? max2 - min2 : 0;
    stats.reads = reads;

    const long maxRange = getStableRawRange();
    if ((_cell1_present && stats.range1 > maxRange) || (_cell2_present && stats.range2 > maxRange)) {
        ESP_LOGW(LOG_TAG, "%s rejected: unstable readings (range1=%ld, range2=%ld, limit=%ld)", operation, stats.range1,
                 stats.range2, maxRange);
        return false;
    }

    return true;
}

long HardwareScale::getStableRawRange() const {
    if (!isValidScaleFactor(_scale_factor)) {
        return 1000;
    }
    return std::max(100L, static_cast<long>(std::fabs(_scale_factor) * 0.25f));
}

bool HardwareScale::isValidScaleFactor(float scale_factor) const {
    const float magnitude = std::fabs(scale_factor);
    return std::isfinite(scale_factor) && magnitude >= MIN_SCALE_FACTOR_MAGNITUDE && magnitude <= MAX_SCALE_FACTOR_MAGNITUDE;
}

bool HardwareScale::doTare() {
    RawSampleStats stats;
    if (!collectStableRawSamples(stats, TARE_SAMPLE_COUNT, TARE_DISCARD_COUNT, "tare")) {
        ESP_LOGE(LOG_TAG, "Tare failed; keeping previous offsets");
        return false;
    }

    if (_cell1_present)
        _offset1 = stats.average1;
    if (_cell2_present)
        _offset2 = stats.average2;
    _weight = 0.0f;
    _has_weight = true;
    _skip_readings = 2;
    _reading_callback(0.0f, 0.0f, 0.0f);
    ESP_LOGI(LOG_TAG, "Tared scale offsets: %.3f, %.3f (%d reads, range1=%ld, range2=%ld)", _offset1, _offset2, stats.reads,
             stats.range1, stats.range2);
    return true;
}

void HardwareScale::doCalibrate(float calibrationWeight) {
    if (!std::isfinite(calibrationWeight) || calibrationWeight < MIN_CALIBRATION_WEIGHT_GRAMS) {
        ESP_LOGE(LOG_TAG, "Calibration failed: invalid weight %.2f", calibrationWeight);
        return;
    }

    RawSampleStats stats;
    if (!collectStableRawSamples(stats, CALIBRATION_SAMPLE_COUNT, CALIBRATION_DISCARD_COUNT, "calibration")) {
        ESP_LOGE(LOG_TAG, "Calibration failed: unstable or missing reads");
        return;
    }

    float rawSum = 0.0f;
    if (_cell1_present)
        rawSum += stats.average1 - _offset1;
    if (_cell2_present)
        rawSum += stats.average2 - _offset2;
    if (!std::isfinite(rawSum) || std::fabs(rawSum) < MIN_CALIBRATION_RAW_DELTA) {
        ESP_LOGE(LOG_TAG, "Calibration failed: raw delta %.3f too small", rawSum);
        return;
    }

    const float scaleFactor = rawSum / calibrationWeight;
    if (!isValidScaleFactor(scaleFactor)) {
        ESP_LOGE(LOG_TAG, "Calibration failed: invalid scale factor %.3f", scaleFactor);
        return;
    }

    _scale_factor = scaleFactor;
    _has_weight = false;

    _skip_readings = 3;
    ESP_LOGI(LOG_TAG, "Calibrated with combined factor: %.3f (weight=%.2f, reads=%d, rawDelta=%.3f)", _scale_factor,
             calibrationWeight, stats.reads, rawSum);
    _configuration_callback(_scale_factor);
}

[[noreturn]] void HardwareScale::loopTask(void *arg) {
    TickType_t lastWake = xTaskGetTickCount();
    auto *scale = static_cast<HardwareScale *>(arg);
    while (true) {
        scale->loop();
        xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SCALE_READ_INTERVAL_MS));
    }
}
