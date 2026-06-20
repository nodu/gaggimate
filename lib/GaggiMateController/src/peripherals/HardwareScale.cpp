#include "HardwareScale.h"
#include <Arduino.h>

#define MAX_SCALE_GRAMS 1500.0f
#define MAX_WAIT_READ_MS 250
#define MAX_STARTUP_WAIT_MS 1200

HardwareScale::HardwareScale(uint8_t data_pin1, uint8_t data_pin2, uint8_t clock_pin,
                             const scale_reading_callback_t &reading_callback,
                             const scale_factor_callback_t &config_callback)
    : _data_pin1(data_pin1), _data_pin2(data_pin2), _clock_pin(clock_pin),
      _reading_callback(reading_callback), _configuration_callback(config_callback) {}

void HardwareScale::setup() {
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
    ESP_LOGI(LOG_TAG, "Initializing hardware scale on DATA1: GPIO%d, DATA2: GPIO%d, CLOCK: GPIO%d", _data_pin1, _data_pin2, _clock_pin);

    // HX711 needs ~400ms to wake from power-down after SCK goes LOW
    delay(1000);

    long start = millis();
    while (!isReady() && (millis() - start) < MAX_STARTUP_WAIT_MS) {
        delay(10);
    }
    if (!isReady()) {
        ESP_LOGE(LOG_TAG, "HX711 not ready after %ld ms, aborting", millis() - start);
        is_initialized = false;
        return;
    }

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

    tare();
    is_initialized = true;
    ESP_LOGI(LOG_TAG, "Hardware scale initialized successfully");

    _configuration_callback(_scale_factor);

    xTaskCreate(loopTask, "HardwareScale::loop", configMINIMAL_STACK_SIZE * 4, this, 1, &taskHandle);
}

// Uses || so init/loop work when at least one HX711 is present.
// tare() and calibrate() require both cells ready (&&) for accuracy.
bool HardwareScale::isReady() { return digitalRead(_data_pin1) == LOW || digitalRead(_data_pin2) == LOW; }

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

float HardwareScale::rawToWeight(long raw, float offset) const {
    return std::clamp((static_cast<float>(raw) - offset) / _scale_factor, -MAX_SCALE_GRAMS, MAX_SCALE_GRAMS);
}

float HardwareScale::smooth(float previous, float current) {
    return std::clamp(0.5f * current + 0.5f * previous, -MAX_SCALE_GRAMS, MAX_SCALE_GRAMS);
}

void HardwareScale::loop() {
    // Handle tare/calibrate requests on this task to avoid race conditions
    if (_tare_requested) {
        _tare_requested = false;
        doTare();
        return;
    }
    if (_calibrate_requested) {
        _calibrate_requested = false;
        doCalibrate();
        return;
    }

    while (!isReady()) {
        vTaskDelay(1);
    }

    auto raw = readRaw();

    // Discard if a request arrived during the read
    if (_tare_requested || _calibrate_requested) {
        return;
    }

    if (_skip_readings > 0) {
        _skip_readings--;
        return;
    }

    if (_scale_factor == 0.0f) {
        _reading_callback(0.0f, 0.0f, 0.0f);
        return;
    }

    float v1 = rawToWeight(raw.value1, _offset1);
    float v2 = rawToWeight(raw.value2, _offset2);
    float combined = std::round((v1 + v2) * 100.0f) / 100.0f;
    _weight = smooth(_weight, combined);
    ESP_LOGD(LOG_TAG, "v1: %.2f, v2: %.2f, Smoothed: %.2f", v1, v2, _weight);

    // Final check — don't send if tare/calibrate was requested
    if (_tare_requested || _calibrate_requested) {
        return;
    }

    _reading_callback(_weight, v1, v2);
}

void HardwareScale::setScaleFactor(float scale_factor) {
    _scale_factor = scale_factor;
    ESP_LOGI(LOG_TAG, "Set scale factor: %.3f", _scale_factor);
}

void HardwareScale::tare() { _tare_requested = true; }

void HardwareScale::calibrate(float calibrationWeight) {
    _calibrate_weight = calibrationWeight;
    _calibrate_requested = true;
}

bool HardwareScale::waitForBothReady() {
    long start = millis();
    while (!(digitalRead(_data_pin1) == LOW && digitalRead(_data_pin2) == LOW) && (millis() - start) < MAX_WAIT_READ_MS) {
        delay(10);
    }
    return digitalRead(_data_pin1) == LOW && digitalRead(_data_pin2) == LOW;
}

void HardwareScale::doTare() {
    long sum1 = 0, sum2 = 0;
    int reads = 0;
    for (int i = 0; i < 5; i++) {
        if (!waitForBothReady()) {
            ESP_LOGE(LOG_TAG, "HX711 not ready for tare read %d, skipping", i);
            continue;
        }
        auto raw = readRaw();
        sum1 += raw.value1;
        sum2 += raw.value2;
        reads++;
    }

    if (reads == 0) {
        ESP_LOGE(LOG_TAG, "Tare failed: no successful reads");
        return;
    }

    _offset1 = static_cast<float>(sum1) / reads;
    _offset2 = static_cast<float>(sum2) / reads;
    _weight = 0.0f;
    _skip_readings = 3;
    _reading_callback(0.0f, 0.0f, 0.0f);
    ESP_LOGI(LOG_TAG, "Tared scale offsets: %.3f, %.3f (%d reads)", _offset1, _offset2, reads);
}

void HardwareScale::doCalibrate() {
    long sum1 = 0, sum2 = 0;
    int successfulReads = 0;
    for (int i = 0; i < 10; i++) {
        if (!waitForBothReady()) {
            ESP_LOGE(LOG_TAG, "HX711 not ready during calibration read %d, skipping", i);
            continue;
        }
        auto raw = readRaw();
        sum1 += raw.value1;
        sum2 += raw.value2;
        successfulReads++;
    }

    if (successfulReads == 0 || _calibrate_weight == 0.0f) {
        ESP_LOGE(LOG_TAG, "Calibration failed: %d successful reads, weight=%.2f", successfulReads, _calibrate_weight);
        return;
    }

    float avg1 = static_cast<float>(sum1) / successfulReads;
    float avg2 = static_cast<float>(sum2) / successfulReads;
    _scale_factor = ((avg1 - _offset1) + (avg2 - _offset2)) / _calibrate_weight;

    _skip_readings = 3;
    ESP_LOGI(LOG_TAG, "Calibrated with combined factor: %.3f (weight=%.2f, reads=%d)", _scale_factor, _calibrate_weight, successfulReads);
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
