#include "HardwareScalePlugin.h"
#include <display/core/Controller.h>

HardwareScalePlugin HardwareScales;

HardwareScalePlugin::HardwareScalePlugin() = default;

void HardwareScalePlugin::setup(Controller *controller, PluginManager *pluginManager) {
    this->controller = controller;

    pluginManager->on("controller:ready", [this](Event const &) {
        _isAvailable = this->controller->getSystemInfo().capabilities.hwScale;
        _scaleFactor = this->controller->getSettings().getScaleFactor();

        ESP_LOGI(LOG_TAG, "Hardware scale available: %s", _isAvailable ? "true" : "false");

        if (_scaleFactor != 0.0f && _scaleFactor != 1.0f) {
            this->controller->getClientController()->sendScaleCalibration(_scaleFactor);
            delay(50);
        }
        this->controller->getClientController()->sendScaleTare();

        this->controller->setVolumetricOverride(_isAvailable);
    });

    pluginManager->on("controller:brew:start", [this](Event const &) { onProcessStart(); });

    pluginManager->on("controller:scale:measurement", [this](Event const &event) {
        _weight1 = event.getFloat("w1");
        _weight2 = event.getFloat("w2");
        float value = event.getFloat("value");
        this->onMeasurement(value);
    });

    pluginManager->on("controller:scale:cal_update", [this](Event const &event) {
        _scaleFactor = event.getFloat("scaleFactor1");
        this->controller->getSettings().setScaleFactor(_scaleFactor);
    });
}

void HardwareScalePlugin::tare() {
    if (_isAvailable) {
        ESP_LOGI(LOG_TAG, "Taring hardware scale");
        controller->getClientController()->sendScaleTare();
    }
}

void HardwareScalePlugin::calibrate(float calibrationWeight) {
    if (_isAvailable) {
        ESP_LOGI(LOG_TAG, "Calibrating hardware scale with weight %.2f", calibrationWeight);
        controller->getClientController()->sendCalibrateScale(calibrationWeight);
    }
}

void HardwareScalePlugin::onProcessStart() {
    if (_isAvailable) {
        ESP_LOGI(LOG_TAG, "Starting tare process for hardware scale");
        controller->getClientController()->sendScaleTare();
        delay(200);
    }
}

void HardwareScalePlugin::onMeasurement(float value) {
    this->_lastMeasurement = value;
    controller->onVolumetricMeasurement(value, VolumetricMeasurementSource::BLUETOOTH);
}
