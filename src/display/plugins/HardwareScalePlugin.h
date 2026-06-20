#ifndef HARDWARESCALEPLUGIN_H
#define HARDWARESCALEPLUGIN_H
#include "../core/Plugin.h"
#include <stdint.h>

class HardwareScalePlugin : public Plugin {
  public:
    HardwareScalePlugin();

    void setup(Controller *controller, PluginManager *pluginManager) override;
    void loop() override {};
    void tare();
    void calibrate(float calibrationWeight);

    bool isConnected() const { return _isAvailable; }
    float getWeight() const { return _lastMeasurement; }
    float getWeight1() const { return _weight1; }
    float getWeight2() const { return _weight2; }

  private:
    void onMeasurement(float value);
    void onProcessStart();

    const char *LOG_TAG = "HardwareScalePlugin";
    bool _isAvailable;
    float _lastMeasurement = 0.0f;
    float _weight1 = 0.0f, _weight2 = 0.0f;
    float _scaleFactor = 1.0f;

    Controller *controller = nullptr;
};

extern HardwareScalePlugin HardwareScales;

#endif // HARDWARESCALEPLUGIN_H
