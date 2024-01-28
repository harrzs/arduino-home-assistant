#include "HALight.h"
#ifndef EX_ARDUINOHA_LIGHT

#include "../HAMqtt.h"
#include "../utils/HASerializer.h"
#include "../ArduinoHADefines.h"

const uint8_t HALight::RGBStringMaxLength = 3*3 + 2; // 3 characters per color + 2 commas
const uint8_t HALight::RGBWStringMaxLength = 4*3 + 3; // 3 characters per color + 3 commas

void HALight::RGBColor::fromBuffer(const uint8_t* data, const uint16_t length)
{
    if (length > RGBStringMaxLength) {
        return;
    }

    uint8_t firstCommaPos = 0;
    uint8_t secondCommaPos = 0;

    for (uint8_t i = 0; i < length; i++) {
        if (data[i] == ',') {
            if (firstCommaPos == 0) {
                firstCommaPos = i;
            } else if (secondCommaPos == 0) {
                secondCommaPos = i;
            }
        }
    }

    if (firstCommaPos == 0 || secondCommaPos == 0) {
        return;
    }
    
    const uint8_t redLen = firstCommaPos;
    const uint8_t greenLen = secondCommaPos - firstCommaPos - 1; // minus comma
    const uint8_t blueLen = length - redLen - greenLen - 2; // minus two commas

    const HANumeric& r = HANumeric::fromStr(data, redLen);
    const HANumeric& g = HANumeric::fromStr(&data[redLen + 1], greenLen);
    const HANumeric& b = HANumeric::fromStr(&data[redLen + greenLen + 2], blueLen);

    if (r.isUInt8() && g.isUInt8() && b.isUInt8()) {
        red = r.toUInt8();
        green = g.toUInt8();
        blue = b.toUInt8();
        isSet = true;
    }
}

void HALight::RGBWColor::fromBuffer(const uint8_t* data, const uint16_t length)
{
    if (length > RGBWStringMaxLength) {
        return;
    }

    uint8_t firstCommaPos = 0;
    uint8_t secondCommaPos = 0;
    uint8_t thirdCommaPos = 0;

    for (uint8_t i = 0; i < length; i++) {
        if (data[i] == ',') {
            if (firstCommaPos == 0) {
                firstCommaPos = i;
            } else if (secondCommaPos == 0) {
                secondCommaPos = i;
            } else if (thirdCommaPos == 0) {
                thirdCommaPos = i;
            }
        }
    }

    if (firstCommaPos == 0 || secondCommaPos == 0 || thirdCommaPos == 0) {
        return;
    }

    const uint8_t redLen = firstCommaPos;
    const uint8_t greenLen = secondCommaPos - firstCommaPos - 1; // minus comma
    const uint8_t blueLen = thirdCommaPos - secondCommaPos - 1; // minus two commas
    const uint8_t whiteLen = length - thirdCommaPos - 1; // minus two commas

    const HANumeric& r = HANumeric::fromStr(data, redLen);
    const HANumeric& g = HANumeric::fromStr(&data[redLen + 1], greenLen);
    const HANumeric& b = HANumeric::fromStr(&data[redLen + greenLen + 2], blueLen);
    const HANumeric& w = HANumeric::fromStr(&data[redLen + greenLen + blueLen + 3], whiteLen);

    if (r.isUInt8() && g.isUInt8() && b.isUInt8() && w.isUInt8()) {
        red = r.toUInt8();
        green = g.toUInt8();
        blue = b.toUInt8();
        white = w.toUInt8();
        isSet = true;
    }
}

HALight::HALight(const char* uniqueId, const uint8_t features) :
    HABaseDeviceType(AHATOFSTR(HAComponentLight), uniqueId),
    _features(features),
    _icon(nullptr),
    _retain(false),
    _optimistic(false),
    _brightnessScale(),
    _currentState(false),
    _currentBrightness(0),
    _minMireds(),
    _maxMireds(),
    _currentColorTemperature(0),
    _currentRGBColor(),
    _currentRGBWColor(),
    _stateCallback(nullptr),
    _brightnessCallback(nullptr),
    _colorTemperatureCallback(nullptr),
    _rgbColorCallback(nullptr),
    _rgbwColorCallback(nullptr)
{

}

bool HALight::setState(const bool state, const bool force)
{
    if (!force && state == _currentState) {
        return true;
    }

    if (publishState(state)) {
        _currentState = state;
        return true;
    }

    return false;
}

bool HALight::setBrightness(const uint8_t brightness, const bool force)
{
    if (!force && brightness == _currentBrightness) {
        return true;
    }

    if (publishBrightness(brightness)) {
        _currentBrightness = brightness;
        return true;
    }

    return false;
}

bool HALight::setColorTemperature(const uint16_t temperature, const bool force)
{
    if (!force && temperature == _currentColorTemperature) {
        return true;
    }

    if (publishColorTemperature(temperature)) {
        _currentColorTemperature = temperature;
        return true;
    }

    return false;
}

bool HALight::setRGBColor(const RGBColor& color, const bool force)
{
    if (!force && color == _currentRGBColor) {
        return true;
    }

    if (publishRGBColor(color)) {
        _currentRGBColor = color;
        return true;
    }

    return false;
}

bool HALight::setRGBWColor(const RGBWColor& color, const bool force)
{
    if (!force && color == _currentRGBWColor) {
        return true;
    }

    if (publishRGBWColor(color)) {
        _currentRGBWColor = color;
        return true;
    }

    return false;
}

void HALight::buildSerializer()
{
    if (_serializer || !uniqueId()) {
        return;
    }

    _serializer = new HASerializer(this, 18); // 18 - max properties nb
    _serializer->set(AHATOFSTR(HANameProperty), _name);
    _serializer->set(AHATOFSTR(HAUniqueIdProperty), _uniqueId);
    _serializer->set(AHATOFSTR(HAIconProperty), _icon);

    if (_retain) {
        _serializer->set(
            AHATOFSTR(HARetainProperty),
            &_retain,
            HASerializer::BoolPropertyType
        );
    }

    if (_optimistic) {
        _serializer->set(
            AHATOFSTR(HAOptimisticProperty),
            &_optimistic,
            HASerializer::BoolPropertyType
        );
    }

    if (_features & BrightnessFeature) {
        _serializer->topic(AHATOFSTR(HABrightnessStateTopic));
        _serializer->topic(AHATOFSTR(HABrightnessCommandTopic));

        if (_brightnessScale.isSet()) {
            _serializer->set(
                AHATOFSTR(HABrightnessScaleProperty),
                &_brightnessScale,
                HASerializer::NumberPropertyType
            );
        }
    }

    if (_features & ColorTemperatureFeature) {
        _serializer->topic(AHATOFSTR(HAColorTemperatureStateTopic));
        _serializer->topic(AHATOFSTR(HAColorTemperatureCommandTopic));

        if (_minMireds.isSet()) {
            _serializer->set(
                AHATOFSTR(HAMinMiredsProperty),
                &_minMireds,
                HASerializer::NumberPropertyType
            );
        }

        if (_maxMireds.isSet()) {
            _serializer->set(
                AHATOFSTR(HAMaxMiredsProperty),
                &_maxMireds,
                HASerializer::NumberPropertyType
            );
        }
    }

    if (_features & RGBFeature) {
        _serializer->topic(AHATOFSTR(HARGBCommandTopic));
        _serializer->topic(AHATOFSTR(HARGBStateTopic));
    }

    if (_features & RGBWFeature) {
        _serializer->topic(AHATOFSTR(HARGBWCommandTopic));
        _serializer->topic(AHATOFSTR(HARGBWStateTopic));
    }

    _serializer->set(HASerializer::WithDevice);
    _serializer->set(HASerializer::WithAvailability);
    _serializer->topic(AHATOFSTR(HAStateTopic));
    _serializer->topic(AHATOFSTR(HACommandTopic));
}

void HALight::onMqttConnected()
{
    if (!uniqueId()) {
        return;
    }

    publishConfig();
    publishAvailability();

    if (!_retain) {
        publishState(_currentState);
        publishBrightness(_currentBrightness);
        publishColorTemperature(_currentColorTemperature);
        publishRGBColor(_currentRGBColor);
        publishRGBWColor(_currentRGBWColor);
    }

    subscribeTopic(uniqueId(), AHATOFSTR(HACommandTopic));

    if (_features & BrightnessFeature) {
        subscribeTopic(uniqueId(), AHATOFSTR(HABrightnessCommandTopic));
    }

    if (_features & ColorTemperatureFeature) {
        subscribeTopic(uniqueId(), AHATOFSTR(HAColorTemperatureCommandTopic));
    }

    if (_features & RGBFeature) {
        subscribeTopic(uniqueId(), AHATOFSTR(HARGBCommandTopic));
    }

    if (_features & RGBWFeature) {
        subscribeTopic(uniqueId(), AHATOFSTR(HARGBWCommandTopic));
    }
}

void HALight::onMqttMessage(
    const char* topic,
    const uint8_t* payload,
    const uint16_t length
)
{
    if (HASerializer::compareDataTopics(
        topic,
        uniqueId(),
        AHATOFSTR(HACommandTopic)
    )) {
        handleStateCommand(payload, length);
    } else if (HASerializer::compareDataTopics(
        topic,
        uniqueId(),
        AHATOFSTR(HABrightnessCommandTopic)
    )) {
        handleBrightnessCommand(payload, length);
    } else if (HASerializer::compareDataTopics(
        topic,
        uniqueId(),
        AHATOFSTR(HAColorTemperatureCommandTopic)
    )) {
        handleColorTemperatureCommand(payload, length);
    } else if (
        HASerializer::compareDataTopics(
            topic,
            uniqueId(),
            AHATOFSTR(HARGBCommandTopic)
        )
    ) {
        handleRGBCommand(payload, length);
    } else if (
        HASerializer::compareDataTopics(
            topic,
            uniqueId(),
            AHATOFSTR(HARGBWCommandTopic)
        )
    ) {
        handleRGBWCommand(payload, length);
    }
}

bool HALight::publishState(const bool state)
{
    return publishOnDataTopic(
        AHATOFSTR(HAStateTopic),
        AHATOFSTR(state ? HAStateOn : HAStateOff),
        true
    );
}

bool HALight::publishBrightness(const uint8_t brightness)
{
    if (!(_features & BrightnessFeature)) {
        return false;
    }

    char str[3 + 1] = {0}; // uint8_t digits with null terminator
    HANumeric(brightness, 0).toStr(str);

    return publishOnDataTopic(AHATOFSTR(HABrightnessStateTopic), str, true);
}

bool HALight::publishColorTemperature(const uint16_t temperature)
{
    if (!(_features & ColorTemperatureFeature)) {
        return false;
    }

    char str[5 + 1] = {0}; // uint16_t digits with null terminator
    HANumeric(temperature, 0).toStr(str);

    return publishOnDataTopic(AHATOFSTR(HAColorTemperatureStateTopic), str, true);
}

bool HALight::publishRGBColor(const RGBColor& color)
{
    if (!(_features & RGBFeature) || !color.isSet) {
        return false;
    }

    char str[RGBStringMaxLength + 1];
    sprintf(str, AHAFROMFSTR(F("%d,%d,%d")), color.red, color.green, color.blue);

    return publishOnDataTopic(AHATOFSTR(HARGBStateTopic), str, true);
}

bool HALight::publishRGBWColor(const RGBWColor& color)
{
    if (!(_features & RGBWFeature) || !color.isSet) {
        return false;
    }

    char str[RGBWStringMaxLength + 1];
    sprintf(str, AHAFROMFSTR(F("%d,%d,%d,%d")), color.red, color.green, color.blue, color.white);

    return publishOnDataTopic(AHATOFSTR(HARGBWStateTopic), str, true);
}

void HALight::handleStateCommand(const uint8_t* cmd, const uint16_t length)
{
    (void)cmd;

    if (!_stateCallback) {
        return;
    }

    bool state = length == strlen_P(HAStateOn);
    _stateCallback(state, this);
}

void HALight::handleBrightnessCommand(const uint8_t* cmd, const uint16_t length)
{
    if (!_brightnessCallback) {
        return;
    }

    const HANumeric& number = HANumeric::fromStr(cmd, length);
    if (number.isUInt8()) {
        _brightnessCallback(number.toUInt8(), this);
    }
}

void HALight::handleColorTemperatureCommand(
    const uint8_t* cmd,
    const uint16_t length
)
{
    if (!_colorTemperatureCallback) {
        return;
    }

    const HANumeric& number = HANumeric::fromStr(cmd, length);
    if (number.isUInt16()) {
        _colorTemperatureCallback(number.toUInt16(), this);
    }
}

void HALight::handleRGBCommand(const uint8_t* cmd, const uint16_t length)
{
    if (!_rgbColorCallback) {
        return;
    }

    RGBColor color;
    color.fromBuffer(cmd, length);

    if (color.isSet) {
        _rgbColorCallback(color, this);
    }
}

void HALight::handleRGBWCommand(const uint8_t* cmd, const uint16_t length)
{
    if (!_rgbwColorCallback) {
        return;
    }

    RGBWColor color;
    color.fromBuffer(cmd, length);

    if (color.isSet) {
        _rgbwColorCallback(color, this);
    }
}

#endif
