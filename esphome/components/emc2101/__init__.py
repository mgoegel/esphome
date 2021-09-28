from esphome.components import fan
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    CONF_INDEX,
    CONF_TEMPERATURE,
    CONF_SPEED,
    # CONF_
    DEVICE_CLASS_TEMPERATURE,
    ICON_EMPTY,
    UNIT_CELSIUS,
)

"""
SMBus  Fan  Control  with  1°C  Accurate  Temperature Monitoring

* Embedded  Application  Fan  Drive
* PWM  Controller  +  Temp  Sensor

* Get external temp
* Get internal temp
* Set duty cycle
* Get duty cycle
* Set min/max RPM
* Get RPM
* Set invert fan speed
* Set temp LUT (temp to duty cycle in %)
* Get temp LUT
* En/disable LUT
* Set LUT hysteresis (switch back to previous level after -x degres)
"""

DEPENDENCIES = ["i2c"]

emc2101_ns = cg.esphome_ns.namespace("emc2101")
EMC2101Component = emc2101_ns.class_(
    "EMC2101Component", cg.PollingComponent, i2c.I2CDevice
)

CONF_DUTY = "duty"
CONF_EXTERNAL_TEMP_SENSOR = "external_temp_sensor"
CONF_MIN_RPM = "min_rpm"
CONF_MAX_RPM = "max_rpm"
CONF_INVERT_SPEED = "invert_speed"
CONF_LUT = "temp_lut"
CONF_LUT_HYSTERESIS = "hysteresis_lut"
CONF_LUT_ENABLE = "enable_lut"

CONF_EMC2101 = "emc2101"

LUT_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_EMC2101): cv.use_id(EMC2101Component),
        cv.Unique(CONF_INDEX, required=True): cv.uint8_t,
        cv.Required(CONF_TEMPERATURE): cv.uint8_t,
        cv.Required(CONF_SPEED): cv.uint16_t,
    }
)

CONFIG_SCHEMA = cv.All(
    # cv.Schema(
    fan.FAN_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(EMC2101Component),
            # missing further configuration here
            cv.Optional(CONF_DUTY): cv.uint8_t,
            cv.Optional(CONF_EXTERNAL_TEMP_SENSOR, default=False): cv.boolean,
            cv.Optional(CONF_MIN_RPM, default=0): cv.uint16_t,
            cv.Optional(CONF_MAX_RPM, default=2000): cv.uint16_t,
            cv.Optional(CONF_INVERT_SPEED, default=False): cv.boolean,
            cv.Optional(CONF_LUT): cv.Schema(LUT_SCHEMA),
            cv.Optional(CONF_LUT_HYSTERESIS): cv.uint8_t,
            cv.Optional(CONF_LUT_ENABLE): cv.boolean,
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                UNIT_CELSIUS, ICON_EMPTY, 1, DEVICE_CLASS_TEMPERATURE
            ),
        }
    )
    # include component schema
    .extend(cv.polling_component_schema("60s"))
    # include I2C schema
    .extend(i2c.i2c_device_schema(0x4C))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature(sens))
