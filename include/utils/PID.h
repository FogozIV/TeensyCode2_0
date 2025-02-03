//
// Created by fogoz on 02/01/2025.
//

#ifndef PID_H
#define PID_H
#include "Arduino.h"
#include <chrono>
#include "ArduinoJson.h"

class PID {
    double kp, ki, kd;
    double last_error = 0.0f;
    double iTerm = 0.0f;
    double anti_windup;
    std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double>> previousTime = std::chrono::system_clock::now();
  public:
    /**
     * A PID class
     * @param kp the proportional term
     * @param ki the integral term
     * @param kd the derivative term
     * @param anti_windup the saturation value
     */
    PID(double kp=0, double ki=0, double kd=0, double anti_windup=2000);

    /**
     *
     * @param error the current error
     * @return the
     */
    double compute(double error);

    /**
     * Reset the values
     */
    void reset();

    void set(double kp=NAN, double ki=NAN, double kd=NAN, double anti_windup=NAN);

    double getKp() const;

    double getKi() const;

    double getKd() const;

    double getAntiWindup() const;
};

namespace ArduinoJson {
    template <>
    struct Converter<PID> {
        static bool deserialize(JsonVariantConst src, PID &pid) {
            if (!src.is<JsonObjectConst>()) return false;
            JsonObjectConst obj = src.as<JsonObjectConst>();

            if (!obj["kp"].is<double>() || !obj["ki"].is<double>() || !obj["kd"].is<double>() || !obj["anti_windup"].is<double>()) {
                return false;
            }

            pid.set(obj["kp"].as<double>(), obj["ki"].as<double>(), obj["kd"].as<double>(), obj["anti_windup"].as<double>());

            return true;
        }

        static JsonVariant serialize(const PID &pid, JsonVariant dst) {
            if (!dst.is<JsonObject>()) return dst;
            JsonObject obj = dst.as<JsonObject>();

            obj["kp"] = pid.getKp();
            obj["ki"] = pid.getKi();
            obj["kd"] = pid.getKd();
            obj["anti_windup"] = pid.getAntiWindup();

            return dst;
        }
        static bool checkJson(JsonVariantConst src){
            JsonObjectConst obj = src.as<JsonObjectConst>();
            if (!obj["kp"].is<double>() || !obj["ki"].is<double>() || !obj["kd"].is<double>() || !obj["anti_windup"].is<double>()) {
                return false;
            }
            return true;
        }
    };
} //

#endif //PID_H
