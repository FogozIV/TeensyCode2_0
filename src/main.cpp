#include <Arduino.h>
#include <memory>
#include <utils/NetworkInitialiser.h>
#include "Teensy41_AsyncTCP.h"
#include "vector"
#include "motors/DirPWMMotor.h"
#include "encoders/QuadEncoderImpl.h"
#include "utils/SpeedEstimator.h"
#include "robot/ThetaAngleRobotImpl.h"
std::shared_ptr<AbstractRobot> robot;
std::shared_ptr<AsyncClient> client;
#include "CommandParser.h"
#include "controllers/AbstractController.h"
#include "utils/PID.h"
#include "utils/QuadRamp.h"
#include "controllers/PIDController.h"
#include "controllers/PIDControllerDeterministicRamp.h"
#include <TeensyThreads.h>

std::shared_ptr<CommandParser> commandParser;
bool connect = false;
//33 34 35 36
#define MOT_1_PWM 33
#define MOT_1_DIR 34
#define MOT_2_DIR 35
#define MOT_2_PWM 36



#define SEND_DATA(client, data, len) client->write(data, len, 0x01)
using namespace std::chrono;
void handle(){
    auto data_point = std::chrono::steady_clock::now();
    Serial.println("thread created");
    while(true){
        if(std::chrono::steady_clock::now() - data_point < 5ms){
            continue;
        }
        Serial.println("Updating");
        data_point = std::chrono::steady_clock::now();
        robot->update();
    }
}

std::shared_ptr<std::thread> t;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    commandParser = std::make_shared<CommandParser>();
    commandParser->registerCommand("help", "", [](std::vector<CommandParser::Argument> args) {
        return "There is no help available yet";
    });
    commandParser->registerCommand("begin_calibration", "", [](std::vector<CommandParser::Argument> args) {
        robot->reset_robot_to({});
        robot->begin_calibration();
        return "Success";
    });
    commandParser->registerCommand("calibration_end_line", "d", [](std::vector<CommandParser::Argument> args) {
        robot->reset_robot_to({});
        robot->end_calibration_straight(args[0].asDouble());
        return "Success";
    });
    commandParser->registerCommand("calibration_end_rotation", "d", [](std::vector<CommandParser::Argument> args) {
        robot->reset_robot_to({});
        robot->end_calibration_angle(args[0].asDouble());
        return "Success";
    });
    commandParser->registerCommand("find_motor_calibration","", [](std::vector<CommandParser::Argument> args) {
        robot->reset_robot_to({});
        robot->find_motor_calibration();
        return "Success";
    });

    qindesign::network::Ethernet.onLinkState([](bool state){
        if(state) {
            connect = true;
        }
    });
    Serial.begin(115200);
    Serial.println("Hello World");
    std::shared_ptr<PID> pidDistance = std::make_shared<PID>(20,8000,0.3, 400, 1000);
    std::shared_ptr<PID> pidAngle= std::make_shared<PID>(800,5000,0.1, 400, 1000);
    /*
    robot = std::make_shared<ThetaAngleRobotImpl>(
        std::make_unique<QuadEncoderImpl>(0,1,1),
        std::make_unique<QuadEncoderImpl>(2,3,2),
        std::make_unique<DirPWMMotor>(MOT_1_PWM,MOT_1_DIR),
        std::make_unique<DirPWMMotor>(MOT_2_PWM,MOT_2_DIR),
        std::make_unique<PIDController>(
                pidDistance,
                pidAngle,
                std::make_shared<QuadRamp>(600,600),
                std::make_shared<QuadRamp>(M_PI_2, M_PI_2)),
        50,20);*/
    robot = std::make_shared<ThetaAngleRobotImpl>(
            std::make_unique<QuadEncoderImpl>(0,1,1),
            std::make_unique<QuadEncoderImpl>(2,3,2),
            std::make_unique<DirPWMMotor>(MOT_1_PWM,MOT_1_DIR),
            std::make_unique<DirPWMMotor>(MOT_2_PWM,MOT_2_DIR),
            std::make_unique<PIDControllerDeterministicRamp>(
                    pidDistance,
                    pidAngle,
                    std::make_shared<Ramp>(600,600, 600)),
            50,20);

    utils::network::initialiseNetwork();
    if (!qindesign::network::Ethernet.waitForLink(6000)) {
        Serial.println("No link detected");
    }
    client = std::make_shared<AsyncClient>();
    client->onPacket([](void*, AsyncClient*, struct pbuf *pb) {
        std::string str(static_cast<const char *>(pb->payload), static_cast<const char *>(pb->payload)+pb->len);
        std::string result;
        commandParser->processCommand(str, result);
        result += "\n";
        Serial.println(SEND_DATA(client, result.data(), result.length()));
        Serial.print(result.data());
    }, nullptr);
    client->onConnect([](void* arg, AsyncClient* client) {
        Serial.println("Connected");
        client->write("Hello World\n");
        connect = false;
    });
    client->onDisconnect([](void* arg, AsyncClient* client) {
        connect = true;
    });
    client->onError([](void* arg, AsyncClient* client, err_t error) {
        Serial.println("Error");
        Serial.println(error);
    });
    robot->reset_robot_to({0,0,0});
    robot->setTargetPos({500,0,0});
    threads.setDefaultStackSize(10000);
    t = std::make_shared<std::thread>(handle);
    t->detach();
    threads.setDefaultTimeSlice(10000);
    //client->connect(IPAddress(192,168,1,68), 8089);
// write your initialization code here

}
uint8_t link_tentative =0;
bool first = true;
void loop() {
    if(connect) {
        if (link_tentative < 5&& qindesign::network::Ethernet.waitForLocalIP(6000)) {
            delay(100);
            Serial.println("Try to Connect");
            client->connect(IPAddress(192,168,1,68), 8089);
            delay(100);
            connect = false;
            link_tentative = 0;
        }else {
            link_tentative++;
        }
    }
    /*Serial.printf("Total distance: %f, total angle: %f, speed distance : %f, speed angle : %f\n",
                  robot->getTranslationalPosition(), robot->getZAxisRotation(), robot->getTranslationalSpeed(),
                  robot->getRotationalSpeed());
*/
}
