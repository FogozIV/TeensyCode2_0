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
std::shared_ptr<CommandParser> commandParser;
bool connect = false;

#define SEND_DATA(client, data, len) client->write(data, len, 0x01)

void setup() {
    commandParser = std::make_shared<CommandParser>();
    commandParser->registerCommand("help", "", [](std::vector<CommandParser::Argument> args) {
        return "There is no help available yet";
    });
    commandParser->registerCommand("begin_calibration", "", [](std::vector<CommandParser::Argument> args) {
        robot->begin_calibration();
        return "Success";
    });
    commandParser->registerCommand("calibration_end_line", "d", [](std::vector<CommandParser::Argument> args) {
        robot->end_calibration_straight(args[0].asDouble());
        return "Success";
    });
    commandParser->registerCommand("calibration_end_rotation", "d", [](std::vector<CommandParser::Argument> args) {
        robot->end_calibration_angle(args[0].asDouble());
        return "Success";
    });

    qindesign::network::Ethernet.onLinkState([](bool state){
        if(state == true) {
            Serial.println("Link detected");
            Serial.println(qindesign::network::Ethernet.linkState());
            Serial.println(qindesign::network::Ethernet.localIP());
            connect = true;
        }
    });
    Serial.begin(115200);
    Serial.println("Hello World");
    robot = std::make_shared<ThetaAngleRobotImpl>(
        std::make_unique<QuadEncoderImpl>(0,1,1),
        std::make_unique<QuadEncoderImpl>(2,3,2),
        std::make_unique<DirPWMMotor>(-1,-1),
        std::make_unique<DirPWMMotor>(-1,-1),
        50,20);
    utils::network::initialiseNetwork();
    if (!qindesign::network::Ethernet.waitForLink(6000)) {
        Serial.println("No link detected");
    }
    client = std::make_shared<AsyncClient>();
    client->onPacket([](void*, AsyncClient*, struct pbuf *pb) {
        std::string str(std::string(static_cast<const char *>(pb->payload), static_cast<const char *>(pb->payload)+pb->len));
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
       Serial.println("Disconnected");
        connect = true;
    });
    client->onError([](void* arg, AsyncClient* client, err_t error) {
        Serial.println("Error");
        Serial.println(error);
    });
    //client->connect(IPAddress(192,168,1,68), 8089);
// write your initialization code here
}
uint8_t link_tentative =0;
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
    robot->update();
    //Serial.println(robot->getPosition());
    /*Serial.printf("Total distance: %f, total angle: %f, speed distance : %f, speed angle : %f\n",
                  robot->getTranslationalPosition(), robot->getZAxisRotation(), robot->getTranslationalSpeed(),
                  robot->getRotationalSpeed());
*/
}
