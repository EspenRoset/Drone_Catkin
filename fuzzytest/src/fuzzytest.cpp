#include <fl/Headers.h>
#include <ros/ros.h>

int main(int argc, char* argv[]){
    using namespace fl;
    Engine* engine = FllImporter().fromFile("/home/espros/catkin_test/src/fuzzytest/src/ObstacleAvoidance.fll");

    std::string status;
    if (not engine->isReady(&status))
        throw Exception("[engine error] engine is not ready:n" + status, FL_AT);

    InputVariable* ScreenLS = engine->getInputVariable("ScreenLS");
    InputVariable* ScreenMS = engine->getInputVariable("ScreenMS");
    InputVariable* ScreenRS = engine->getInputVariable("ScreenRS");
    OutputVariable* steer = engine->getOutputVariable("Roll");

    for (int i = 0; i <= 50; ++i){
        scalar location = ScreenLS->getMinimum() + i * (ScreenLS->range() / 50);
        ScreenLS->setValue(location);
        engine->process();
        FL_LOG("obstacle.input = " << Op::str(location) << 
            " => " << "steer.output = " << Op::str(steer->getValue()));
    }
}