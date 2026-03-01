#include "path_motion_planning/motion/controller.h"

int main(){
    path_motion_planning::motion::Controller::Ptr controller = std::make_shared<path_motion_planning::motion::Controller>();
    auto& pt_cfg = controller->trackerConfig();
    pt_cfg.max_speed = 10;

    return 0;
}