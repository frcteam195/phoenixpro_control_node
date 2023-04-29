#pragma once
#include "parameter_helper.hpp"
#include "rclcpp/rclcpp.hpp"

//TODO: In the future, auto generate this from the yaml file
DEFINE_PARAMS(Parameters,
    (canivore_name) ,
    (rclcpp::PARAMETER_STRING)
)