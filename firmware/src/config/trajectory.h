#pragma once

#include "SlalomDesigner.h"

using namespace ctrl;

static auto sd_SL90 =
    SlalomDesigner(SlalomDesigner::Shape(Position(45, 45, M_PI / 2), 40));
static auto sd_SR90 =
    SlalomDesigner(SlalomDesigner::Shape(Position(45, -45, -M_PI / 2), -40));

static auto sd_FL45 =
    SlalomDesigner(SlalomDesigner::Shape(Position(90, 45, M_PI / 4), 30));

static auto sd_FL90 =
    SlalomDesigner(SlalomDesigner::Shape(Position(90, 90, M_PI / 2), 70));

static auto sd_FL135 =
    SlalomDesigner(SlalomDesigner::Shape(Position(45, 90, M_PI * 3 / 4), 80));

static auto sd_FL180 =
    SlalomDesigner(SlalomDesigner::Shape(Position(0, 90, M_PI), 90, 30));

static auto sd_FLV90 = SlalomDesigner(SlalomDesigner::Shape(
    Position(45 * std::sqrt(2), 45 * std::sqrt(2), M_PI / 2), 42));
