#pragma once

#include "SlalomDesigner.h"

static auto sd_SL90 = signal_processing::SlalomDesigner(
    signal_processing::SlalomDesigner::Constraint(M_PI / 2, 40, 45, 45));

static auto sd_SR90 = signal_processing::SlalomDesigner(
    signal_processing::SlalomDesigner::Constraint(-M_PI / 2, -40, 45, -45));
