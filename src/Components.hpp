#pragma once
#include "plugin.hpp"
#include "ui.hpp"

struct BananutBlackPort : app::SvgPort {
    BananutBlackPort() {
        setSvg(trySvg("res/blacknut.svg", "res/ComponentLibrary/PJ301M.svg"));
    }
};

struct BananutRedPort : app::SvgPort {
    BananutRedPort() {
        setSvg(trySvg("res/rednut.svg", "res/ComponentLibrary/PJ301M.svg"));
    }
};
