#pragma once
#include "display/lvgl.h"

namespace devils
{
    /**
     * Represents a renderer. Uses LVGL to create an object to render to the screen.
     */
    struct Renderer
    {
    public:
        virtual void create(lv_obj_t *root) {}
        virtual void update() {}
    };
}