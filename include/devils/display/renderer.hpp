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
        /**
         * Called when the renderer is initialized with a display root.
         * @param root The root canvas object to append to.
         */
        virtual void create(lv_obj_t *root) = 0;

        /**
         * Called when the renderer should refresh its display.
         */
        virtual void update() {}
    };
}