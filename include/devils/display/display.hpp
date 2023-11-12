#pragma once
#include "../odom/pose.hpp"
#include "renderer.hpp"

namespace devils
{
    /**
     * A display that manages multiple sub-renderers.
     */
    class Display
    {
    public:
        /**
         * Creates a new display with the given renderers.
         * @param renderers The renderers to manage.
         */
        Display(std::initializer_list<Renderer *> renderers)
            : renderers(renderers)
        {
            // Create Root Object
            static lv_obj_t *rootObject = lv_obj_create(NULL, NULL);
            lv_scr_load(rootObject);

            // Init Renderers
            for (Renderer *renderer : renderers)
                renderer->create(rootObject);
        }

        /**
         * Updates all renderers.
         */
        void update()
        {
            for (Renderer *renderer : renderers)
                renderer->update();
        }

    private:
        std::vector<Renderer *> renderers;
    };
}