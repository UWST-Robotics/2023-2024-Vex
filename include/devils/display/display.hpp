#pragma once
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
            rootObject = lv_obj_create(NULL, NULL);
            lv_scr_load(rootObject);

            // Init Renderers
            for (Renderer *renderer : renderers)
                renderer->create(rootObject);
        }

        /**
         * Destroys the display.
         */
        ~Display()
        {
            lv_obj_del(rootObject);
        }

        /**
         * Updates all renderers.
         */
        void update()
        {
            for (Renderer *renderer : renderers)
                renderer->update();

            // Uncomment to force a redraw every frame
            // lv_obj_invalidate(rootObject);
        }

    private:
        lv_obj_t *rootObject;
        std::vector<Renderer *> renderers;
    };
}