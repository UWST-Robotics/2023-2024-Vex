#pragma once
#include "renderer.hpp"
#include "../utils/runnable.hpp"

namespace devils
{
    /**
     * A display that manages multiple sub-renderers.
     */
    class Display : public Runnable
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
            for (Renderer *renderer : this->renderers)
                renderer->create(rootObject);
        }

        /**
         * Destroys the display.
         */
        ~Display()
        {
            lv_obj_del(rootObject);

            for (Renderer *renderer : renderers)
                delete renderer;
        }

        /**
         * Updates all renderers.
         */
        void update() override
        {
            for (Renderer *renderer : renderers)
                renderer->update();

            // Uncomment to force a redraw every frame
            lv_obj_invalidate(rootObject);
        }

        /**
         * Adds a renderer to the display.
         * @param renderer The renderer to add.
         */
        void addRenderer(Renderer *renderer)
        {
            renderers.push_back(renderer);
            renderer->create(rootObject);
        }

        /**
         * Gets a renderer of the given type.
         * @return The renderer or nullptr if not found.
         * @tparam T The type of renderer to get.
         */
        template <typename T>
        T *getRenderer()
        {
            for (Renderer *renderer : renderers)
            {
                T *casted = dynamic_cast<T *>(renderer);
                if (casted != nullptr)
                    return casted;
            }

            return nullptr;
        }

    private:
        lv_obj_t *rootObject;
        std::vector<Renderer *> renderers;
    };
}