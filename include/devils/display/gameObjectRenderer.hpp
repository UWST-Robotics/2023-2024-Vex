#pragma once
#include "renderer.hpp"
#include "../utils/logger.hpp"
#include "../geometry/units.hpp"
#include "../geometry/polygon.hpp"
#include "../control/autoController.hpp"
#include "../gameobject/gameObjectManager.hpp"
#include "../display/displayUtils.hpp"
#include <cmath>
#include <string>
#include <map>

namespace devils
{
    /**
     * Renderer that displays the field as a grid.
     */
    class GameObjectRenderer : public Renderer
    {
    public:
        /**
         * Creates a new GameObjectRenderer
         * @param gameObjects The game objects to render
         */
        GameObjectRenderer(GameObjectManager &gameObjectManager) : gameObjectManager(gameObjectManager)
        {
        }

        ~GameObjectRenderer()
        {
            for (auto objectRenderer : rendererPool)
                lv_obj_del(objectRenderer);
        }

        void create(lv_obj_t *root) override
        {
            rootObject = root;

            // Define Styles
            lv_style_copy(&gameObjectStyle, &lv_style_plain);
            gameObjectStyle.body.main_color = LV_COLOR_MAKE(0x00, 0xff, 0x00);
            gameObjectStyle.body.grad_color = LV_COLOR_MAKE(0x00, 0xff, 0x00);
            gameObjectStyle.body.radius = 1;

            lv_style_copy(&gameObjectDisabledStyle, &lv_style_plain);
            gameObjectDisabledStyle.body.main_color = LV_COLOR_MAKE(0x00, 0x55, 0x00);
            gameObjectDisabledStyle.body.grad_color = LV_COLOR_MAKE(0x00, 0x55, 0x00);
            gameObjectDisabledStyle.body.radius = 1;

            // Create game object renderers
            auto gameObjects = gameObjectManager.getGameObjects();
            for (auto gameObject : *gameObjects)
            {
                auto renderer = _addRendererToPool();
                _renderObject(renderer, gameObject);
            }
        }

        void update() override
        {
            // Get GameObject List
            auto gameObjects = gameObjectManager.getGameObjects();

            // Add additional renderers if needed
            while (rendererPool.size() < gameObjects->size())
                _addRendererToPool();

            // Render Game Objects
            for (int i = 0; i < gameObjects->size(); i++)
                _renderObject(rendererPool[i], gameObjects->at(i));

            // Hide unused renderers
            for (int i = gameObjects->size(); i < rendererPool.size(); i++)
                lv_obj_set_hidden(rendererPool[i], true);

            // Get Last Input
            pros::screen_touch_status_s_t touchStatus = pros::screen::touch_status();
            if (touchStatus.touch_status != pros::E_TOUCH_PRESSED)
                return;

            // Get Touch Position
            double touchX = touchStatus.x - (DisplayUtils::DISPLAY_WIDTH / 2);
            double touchY = touchStatus.y - (DisplayUtils::DISPLAY_HEIGHT / 2);
            GameObject newObj(
                touchX / DisplayUtils::PX_PER_IN,
                touchY / DisplayUtils::PX_PER_IN,
                0.0);

            // Add the new object
            gameObjectManager.add(newObj);

            Logger::info("Added GO: " + newObj.toString());
        }

        /**
         * Highlights all gameobjects within the area
         * @param polygon The area to highlight
         */
        void useArea(Polygon *polygon)
        {
            highlightArea = polygon;
        }

        /**
         * Adds a new object renderer to the pool
         * @return The new object renderer
         */
        lv_obj_t *_addRendererToPool()
        {
            // Create Renderer
            lv_obj_t *objectRenderer = lv_obj_create(this->rootObject, NULL);
            {
                lv_obj_set_size(objectRenderer, OBJ_WIDTH, OBJ_HEIGHT);
                lv_obj_set_style(objectRenderer, &gameObjectDisabledStyle);

                // Add to Pool
                rendererPool.push_back(objectRenderer);
            }

            return objectRenderer;
        }

        /**
         * Renders the game object using a renderer from the pool
         * @param renderer The renderer to use
         * @param gameObject The game object to render
         */
        void _renderObject(lv_obj_t *renderer, GameObject &gameObject)
        {
            // Show Renderer
            lv_obj_set_hidden(renderer, false);

            // Set Position
            int x = gameObject.x * DisplayUtils::PX_PER_IN + OFFSET_X;
            int y = gameObject.y * DisplayUtils::PX_PER_IN + OFFSET_Y;
            lv_obj_set_pos(renderer, x, y);

            // Set Style
            if (highlightArea != nullptr && highlightArea->contains(gameObject))
                lv_obj_set_style(renderer, &gameObjectStyle);
            else
                lv_obj_set_style(renderer, &gameObjectDisabledStyle);
        }

    private:
        static constexpr int OBJ_WIDTH = 14;  // px
        static constexpr int OBJ_HEIGHT = 14; // px
        static constexpr int OFFSET_X = (DisplayUtils::DISPLAY_WIDTH - OBJ_WIDTH) / 2;
        static constexpr int OFFSET_Y = (DisplayUtils::DISPLAY_HEIGHT - OBJ_HEIGHT) / 2;

        lv_style_t gameObjectStyle;
        lv_style_t gameObjectDisabledStyle;

        lv_obj_t *rootObject = nullptr;
        Polygon *highlightArea = nullptr;
        GameObjectManager &gameObjectManager;

        std::vector<lv_obj_t *> rendererPool;
    };
}