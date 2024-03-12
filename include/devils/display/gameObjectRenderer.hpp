#pragma once
#include "renderer.hpp"
#include "../utils/logger.hpp"
#include "../utils/units.hpp"
#include "../control/autoController.hpp"
#include "../gameobject/gameobject.hpp"
#include "../display/displayUtils.hpp"
#include "../utils/rect.hpp"
#include <cmath>
#include <string>

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
        GameObjectRenderer(std::vector<GameObject> *gameObjects) : gameObjects(gameObjects)
        {
        }

        ~GameObjectRenderer()
        {
            for (int i = 0; i < objectRenderers.size(); i++)
                lv_obj_del(objectRenderers[i]);
        }

        /**
         * Highlights all gameobjects within the rectangle
         * @param rect The rectangle to highlight
         */
        void useRect(Rect *rect)
        {
            highlightRect = rect;
        }

        void create(lv_obj_t *root) override
        {
            rootObject = root;

            // Define Styles
            static lv_style_t gameObjectStyle;
            lv_style_copy(&gameObjectStyle, &lv_style_plain);
            gameObjectStyle.body.main_color = LV_COLOR_MAKE(0x00, 0xff, 0x00);
            gameObjectStyle.body.grad_color = LV_COLOR_MAKE(0x00, 0xff, 0x00);
            gameObjectStyle.body.radius = 1;

            static lv_style_t gameObjectDisabledStyle;
            lv_style_copy(&gameObjectDisabledStyle, &lv_style_plain);
            gameObjectDisabledStyle.body.main_color = LV_COLOR_MAKE(0x00, 0x55, 0x00);
            gameObjectDisabledStyle.body.grad_color = LV_COLOR_MAKE(0x00, 0x55, 0x00);
            gameObjectDisabledStyle.body.radius = 1;

            // Create game object renderers
            for (int i = 0; i < gameObjects->size(); i++)
            {
                lv_obj_t *objectRenderer = lv_obj_create(root, NULL);
                {
                    int x = gameObjects->at(i).x * DisplayUtils::PX_PER_IN + OFFSET_X;
                    int y = gameObjects->at(i).y * DisplayUtils::PX_PER_IN + OFFSET_Y;

                    lv_obj_set_size(objectRenderer, OBJ_WIDTH, OBJ_HEIGHT);
                    lv_obj_set_pos(objectRenderer, x, y);

                    // Conditional Style
                    bool outsideRectangle = highlightRect != nullptr && !highlightRect->contains(gameObjects->at(i));
                    if (outsideRectangle)
                        lv_obj_set_style(objectRenderer, &gameObjectDisabledStyle);
                    else
                        lv_obj_set_style(objectRenderer, &gameObjectStyle);

                    objectRenderers.push_back(objectRenderer);
                }
            }
        }

        void update() override
        {
            if (gameObjects->size() != objectRenderers.size() && rootObject != nullptr)
            {
                // Regenerate renderers
                for (int i = 0; i < objectRenderers.size(); i++)
                    lv_obj_del(objectRenderers[i]);
                objectRenderers.clear();
                create(rootObject);
            }
        }

    private:
        static constexpr int OBJ_WIDTH = 14;  // px
        static constexpr int OBJ_HEIGHT = 14; // px
        static constexpr int OFFSET_X = (DisplayUtils::DISPLAY_WIDTH - OBJ_WIDTH) / 2;
        static constexpr int OFFSET_Y = (DisplayUtils::DISPLAY_HEIGHT - OBJ_HEIGHT) / 2;

        lv_obj_t *rootObject = nullptr;
        Rect *highlightRect = nullptr;
        std::vector<GameObject> *gameObjects;
        std::vector<lv_obj_t *> objectRenderers;
    };
}