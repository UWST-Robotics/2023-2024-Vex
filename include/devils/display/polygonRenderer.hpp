#pragma once
#include "renderer.hpp"
#include "../utils/logger.hpp"
#include "../geometry/units.hpp"
#include "../geometry/polygon.hpp"
#include "displayUtils.hpp"
#include <cmath>
#include <string>

namespace devils
{
    /**
     * Renderer that displays a polygon.
     */
    class PolygonRenderer : public Renderer
    {
    public:
        /**
         * Creates a new PolygonRenderer
         * @param polygon The polygon to render
         */
        PolygonRenderer(Polygon *polygon) : polygon(polygon)
        {
        }

        ~PolygonRenderer()
        {
            lv_obj_del(lineObject);
        }

        void create(lv_obj_t *root) override
        {
            // Calculate Dimensions
            double offsetX = DisplayUtils::DISPLAY_WIDTH / 2;
            double offsetY = DisplayUtils::DISPLAY_HEIGHT / 2;

            // Bounding Rect
            lineObject = lv_line_create(root, NULL);
            {
                // Convert path to vector of points
                linePointVector.clear();
                for (int i = 0; i < polygon->points.size() + 1; i++)
                {
                    Vector2 &point = polygon->points[i % polygon->points.size()];
                    linePointVector.push_back({(short)(point.x * DisplayUtils::PX_PER_IN + offsetX),
                                               (short)(point.y * DisplayUtils::PX_PER_IN + offsetY)});
                }

                // Create Line
                linePoints = linePointVector.data();
                lv_line_set_points(lineObject, linePoints, linePointVector.size());

                // Style
                static lv_style_t pathStyle;
                lv_style_copy(&pathStyle, &lv_style_plain);
                pathStyle.line.color = LV_COLOR_MAKE(0xff, 0xff, 0xff);
                pathStyle.line.opa = 100;
                pathStyle.line.width = 4;
                lv_obj_set_style(lineObject, &pathStyle);
            }
        }

    private:
        std::vector<lv_point_t> linePointVector;
        lv_point_t *linePoints;

        Polygon *polygon;
        lv_obj_t *lineObject;
    };
}