#include "planar_quadrotor_visualizer.h"

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr)
    : quadrotor_ptr(quadrotor_ptr), propeller_angle(0.0f), prev_theta(0.0f) {}
//smigla
std::pair<float, float> rotatePoint(float x, float y, float centerX, float centerY, float angle) {
    float translatedX = x - centerX;
    float translatedY = y - centerY;
    float rotatedX = translatedX * cos(angle) - translatedY * sin(angle);
    float rotatedY = translatedX * sin(angle) + translatedY * cos(angle);
    return { rotatedX + centerX, rotatedY + centerY };
}
//interpolacja
float interpolateAngle(float prev_angle, float current_angle, float interpolation_factor) {
    float diff = current_angle - prev_angle;
    if (diff > M_PI) {
        diff -= 2 * M_PI;
    }
    else if (diff < -M_PI) {
        diff += 2 * M_PI;
    }
    return prev_angle + diff * interpolation_factor;
}
//renderowanie drona 
void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer>& gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x = state[0], q_y = state[1], q_theta = state[2];


    float interpolation_factor = 0.001f;
    q_theta = interpolateAngle(prev_theta, q_theta, interpolation_factor);
    prev_theta = q_theta;



    int screen_width, screen_height;
    SDL_GetRendererOutputSize(gRenderer.get(), &screen_width, &screen_height);

    float screen_center_x = screen_width / 2.0f;
    float screen_center_y = screen_height / 2.0f;
    float rect_center_x = screen_center_x + q_x;
    float rect_center_y = screen_center_y - q_y;

    float body_width = 150.0f, body_height = 20.0f;
    float half_width = body_width / 2.0f, half_height = body_height / 2.0f;

    std::vector<std::pair<float, float>> rect_points = {
        rotatePoint(rect_center_x - half_width, rect_center_y - half_height, rect_center_x, rect_center_y, -q_theta),
        rotatePoint(rect_center_x + half_width, rect_center_y - half_height, rect_center_x, rect_center_y, -q_theta),
        rotatePoint(rect_center_x + half_width, rect_center_y + half_height, rect_center_x, rect_center_y, -q_theta),
        rotatePoint(rect_center_x - half_width, rect_center_y + half_height, rect_center_x, rect_center_y, -q_theta)
    };

    Sint16 vx[4], vy[4];
    for (int i = 0; i < 4; i++) {
        vx[i] = static_cast<Sint16>(rect_points[i].first);
        vy[i] = static_cast<Sint16>(rect_points[i].second);
    }
    SDL_SetRenderDrawColor(gRenderer.get(), 0x80, 0x80, 0x80, 0xFF);
    filledPolygonRGBA(gRenderer.get(), vx, vy, 4, 0x80, 0x80, 0x80, 0xFF);

    SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x00, 0x00, 0xFF);
    for (int i = 0; i < 4; i++) {
        SDL_RenderDrawLine(gRenderer.get(), vx[i], vy[i], vx[(i + 1) % 4], vy[(i + 1) % 4]);
    }

    float propeller_offset = 15.0f;
    float propeller_width = 15.0f, propeller_height = 7.0f;
    propeller_angle += 0.1f;

    std::vector<std::pair<float, float>> propeller_centers = {
        { rect_points[0].first, rect_points[0].second - propeller_offset },
        { rect_points[1].first, rect_points[1].second - propeller_offset }
    };

    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0x00, 0x00, 0xFF);
    SDL_RenderDrawLine(gRenderer.get(), rect_points[0].first, rect_points[0].second, rect_points[0].first, rect_points[0].second - propeller_offset);
    SDL_RenderDrawLine(gRenderer.get(), rect_points[1].first, rect_points[1].second, rect_points[1].first, rect_points[1].second - propeller_offset);

    Uint32 propeller_color = 0x0000FFFF;
    for (auto& center : propeller_centers) {
        auto [rotated_x1, rotated_y1] = rotatePoint(center.first - propeller_width, center.second, center.first, center.second, propeller_angle);
        auto [rotated_x2, rotated_y2] = rotatePoint(center.first + propeller_width, center.second, center.first, center.second, propeller_angle);
        auto [rotated_x3, rotated_y3] = rotatePoint(center.first, center.second - propeller_height, center.first, center.second, propeller_angle);
        auto [rotated_x4, rotated_y4] = rotatePoint(center.first, center.second + propeller_height, center.first, center.second, propeller_angle);

        filledEllipseRGBA(gRenderer.get(), rotated_x1, rotated_y1, propeller_width, propeller_height, 0x00, 0x00, 0xFF, 0xFF);
        filledEllipseRGBA(gRenderer.get(), rotated_x2, rotated_y2, propeller_width, propeller_height, 0x00, 0x00, 0xFF, 0xFF);
        filledEllipseRGBA(gRenderer.get(), rotated_x3, rotated_y3, propeller_width, propeller_height, 0x00, 0x00, 0xFF, 0xFF);
        filledEllipseRGBA(gRenderer.get(), rotated_x4, rotated_y4, propeller_width, propeller_height, 0x00, 0x00, 0xFF, 0xFF);
    }
}