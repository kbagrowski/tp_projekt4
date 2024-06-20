#pragma once

#include <memory>
#include <cmath>

#include <SDL.h>
#include <SDL2_gfx/SDL2_gfxPrimitives.h>

#include "planar_quadrotor.h"

class PlanarQuadrotorVisualizer {
private:
    PlanarQuadrotor* quadrotor_ptr;
    float propeller_angle;
    float prev_theta;
public:
    PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr);
    void render(std::shared_ptr<SDL_Renderer>& gRenderer);
};