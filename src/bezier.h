#pragma once

#include <iostream>
#include "glm/vec3.hpp"
#include "glm/glm.hpp"
#include <math.h>
#include <vector>

using namespace glm;

//cubic bezier curve
class Bezier
{
  public:
    std::vector<dvec3> contorl_pts;

    Bezier(std::vector<dvec3> contorl_pts)
    {
        this->contorl_pts = contorl_pts;
    }
};
