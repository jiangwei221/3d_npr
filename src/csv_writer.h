#pragma once

#include <iostream>
#include <fstream>
#include "bezier.h"
#include "glm/vec3.hpp"
#include "glm/glm.hpp"
#include <math.h>
#include <vector>

using namespace glm;

class Writer
{
  public:
    Writer()
    {
        ;
    }

    void writeBezierToCSV(const char *path, const std::vector<std::vector<dvec3>> &control_pts)
    {
        std::ofstream myfile (path);
        if (myfile.is_open())
        {
            for(int i=0;i<control_pts.size();i++)
            {
                for(int j=0;j<control_pts[i].size();j++)
                {
                    myfile << control_pts[i][j].x << ", " <<control_pts[i][j].y << ", "<< control_pts[i][j].z << ", ";
                }
                myfile << std::endl;
            }
            myfile.close();
        }
        else std::cout << "Unable to open file";
    }
};