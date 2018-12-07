/*
select a set of vector of points, basically, vector<vector<dvec3>>
from a point cloud

jiang wei
uvic
start: 2018.10.9
*/

#pragma once

#include <iostream>
#include <limits>
#include <algorithm>
#include "bezier.h"
#include "glm/vec3.hpp"
#include "glm/glm.hpp"
#include <math.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

using namespace glm;

#define SLICE_AXIS y
#define BUILD_AXIS z

class Selector
{
  public:
    int num_slices;
    int num_max_line;
    int num_max_pts;
    Selector()
    {
        num_slices = 10;
        num_max_line = 50;
        num_max_pts = 50;

    }    

    std::vector<std::vector<dvec3>> selectPoints(const pcl::PointCloud<pcl::PointXYZRGBNormal> &in_cloud, int num_curves)
    {
        std::vector<std::vector<dvec3>> result;
        std::vector<std::vector<dvec3>> sliced_cloud = sliceCloud(in_cloud, num_slices);
        for(int i=0; i<sliced_cloud.size(); i++)
        {
            std::vector<std::vector<dvec3>> lines = buildLine(sliced_cloud[i], num_max_pts, num_max_line);
            result.insert( result.end(), lines.begin(), lines.end() );
        }

        return result;
    }

    std::vector<dvec3> copyPoints(const pcl::PointCloud<pcl::PointXYZRGBNormal> &in_cloud)
    {
        std::vector<dvec3> result;
        for(int i=0; i<in_cloud.size(); i++)
        {
            result.push_back(dvec3(in_cloud.points[i].x, in_cloud.points[i].y, in_cloud.points[i].z));
        }

        return result;
    }

    std::vector<float> getHighAndLow(const pcl::PointCloud<pcl::PointXYZRGBNormal> &in_cloud)
    {
        int num_pts = in_cloud.width;
        assert(in_cloud.height == 1);
        float height_max = std::numeric_limits<float>::min();
        float height_min = std::numeric_limits<float>::max();
        for(int i=0;i<num_pts;i++)
        {
            if(in_cloud.points[i].SLICE_AXIS > height_max)
                height_max = in_cloud.points[i].SLICE_AXIS;
            if(in_cloud.points[i].SLICE_AXIS < height_min)
                height_min = in_cloud.points[i].SLICE_AXIS;
        }

        std::vector<float> result;
        result.push_back(height_max+0.000001);
        result.push_back(height_min);

        return result;
    }

    std::vector<std::vector<dvec3>> sliceCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal> &in_cloud, int num_slices)
    {
        std::vector<std::vector<dvec3>> result (num_slices);
        std::vector<float> high_low = getHighAndLow(in_cloud);
        float high = high_low[0];
        float low = high_low[1];

        int num_pts = in_cloud.width;
        for(int i=0;i<num_pts;i++)
        {
            int index = int((in_cloud.points[i].SLICE_AXIS - low) / ((high-low)/num_slices));
            // std::cout<<"index: "<<index<<" num_slices: "<<num_slices<<std::endl;
            assert(index >= 0 && index < num_slices);
            result[index].push_back(dvec3(in_cloud.points[i].x, in_cloud.points[i].y, in_cloud.points[i].z));
        }

        return result;
    }

    std::vector<std::vector<dvec3>> buildLine(const std::vector<dvec3> &in_pts, int num_max_pts, int num_max_line)
    {
        std::vector<std::vector<dvec3>> result;
        std::vector<dvec3> temp = in_pts;
        // temp.swap(in_pts)
        std::vector<bool> temp_flag (temp.size(), true);
        std::vector<std::vector<float>> dist_mtrx = buildDistanceMtrx(in_pts);
        for(int i=0; i<num_max_line; i++)
        {
            // std::vector<bool>::iterator itr = std::find(temp_flag.begin(), temp_flag.end(), true);
            // int start_id = std::distance(temp_flag.begin(), itr);
            // for(int j=start_id; j<temp_flag.size(); j++)
            // {

            // }
            bool is_new_line = false;
            std::vector<dvec3> c_line;

            // std::vector<bool>::iterator itr = std::find(temp_flag.begin(), temp_flag.end(), true);
            // int start_id = std::distance(temp_flag.begin(), itr);
            float start_pos = std::numeric_limits<float>::max();
            int start_id = -1;
            for(int j=0; j<temp_flag.size(); j++)
            {
                if(temp_flag[j]==true)
                {
                    if(temp[j].BUILD_AXIS < start_pos)
                    {
                        start_pos = temp[j].BUILD_AXIS;
                        start_id = j;
                    }
                }
            }

            if(start_id == -1)
                break;

            
            float prev_dist = std::numeric_limits<float>::max();
            for(int j=0; j<num_max_pts; j++)
            {
                //find the closest pt of start_id
                float dist_min = std::numeric_limits<float>::max();
                int dist_min_id = -1;
                for(int k=0; k<temp.size(); k++)
                {
                    if(temp_flag[k] == true && k!=start_id)
                    {
                        if(dist_mtrx[start_id][k] < dist_min)
                        {
                            dist_min = dist_mtrx[start_id][k];
                            dist_min_id = k;
                        }
                    }
                }

                if(dist_min / 200.0f > prev_dist)
                    break;

                prev_dist = dist_min;

                //set temp_flag[start_id] = false
                temp_flag[start_id] = false;

                //add temp[start_id] to c_line
                c_line.push_back(temp[start_id]);

                //set start_id to the closest pt's index
                start_id = dist_min_id;

                //if no point to add
                if(dist_min_id == -1)
                {
                    break;
                }


            }
            if(c_line.size() >= 4)
            {
                result.push_back(c_line);
                is_new_line = true;
            }

            if(is_new_line == false)
                break;
        }

        return result;
    }

    std::vector<std::vector<float>> buildDistanceMtrx(const std::vector<dvec3> &in_pts)
    {
        int num_pts = in_pts.size();
        std::vector<std::vector<float>> result (num_pts, std::vector<float>(num_pts));
        for(int i=0; i<num_pts; i++)
        {
            for(int j=0; j<num_pts; j++)
            {
                result[i][j] = distance(in_pts[i], in_pts[j]);
            }
        }
        return result;
    }
};