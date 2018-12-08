#pragma once

// OBJ_Loader - .obj Loader
#include "OBJ_Loader.h"

#include <iostream>
#include "glm/vec3.hpp"
#include "glm/glm.hpp"
#include <math.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

using namespace glm;

class Sampler
{
  public:
    std::vector<dvec3> tris;
    std::vector<dvec2> uvs;
    std::vector<dvec3> normals;
    int num_tris;
    double area_sum;
    double *weights;

    //two optioanl flags
    bool flip_flag;   //default on
    bool normal_flag; //default off
    bool color_flag; //default off

    Sampler(const char *path, bool normal_flag, bool flip_flag, bool color_flag)
    {
        // bool res = loadOBJ(path, tris, uvs, normals);
        //convert the loaded data to dvec3
        // Initialize Loader
        std::cout << "loading mesh" << std::endl;
	    objl::Loader Loader;
        bool res = Loader.LoadFile(path);
        objl::Mesh curMesh = Loader.LoadedMeshes[0];
        std::cout << "mesh loaded" << std::endl;
        std::cout << "converting to vector" << std::endl;
        for (int i = 0; i < curMesh.Indices.size(); i += 3)
        {
            for (int j=0; j<3; j++)
            {
                int cur_index = curMesh.Indices[i+j];

                dvec3 pos = dvec3(curMesh.Vertices[cur_index].Position.X, curMesh.Vertices[cur_index].Position.Y, curMesh.Vertices[cur_index].Position.Z);
                tris.push_back(pos);

                dvec3 normal = dvec3(curMesh.Vertices[cur_index].Normal.X, curMesh.Vertices[cur_index].Normal.Y, curMesh.Vertices[cur_index].Normal.Z);
                normals.push_back(normal);

                dvec2 uv = dvec2(curMesh.Vertices[cur_index].TextureCoordinate.X, curMesh.Vertices[cur_index].TextureCoordinate.Y);
                uvs.push_back(uv);
            }
        }

        std::cout << "read: " << (res ? "sucessful" : "fail") << std::endl;

        num_tris = int(this->tris.size() / 3);
        weights = new double[num_tris];
        area_sum = 0;
        for (int i = 0; i < num_tris; i++)
        {
            double carea = getTriArea(tris[i * 3], tris[i * 3 + 1], tris[i * 3 + 2]);
            weights[i] = carea;
            area_sum += carea;
        }
        for (int i = 0; i < num_tris; i++)
        {
            weights[i] /= area_sum;
        }

        this->flip_flag = flip_flag;
        this->normal_flag = normal_flag && normals.size() > 0;
        this->color_flag = color_flag;
        std::cout << "modela surface area: " << this->area_sum << std::endl;
        std::cout << "num of vertices: " << this->tris.size() << std::endl;
        std::cout << "num of normals: " << this->normals.size() << std::endl;
        std::cout << "num of uvs: " << this->uvs.size() << std::endl;
    }

    //get the area of a triangle
    double getTriArea(const dvec3 &a, const dvec3 &b, const dvec3 &c)
    {
        double e0 = distance(a, b);
        double e1 = distance(b, c);
        double e2 = distance(c, a);
        double s = (e0 + e1 + e2) / 2;
        double area = std::sqrt(s * (s - e0) * (s - e1) * (s - e2));
        // if(area > 0.001)
        //     std::cout<<"area: "<<area<<std::endl;
        return area;
    }

    //sample the mesh to point cloud
    pcl::PointCloud<pcl::PointXYZRGBNormal> getPointCloud(int sample_density)
    {
        int num_samples = int(sample_density * area_sum + 0.5);
        pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
        cloud.width = num_samples;
        cloud.height = 1;
        cloud.is_dense = false;
        cloud.points.resize(num_samples);

        for (int i = 0; i < num_samples; i++)
        {
            //for every sample, randomly choose a tri
            int tri_index = 0;
            double x = ((double)std::rand() / (RAND_MAX));
            double temp = 1;
            // std::cout<<"******"<<std::endl;
            // std::cout<<"original x: "<<x<<std::endl;
            for (int j = 0; j < num_tris; j++)
            {
                if (temp <= x)
                {
                    tri_index = j;
                    // std::cout<<"uodate index: "<<tri_index<<std::endl;
                    break;
                }
                temp -= weights[j];
            }
            // std::cout<<"tri_index: "<<tri_index<<", temp: "<<temp<<std::endl;
            // std::cout<<"******"<<std::endl;
            //for a random tri, randomly select a point
            if (normal_flag)
            {
                //interpolate the normal
                dvec3 pt_n;
                dvec3 pt = getRandomPtOnTri(tris[tri_index * 3], tris[tri_index * 3 + 1], tris[tri_index * 3 + 2], normals[tri_index * 3], normals[tri_index * 3 + 1], normals[tri_index * 3 + 2], pt_n);
                //write to cloud
                cloud.points[i].r = 100;
                cloud.points[i].g = 100;
                cloud.points[i].b = 100;
                cloud.points[i].x = pt.x;
                cloud.points[i].y = pt.y;
                cloud.points[i].z = pt.z;
                cloud.points[i].normal_x = pt_n.x;
                cloud.points[i].normal_y = pt_n.y;
                cloud.points[i].normal_z = pt_n.z;
            }
            else
            //no normal, save normal as (0,0,0)
            {
                dvec3 pt_n;
                dvec3 pt = getRandomPtOnTri(tris[tri_index * 3], tris[tri_index * 3 + 1], tris[tri_index * 3 + 2], dvec3(0, 0, 0), dvec3(0, 0, 0), dvec3(0, 0, 0), pt_n);
                //write to cloud
                cloud.points[i].x = pt.x;
                cloud.points[i].y = pt.y;
                cloud.points[i].z = pt.z;
            }
        }
        return cloud;
    }

    //pick a random point on the triangle, and interpolate the normal
    //const dvec2 &a_uv, const dvec2 &b_uv, const dvec2 &c_uv,
    dvec3 getRandomPtOnTri(const dvec3 &a, const dvec3 &b, const dvec3 &c,
                          const dvec3 &a_n, const dvec3 &b_n, const dvec3 &c_n,
                          dvec3 &normal)
    {
        double tarea = double(getTriArea(a, b, c));
        double r0 = ((double)std::rand() / (RAND_MAX));
        double r1 = ((double)std::rand() / (RAND_MAX));
        dvec3 e0 = b - a;
        dvec3 e1 = c - a;
        dvec3 pt = a + r0 * e0 + r1 * e1;
        //is pt in abc?
        //http://blackpawn.com/texts/pointinpoly/
        //if not r0=1-r0, r1=1-r1
        //calculate the barycentric coord
        //interpolate normal
        dvec3 e2 = pt - a;
        double dot00 = dot(e0, e0);
        double dot01 = dot(e0, e1);
        double dot02 = dot(e0, e2);
        double dot11 = dot(e1, e1);
        double dot12 = dot(e1, e2);
        double invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
        double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        double v = (dot00 * dot12 - dot01 * dot02) * invDenom;
        if ((u >= -0.00001) && (v >= -0.00001) && (u + v <= 1))
        {
            double area0 = double(getTriArea(b, c, pt));
            double area1 = double(getTriArea(c, a, pt));
            double area2 = double(getTriArea(a, c, pt));
            double w0 = area0 / tarea;
            double w1 = area1 / tarea;
            double w2 = area2 / tarea;
            dvec3 pt_n = w0 * a_n + w1 * b_n + w2 * c_n;
            if (flip_flag)
                pt_n = -pt_n;
            normal = normalize(pt_n);
            return pt;
        }
        else
        {
            r0 = 1.0 - r0;
            r1 = 1.0 - r1;
            pt = a + r0 * e0 + r1 * e1;
            e2 = pt - a;
            dot00 = dot(e0, e0);
            dot01 = dot(e0, e1);
            dot02 = dot(e0, e2);
            dot11 = dot(e1, e1);
            dot12 = dot(e1, e2);
            invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
            u = (dot11 * dot02 - dot01 * dot12) * invDenom;
            v = (dot00 * dot12 - dot01 * dot02) * invDenom;
            if ((u >= -0.00001) && (v >= -0.00001) && (u + v <= 1))
            {
                double area0 = double(getTriArea(b, c, pt));
                double area1 = double(getTriArea(c, a, pt));
                double area2 = double(getTriArea(a, c, pt));
                double w0 = area0 / tarea;
                double w1 = area1 / tarea;
                double w2 = area2 / tarea;
                dvec3 pt_n = w0 * a_n + w1 * b_n + w2 * c_n;
                if (flip_flag)
                    pt_n = -pt_n;
                normal = normalize(pt_n);
                return pt;
            }
            else
            {
                //should never be executed
                std::cout << "tri area " << tarea << std::endl;
                std::cout << "u: " << u << ", v: " << v << std::endl;
                std::cout << "r0: " << r0 << ", r1: " << r1 << std::endl;
                std::cout << "pt: " << to_string(pt) << std::endl;
                std::cout << "a: " << to_string(a) << ", b: " << to_string(b) << ", c: " << to_string(c) << std::endl;
                std::cout << "wrong implementation" << std::endl;
                while (true)
                {
                    ;
                }
            }
        }
    }
};