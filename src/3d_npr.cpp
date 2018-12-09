#include <iostream>
#include <vector>
#include <stdlib.h> /* srand, rand */
#include <time.h>   /* time */
#include "glm/glm.hpp"
#include "glm/gtx/string_cast.hpp"
#include "OBJ_Loader.h"
#include "ppm_loader.h"
#include "sampler.h"
#include "bezier.h"
#include "fitter.h"
#include "selector.h"
#include "csv_writer.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>


using namespace glm;
//do not using namespace std overlap with glm; some common functions abiguous, like min/max


int main(int argc, char **argv)
{
    if (argc < 3)
    {
        pcl::console::print_error("argv: objfile_path pcdfile_path sample_density optioanl_normal_flag optional_flip_flag\n", argv[0]);
        return 1;
    }

    //set sample density
    int sample_density = 100;
    pcl::console::parse_argument(argc, argv, "-sample_density", sample_density);

    bool normal_flag = true;
    pcl::console::parse_argument(argc, argv, "-normal_flag", normal_flag);

    bool flip = false;
    pcl::console::parse_argument(argc, argv, "-flip_flag", flip);

    std::cout << "sample_density: " << sample_density << std::endl;
    std::cout << "normal_flag: " << normal_flag << std::endl;
    std::cout << "flip_flag: " << flip << std::endl;

    //set random seed
    srand(time(NULL));
    std::cout << "initializing sampler" << std::endl;
    Sampler sampler = Sampler(argv[1], normal_flag, flip, false);
    std::cout << "loading texture" << std::endl;
    Image texture = readPPM("../models/bunny.ppm");
    std::cout << "w: " <<  texture.w << ", h: " << texture.h << std::endl;
    std::cout << "sampling point cloud" << std::endl;

    pcl::PointCloud<pcl::PointXYZRGBNormal> out_cloud = sampler.getPointCloud(sample_density, &texture);

    pcl::io::savePCDFileASCII(argv[2], out_cloud);
    std::cerr << "saved " << out_cloud.points.size() << " data points to " << argv[2] << "." << std::endl;

    
    Selector test_selector = Selector();
    Fitter test_fitter = Fitter();
    // std::vector<dvec3> pts = test_selector.copyPoints(out_cloud);
    // for(int i=0;i<pts.size();i++)
    // {
    //     std::cout<<to_string(pts[i])<<" ";
    // }
    std::vector<std::vector<dvec3>> color_list;
    std::vector<std::vector<dvec3>> lines = test_selector.selectPoints(out_cloud, color_list, 0);
    // std::cout<<color_list.size()<<std::endl;
    // for(int i=0;i<lines[1].size();i++)
    // {
    //     std::cout<<to_string(lines[1][i]);
    // }
    // std::cout<<lines.size()<<std::endl;
    // for(int i=0;i<lines.size();i++)
    // {
    //     std::cout<<lines[i].size()<<std::endl;
    //     for(int j=0;j<lines[i].size();j++)
    //     {
    //         std::cout<<to_string(lines[i][j])<<" ";
    //     }
    //     std::cout<<std::endl;
    // }
    std::vector<std::vector<dvec3>> result;
    for(int i=0;i<lines.size();i++)
    {
        
        // std::vector<dvec3> in = lines[i];
        // std::cout<<"in size: "<<(in.size())<<std::endl;
        std::vector<dvec3> out = test_fitter.fitCubic(lines[i]);
        // std::cout<<"out size: "<<(out.size())<<std::endl;
        // std::cout<<"line: "<<i<<std::endl;
        // for(int j=0;j<lines[i].size();j++)
        // {
        //     std::cout<<to_string(lines[i][j]*double(100))<<" ";
        // }
        // std::cout<<std::endl;
        // for(int j=0;j<out.size();j++)
        // {
        //     std::cout<<to_string(out[j]*double(100))<<" ";
        // }
        // std::cout<<std::endl;
        result.push_back(out);

    }

    Writer test_writer = Writer();
    test_writer.write_bezier_to_csv("../output/test.txt", result);
    test_writer.write_bezier_to_csv("../output/test_color.txt", color_list);
    
    return 0;
}


/*
int main(int argc, char **argv)
{
    std::cout<<"hello, world!"<<std::endl;
    Fitter test_fitter = Fitter();
    std::cout<<test_fitter.tolerance<<std::endl;
    dvec3 p0 = dvec3(0,0,0);
    dvec3 p1 = dvec3(0,1,0);
    dvec3 p2 = dvec3(2,8,0);
    dvec3 p3 = dvec3(2,0,0);

    std::vector<dvec3> cur_pts;

    cur_pts.push_back(p0);
    cur_pts.push_back(p1);
    cur_pts.push_back(p2);
    cur_pts.push_back(p3);

    std::vector<dvec3> out = test_fitter.fitCubic(cur_pts);
    std::cout<<"control points:"<<std::endl;

    for(int i=0;i<out.size();i++)
    {
        std::cout<<to_string(out[i])<<std::endl;
    }
    for(int i=0;i<cur_pts.size();i++)
    {
        std::cout<<to_string(cur_pts[i])<<std::endl;
    }

    out = test_fitter.fitCubic(cur_pts);
    std::cout<<"control points:"<<std::endl;

    for(int i=0;i<out.size();i++)
    {
        std::cout<<to_string(out[i])<<std::endl;
    }
    for(int i=0;i<cur_pts.size();i++)
    {
        std::cout<<to_string(cur_pts[i])<<std::endl;
    }
}
*/