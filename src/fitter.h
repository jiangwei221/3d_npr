#pragma once

#include <iostream>
#include "bezier.h"
#include "glm/vec3.hpp"
#include "glm/glm.hpp"
#include <math.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

using namespace glm;

class Fitter
{
  public:
    float tolerance;
    int max_pieces;

    struct val_index 
    {
      double value;
      int index;
    };

    Fitter()
    {
      this->tolerance = 0.2;
      this->max_pieces = 100;
    }

    std::vector<dvec3> fitCubic(const std::vector<dvec3> &cur_pts)
    {
      // std::cout<<"*****fitCubic******"<<std::endl;
      // std::cout<<cur_pts.size()<<std::endl;
      std::vector<float> t_values = chordLengthParameterize(cur_pts);
      std::vector<dvec3> tan_v = computeTangentVectors(cur_pts);
      std::vector<dvec3> bz_curve = generateBezier(cur_pts);
      val_index max_error = computeMaxError(cur_pts, t_values, bz_curve);
      float max_error_val = max_error.value;
      float max_error_index = max_error.index;
      if(max_error_val>tolerance & t_values.size()>0)
      {
        //magic code
        dvec3 actual_pt = cur_pts[max_error_index];
        dvec3 prev_pt = cur_pts[max_error_index-1];
        dvec3 next_pt = cur_pts[max_error_index+1];
        //dvec3 center_pt = (prev_pt + next_pt)/2.0;//dvec3((prev_pt.x+next_pt.x)/2.0, (prev_pt.y+next_pt.y)/2.0, (prev_pt.z+next_pt.z)/2.0);
        dvec3 prev_add = (actual_pt + prev_pt)/2.0;//new PVector((actualPoint.x + lastButOnePointInLeft.x)/2, (actualPoint.y + lastButOnePointInLeft.y)/2);
        dvec3 next_add = (actual_pt + next_pt)/2.0;
        dvec3 center_pt_new = (prev_add + next_add)/2.0;
        dvec3 prev_tan = prev_add - center_pt_new;//new PVector((pointAddToLeft.x - assumedCenterPoint2.x), (pointAddToLeft.y - assumedCenterPoint2.y));
        dvec3 next_tan = next_add - center_pt_new;

        std::vector<dvec3>::const_iterator s0 = cur_pts.begin() + 0;
        std::vector<dvec3>::const_iterator s1 = cur_pts.begin() + max_error_index;
        std::vector<dvec3>::const_iterator s2 = cur_pts.begin() + max_error_index + 1;
        std::vector<dvec3>::const_iterator s3 = cur_pts.end();
        
        std::vector<dvec3> prev_pts(s0, s1);
        prev_pts.push_back(prev_add);
        prev_pts.push_back(center_pt_new);
        std::vector<dvec3> prev_cubic = fitCubic(prev_pts);

        std::vector<dvec3> next_pts;
        next_pts.push_back(center_pt_new);
        next_pts.push_back(next_add);
        next_pts.insert(next_pts.end(), s2, s3);
        std::vector<dvec3> next_cubic = fitCubic(next_pts);

        prev_cubic.pop_back();

        std::vector<dvec3> result;
        result.reserve( prev_cubic.size() + next_cubic.size() ); // preallocate memory
        result.insert( result.end(), prev_cubic.begin(), prev_cubic.end() );
        result.insert( result.end(), next_cubic.begin(), next_cubic.end() );
        return result;

        // leftCubic = fitCubic(leftPointsXY);
        


      }
      // std::cout<<"***"<<std::endl;
      // std::cout<<bz_curve.size()<<std::endl;
      // for(int j;j<bz_curve.size();j++)
      // {
      //     std::cout<<to_string(bz_curve[j])<<" ";
      // }
      // std::cout<<"***"<<std::endl;
      return bz_curve;
    }

    std::vector<float> chordLengthParameterize(const std::vector<dvec3> &cur_pts)
    {
      int num_cur_pts = cur_pts.size();
      std::vector<float> t_values;
      t_values.push_back(0.0);

      for(int i=1; i<num_cur_pts; i++)
      {
        float dist = distance(cur_pts[i-1], cur_pts[i]);
        t_values.push_back(dist+t_values[i-1]);
      }

      float total_len = t_values[t_values.size()-1];
      //std::cout<<"last val: "<<total_len<<std::endl;
      for(int i=0; i<t_values.size(); i++)
      {

        t_values[i] /= total_len;
      }

      return t_values;
    }

    std::vector<dvec3> generateBezier(const std::vector<dvec3> &cur_pts)
    {
      std::vector<float> t_values = chordLengthParameterize(cur_pts);

      // std::cout<<"t_values:"<<std::endl;
      // for(int i=0;i<t_values.size();i++)
      // {
      //   std::cout<<t_values[i]<<std::endl;
      // }

      int num_cur_pts = cur_pts.size();

      // std::cout<<"tangent vectors:"<<std::endl;
      
      std::vector<dvec3> tan_v = computeTangentVectors(cur_pts);
      // for(int i=0;i<tan_v.size();i++)
      // {
      //   std::cout<<to_string(tan_v[i])<<std::endl;
      // }

      std::vector<dvec3> result;
      
      //magic code
      float C [2][2] = { 0 };
      float X [2] = { 0 };
      float det_C0_C1, det_C0_X, det_X_C1, alpha1, alpha2;
      std::vector<std::vector<dvec3>> A;
      dvec3 first_pt = cur_pts[0];
      dvec3 last_pt = cur_pts[num_cur_pts-1];
      for(int i=0;i<num_cur_pts;i++)
      {
        dvec3 v1 = tan_v[0];
        dvec3 v2 = tan_v[1];
        v1 = normalize(v1);
        v1 = v1 * B1(t_values[i]);
        v2 = normalize(v2);
        v2 = v2 * B2(t_values[i]);
        //
        std::vector<dvec3> temp;
        temp.push_back(v1);
        temp.push_back(v2);
        A.push_back(temp);
      }

      // std::cout<<"A:"<<std::endl;
      // for(int i=0;i<num_cur_pts;i++)
      // {
      //   for(int j=0;j<2;j++)
      //   {
      //     std::cout<<to_string(A[i][j])<<" ";
      //   }
      //   std::cout<<std::endl;
      // }

      for(int i=0;i<num_cur_pts;i++)
      {
        C[0][0] += dot(A[i][0], A[i][0]);
        C[0][1] += dot(A[i][0], A[i][1]);
        C[1][0] = C[0][1];
        C[1][1] += dot(A[i][1], A[i][1]);
        dvec3 temp;
        dvec3 i_pt = cur_pts[i];
        temp = (i_pt - ( (first_pt * B0(t_values[i]) ) + ( (first_pt * B1(t_values[i])) + ( (last_pt * B2(t_values[i])) + (last_pt * B3(t_values[i])) )) ));
        X[0] += dot(A[i][0], temp);
        X[1] += dot(A[i][1], temp);
      }
      det_C0_C1 = C[0][0] * C[1][1] - C[1][0] * C[0][1];
      det_C0_X  = C[0][0] * X[1]    - C[1][0] * X[0];
      det_X_C1  = X[0]    * C[1][1] - X[1]    * C[0][1];

      // std::cout<<"X: "<<X[0]<<" "<<X[1]<<std::endl;
      // std::cout<<"C: "<<C[0][0]<<" "<<C[0][1]<<" "<<C[1][0]<<" "<<C[1][1]<<std::endl;
      // std::cout<<"det_C0_C1: "<<det_C0_C1<<std::endl;
      // std::cout<<"det_C0_X: "<<det_C0_X<<std::endl;
      // std::cout<<"det_X_C1: "<<det_X_C1<<std::endl;
      

      alpha1 = (abs(det_C0_C1) < 0.01) ? 0.0 : (det_X_C1 / det_C0_C1);
      alpha2 = (abs(det_C0_C1) < 0.01) ? 0.0 : (det_C0_X / det_C0_C1);
      
      dvec3 v1 = tan_v[0];
      dvec3 v2 = tan_v[1];
      v1 = normalize(v1);
      v2 = normalize(v2);

      result.push_back(cur_pts[0]);

      dvec3 temp = dvec3(cur_pts[0].x + v1.x*alpha1, cur_pts[0].y + v1.y*alpha1, cur_pts[0].z + v1.z*alpha1);
      result.push_back(temp);

      dvec3 temp_2 = dvec3(cur_pts[num_cur_pts-1].x + v2.x*alpha2, cur_pts[num_cur_pts-1].y + v2.y*alpha2, cur_pts[num_cur_pts-1].z + v2.z*alpha2);
      result.push_back(temp_2);

      result.push_back(cur_pts[num_cur_pts-1]);

      // std::cout<<"alpha1: "<<alpha1<<std::endl;
      // std::cout<<"alpha2: "<<alpha2<<std::endl;

      return result;
    }

    std::vector<dvec3> computeTangentVectors(const std::vector<dvec3> &cur_pts)
    {
      assert(cur_pts.size() >= 2);
      std::vector<dvec3> result; //index 0: tangent vector at p0, index 1: p3
      dvec3 v0 = cur_pts[1] - cur_pts[0]; //p0 to p1
      dvec3 v1 = cur_pts[cur_pts.size()-2] - cur_pts[cur_pts.size()-1]; //p3 to p2
      result.push_back(v0);
      result.push_back(v1);

      return result;
    }

    val_index computeMaxError(const std::vector<dvec3> &cur_pts, const std::vector<float> &t_values, const std::vector<dvec3> &bz_curve)
    {
      if(cur_pts.size() == 2)
      {
        val_index result = {0.0, 1};
        return result;
      }

      assert(bz_curve.size() == 4);
      mat4 basic_mat = mat4(
            1.0f, 0.0f, 0.0f, 0.0f,
            -3.0f, 3.0f, 0.0f, 0.0f,
            3.0f, -6.0f, 3.0f, 0.0f,
            -1.0f, 3.0f, -3.0f, 1.0f);

      // dvec3 actual_pt, bz_pt;
      float max_dist = 0;
      int max_index = 0;
      for(int i=0;i<cur_pts.size();i++)
      {
        dvec3 actual_pt = cur_pts[i];
        float t = t_values[i];
        vec4 vec_x = vec4(bz_curve[0][0], bz_curve[1][0], bz_curve[2][0], bz_curve[3][0]);
        vec4 vec_y = vec4(bz_curve[0][1], bz_curve[1][1], bz_curve[2][1], bz_curve[3][1]);
        vec4 vec_z = vec4(bz_curve[0][2], bz_curve[1][2], bz_curve[2][2], bz_curve[3][2]);

        vec4 t_coeff = vec4(1, t, t*t, t*t*t);

        float c_x = dot(t_coeff, (vec_x * basic_mat));
        float c_y = dot(t_coeff, (vec_y * basic_mat));
        float c_z = dot(t_coeff, (vec_z * basic_mat));

        dvec3 bz_pt = vec3(c_x, c_y, c_z);

        float dist = distance(actual_pt, bz_pt);
        if(dist > max_dist)
        {
          max_dist = dist;
          max_index = i;
        }
      }

      val_index result = {max_dist, max_index};
      return result;
    }

    double B0(double u)
    {
      double temp = 1-u;
      return (temp*temp*temp);
    }

    double B1(double u)
    {
      double temp = 1-u;
      return (3*u*temp*temp);
    }

    double B2(double u)
    {
      double temp = 1-u;
      return (3*u*u*temp);
    }

    double B3(double u)
    {
      return (u*u*u);
    }

};