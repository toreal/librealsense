// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.
//
// Plane Fit implementation follows http://www.ilikebigbits.com/blog/2015/3/2/plane-from-points algorithm

#pragma once
#include <vector>
#include <mutex>
#include <array>
#include <imgui.h>
#include <librealsense2/rsutil.h>
#include <librealsense2/rs.hpp>
//#include "rendering.h"

namespace rs2
{



	struct plane
	{
		float a;
		float b;
		float c;
		float d;
	};
	inline bool operator==(const plane& lhs, const plane& rhs) { return lhs.a == rhs.a && lhs.b == rhs.b && lhs.c == rhs.c && lhs.d == rhs.d; }

	struct float3
	{
		float x, y, z;

		float length() const { return sqrt(x * x + y * y + z * z); }

		float3 normalize() const
		{
			return (length() > 0) ? float3{ x / length(), y / length(), z / length() } : *this;
		}
	};


	inline float3 cross(const float3& a, const float3& b)
	{
		return { a.y * b.z - b.y * a.z, a.x * b.z - b.x * a.z, a.x * b.y - a.y * b.x };
	}

	inline float evaluate_plane(const plane& plane, const float3& point)
	{
		return plane.a * point.x + plane.b * point.y + plane.c * point.z + plane.d;
	}

	inline float3 operator*(const float3& a, float t)
	{
		return { a.x * t, a.y * t, a.z * t };
	}

	inline float3 operator/(const float3& a, float t)
	{
		return { a.x / t, a.y / t, a.z / t };
	}

	inline float3 operator+(const float3& a, const float3& b)
	{
		return { a.x + b.x, a.y + b.y, a.z + b.z };
	}

	inline float3 operator-(const float3& a, const float3& b)
	{
		return { a.x - b.x, a.y - b.y, a.z - b.z };
	}

	inline float3 lerp(const float3& a, const float3& b, float t)
	{
		return b * t + a * (1 - t);
	}

	inline float operator*(const float3& a, const float3& b)
	{
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}


	using plane_3d = std::array<float3, 4>;
    //namespace depth_quality
    //{
			   
        struct snapshot_metrics
        {
            int width;
            int height;

            rs2::region_of_interest roi;

            float distance;
            float angle;
            float angle_x;
            float angle_y;

            plane p;
            std::array<float3, 4> plane_corners;
        };

        struct single_metric_data
        {
            single_metric_data(std::string name, float val) :
                val(val), name(name) {}

            float val;
            std::string name;
        };

        using callback_type = std::function<void(
            const std::vector<rs2::float3>& points,
            const plane p,
            const rs2::region_of_interest roi,
            const float baseline_mm,
            const float focal_length_pixels,
            const int ground_thruth_mm,
            const bool plane_fit,
            const float plane_fit_to_ground_truth_mm,
            const float distance_mm,
            bool record,
            std::vector<single_metric_data>& samples)>;

        inline plane plane_from_point_and_normal(const rs2::float3& point, const rs2::float3& normal)
        {
            return{ normal.x, normal.y, normal.z, -(normal.x*point.x + normal.y*point.y + normal.z*point.z) };
        }

        //Based on: http://www.ilikebigbits.com/blog/2015/3/2/plane-from-points
        inline plane plane_from_points(const std::vector<rs2::float3> points)
        {
            if (points.size() < 3) throw std::runtime_error("Not enough points to calculate plane");

            rs2::float3 sum = { 0,0,0 };
            for (auto point : points) sum = sum + point;

            rs2::float3 centroid = sum / float(points.size());

            double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
            for (auto point : points) {
                rs2::float3 temp = point - centroid;
                xx += temp.x * temp.x;
                xy += temp.x * temp.y;
                xz += temp.x * temp.z;
                yy += temp.y * temp.y;
                yz += temp.y * temp.z;
                zz += temp.z * temp.z;
            }

            double det_x = yy*zz - yz*yz;
            double det_y = xx*zz - xz*xz;
            double det_z = xx*yy - xy*xy;

            double det_max = std::max({ det_x, det_y, det_z });
            if (det_max <= 0) return{ 0, 0, 0, 0 };

            rs2::float3 dir{};
            if (det_max == det_x)
            {
                float a = static_cast<float>((xz*yz - xy*zz) / det_x);
                float b = static_cast<float>((xy*yz - xz*yy) / det_x);
                dir = { 1, a, b };
            }
            else if (det_max == det_y)
            {
                float a = static_cast<float>((yz*xz - xy*zz) / det_y);
                float b = static_cast<float>((xy*xz - yz*xx) / det_y);
                dir = { a, 1, b };
            }
            else
            {
                float a = static_cast<float>((yz*xy - xz*yy) / det_z);
                float b = static_cast<float>((xz*xy - yz*xx) / det_z);
                dir = { a, b, 1 };
            }

            return plane_from_point_and_normal(centroid, dir.normalize());
        }

        inline double evaluate_pixel(const plane& p, const rs2_intrinsics* intrin, float x, float y, float distance, float3& output)
        {
            float pixel[2] = { x, y };
            rs2_deproject_pixel_to_point(&output.x, intrin, pixel, distance);
            return evaluate_plane(p, output);
        }

        inline float3 approximate_intersection(const plane& p, const rs2_intrinsics* intrin, float x, float y, float min, float max)
        {
            float3 point;
            auto f = evaluate_pixel(p, intrin, x, y, max, point);
            if (fabs(max - min) < 1e-3) return point;
            auto n = evaluate_pixel(p, intrin, x, y, min, point);
            if (f*n > 0) return{ 0, 0, 0 };

            auto avg = (max + min) / 2;
            auto mid = evaluate_pixel(p, intrin, x, y, avg, point);
            if (mid*n < 0) return approximate_intersection(p, intrin, x, y, min, avg);
            return approximate_intersection(p, intrin, x, y, avg, max);
        }

        inline float3 approximate_intersection(const plane& p, const rs2_intrinsics* intrin, float x, float y)
        {
            return approximate_intersection(p, intrin, x, y, 0.f, 1000.f);
        }

      



		inline bool is_valid(const plane_3d& p)
		{
			std::vector<float> angles;
			angles.reserve(4);
			for (size_t i = 0; i < p.size(); i++)
			{
				auto p1 = p[i];
				auto p2 = p[(i + 1) % p.size()];
				if ((p2 - p1).length() < 1e-3) return false;

				p1 = p1.normalize();
				p2 = p2.normalize();

				angles.push_back(acos((p1 * p2) / sqrt(p1.length() * p2.length())));
			}
			return std::all_of(angles.begin(), angles.end(), [](float f) { return f > 0; }) ||
				std::all_of(angles.begin(), angles.end(), [](float f) { return f < 0; });
		}

    }
//}
