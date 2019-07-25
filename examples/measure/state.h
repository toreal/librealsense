#pragma once

//#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
//#include <librealsense2/rsutil.h>


#include <Windows.h>

#define _USE_MATH_DEFINES

#include <math.h>
#include <algorithm> 



#include "depth-metrics.h"

 



using pixel = std::pair<int, int>;

// Toggle helper class will be used to render the two buttons
// controlling the edges of our ruler
 struct toggle
{
	toggle() : x(0.f), y(0.f) {}
	toggle(float xl, float yl)
		: x(std::min(std::max(xl, 0.f), 1.f)),
		y(std::min(std::max(yl, 0.f), 1.f))
	{}

	// Move from [0,1] space to pixel space of specific frame
	pixel get_pixel(depth_frame frm) const
	{
		int px = x * frm.get_width();
		int py = y * frm.get_height();
		return{ px, py };
	}

	void render(const window& app)
	{
		glColor4f(0.f, 0.0f, 0.0f, 0.2f);
		render_circle(app, 10);
		render_circle(app, 8);
		glColor4f(1.f, 0.9f, 1.0f, 1.f);
		render_circle(app, 6);
	}

	void render_circle(const window& app, float r)
	{
		const float segments = 16;
		glBegin(GL_TRIANGLE_STRIP);
		for (auto i = 0; i <= segments; i++)
		{
			auto t = 2 * M_PI * float(i) / segments;

			glVertex2f(x * app.width() + cos(t) * r,
				y * app.height() + sin(t) * r);

			glVertex2f(x * app.width(),
				y * app.height());
		}
		glEnd();
	}

	// This helper function is used to find the button
	// closest to the mouse cursor
	// Since we are only comparing this distance, sqrt can be safely skipped
	float dist_2d(const toggle& other) const
	{
		return pow(x - other.x, 2) + pow(y - other.y, 2);
	}

	float x;
	float y;
	bool selected = false;
};

// Application state shared between the main-thread and GLFW events



class state
{
public:

	bool mouse_down = false;
	toggle ruler_start;
	toggle ruler_end;
	toggle ruler_bottomS;
	toggle ruler_bottomE;

	float dfrom;
	float dto;
	float dbottomS;
	float dbottomE;

	int depthw = 640;
	int depthh = 480;
	rs2_intrinsics intr;
	uchar* buf = NULL;
	float baseD = -1;
	plane basep;
	std::vector<float> baselist;
	float* dbuf =NULL;
	snapshot_metrics        _latest_metrics;
	float                   _depth_scale_units;
	std::mutex      _m;

	float best = FLT_MAX;
	float3 pos[5][4];
	int    ipos = 0;
	void reset()
	{
		if (best < FLT_MAX)
		{
			for (int i = 0; i < 5; i++)
			{
				for (int j = 0; j < 4; j++)
				{

					pos[i][j] = float3{ 0,0,0 };

				}
			}
			best = FLT_MAX;
			ruler_start = { 0.45f, 0.4f };
			ruler_end = { 0.55f, 0.4f };
			ruler_bottomS = { 0.45f, 0.6f };
			ruler_bottomE = { 0.55f, 0.6f };
		}
	}


	float3 to3d(pixel u ,float & dep)
	{
		float upixel[2]; // From pixel
		float upoint[3]; // From point (in 3D)
		float bpoint[3]; // From point (in 3D)
		upixel[0] = u.first;
		upixel[1] = u.second;
		float udis = dbuf[u.second * depthw + u.first];
		float bdis = udis = udis * _depth_scale_units;
		udis = bdis * _latest_metrics.distance / baseD;
		rs2_deproject_pixel_to_point(upoint, &intr, upixel, udis);
		rs2_deproject_pixel_to_point(bpoint, &intr, upixel, bdis);
		plane p = _latest_metrics.p;
		
		// Find distance from point to the reconstructed plane
		auto dist2plane = p.a * upoint[0] + p.b * upoint[1] + p.c * upoint[2] + p.d;
		auto dist2bplane = basep.a * bpoint[0] + basep.b * bpoint[1] + basep.c * bpoint[2] + basep.d;
		//		// Project the point to plane in 3D and find distance to the intersection point
		float3 upointi = { float(upoint[0] - dist2plane * p.a),
										float(upoint[1] - dist2plane * p.b),
										float(upoint[2] - dist2plane * p.c) };

		float3 bpointi = { float(bpoint[0] - dist2bplane * basep.a),
										float(bpoint[1] - dist2bplane * basep.b),
										float(bpoint[2] - dist2bplane * basep.c) };

		float3 dis = bpointi - upointi;
		dep = dis.length();



		return upointi;

	}

	float dist_3d(pixel u, pixel v)
	{
		if (baseD < 0 || buf == NULL)
			return 0;


		float upixel[2]; // From pixel
		float upoint[3]; // From point (in 3D)

		float vpixel[2]; // To pixel
		float vpoint[3]; // To point (in 3D)

		// Copy pixels into the arrays (to match rsutil signatures)
		upixel[0] = u.first;
		upixel[1] = u.second;
		vpixel[0] = v.first;
		vpixel[1] = v.second;


		float udis = dbuf[u.second * depthw + u.first];
		float vdis = dbuf[v.second * depthw + v.first];

		udis = udis * _depth_scale_units * _latest_metrics.distance / baseD;
		vdis = vdis * _depth_scale_units * _latest_metrics.distance / baseD;
		//rs2_intrinsics intr = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
		rs2_deproject_pixel_to_point(upoint, &intr, upixel, udis);
		rs2_deproject_pixel_to_point(vpoint, &intr, vpixel, vdis);

		plane p = _latest_metrics.p;
		// Find distance from point to the reconstructed plane
		auto dist2plane = p.a * upoint[0] + p.b * upoint[1] + p.c * upoint[2] + p.d;
		//		// Project the point to plane in 3D and find distance to the intersection point
		float3 upointi = { float(upoint[0] - dist2plane * p.a),
										float(upoint[1] - dist2plane * p.b),
										float(upoint[2] - dist2plane * p.c) };

		auto dist2plane2 = p.a * vpoint[0] + p.b * vpoint[1] + p.c * vpoint[2] + p.d;
		//		// Project the point to plane in 3D and find distance to the intersection point
		float3 vpointi = { float(vpoint[0] - dist2plane2 * p.a),
										float(vpoint[1] - dist2plane2 * p.b),
										float(vpoint[2] - dist2plane2 * p.c) };


		return sqrt(pow(upointi.x - vpointi.x, 2) +
			pow(upointi.y - vpointi.y, 2) +
			pow(upointi.z - vpointi.z, 2));
	}


	void update(cv::Point2f* var) {
		int corner1 = -1;
		int corner2 = -1;
		int corner3 = -1;
		int corner4 = -1;

		bool bnice = true;
		for (int i = 0; i < 4; i++)
		{
			if (var[i].x < 0)
			{
				var[i].x = 0;
				bnice = false;

			}


			if (var[i].x >= depthw)
			{
				var[i].x = depthw - 1;
				bnice = false;
			}

			if (var[i].y < 0)
			{
				var[i].y = 0;
				bnice = false;
			}

			if (var[i].y >= depthh)
			{
				var[i].y = depthh - 1;
				bnice = false;
			}
		}
		
		
		

		for (int k = 0; k < 4; k++)
		{
			float d1 = var[k].x * var[k].x + var[k].y * var[k].y;
			float d2 = var[k].x * var[k].x + (depthh - var[k].y) * (depthh - var[k].y);
			float d3 = (depthw - var[k].x) * (depthw - var[k].x) + var[k].y * var[k].y;
			float d4 = (depthw - var[k].x) * (depthw - var[k].x) + (depthh - var[k].y) * (depthh - var[k].y);

			if (d1 < d2 && d1 < d3 && d1 < d4)
			{
				corner1 = k;
			}
			if (d2 < d1 && d2 < d3 && d2 < d4)
			{
				corner2 = k;
			}
			if (d3 < d2 && d3 < d1 && d3 < d4)
			{
				corner3 = k;
			}
			if (d4 < d2 && d4 < d3 && d4 < d1)
			{
				corner4 = k;
			}

		}
		if (corner1 == corner2 || corner1 == corner3 || corner1 == corner4 || corner2 == corner3 ||
			corner2 == corner4 || corner3 == corner4 || corner1 == -1 || corner2 == -1 ||
			corner3 ==-1 || corner4== -1)
			bnice = false;

		if (bnice)
		{
			

			auto from_pixel = pixel{ var[corner1].x , var[corner1].y };
			auto to_pixel = pixel{ var[corner2].x , var[corner2].y };
			auto bottomE_pixel = pixel{ var[corner4].x , var[corner4].y };
			auto bottomS_pixel = pixel{ var[corner3].x , var[corner3].y };
			auto from = to3d( from_pixel, dfrom);
			auto to = to3d(to_pixel, dto);
			auto bottomE = to3d(bottomE_pixel, dbottomE);
			auto bottomS = to3d(bottomS_pixel, dbottomS);


			pos[ipos][0] = from;
			pos[ipos][1] = to;
			pos[ipos][2] = bottomE;
			pos[ipos][3] = bottomS;
		
			
			std::cout << "****depth: "<< dfrom << "," << dto << "," << dbottomE << "," << dbottomS << std::endl;
			auto cost = estimate(from, to, bottomE, bottomS);
			
			ipos++;
			ipos = ipos % 5;


			if (cost < best)
			{
				ruler_start = toggle(var[corner1].x / depthw, var[corner1].y / depthh);
				ruler_end = toggle(var[corner2].x / depthw, var[corner2].y / depthh);
				ruler_bottomE = toggle(var[corner4].x / depthw, var[corner4].y / depthh);
				ruler_bottomS = toggle(var[corner3].x / depthw, var[corner3].y / depthh);
				best = cost;
			}
		}
	}

	float estimate(float3 from, float3 to, float3  bottomE, float3 bottomS)
	{

		bool bfind = false;

		for (int i = 0; i < 5; i++)
		{
			if (i == ipos)
				continue;

			auto k1 = from - pos[i][0];
			auto k2 = to - pos[i][1];
			auto k3 = bottomE - pos[i][2];
			auto k4 = bottomS - pos[i][3];

			float v1 = abs(k1.length());
			float v2 = abs(k2.length());
			float v3 = abs(k3.length());
			float v4 = abs(k4.length());

			float total = v1 + v2 + v3 + v4;
			if (total < 0.01 && total > 0 )
			{
				bfind = true;
				break;

			}


		}

		if (!bfind)
			return FLT_MAX;


		auto d1 = from - to;
		auto d2 = bottomS - bottomE;
		auto d3 = from - bottomS;
		auto d4 = to - bottomE;
		float w = d1.length() + d2.length() + d3.length() + d4.length();
		if ( w < 0.001)//too short
			return FLT_MAX;

		float w0 = d1.length() + d2.length()- (d3.length()+d4.length());
		float w1 = d1.length() - d2.length();
		float w2 = d3.length() - d4.length();

		float w3=d1.normalize()* d3.normalize();
		float w4 = d1.normalize() * d4.normalize();
		float w5 = d4.normalize() * d2.normalize();
		float w6 = d2.normalize() * d3.normalize();

		const float  p1=1;
		const float  p2 = 100;
		const float  p3 = 25;
		float c0 = 1 / w;
		float c1 = abs(w0)*p1;
		float c2 = (abs(w1) + abs(w2))*p2;
		float c3 = (abs(w3) + abs(w4) + abs(w5) + abs(w6))*p3;
		std::cout << c0 << "," << c1 << "," << c2 << "," << c3 << std::endl;
		return c0+c1+c2+c3;
	}

	void render(const window& app)
	{
		// Render the ruler
		ruler_start.render(app);
		ruler_end.render(app);
		ruler_bottomS.render(app);
		ruler_bottomE.render(app);

		
	}


	boolean check()
	{
		if (ruler_start.x < 0 || ruler_start.x >= 1)
			return true;

		if (ruler_start.y < 0 || ruler_start.y >= 1)
			return true;

		if (ruler_end.x < 0 || ruler_end.x >= 1)
			return true;

		if (ruler_end.y < 0 || ruler_end.y >= 1)
			return true;

		
		if (ruler_bottomS.x < 0 || ruler_bottomS.x >= 1)
			return true;

		if (ruler_bottomS.y < 0 || ruler_bottomS.y >= 1)
			return true;
		if (ruler_bottomE.x < 0 || ruler_bottomE.x >= 1)
			return true;

		if (ruler_bottomE.y < 0 || ruler_bottomE.y >= 1)
			return true;

		return false;

	}

	boolean render_simple_distance(
		 state& s,
		const window& app)
	{
		boolean ret=false;


		if (s.check())
		{
			reset();
			return false;
		}

		pixel center, h, bottom, bsh;

		glColor4f(0.f, 0.0f, 0.0f, 0.2f);
		draw_line(s.ruler_start.x * app.width(),
			s.ruler_start.y * app.height(),
			s.ruler_end.x * app.width(),
			s.ruler_end.y * app.height(), 9);
		draw_line(s.ruler_bottomE.x * app.width(),
			s.ruler_bottomE.y * app.height(),
			s.ruler_end.x * app.width(),
			s.ruler_end.y * app.height(), 9);

		draw_line(s.ruler_start.x * app.width(),
			s.ruler_start.y * app.height(),
			s.ruler_bottomS.x * app.width(),
			s.ruler_bottomS.y * app.height(), 9);
		draw_line(s.ruler_bottomE.x * app.width(),
			s.ruler_bottomE.y * app.height(),
			s.ruler_bottomS.x * app.width(),
			s.ruler_bottomS.y * app.height(), 9);



		glColor4f(0.f, 0.0f, 0.0f, 0.3f);
		draw_line(s.ruler_start.x * app.width(),
			s.ruler_start.y * app.height(),
			s.ruler_end.x * app.width(),
			s.ruler_end.y * app.height(), 7);

		draw_line(s.ruler_bottomE.x * app.width(),
			s.ruler_bottomE.y * app.height(),
			s.ruler_end.x * app.width(),
			s.ruler_end.y * app.height(), 7);


		draw_line(s.ruler_start.x * app.width(),
			s.ruler_start.y * app.height(),
			s.ruler_bottomS.x * app.width(),
			s.ruler_bottomS.y * app.height(), 7);

		draw_line(s.ruler_bottomE.x * app.width(),
			s.ruler_bottomE.y * app.height(),
			s.ruler_bottomS.x * app.width(),
			s.ruler_bottomS.y * app.height(), 7);

		glColor4f(1.f, 1.0f, 1.0f, 1.f);
		draw_line(s.ruler_start.x * app.width(),
			s.ruler_start.y * app.height(),
			s.ruler_end.x * app.width(),
			s.ruler_end.y * app.height(), 3);

		draw_line(s.ruler_bottomE.x * app.width(),
			s.ruler_bottomE.y * app.height(),
			s.ruler_end.x * app.width(),
			s.ruler_end.y * app.height(), 3);
		draw_line(s.ruler_start.x * app.width(),
			s.ruler_start.y * app.height(),
			s.ruler_bottomS.x * app.width(),
			s.ruler_bottomS.y * app.height(), 3);

		draw_line(s.ruler_bottomE.x * app.width(),
			s.ruler_bottomE.y * app.height(),
			s.ruler_bottomS.x * app.width(),
			s.ruler_bottomS.y * app.height(), 3);



		auto from_pixel = pixel{ ruler_start.x * depthw, ruler_start.y * depthh };
		auto to_pixel = pixel{ ruler_end.x * depthw, ruler_end.y * depthh };
		auto be_pixel = pixel{ ruler_bottomE.x * depthw, ruler_bottomE.y * depthh };
		auto bs_pixel = pixel{ ruler_bottomS.x * depthw, ruler_bottomS.y * depthh };

		std::stringstream ds, dh, db, de;
		ds << int(dfrom * 1000) << " mm";
		dh << int(dto * 1000) << " mm";
		db << int(dbottomS * 1000) << " mm";
		de << int(dbottomE * 1000) << " mm";

		draw_text(from_pixel.first +15, from_pixel.second+15, ds.str().c_str());
		draw_text(to_pixel.first + 15, to_pixel.second + 15, dh.str().c_str());
		draw_text(be_pixel.first + 15, be_pixel.second + 15, de.str().c_str());
		draw_text(bs_pixel.first + 15, bs_pixel.second + 15, db.str().c_str());


		float air_dist = dist_3d( from_pixel, to_pixel);
		float h_dist = dist_3d( be_pixel, to_pixel);
		float b_dist = dist_3d( be_pixel, bs_pixel);
		float eh_dist = dist_3d( from_pixel, bs_pixel);


		center.first = (from_pixel.first + to_pixel.first) / 2;
		center.second = (from_pixel.second + to_pixel.second) / 2;

		h.first = (be_pixel.first + to_pixel.first) / 2;
		h.second = (be_pixel.second + to_pixel.second) / 2;

		bottom.first = (bs_pixel.first + be_pixel.first) / 2;
		bottom.second = (bs_pixel.second + be_pixel.second) / 2;

		bsh.first = (bs_pixel.first + from_pixel.first) / 2;
		bsh.second = (bs_pixel.second + from_pixel.second) / 2;




		std::stringstream ss, sh, sb, se;
		ss << int(air_dist * 1000) << " mm";
		sh << int(h_dist * 1000) << " mm";
		sb << int(b_dist * 1000) << " mm";
		se << int(eh_dist * 1000) << " mm";


		auto str = ss.str();
		auto strh = sh.str();
		auto strb = sb.str();
		auto stre = se.str();

		auto x = (float(center.first) / depthw) * app.width() + 15;
		auto y = (float(center.second) / depthh) * app.height() + 15;

		auto xh = (float(h.first) / depthw) * app.width() + 15;
		auto yh = (float(h.second) / depthh) * app.height() + 15;

		auto xb = (float(bottom.first) / depthw) * app.width() + 15;
		auto yb = (float(bottom.second) / depthh) * app.height() + 15;

		auto xe = (float(bsh.first) / depthw) * app.width() + 15;
		auto ye = (float(bsh.second) / depthh) * app.height() + 15;


		auto w = stb_easy_font_width((char*)str.c_str());
		auto wh = stb_easy_font_width((char*)strh.c_str());

		// Draw dark background for the text label
		glColor4f(0.f, 0.f, 0.f, 0.4f);
		glBegin(GL_TRIANGLES);
		glVertex2f(x - 3, y - 10);
		glVertex2f(x + w + 2, y - 10);
		glVertex2f(x + w + 2, y + 2);
		glVertex2f(x + w + 2, y + 2);
		glVertex2f(x - 3, y + 2);
		glVertex2f(x - 3, y - 10);
		glEnd();

		glBegin(GL_TRIANGLES);
		glVertex2f(xh - 3, yh - 10);
		glVertex2f(xh + wh + 2, yh - 10);
		glVertex2f(xh + wh + 2, yh + 2);
		glVertex2f(xh + wh + 2, yh + 2);
		glVertex2f(xh - 3, yh + 2);
		glVertex2f(xh - 3, yh - 10);
		glEnd();


		glBegin(GL_TRIANGLES);
		glVertex2f(xb - 3, yb - 10);
		glVertex2f(xb + wh + 2, yb - 10);
		glVertex2f(xb + wh + 2, yb + 2);
		glVertex2f(xb + wh + 2, yb + 2);
		glVertex2f(xb - 3, yb + 2);
		glVertex2f(xb - 3, yb - 10);
		glEnd();

		glBegin(GL_TRIANGLES);
		glVertex2f(xe - 3, ye - 10);
		glVertex2f(xe + wh + 2, ye - 10);
		glVertex2f(xe + wh + 2, ye + 2);
		glVertex2f(xe + wh + 2, ye + 2);
		glVertex2f(xe - 3, ye + 2);
		glVertex2f(xe - 3, ye - 10);
		glEnd();

		// Draw white text label
		glColor4f(1.f, 1.f, 1.f, 1.f);
		draw_text(x, y, str.c_str());
		draw_text(xh, yh, strh.c_str());
		draw_text(xb, yb, strb.c_str());
		draw_text(xe, ye, stre.c_str());


		if (baseD < 0)
		{
			if (baselist.size() < 100)
			{
				if (_latest_metrics.distance > 100 && _latest_metrics.distance < 1000)
					baselist.push_back(_latest_metrics.distance);
			}
			else
			{
				float sum = 0;

				for (int i = 0; i < 100; i++) {

					float v = baselist[i];
					sum += v;

				}

				baseD = sum / 100;
				basep = _latest_metrics.p;
				ret = true;
				//constructPlane(depth);
			}



		}


		std::stringstream sdis;
		sdis << "depth:" << baseD - _latest_metrics.distance << " mm";
		auto sstr = sdis.str();
		draw_text(x, y + 50, sstr.c_str());

		return ret;
	}



	void draw_line(float x0, float y0, float x1, float y1, int width)
	{
		glPushAttrib(GL_ENABLE_BIT);
		glLineStipple(1, 0x00ff);
		glEnable(GL_LINE_STIPPLE);
		glLineWidth(width);
		glBegin(GL_LINE_STRIP);
		glVertex2f(x0, y0);
		glVertex2f(x1, y1);
		glEnd();
		glPopAttrib();
	}



	inline snapshot_metrics analyze_depth_image(
		const rs2::video_frame& frame,
		float units, float baseline_mm,
		const rs2_intrinsics* intrin,
		rs2::region_of_interest roi,
		const int ground_truth_mm,
		bool plane_fit_present,
		std::vector<single_metric_data>& samples,
		bool record,
		callback_type callback)
	{


		auto pixels = (const uint16_t*)frame.get_data();
		const auto w = frame.get_width();
		const auto h = frame.get_height();
		if (buf != NULL)
			memset(buf, 0x00, w * h);
		snapshot_metrics result{ w, h, roi, {} };



		std::vector<float3> roi_pixels;

		//#pragma omp parallel for - TODO optimization envisaged
		for (int y = roi.min_y; y < roi.max_y; ++y)
			for (int x = roi.min_x; x < roi.max_x; ++x)
			{
				auto depth_raw = pixels[y * w + x];

				if (depth_raw)
				{
					// units is float
					float pixel[2] = { float(x), float(y) };
					float point[3];
					auto distance = depth_raw * units;

					rs2_deproject_pixel_to_point(point, intrin, pixel, distance);

					std::lock_guard<std::mutex> lock(_m);
					roi_pixels.push_back({ point[0], point[1], point[2] });
				}
			}



		if (roi_pixels.size() < 3) { // Not enough pixels in RoI to fit a plane
			return result;
		}

		plane p = plane_from_points(roi_pixels);

		if (p == plane{ 0, 0, 0, 0 }) { // The points in RoI don't span a valid plane
			return result;
		}

		//找出error 大的點
		for (int y = 0; y < h; y++)
			for (int x = 0; x < w; x++)
			{

				//if (x >= roi.min_x && x <= roi.max_x && y >=roi.min_y && y <= roi.max_y)
				//	continue;

				auto depth_raw = pixels[y * w + x];

				if (buf != NULL)
				{
					auto dbuf_raw = dbuf[y * w + x];

					if (depth_raw && dbuf_raw)
					{
						auto v = 255 - abs(depth_raw - dbuf_raw) * 3;
						if (v < 0)
							v = 0;

						buf[y * w + x] = v;


					}
					else
						buf[y * w + x] = 0;



				}



			}



		// Calculate intersection of the plane fit with a ray along the center of ROI
		// that by design coincides with the center of the frame
		float3 plane_fit_pivot = approximate_intersection(p, intrin, intrin->width / 2.f, intrin->height / 2.f);
		float plane_fit_to_gt_offset_mm = (ground_truth_mm > 0.f) ? (plane_fit_pivot.z * 1000 - ground_truth_mm) : 0;

		result.p = p;
		result.plane_corners[0] = approximate_intersection(p, intrin, float(roi.min_x), float(roi.min_y));
		result.plane_corners[1] = approximate_intersection(p, intrin, float(roi.max_x), float(roi.min_y));
		result.plane_corners[2] = approximate_intersection(p, intrin, float(roi.max_x), float(roi.max_y));
		result.plane_corners[3] = approximate_intersection(p, intrin, float(roi.min_x), float(roi.max_y));

		// Distance of origin (the camera) from the plane is encoded in parameter D of the plane
		// The parameter represents the euclidian distance (along plane normal) from camera to the plane
		result.distance = static_cast<float>(-p.d * 1000);
		// Angle can be calculated from param C
		result.angle = static_cast<float>(std::acos(std::abs(p.c)) / PI * 180.);


		return result;
	}



	void constructPlane(const rs2::depth_frame& frame)
	{
		if (dbuf == NULL && buf == NULL)
		{

			float upixel[2];
			auto pixels = (const uint16_t*)frame.get_data();
			depthw = frame.get_width();
			depthh = frame.get_height();

			dbuf = new float[depthw * depthh];
			buf = new uchar[depthw * depthh];
			for (int x = 0; x < depthw; x++)
				for (int y = 0; y < depthh; y++)
				{
					dbuf[y * depthw + x] = pixels[y * depthw + x];

				}
		}

	}

};