#pragma once

//#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
//#include <librealsense2/rsutil.h>


#include <Windows.h>

#define _USE_MATH_DEFINES

#include <math.h>
#include <algorithm> 



#include "depth-metrics.h"

 
void intersectLineWithPlane3D(const float* q,
	const float* v,
	const float* w,
	float* p,
	float& depth);



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
	/*pixel get_pixel(depth_frame frm) const
	{
		int px = x * frm.get_width();
		int py = y * frm.get_height();
		return{ px, py };
	}*/

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
	bool                    bdo = true;
	
	float real = 0.0012;
	std::vector<cv::Point2f>  conerPoints;
	float  finalDepth;
	bool bfinal=false;


	rs2::region_of_interest roi{};
	bool mouse_down = false;
	toggle ruler_start;
	toggle ruler_end;
	toggle ruler_bottomS;
	toggle ruler_bottomE;

	float dfrom;
	float dto;
	float dbottomS;
	float dbottomE;
	float3 bpos[4];


	int depthw = 640;
	int depthh = 480;
	rs2_intrinsics intr;
	uchar* buf = NULL;
	float baseD = -1;
	plane basep;
	plane pinside;
	plane poutside;
	std::vector<float> baselist;
	float* dbuf =NULL;
	float* mbuf = NULL;
	snapshot_metrics        _latest_metrics;
	float                   _depth_scale_units;
	std::mutex      _m;

	float best = FLT_MAX;
	float3 pos[5][4];
	int    ipos = 0;
	void reset( bool begin )
	{
		if (begin || best < FLT_MAX)
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
			finalDepth = -1;
			bfinal = false;
		}

		if (begin)
		{
			baseD = -1;
			baselist.clear();

		}

	}

	float3 to3dbyPlane(pixel u,  plane pp )
	{
		float upixel[2]; // From pixel
		float upoint[3]; // From point (in 3D)
		//float dis = static_cast<float>(-pp.d * 1000);
		
		upixel[0] = u.first;
		upixel[1] = u.second;
		/*float udis = dbuf[u.second * depthw + u.first];
		udis = udis * _depth_scale_units;
		udis = udis * dis/ baseD;*/
		rs2_deproject_pixel_to_point(upoint, &intr, upixel, 1);

		


		// Find distance from point to the reconstructed plane
		//auto dist2plane = pp.a * upoint[0] + pp.b * upoint[1] + pp.c * upoint[2] + pp.d;
		float3 ray={ upoint[0],upoint[1],upoint[2] };

		//if (abs(dist2plane) < 0.00001)
		//	return ray;

		//float3 normal={pp.a,pp.b,pp.c };
		ray=ray.normalize();


		float v[3];
		v[0] = ray.x;
		v[1] = ray.y;
		v[2] = ray.z;
		float q[3];
		q[0] = 0;
		q[1] = 0;
		q[2] = 0;
		float p[3];
		float plane[4];
		plane[0] = pp.a;
		plane[1] = pp.b;
		plane[2] = pp.c;
		plane[3] = pp.d;
		float depth;
		intersectLineWithPlane3D(q, v, plane, p, depth);
		float3 ret;
		ret.x = -p[0];
		ret.y = -p[1];
		ret.z = -p[2];

		return ret;


//		normal = normal.normalize();
//		float costheta = ray * normal;
//		if (abs(costheta) < 0.00001)
//		{
//			ray = float3{ upoint[0],upoint[1],upoint[2] }+normal * dist2plane;
//			return ray;
//		}
//
//		float raydis = dist2plane / costheta;
//		ray = ray / ray.z;
//		ray = ray * (udis - raydis);
//
//		//		// Project the point to plane in 3D and find distance to the intersection point
//		/*float3 upointi = { float(upoint[0] - dist2plane * p.a),
//										float(upoint[1] - dist2plane * p.b),
//										float(upoint[2] - dist2plane * p.c) };
//*/
//		return ray;
//

	}



	//float3 to3d(pixel u ,float & dep)
	//{
	//	float upixel[2]; // From pixel
	//	float upoint[3]; // From point (in 3D)
	//	float bpoint[3]; // From point (in 3D)
	//	upixel[0] = u.first;
	//	upixel[1] = u.second;
	//	float udis = dbuf[u.second * depthw + u.first];
	//	float bdis = udis = udis * _depth_scale_units;
	//	udis = bdis * _latest_metrics.distance / baseD;
	//	rs2_deproject_pixel_to_point(upoint, &intr, upixel, udis);
	//	rs2_deproject_pixel_to_point(bpoint, &intr, upixel, bdis);
	//	plane p = _latest_metrics.p;
	//	
	//	// Find distance from point to the reconstructed plane
	//	auto dist2plane = p.a * upoint[0] + p.b * upoint[1] + p.c * upoint[2] + p.d;
	//	auto dist2bplane = basep.a * bpoint[0] + basep.b * bpoint[1] + basep.c * bpoint[2] + basep.d;
	//	//		// Project the point to plane in 3D and find distance to the intersection point
	//	float3 upointi = { float(upoint[0] - dist2plane * p.a),
	//									float(upoint[1] - dist2plane * p.b),
	//									float(upoint[2] - dist2plane * p.c) };
	//	float3 bpointi = { float(bpoint[0] - dist2bplane * basep.a),
	//									float(bpoint[1] - dist2bplane * basep.b),
	//									float(bpoint[2] - dist2bplane * basep.c) };
	//	float3 dis = bpointi - upointi;
	//	dep = dis.length();
	//	return upointi;
	//}

	//float dist_3d(pixel u, pixel v)
	//{
	//	if (baseD < 0 || buf == NULL)
	//		return 0;
	//	float upixel[2]; // From pixel
	//	float upoint[3]; // From point (in 3D)
	//	float vpixel[2]; // To pixel
	//	float vpoint[3]; // To point (in 3D)
	//	// Copy pixels into the arrays (to match rsutil signatures)
	//	upixel[0] = u.first;
	//	upixel[1] = u.second;
	//	vpixel[0] = v.first;
	//	vpixel[1] = v.second;
	//	float udis = dbuf[u.second * depthw + u.first];
	//	float vdis = dbuf[v.second * depthw + v.first];
	//	udis = udis * _depth_scale_units * _latest_metrics.distance / baseD;
	//	vdis = vdis * _depth_scale_units * _latest_metrics.distance / baseD;
	//	//rs2_intrinsics intr = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
	//	rs2_deproject_pixel_to_point(upoint, &intr, upixel, udis);
	//	rs2_deproject_pixel_to_point(vpoint, &intr, vpixel, vdis);
	//	plane p = _latest_metrics.p;
	//	// Find distance from point to the reconstructed plane
	//	auto dist2plane = p.a * upoint[0] + p.b * upoint[1] + p.c * upoint[2] + p.d;
	//	//		// Project the point to plane in 3D and find distance to the intersection point
	//	float3 upointi = { float(upoint[0] - dist2plane * p.a),
	//									float(upoint[1] - dist2plane * p.b),
	//									float(upoint[2] - dist2plane * p.c) };
	//	auto dist2plane2 = p.a * vpoint[0] + p.b * vpoint[1] + p.c * vpoint[2] + p.d;
	//	//		// Project the point to plane in 3D and find distance to the intersection point
	//	float3 vpointi = { float(vpoint[0] - dist2plane2 * p.a),
	//									float(vpoint[1] - dist2plane2 * p.b),
	//									float(vpoint[2] - dist2plane2 * p.c) }
	//	return sqrt(pow(upointi.x - vpointi.x, 2) +
	//		pow(upointi.y - vpointi.y, 2) +
	//		pow(upointi.z - vpointi.z, 2));
	//}



	void update(cv::Point2f* var) {
		
		//if (bnice)
		{
			
		
			
			auto from_pixel = pixel{ var[0].x , var[0].y };
			auto to_pixel = pixel{ var[1].x , var[1].y };
			auto bottomE_pixel = pixel{ var[2].x , var[2].y };
			auto bottomS_pixel = pixel{ var[3].x , var[3].y };

			//float dfrom2, dto2, dbottomE2, dbottomS2;

			//auto from2 = to3d( from_pixel, dfrom2);
			//auto test0 = from2.length();
			//auto to2 = to3d(to_pixel, dto2);
			//auto bottomE2 = to3d(bottomE_pixel, dbottomE2);
			//auto bottomS2 = to3d(bottomS_pixel, dbottomS2);

			auto from = to3dbyPlane(from_pixel, pinside);
			//auto test1 = from.length();
			auto to= to3dbyPlane(to_pixel, pinside);
			auto bottomE = to3dbyPlane(bottomE_pixel, pinside);
			auto bottomS = to3dbyPlane(bottomS_pixel, pinside);
			//auto test2 = from.x * _latest_metrics.p.a + from.y * _latest_metrics.p.b + from.z * _latest_metrics.p.c + _latest_metrics.p.d;

			dfrom = abs(from.x* poutside.a +from.y* poutside.b+from.z* poutside.c + poutside.d);
			dto =abs( to.x * poutside.a + to.y * poutside.b + to.z * poutside.c + poutside.d);
			dbottomE = abs(bottomE.x * poutside.a + bottomE.y * poutside.b + bottomE.z * poutside.c + poutside.d);
			dbottomS =abs( bottomS.x * poutside.a + bottomS.y * poutside.b + bottomS.z * poutside.c + poutside.d);



			float mindis = dfrom;
			if (dto < mindis)
				mindis = dto;
			if (dbottomE < mindis)
				mindis = dbottomE;

			if (dbottomS < mindis)
				mindis = dbottomS;

			



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
				bfinal = false;

				conerPoints.clear();
				for (int i = 0; i < 4; i++)
				{
					conerPoints.push_back(var[i]);
				}

				finalDepth = (mindis * 1000 + baseD - _latest_metrics.distance)/2;

				ruler_start = toggle(var[0].x / depthw, var[0].y / depthh);
				ruler_end = toggle(var[1].x / depthw, var[1].y / depthh);
				ruler_bottomE = toggle(var[2].x / depthw, var[2].y / depthh);
				ruler_bottomS = toggle(var[3].x / depthw, var[3].y / depthh);
				best = cost;

				bpos[0] = from;
				bpos[1] = to;
				bpos[2] = bottomE;
				bpos[3] = bottomS;

			}
		}
	}

	// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
	bool intersection(cv::Vec4f p1, cv::Vec4f p2,
		cv::Point2f& r)
	{
		cv::Point2f x,d1,d2,o1;
			x.x= p2[2] - p1[2];
			x.y = p2[3] - p1[3];
			d1.x = p1[0];
			d1.y = p1[1];
			d2.x = p2[0];
			d2.y = p2[1];
			o1.x = p1[2];
			o1.y = p1[3];


		
		float cross = d1.x * d2.y - d1.y * d2.x;
		if (abs(cross) < /*EPS*/1e-8)
			return false;

		double t1 = (x.x * d2.y - x.y * d2.x) / cross;
		r = o1 + d1 * t1;
		return true;
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
		boolean ret = false;
		float x = 15;
		float y = 15;


		if (s.bdo && s.check())
		{
			reset(false);
			return false;
		}

		pixel center, h, bottom, bsh;
		glColor4f(1.f, 0.0f, 0.0f, 0.2f);
		draw_line(roi.min_x, roi.max_y, roi.min_x, roi.min_y, 3);
		draw_line(roi.min_x, roi.min_y, roi.max_x, roi.min_y, 3);
		draw_line(roi.max_x, roi.min_y, roi.max_x, roi.max_y, 3);
		draw_line(roi.max_x, roi.max_y, roi.min_x, roi.max_y, 3);


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


		if (baseD > 0 && best < FLT_MAX){

		auto from_pixel = pixel{ ruler_start.x * depthw, ruler_start.y * depthh };
		auto to_pixel = pixel{ ruler_end.x * depthw, ruler_end.y * depthh };
		auto be_pixel = pixel{ ruler_bottomE.x * depthw, ruler_bottomE.y * depthh };
		auto bs_pixel = pixel{ ruler_bottomS.x * depthw, ruler_bottomS.y * depthh };

		// auto from = to3dbyPlane(from_pixel, this->_latest_metrics.p);
		// auto to = to3dbyPlane(to_pixel, this->_latest_metrics.p);
		// auto bottomE = to3dbyPlane(be_pixel, this->_latest_metrics.p);
		// auto bottomS = to3dbyPlane(bs_pixel, this->_latest_metrics.p);

		auto from =bpos[0] ;
		auto to=bpos[1] ;
		auto bottomE= bpos[2];
		auto bottomS= bpos[3];

		std::stringstream ds, dh, db, de;
		ds << int(dfrom * 1000) << " mm";
		dh << int(dto * 1000) << " mm";
		db << int(dbottomS * 1000) << " mm";
		de << int(dbottomE * 1000) << " mm";

		draw_text(from_pixel.first + 15, from_pixel.second + 15, ds.str().c_str());
		draw_text(to_pixel.first + 15, to_pixel.second + 15, dh.str().c_str());
		draw_text(be_pixel.first + 15, be_pixel.second + 15, de.str().c_str());
		draw_text(bs_pixel.first + 15, bs_pixel.second + 15, db.str().c_str());


		float air_dist = (bpos[0] - bpos[1]).length();//dist_3d( from_pixel, to_pixel);
		float h_dist = (bpos[2] - bpos[1]).length();//dist_3d( be_pixel, to_pixel);
		float b_dist = (bpos[3] - bpos[2]).length(); //dist_3d( be_pixel, bs_pixel);
		float eh_dist = (bpos[0] - bpos[3]).length();//dist_3d( from_pixel, bs_pixel);


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

		 x = (float(center.first) / depthw) * app.width() + 15;
		 y = (float(center.second) / depthh) * app.height() + 15;

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
	}

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



		//找出error 大的點
		if (buf != NULL)
		{
			for (int y = 0; y < h; y++)
				for (int x = 0; x < w; x++)
				{

					//if (x >= roi.min_x && x <= roi.max_x && y >=roi.min_y && y <= roi.max_y)
					//	continue;

					auto depth_raw = pixels[y * w + x];

					if (depth_raw)
					{

						// units is float
						float pixel[2] = { float(x), float(y) };
						float point[3];
						auto distance = depth_raw * units;

						rs2_deproject_pixel_to_point(point, intrin, pixel, distance);
						auto modify = mbuf[y * w + x];
						auto dist2plane = p.a * point[0] + p.b * point[1] + p.c * point[2] + p.d-modify;
						
						auto dist2bplane = basep.a * point[0] + basep.b * point[1] + basep.c * point[2] + basep.d -modify;
						auto dist =   abs(dist2bplane)- abs(dist2plane);
						if (dist < 0)
						{
							

							buf[y * w + x] = 0 ;

						}
						else
						{
							/*dist = dist * 10000;

							if (dist > 255)
								dist = 255;*/
							buf[y * w + x] = 255;
						}



					}
					else
						buf[y * w + x] = 200;


					/*if (buf != NULL)
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



					}*/



				}

		}//buf != NULL


		return result;
	}



	bool updatePlane(depth_frame& frame, cv::Mat& checkM)
	{
		std::vector<float3> inside_pixels;
		std::vector<float3> outside_pixels;
		auto pixels = (const uint16_t*)frame.get_data();
		auto data = checkM.data;

		//#pragma omp parallel for - TODO optimization envisaged
		for (int y = 0; y < depthh; y++)
			for (int x = 0; x < depthw; x++)
			{
				auto depth_raw = pixels[y * depthw + x];
				auto check = data[y * depthw + x];

				if (depth_raw && data && check > 0)
				{
					int value = (int) check;

					// units is float
					float pixel[2] = { float(x), float(y) };
					float point[3];
					auto distance = depth_raw * _depth_scale_units;

					rs2_deproject_pixel_to_point(point, &intr, pixel, distance);

					std::lock_guard<std::mutex> lock(_m);
					
					if (value == 100) //outside
					{
						outside_pixels.push_back({ point[0], point[1], point[2] });
					}
					else if (value == 200)//inside
					{
						inside_pixels.push_back({ point[0], point[1], point[2] });
					}
				}
			}



		if (inside_pixels.size() < 3 || outside_pixels.size() < 3) { // Not enough pixels in RoI to fit a plane
			return false;
		}

		 pinside = plane_from_points(inside_pixels);
		 poutside = plane_from_points(outside_pixels);

		return true;

	}



	void constructPlane(const rs2::depth_frame& frame)
	{
		if (dbuf == NULL && buf == NULL)
		{

			auto pixels = (const uint16_t*)frame.get_data();
			depthw = frame.get_width();
			depthh = frame.get_height();

			dbuf = new float[depthw * depthh];
			mbuf = new float[depthw * depthh];
			buf = new uchar[depthw * depthh];
			for (int x = 0; x < depthw; x++)
				for (int y = 0; y < depthh; y++)
				{
					dbuf[y * depthw + x] = pixels[y * depthw + x];

				}
		}
		else
		{

			auto pixels = (const uint16_t*)frame.get_data();
			for (int x = 0; x < depthw; x++)
				for (int y = 0; y < depthh; y++)
				{
					dbuf[y * depthw + x] = pixels[y * depthw + x];

				}


		}

		modifyBuffer(frame);
	}

	void modifyBuffer(const rs2::depth_frame& frame)
	{
		auto pixels = (const uint16_t*)frame.get_data();

		for (int x = 0; x < depthw; x++)
			for (int y = 0; y < depthh; y++)
			{

				auto depth_raw = pixels[y * depthw + x];
				float dist2bplane=0;
				if (depth_raw)
				{

					// units is float
					float pixel[2] = { float(x), float(y) };
					float point[3];
					auto distance = depth_raw * _depth_scale_units;

					rs2_deproject_pixel_to_point(point, &intr, pixel, distance);
					
					 dist2bplane = basep.a * point[0] + basep.b * point[1] + basep.c * point[2] + basep.d;
				}

				mbuf[y * depthw + x] = dist2bplane;



			}

	}

};