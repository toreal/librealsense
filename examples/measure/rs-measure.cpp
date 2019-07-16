// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>




#include "example.hpp"          // Include short list of convenience functions for rendering

#include "depth-metrics.h"

using namespace rs2;


// This example will require several standard data-structures and algorithms:
#define _USE_MATH_DEFINES
#include <math.h>
#include <queue>
#include <unordered_set>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>

#include "opencv2/core.hpp"

#include "opencv2/imgproc.hpp"

#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <stdio.h>



using namespace cv;

using pixel = std::pair<int, int>;

// Distance 3D is used to calculate real 3D distance between two pixels
float dist_3d(const rs2::depth_frame& frame, pixel u, pixel v);

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
    pixel get_pixel(rs2::depth_frame frm) const
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
struct state
{
    bool mouse_down = false;
    toggle ruler_start;
    toggle ruler_end;
	toggle ruler_mid;
};

// Helper function to register to UI events
void register_glfw_callbacks(window& app, state& app_state);

// Distance rendering functions:

// Simple distance is the classic pythagorean distance between 3D points
// This distance ignores the topology of the object and can cut both through
// air and through solid
void render_simple_distance(const rs2::depth_frame& depth,
                            const state& s,
                            const window& app);



rs2_intrinsics          _depth_intrinsic;

float                   _depth_scale_units;
float                   _stereo_baseline_mm;
int                     _ground_truth_mm;
bool                    _use_gt;
bool                    _plane_fit=true;
rs2::region_of_interest      _roi;
float                   _roi_percentage=0.4f;
rs2::snapshot_metrics        _latest_metrics;
bool                    _active;

std::mutex      _m;
uchar* buf;
float baseD = -1;
std::vector<float> baselist;
rs2::float3 *  dbuf = new rs2::float3[320*240];

void constructPlane(const rs2::depth_frame& frame)
{
	float upixel[2];

	
	for ( int x=0 ; x < 320; x++)
		for (int y = 0; y < 240; y++)
		{
			upixel[0] = x;
			upixel[1] = y;
			auto udist = frame.get_distance(x, y);			

			// Deproject from pixel to point in 3D
			rs2_intrinsics intr = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
			rs2_deproject_pixel_to_point(&dbuf[y*320+x].x, &intr, upixel, udist);


		}

}

std::tuple<int, bool> get_inputs() 
{
	std::lock_guard<std::mutex> lock(_m);
	return std::make_tuple(_ground_truth_mm, _plane_fit);
}

std::array<rs2::float3, 4> get_plane()
{
	
	return _latest_metrics.plane_corners;
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
	
	memset(buf, 0x00, 640 * 480);
	auto pixels = (const uint16_t*)frame.get_data();
	const auto w = frame.get_width();
	const auto h = frame.get_height();

	snapshot_metrics result{ w, h, roi, {} };



	std::vector<rs2::float3> roi_pixels;

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
	for (int y = 0; y < h; ++y)
		for (int x = 0; x < w; ++x)
		{

			//if (x >= roi.min_x && x <= roi.max_x && y >=roi.min_y && y <= roi.max_y)
			//	continue;

			auto depth_raw = pixels[y * w + x];

			if (depth_raw)
			{
				// units is float
				float pixel[2] = { float(x), float(y) };
				//float point[3];
				rs2::float3 pp;
				auto distance = depth_raw * units;

				rs2_deproject_pixel_to_point(&pp.x, intrin, pixel, distance);
				
				float ret=evaluate_plane(p, pp);		
				if (ret < 0)
					ret = 0;
				int val = (int)(ret * 50000);
				if (val > 255)
					val = 255;
					buf[y * 320 + x] =val ;
			}
		}



	// Calculate intersection of the plane fit with a ray along the center of ROI
	// that by design coincides with the center of the frame
	rs2::float3 plane_fit_pivot = approximate_intersection(p, intrin, intrin->width / 2.f, intrin->height / 2.f);
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

	//callback(roi_pixels, p, roi, baseline_mm, intrin->fx, ground_truth_mm, plane_fit_present,
	//	plane_fit_to_gt_offset_mm, result.distance, record, samples);

	//// Calculate normal
	//auto n = rs2::float3{ p.a, p.b, p.c };
	//auto cam = rs2::float3{ 0.f, 0.f, -1.f };
	//auto dot = n * cam;
	//auto u = cam - n * dot;

	//result.angle_x = u.x;
	//result.angle_y = u.y;

	return result;
}



int main(int argc, char * argv[]) try
{
    // OpenGL textures for the color and depth frames
    texture depth_image, color_image;
	buf = new uchar[640 * 480];
	

    // Colorizer is used to visualize depth data
    rs2::colorizer color_map;
    // Use black to white color map
    color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2.f);
    // Decimation filter reduces the amount of data (while preserving best samples)
    rs2::decimation_filter dec;
    // If the demo is too slow, make sure you run in Release (-DCMAKE_BUILD_TYPE=Release)
    // but you can also increase the following parameter to decimate depth more (reducing quality)
    dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    // Define transformations from and to Disparity domain
    rs2::disparity_transform depth2disparity;
    rs2::disparity_transform disparity2depth(false);
    // Define spatial filter (edge-preserving)
    rs2::spatial_filter spat;
    // Enable hole-filling
    // Hole filling is an agressive heuristic and it gets the depth wrong many times
    // However, this demo is not built to handle holes
    // (the shortest-path will always prefer to "cut" through the holes since they have zero 3D distance)
    spat.set_option(RS2_OPTION_HOLES_FILL, 5); // 5 = fill all the zero pixels
    // Define temporal filter
    rs2::temporal_filter temp;
    // Spatially align all streams to depth viewport
    // We do this because:
    //   a. Usually depth has wider FOV, and we only really need depth for this demo
    //   b. We don't want to introduce new holes
    rs2::align align_to(RS2_STREAM_DEPTH);

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    rs2::config cfg;
		

//std::vector<std::shared_ptr<metric_plot>> _plots;
	//metrics_recorder _recorder;
	std::string  _camera_info;
	

	//rs2::callback_type callback = [&](
	//	const std::vector<rs2::float3>& points,
	//	const rs2::plane p,
	//	const rs2::region_of_interest roi,
	//	const float baseline_mm,
	//	const float focal_length_pixels,
	//	const int ground_truth_mm,
	//	const bool plane_fit,
	//	const float plane_fit_to_ground_truth_mm,
	//	const float distance_mm,
	//	bool record,		
	//	std::vector<rs2::single_metric_data>& samples)
	//{
	//	float TO_METERS = _depth_scale_units;// model.get_depth_scale();
	//	static const float TO_MM = 1000.f;
	//	static const float TO_PERCENT = 100.f;
	//	// Calculate fill rate relative to the ROI
	//	auto fill_rate = points.size() / float((roi.max_x - roi.min_x) * (roi.max_y - roi.min_y)) * TO_PERCENT;
	//	//fill->add_value(fill_rate);
	//	//if (record) samples.push_back({ fill->get_name(),  fill_rate });
	//	if (!plane_fit) return;
	//	const float bf_factor = baseline_mm * focal_length_pixels * TO_METERS; // also convert point units from mm to meter
	//	std::vector<rs2::float3> points_set = points;
	//	std::vector<float> distances;
	//	std::vector<float> disparities;
	//	std::vector<float> gt_errors;
	//	// Reserve memory for the data
	//	distances.reserve(points.size());
	//	disparities.reserve(points.size());
	//	if (ground_truth_mm) gt_errors.reserve(points.size());
	//	// Remove outliers [below 0.5% and above 99.5%)
	//	std::sort(points_set.begin(), points_set.end(), [](const rs2::float3& a, const rs2::float3& b) { return a.z < b.z; });
	//	size_t outliers = points_set.size() / 200;
	//	points_set.erase(points_set.begin(), points_set.begin() + outliers); // crop min 0.5% of the dataset
	//	points_set.resize(points_set.size() - outliers); // crop max 0.5% of the dataset
	//	// Convert Z values into Depth values by aligning the Fitted plane with the Ground Truth (GT) plane
	//	// Calculate distance and disparity of Z values to the fitted plane.
	//	// Use the rotated plane fit to calculate GT errors
	//	for (auto point : points_set)
	//	{
	//		// Find distance from point to the reconstructed plane
	//		auto dist2plane = p.a * point.x + p.b * point.y + p.c * point.z + p.d;
	//		// Project the point to plane in 3D and find distance to the intersection point
	//		rs2::float3 plane_intersect = { float(point.x - dist2plane * p.a),
	//										float(point.y - dist2plane * p.b),
	//										float(point.z - dist2plane * p.c) };
	//		// Store distance, disparity and gt- error
	//		distances.push_back(dist2plane * TO_MM);
	//		disparities.push_back(bf_factor / point.length() - bf_factor / plane_intersect.length());
	//		// The negative dist2plane represents a point closer to the camera than the fitted plane
	//		if (ground_truth_mm) gt_errors.push_back(plane_fit_to_ground_truth_mm + (dist2plane * TO_MM));
	//	}
	//	// Show Z accuracy metric only when Ground Truth is available
	//	//z_accuracy->enable(ground_truth_mm > 0);
	//	if (ground_truth_mm)
	//	{
	//		std::sort(begin(gt_errors), end(gt_errors));
	//		auto gt_median = gt_errors[gt_errors.size() / 2];
	//		auto accuracy = TO_PERCENT * (gt_median / ground_truth_mm);
	//		//z_accuracy->add_value(accuracy);
	//		//if (record) samples.push_back({ z_accuracy->get_name(),  accuracy });
	//	}
	//	// Calculate Sub-pixel RMS for Stereo-based Depth sensors
	//	double total_sq_disparity_diff = 0;
	//	for (auto disparity : disparities)
	//	{
	//		total_sq_disparity_diff += disparity * disparity;
	//	}
	//	auto rms_subpixel_val = static_cast<float>(std::sqrt(total_sq_disparity_diff / disparities.size()));
	//	//sub_pixel_rms_error->add_value(rms_subpixel_val);
	//	//if (record) samples.push_back({ sub_pixel_rms_error->get_name(),  rms_subpixel_val });
	//	// Calculate Plane Fit RMS  (Spatial Noise) mm
	//	//double plane_fit_err_sqr_sum = std::inner_product(distances.begin(), distances.end(), distances.begin(), 0.);
	//	//auto rms_error_val = static_cast<float>(std::sqrt(plane_fit_err_sqr_sum / distances.size()));
	//	//auto rms_error_val_per = TO_PERCENT * (rms_error_val / distance_mm);
	//	//plane_fit_rms_error->add_value(rms_error_val_per);
	//	//if (record) samples.push_back({ plane_fit_rms_error->get_name(),  rms_error_val });
	//};


    cfg.enable_stream(RS2_STREAM_DEPTH); // Enable default depth
    // For the color stream, set format to RGBA
    // To allow blending of the color frame on top of the depth frame
    cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGBA8);
    auto profile = pipe.start(cfg);

    auto sensor = profile.get_device().first<rs2::depth_sensor>();

	_depth_scale_units =sensor.get_depth_scale();


	// Retrieve stereo baseline for supported devices
	auto baseline_mm = -1.f;
	//auto profiles = sensor.get_stream_profiles();
	//auto right_sensor = std::find_if(profiles.begin(), profiles.end(), [](rs2::stream_profile& p)
	//	{ return (p.stream_index() == 2) && (p.stream_type() == RS2_STREAM_INFRARED); });
	//if (right_sensor != profiles.end())
	//{
	//	auto left_sensor = std::find_if(profiles.begin(), profiles.end(), [](rs2::stream_profile& p)
	//		{ return (p.stream_index() == 0) && (p.stream_type() == RS2_STREAM_DEPTH); });
	//	try
	//	{
	//		auto extrin = (*left_sensor).get_extrinsics_to(*right_sensor);
	//		baseline_mm = fabs(extrin.translation[0]) * 1000;  // baseline in mm
	//	}
	//	catch (...) {
	//		auto _error_message = "Extrinsic parameters are not available";
	//	}
	//}

	

    // Set the device to High Accuracy preset of the D400 stereoscopic cameras
    if (sensor && sensor.is<rs2::depth_stereo_sensor>())
    {
        sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
    }

    auto stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();

    // Create a simple OpenGL window for rendering:
    window app(stream.width(), stream.height(), "RealSense Measure Example");
	//window app2(stream.width(), stream.height(), "debug");

    // Define application state and position the ruler buttons
    state app_state;
    app_state.ruler_start = { 0.45f, 0.5f };
    app_state.ruler_end   = { 0.55f, 0.5f };
	app_state.ruler_mid = { 0.55f, 0.5f };
    register_glfw_callbacks(app, app_state);

    // After initial post-processing, frames will flow into this queue:
    rs2::frame_queue postprocessed_frames;

    // Alive boolean will signal the worker threads to finish-up
    std::atomic_bool alive{ true };

    // Video-processing thread will fetch frames from the camera,
    // apply post-processing and send the result to the main thread for rendering
    // It recieves synchronized (but not spatially aligned) pairs
    // and outputs synchronized and aligned pairs
    std::thread video_processing_thread([&]() {
        while (alive)
        {
            // Fetch frames from the pipeline and send them for processing
            rs2::frameset data;
            if (pipe.poll_for_frames(&data))
            {
                // First make the frames spatially aligned
                data = data.apply_filter(align_to);

                // Decimation will reduce the resultion of the depth image,
                // closing small holes and speeding-up the algorithm
                data = data.apply_filter(dec);

                // To make sure far-away objects are filtered proportionally
                // we try to switch to disparity domain
                data = data.apply_filter(depth2disparity);

                // Apply spatial filtering
                data = data.apply_filter(spat);

                // Apply temporal filtering
                data = data.apply_filter(temp);

                // If we are in disparity domain, switch back to depth
                data = data.apply_filter(disparity2depth);

                //// Apply color map for visualization of depth
                data = data.apply_filter(color_map);

                // Send resulting frames for visualization in the main thread
                postprocessed_frames.enqueue(data);
            }


			std::vector<rs2::single_metric_data> sample;
			for (auto&& f : data)
			{
				auto profile = f.get_profile();
				auto stream_type = profile.stream_type();
				auto stream_format = profile.format();

				if ((RS2_STREAM_DEPTH == stream_type) && (RS2_FORMAT_Z16 == stream_format))
				{
					float su = 0, baseline = -1.f;
					rs2_intrinsics intrin{};
					int gt_mm{};
					bool plane_fit_set{};
					rs2::region_of_interest roi{};
					{
						std::lock_guard<std::mutex> lock(_m);
						su = _depth_scale_units;
						baseline = _stereo_baseline_mm;
						auto depth_profile = profile.as<rs2::video_stream_profile>();
						intrin = depth_profile.get_intrinsics();
						_depth_intrinsic = intrin;
						_roi = { int(intrin.width * (0.5f - 0.5f * _roi_percentage)),
							int(intrin.height * (0.5f - 0.5f * _roi_percentage)),
							int(intrin.width * (0.5f + 0.5f * _roi_percentage)),
							int(intrin.height * (0.5f + 0.5f * _roi_percentage)) };

						roi = _roi;
					}

					std::tie(gt_mm, plane_fit_set) = get_inputs();

					auto metrics = analyze_depth_image(f, su, baseline, &intrin, roi, gt_mm, plane_fit_set, sample, false, NULL);

					{
						std::lock_guard<std::mutex> lock(_m);
						_latest_metrics = metrics;
						_plane_fit = is_valid(get_plane());
					}
				}
			}
			//if (_recorder.is_recording())
			//	_recorder.add_sample(frames, std::move(sample));

			// Artificially slow down the calculation, so even on small ROIs / resolutions
			// the output is updated within reasonable interval (keeping it human readable)
			std::this_thread::sleep_for(std::chrono::milliseconds(80));


        }
    });

    rs2::frameset current_frameset;
	namedWindow("debug", WINDOW_AUTOSIZE);
	namedWindow("dist", WINDOW_AUTOSIZE);
	namedWindow("src", WINDOW_AUTOSIZE);

	std::vector<std::vector<cv::Point>> contours;

	std::vector<cv::Vec4i> hierarchy;
	RNG rng(12345);


    while(app) // Application still alive?
    {
        // Fetch the latest available post-processed frameset
        postprocessed_frames.poll_for_frame(&current_frameset);

        if (current_frameset)
        {
            auto depth = current_frameset.get_depth_frame();
            auto color = current_frameset.get_color_frame();
            auto colorized_depth = current_frameset.first(RS2_STREAM_DEPTH, RS2_FORMAT_RGB8);

            glEnable(GL_BLEND);
            // Use the Alpha channel for blending
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

			

            // First render the colorized depth image
           depth_image.render(colorized_depth, { 0, 0, app.width(), app.height() });

            // Render the color frame (since we have selected RGBA format
            // pixels out of FOV will appear transparent)
            color_image.render(color, { 0, 0, app.width(), app.height() });

            // Render the simple pythagorean distance
            render_simple_distance(depth, app_state, app);

            // Render the ruler
            app_state.ruler_start.render(app);
            app_state.ruler_end.render(app);
			app_state.ruler_mid.render(app);

            glColor3f(1.f, 1.f, 1.f);
            glDisable(GL_BLEND);

			

			// Creating OpenCV Matrix from a color image
			Mat colormat(Size(depth.get_width(), depth.get_height()), CV_8UC1, (void*)buf, Mat::AUTO_STEP);

			Mat dst1, dst2, canny_output;

			Mat ele = getStructuringElement(cv::MORPH_DILATE, Size(10, 10));

			Mat ele2 = getStructuringElement(cv::MORPH_ERODE, Size(10, 10));

			//		cvtColor(gray1, src_gray, CV_BGR2GRAY);

			
			float bd = (baseD - _latest_metrics.distance) * 3;
			if (baseD > 0 && bd > 10)
			{
				

				if (bd > 128)
					bd = 128;



				threshold(colormat, dst1, bd, 255, cv::THRESH_BINARY);

				cv::erode(dst1, dst2, ele2);
				cv:dilate(dst2, dst1, ele);

				imshow("dist", dst1);
				imshow("src", colormat);

				int thresh = 100;


				/// Detect edges using canny

				Canny(dst1, canny_output, thresh, thresh * 2, 3);

				//	cv::imshow("canny_output", canny_output);

					/// Find contours
			
				contours.clear();
				hierarchy.clear();

				findContours(canny_output, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, Point(0, 0));
				
				if (contours.size() > 0)
				{
					std::vector<cv::Point> cnt = contours[0];
					for (int i = 1; i < contours.size(); i++)
					{
						if (contours[i].size() > cnt.size())
							cnt = contours[i];
					}
					RotatedRect rect= minAreaRect(cnt);
					Point2f var[4];
					rect.points(var);

					circle(canny_output, var[0], 5, Scalar(128));
					circle(canny_output, var[1], 5, Scalar(128));
					circle(canny_output, var[2], 5, Scalar(128));
					circle(canny_output, var[3], 5, Scalar(128));

					app_state.ruler_start = toggle(var[0].x / 320, var[0].y / 240);
					app_state.ruler_end = toggle(var[1].x / 320, var[1].y / 240);
					app_state.ruler_mid = toggle(var[2].x / 320, var[2].y / 240);

					
				}


				//uchar* dbuf = colormat.data;


				imshow("debug", canny_output);
				//waitKey(0);
			}//has baseD
        }
    }

    // Signal threads to finish and wait until they do
    alive = false;
    video_processing_thread.join();

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

float dist_3d(const rs2::depth_frame& frame, pixel u, pixel v)
{
    float upixel[2]; // From pixel
	float* upoint;// [3] ; // From point (in 3D)

    float vpixel[2]; // To pixel
	float* vpoint;// [3] ; // To point (in 3D)

    // Copy pixels into the arrays (to match rsutil signatures)
    upixel[0] = u.first;
    upixel[1] = u.second;
    vpixel[0] = v.first;
    vpixel[1] = v.second;

    // Query the frame for distance
    // Note: this can be optimized
    // It is not recommended to issue an API call for each pixel
    // (since the compiler can't inline these)
    // However, in this example it is not one of the bottlenecks
    //auto udist = frame.get_distance(upixel[0], upixel[1]);
    //auto vdist = frame.get_distance(vpixel[0], vpixel[1]);

    // Deproject from pixel to point in 3D
    //rs2_intrinsics intr = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
    //rs2_deproject_pixel_to_point(upoint, &intr, upixel, udist);
    //rs2_deproject_pixel_to_point(vpoint, &intr, vpixel, vdist);

    // Calculate euclidean distance between the two points

	upoint = &dbuf[u.second * 320 + u.first].x;
	vpoint = &dbuf[v.second * 320 + v.first].y;

    return sqrt(pow(upoint[0] - vpoint[0], 2) +
                pow(upoint[1] - vpoint[1], 2) +
                pow(upoint[2] - vpoint[2], 2));
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

void render_simple_distance(const rs2::depth_frame& depth,
                            const state& s,
                            const window& app)
{
    pixel center,h;

    glColor4f(0.f, 0.0f, 0.0f, 0.2f);
    draw_line(s.ruler_start.x * app.width(),
        s.ruler_start.y * app.height(),
        s.ruler_end.x   * app.width(),
        s.ruler_end.y   * app.height(), 9);
	draw_line(s.ruler_mid.x * app.width(),
		s.ruler_mid.y * app.height(),
		s.ruler_end.x * app.width(),
		s.ruler_end.y * app.height(), 9);

    glColor4f(0.f, 0.0f, 0.0f, 0.3f);
    draw_line(s.ruler_start.x * app.width(),
        s.ruler_start.y * app.height(),
        s.ruler_end.x   * app.width(),
        s.ruler_end.y   * app.height(), 7);

	draw_line(s.ruler_mid.x * app.width(),
		s.ruler_mid.y * app.height(),
		s.ruler_end.x * app.width(),
		s.ruler_end.y * app.height(), 7);

    glColor4f(1.f, 1.0f, 1.0f, 1.f);
    draw_line(s.ruler_start.x * app.width(),
              s.ruler_start.y * app.height(),
              s.ruler_end.x   * app.width(),
              s.ruler_end.y   * app.height(), 3);

	draw_line(s.ruler_mid.x * app.width(),
		s.ruler_mid.y * app.height(),
		s.ruler_end.x * app.width(),
		s.ruler_end.y * app.height(), 3);




    auto from_pixel = s.ruler_start.get_pixel(depth);
    auto to_pixel =   s.ruler_end.get_pixel(depth);
	auto mid_pixel = s.ruler_mid.get_pixel(depth);
    float air_dist = dist_3d(depth, from_pixel, to_pixel);
	float h_dist = dist_3d(depth, mid_pixel, to_pixel);

    center.first  = (from_pixel.first + to_pixel.first) / 2;
    center.second = (from_pixel.second + to_pixel.second) / 2;

	h.first = (mid_pixel.first + to_pixel.first) / 2;
	h.second = (mid_pixel.second + to_pixel.second) / 2;



    std::stringstream ss,sh;
    ss << int(air_dist * 1000) << " mm";
	sh << int(h_dist * 1000) << " mm";
    auto str = ss.str();
	auto strh = sh.str();
    auto x = (float(center.first)  / depth.get_width())  * app.width() + 15;
    auto y = (float(center.second) / depth.get_height()) * app.height() + 15;

	auto xh = (float(h.first) / depth.get_width()) * app.width() + 15;
	auto yh = (float(h.second) / depth.get_height()) * app.height() + 15;

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


    // Draw white text label
    glColor4f(1.f, 1.f, 1.f, 1.f);
    draw_text(x, y, str.c_str());
	draw_text(xh, yh, strh.c_str());


	if (baseD < 0)
	{
		if (baselist.size() < 100 && _latest_metrics.distance > 100 && _latest_metrics.distance < 1000)
			baselist.push_back(_latest_metrics.distance);
		else
		{
			float sum = 0; 
		
			for (int i = 0; i < 100; i++) {

				float v = baselist[i];
				sum += v;

			}

			baseD = sum / 100;
			constructPlane(depth);
		}



	}

	
	std::stringstream sdis;
	sdis << "depth:" << baseD - _latest_metrics.distance  << " mm";
	auto sstr = sdis.str();
	draw_text(x, y+50, sstr.c_str());
}

// Implement drag&drop behaviour for the buttons:
void register_glfw_callbacks(window& app, state& app_state)
{
    app.on_left_mouse = [&](bool pressed)
    {
        app_state.mouse_down = pressed;
    };

    app.on_mouse_move = [&](double x, double y)
    {
        toggle cursor{ float(x) / app.width(), float(y) / app.height() };
        std::vector<toggle*> toggles{
            &app_state.ruler_start,
            &app_state.ruler_end };

        if (app_state.mouse_down)
        {
            toggle* best = toggles.front();
            for (auto&& t : toggles)
            {
                if (t->dist_2d(cursor) < best->dist_2d(cursor))
                {
                    best = t;
                }
            }
            best->selected = true;
        }
        else
        {
            for (auto&& t : toggles) t->selected = false;
        }

        for (auto&& t : toggles)
        {
            if (t->selected) *t = cursor;
        }
    };
}

