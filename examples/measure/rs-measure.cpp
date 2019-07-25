// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

//#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
//#include <librealsense2/rsutil.h>
#include <Windows.h>



        // Include short list of convenience functions for rendering

#include "depth-metrics.h"

using namespace rs2;


// This example will require several standard data-structures and algorithms:


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
#include "state.h"


void colorBound(const rs2::video_frame& frame);

using namespace cv;


//void render_simple_distance(const rs2::depth_frame& depth,
//                            const state& s,
//                            const window& app);




//rs2_intrinsics          _depth_intrinsic;

//float                   _depth_scale_units;
float                   _stereo_baseline_mm;
int                     _ground_truth_mm;
//bool                    _use_gt;
//bool                    _plane_fit=true;
rs2::region_of_interest      _roi;
float                   _roi_percentage=0.4f;
//snapshot_metrics        _latest_metrics;
//bool                    _active;
bool                    bdo = true;


//uchar* buf=NULL;
//float baseD = -1;
//std::vector<float> baselist;
//float* dbuf;
//int depthw=640;
//int depthh=480;







int main(int argc, char * argv[]) try
{
	state app_state;

    // OpenGL textures for the color and depth frames
    texture depth_image, color_image;
	
	

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
	//std::string  _camera_info;
	


    cfg.enable_stream(RS2_STREAM_DEPTH); // Enable default depth
    // For the color stream, set format to RGBA
    // To allow blending of the color frame on top of the depth frame
    cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGBA8);
	//cfg.enable_stream(RS2_STREAM_COLOR,1920,1080, RS2_FORMAT_ANY);
    auto profile = pipe.start(cfg);

    auto sensor = profile.get_device().first<rs2::depth_sensor>();

	app_state._depth_scale_units =sensor.get_depth_scale();


	// Retrieve stereo baseline for supported devices
	auto baseline_mm = -1.f;
	

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
    
    app_state.ruler_start = { 0.45f, 0.4f };
    app_state.ruler_end   = { 0.55f, 0.4f };
	app_state.ruler_bottomS = { 0.45f, 0.6f };
	app_state.ruler_bottomE = { 0.55f, 0.6f };
    register_glfw_callbacks(app, (glfw_state&)app_state);

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
				//data = data.apply_filter(dec);

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


			std::vector<single_metric_data> sample;
			for (auto&& f : data)
			{
				auto profile = f.get_profile();
				auto stream_type = profile.stream_type();
				auto stream_format = profile.format();

				if (RS2_STREAM_COLOR == stream_type)
				{
					colorBound(f);
				}


				if ((RS2_STREAM_DEPTH == stream_type) && (RS2_FORMAT_Z16 == stream_format))
				{
					float su = 0, baseline = -1.f;
					rs2_intrinsics intrin{};
					int gt_mm{};
					bool plane_fit_set{};
					rs2::region_of_interest roi{};
					{
						std::lock_guard<std::mutex> lock(app_state._m);
						su = app_state._depth_scale_units;
						baseline = _stereo_baseline_mm;
						auto depth_profile = profile.as<rs2::video_stream_profile>();
						intrin = depth_profile.get_intrinsics();
						app_state.intr = intrin;
						_roi = { int(intrin.width * (0.5f - 0.5f * _roi_percentage)),
							int(intrin.height * (0.5f - 0.5f * _roi_percentage)),
							int(intrin.width * (0.5f + 0.5f * _roi_percentage)),
							int(intrin.height * (0.5f + 0.5f * _roi_percentage)) };

						roi = _roi;
					}

					//std::tie(gt_mm, plane_fit_set) = get_inputs();

					auto metrics = app_state.analyze_depth_image(f, su, baseline, &intrin, roi, _ground_truth_mm, plane_fit_set, sample, false, NULL);

					{
						std::lock_guard<std::mutex> lock(app_state._m);
						app_state._latest_metrics = metrics;
						//_plane_fit = is_valid(get_plane());
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
	namedWindow("color", WINDOW_AUTOSIZE);
	namedWindow("mask", WINDOW_AUTOSIZE);


	std::vector<std::vector<cv::Point>> contours;
	std::vector<std::vector<cv::Point>> appros;
	std::vector<cv::Vec4i> hierarchy;
	RNG rng(12345);


	bool bpressed = false;

    while(app) // Application still alive?
    {

		if ( GetKeyState(VK_SHIFT) & 0x8000)
		{
			
			// Shift down
			if (!bpressed)
			{
				bdo = !bdo;
				printf(bdo ? "true" : "false");
				bpressed = true;
			}

			
		}
		else
			bpressed = false;

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
            auto ret=app_state.render_simple_distance( app_state, app);
			if (ret)
				app_state.constructPlane(depth);

            
			app_state.render(app);

            glColor3f(1.f, 1.f, 1.f);
            glDisable(GL_BLEND);

			

			
			float bd = (app_state.baseD - app_state._latest_metrics.distance) * 3;

			if (app_state.baseD > 0 && bd > 10 &&bdo && app_state.buf != NULL)
			{			

				// Creating OpenCV Matrix from a color image
				Mat colormat(Size(depth.get_width(), depth.get_height()), CV_8UC1, (void*)app_state.buf, Mat::AUTO_STEP);

				Mat dst1, dst2, canny_output;

				Mat ele = getStructuringElement(cv::MORPH_DILATE, Size(10, 10));
				Mat ele2 = getStructuringElement(cv::MORPH_ERODE, Size(10, 10));

				if (bd > 128)
					bd = 128;

				threshold(colormat, dst1, 128, 255, cv::THRESH_BINARY);
				cv::erode(dst1, dst2, ele2);
				cv:dilate(dst2, dst1, ele);				
				int thresh = 100;

				imshow("dist", dst1);
				imshow("src", colormat);

				Canny(dst1, canny_output, thresh, thresh * 2, 3);

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

					auto epsilon = 0.1 * arcLength(cnt, true);
					appros.clear();
					appros.resize(1);
					 approxPolyDP(cnt, appros[0], epsilon, true);

					 drawContours(canny_output, appros, 0, Scalar(200));

					 std::cout << appros[0].size() << std::endl;

					 Point2f var[4];

					 if (appros[0].size() == 4)
					 {
						 for (int i = 0; i < 4; i++)
						 {
							 var[i].x = appros[0][i].x;
							 var[i].y = appros[0][i].y;

						 }
						 app_state.update(var);
					 }
					 /*else
					 {


						 RotatedRect rect = minAreaRect(cnt);
						 
						 rect.points(var);

						 circle(canny_output, var[0], 5, Scalar(128));
						 circle(canny_output, var[1], 5, Scalar(128));
						 circle(canny_output, var[2], 5, Scalar(128));
						 circle(canny_output, var[3], 5, Scalar(128));					 

					 }*/
					 
					 

				}

				imshow("debug", canny_output);

			}//has baseD
			else
			{
				app_state.reset();
			}
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
            &app_state.ruler_end,
				& app_state.ruler_bottomS,
					& app_state.ruler_bottomE
		};

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
			if (t->selected)
			{
				if (cursor.x < 0)
					cursor.x = 0;
				if (cursor.y < 0)
					cursor.y = 0;
				if (cursor.x >= 1)
					cursor.x = 0.99;
				if (cursor.y >= 1)
					cursor.y = 0.99;

				*t = cursor;
			}
        }
    };
}

