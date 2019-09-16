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

using namespace cv;

void prepare3D(std::vector<cv::Point2f> bookPoints, float book);
bool colorBound(const rs2::video_frame& frame, float dep, bool  );
void drawIR(Mat src);
void initFrame(const rs2::video_frame& frame,int);





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
float                   _roi_percentage=0.25f;
bool                     bsave=false;
bool                    bprepare = false;
bool                    bupdate = false;

int                      ninitmog = 20;
//snapshot_metrics        _latest_metrics;
//bool                    _active;



//uchar* buf=NULL;
//float baseD = -1;
//std::vector<float> baselist;
//float* dbuf;
//int depthw=640;
//int depthh=480;




void myregister_glfw_callbacks(window& app, state& app_state);


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
	cfg.enable_stream(RS2_STREAM_INFRARED);
	//cfg.enable_stream(RS2_STREAM_COLOR,1920,1080, RS2_FORMAT_ANY);
    auto profile = pipe.start(cfg);

    auto sensor = profile.get_device().first<rs2::depth_sensor>();

	app_state._depth_scale_units =  sensor.get_depth_scale();
	//app_state.real= sensor.get_depth_scale();

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
    myregister_glfw_callbacks(app, (state&)app_state);

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
				//data = data.apply_filter(align_to);

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
					//float pos[12];
					if (bsave )
					{
						/*std::vector<cv::Point3f> projectedPoints;
						for (int k = 0; k < 4; k++)
						{
							projectedPoints.push_back(Point3f(app_state.bpos[k].x/ app_state.real, app_state.bpos[k].y/ app_state.real, app_state.bpos[k].z/ app_state.real));
						}*/
					app_state.bfinal=colorBound(f, app_state.finalDepth, app_state.bfinal );

					}
				}
				if (stream_type == RS2_STREAM_INFRARED)
				{
					rs2::video_frame frame = f;
					int h = frame.get_height();
					int w = frame.get_width();
					Mat dsrc(h, w, CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);

					if (!bsave)
					{
						
						imwrite("C:\\james\\images\\d0.bmp", dsrc);
						
						bsave = true;
					}
					else
					{
						/*std::vector<cv::Point3f> projectedPoints;
						for (int k = 0; k < 4; k++)
						{
							projectedPoints.push_back(Point3f(app_state.bpos[k].x/ app_state.real, app_state.bpos[k].y / app_state.real, app_state.bpos[k].z / app_state.real));
						}*/
						drawIR(dsrc);

					}

				}


				if ((RS2_STREAM_DEPTH == stream_type) && (RS2_FORMAT_Z16 == stream_format))
				{
					float su = 0, baseline = -1.f;
					rs2_intrinsics intrin{};
					int gt_mm{};
					bool plane_fit_set{};
					
					{
						std::lock_guard<std::mutex> lock(app_state._m);
						su = app_state._depth_scale_units;
						baseline = _stereo_baseline_mm;
						auto depth_profile = profile.as<rs2::video_stream_profile>();
						intrin = depth_profile.get_intrinsics();
						/*intrin.fx = 480.46025019362213;
						intrin.fy = 480.46025019362213;
						intrin.ppx = 320;
						intrin.ppx = 240;*/

						app_state.intr = intrin;
						_roi = { int(intrin.width * (0.5f - 0.5f * _roi_percentage)),
							int(intrin.height * (0.5f - 0.5f * _roi_percentage)),
							int(intrin.width * (0.5f + 0.5f * _roi_percentage)),
							int(intrin.height * (0.5f + 0.5f * _roi_percentage)) };

						app_state.roi = _roi;
					}

					//std::tie(gt_mm, plane_fit_set) = get_inputs();

					auto metrics = app_state.analyze_depth_image(f, su, baseline, &intrin, app_state.roi, _ground_truth_mm, plane_fit_set, sample, false, NULL);

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
	namedWindow("colordist", WINDOW_AUTOSIZE);
	namedWindow("src", WINDOW_AUTOSIZE);
	namedWindow("color", WINDOW_AUTOSIZE);
	namedWindow("mask", WINDOW_AUTOSIZE);
	namedWindow("grabCut", WINDOW_AUTOSIZE);
	namedWindow("img1", WINDOW_AUTOSIZE);
	namedWindow("img2", WINDOW_AUTOSIZE);
	namedWindow("img3", WINDOW_AUTOSIZE);
	namedWindow("img4", WINDOW_AUTOSIZE);


	std::vector<std::vector<cv::Point>> contours;
	std::vector<std::vector<cv::Point>> appros;
	std::vector<cv::Vec4i> hierarchy;
	//RNG rng(12345);


	bool bpressed = false;

    while(app) // Application still alive?
    {
		bool initf = false;

		if ( GetKeyState(VK_SHIFT) & 0x8000)
		{
			
			// Shift down
			if (!bpressed)
			{
				app_state.bdo = !app_state.bdo;
				printf(app_state.bdo ? "true" : "false");
				bpressed = true;
			}

			
		}
		else
			bpressed = false;

		auto  key = GetKeyState(97); //VK_NUMPAD1 '1'
		std::cout << key << std::endl;
		if (key & 0x8000) //'a'
		{
			app_state.reset(true);
		}


		key = GetKeyState(98); //VK_NUMPAD1 '2'
		std::cout << key << std::endl;
		if (key & 0x8000) //'a'
		{
			initf = true;
			ninitmog = 0;
		}


        // Fetch the latest available post-processed frameset
        postprocessed_frames.poll_for_frame(&current_frameset);

        if (current_frameset)
        {
            auto depth = current_frameset.get_depth_frame();
            auto color = current_frameset.get_color_frame();

			if (initf || bprepare)//|| ninitmog < 20 )
			{
				initFrame(color, ninitmog);
				bprepare = false;
				ninitmog++;
			}
			
            auto colorized_depth = current_frameset.first(RS2_STREAM_DEPTH, RS2_FORMAT_RGB8);

            glEnable(GL_BLEND);
            // Use the Alpha channel for blending
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

			

            // First render the colorized depth image
           depth_image.render(colorized_depth, { 0, 0, app.width(), app.height() });

            // Render the color frame (since we have selected RGBA format
            // pixels out of FOV will appear transparent)
            //color_image.render(color, { 0, 0, app.width(), app.height() });

            // Render the simple pythagorean distance
            auto ret=app_state.render_simple_distance( app_state, app);
			if (ret)
				app_state.constructPlane(depth);

            
			app_state.render(app);

            glColor3f(1.f, 1.f, 1.f);
            glDisable(GL_BLEND);

			

			
			float bd = (app_state.baseD - app_state._latest_metrics.distance) * 3;

			if (app_state.baseD > 0 && bd > 5 && app_state.bdo && app_state.buf != NULL)
			{			

				// Creating OpenCV Matrix from a color image
				Mat colormat(Size(depth.get_width(), depth.get_height()), CV_8UC1, (void*)app_state.buf, Mat::AUTO_STEP);
				
				Mat dst1, dst2, canny_output;

				Mat ele = getStructuringElement(cv::MORPH_DILATE, Size(10, 10));
				Mat ele2 = getStructuringElement(cv::MORPH_ERODE, Size(10, 10));

				if (bd > 128)
					bd = 128;

				threshold(colormat, dst1, 128, 255, cv::THRESH_BINARY|cv::THRESH_OTSU);
				cv::erode(dst1, dst2, ele2);
				cv:dilate(dst2, dst1, ele);				
				int thresh = 100;

				imshow("dist", dst1);
				imshow("src", colormat);

				Canny(dst1, canny_output, thresh, thresh * 2, 3);

				contours.clear();
				hierarchy.clear();

				findContours(canny_output, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, Point(0, 0));
				
	


				



				

				int ind = 0;
				if (contours.size() > 0)
				{
					std::vector<cv::Point> cnt = contours[0];
					float mind = FLT_MAX;
					for (int i = 0; i < contours.size(); i++)
					{
						int csize = contours[i].size();
						float sumx = 0;
						float sumy = 0;
						for (int j = 0; j < csize; j++)
						{
							sumx += contours[i].at(j).x;
							sumy += contours[i].at(j).y;

						}

						sumx = sumx / csize;
						sumy = sumy / csize;

						float dis = (sumx - app_state.depthw / 2)* (sumx - app_state.depthw / 2)+ (sumy - app_state.depthh / 2)* (sumy - app_state.depthh / 2);
						if (dis < mind)
						{
							cnt = contours[i];
							ind = i;
							mind = dis;
						}

						//if (contours[i].size() > cnt.size())
						//	cnt = contours[i];
					}


					auto epsilon = 0.1 * arcLength(cnt, true);
					appros.clear();
					appros.resize(1);
					approxPolyDP(cnt, appros[0], epsilon, true);
					//		 convexHull(cnt, appros[0],false,true);



					

				
				//	 drawContours(canny_output, appros, 0, Scalar(200),5);
					if (appros[0].size() == 4)
					{

					int corner1 = -1;
					int corner2 = -1;
					int corner3 = -1;
					int corner4 = -1;

					bool bnice = true;
					for (int i = 0; i < 4; i++)
					{
						if (appros[0][i].x < 0)
						{
							appros[0][i].x = 0;
							bnice = false;

						}


						if (appros[0][i].x >= app_state.depthw)
						{
							appros[0][i].x = app_state.depthw - 1;
							bnice = false;
						}

						if (appros[0][i].y < 0)
						{
							appros[0][i].y = 0;
							bnice = false;
						}

						if (appros[0][i].y >= app_state.depthh)
						{
							appros[0][i].y = app_state.depthh - 1;
							bnice = false;
						}
					}




					for (int k = 0; k < 4; k++)
					{
						float d1 = appros[0][k].x * appros[0][k].x + appros[0][k].y * appros[0][k].y;
						float d2 = appros[0][k].x * appros[0][k].x + (app_state.depthh - appros[0][k].y) * (app_state.depthh - appros[0][k].y);
						float d3 = (app_state.depthw - appros[0][k].x) * (app_state.depthw - appros[0][k].x) + appros[0][k].y * appros[0][k].y;
						float d4 = (app_state.depthw - appros[0][k].x) * (app_state.depthw - appros[0][k].x) + (app_state.depthh - appros[0][k].y) * (app_state.depthh - appros[0][k].y);

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
							corner4 = k;
						}
						if (d4 < d2 && d4 < d3 && d4 < d1)
						{
							corner3 = k;
						}

					}
					if (corner1 == corner2 || corner1 == corner3 || corner1 == corner4 || corner2 == corner3 ||
						corner2 == corner4 || corner3 == corner4 || corner1 == -1 || corner2 == -1 ||
						corner3 == -1 || corner4 == -1)
						bnice = false;


					if (bnice)
					{

						Point var[4],diff[4];

						
						
							var[0].x = appros[0][corner1].x;
							var[0].y = appros[0][corner1].y;

							var[1].x = appros[0][corner2].x;
							var[1].y = appros[0][corner2].y;

							var[2].x = appros[0][corner3].x;
							var[2].y = appros[0][corner3].y;

							var[3].x = appros[0][corner4].x;
							var[3].y = appros[0][corner4].y;


							float mdis[4];
							int	ind[4];
							for (int k = 0; k < 4; k++)
							{
								mdis[k] = FLT_MAX;
							}
							for (int i = 0; i < cnt.size(); i++)
							{
								for (int k = 0; k < 4; k++)
								{
									diff[k] = var[k] - cnt[i];
									float lens = diff[k].dot(diff[k]);
									if (lens < mdis[k])
									{
										mdis[k] = lens;
										ind[k] = i;
									}

								}

							}

							/*std::vector<Vec4i> lines;
			HoughLinesP(canny_output, lines, 1, CV_PI / 180, 50, 50, 10);
			for (size_t i = 0; i < lines.size(); i++)
			{
				Vec4i l = lines[i];
				line(canny_output, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(128),5);
			}*/
							
							cv::Vec4f line_para[4];
							for (int k = 0; k < 4; k++)
							{

								std::vector<Point> pointset;
								pointset.clear();
								int nbeg = ind[k];
								int nend = ind[(k + 1)%4];
								int ncount = nend - nbeg+1;
								if (ncount < 0)
								{
									ncount = nend + cnt.size() - nbeg+1;
								}
								int index = nbeg;
								for (int i = 0; i < ncount; i++)
								{
									
									pointset.push_back(cnt[index]);
									index = (index + 1) % cnt.size();
								}


								cv::fitLine(pointset, line_para[k], cv::DIST_L2, 0, 1e-2, 1e-2);
							//	line(canny_output, Point(line_para[k][0], line_para[k][1]),
							//		Point(line_para[k][2], line_para[k][3]), Scalar(128), 5);
							}






							Point2f corner[4];
							for (int i = 0; i < 4; i++)
							{
								bnice=app_state.intersection(line_para[i], line_para[(i + 3) % 4], corner[i]);
								if (!bnice)
									break;
								//corner[i].x = var[i].x;
								//corner[i].y = var[i].y;
							}

							if (bnice)
							{
								Mat checkM(Size(depth.get_width(), depth.get_height()), CV_8UC1);

								Point root[1][4];
								for (int i = 0; i < 4; i++)
								{
									root[0][i] = corner[i];
								}
								const Point* ppt[1] = { root[0] };
								int npt[] = { 4 };
								polylines(checkM, ppt, npt, 1, 1, Scalar(100), 100, 8, 0);
								fillPoly(checkM, ppt, npt, 1, Scalar(200));
								polylines(checkM, ppt, npt, 1, 1, Scalar(0), 10, 8, 0);

								imshow("debug", checkM);

								app_state.updatePlane(depth, checkM);
								app_state.update(corner);
								bupdate = true;
								if (app_state.conerPoints.size() > 0 && app_state.finalDepth > 0 )
								{
									prepare3D(app_state.conerPoints, app_state.finalDepth);
								}
							}
					}
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

				//imshow("debug", canny_output);

			}//has baseD
			else
			{	
				if (app_state.bdo)
				{
					app_state.reset(false);
					if (bupdate)
					{
						bprepare = true;
						bupdate = false;
					}

				}
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
void myregister_glfw_callbacks(window& app, state& app_state)
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

