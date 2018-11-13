/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include <se/DenseSLAMSystem.h>
#include <interface.h>
#include <default_parameters.h>
#include <stdint.h>
#include <vector>
#include <sstream>
#include <string>
#include <cstring>
#include <time.h>
#include <csignal>

#include <sys/types.h>
#include <sys/stat.h>
#include <sstream>
#include <iomanip>
#include <getopt.h>
#include <perfstats.h>

PerfStats Stats;

/***
 * This program loop over a scene recording
 */

int main(int argc, char ** argv) {

	Configuration config = parseArgs(argc, argv);

	// ========= CHECK ARGS =====================

	std::ostream* logstream = &std::cout;
	std::ofstream logfilestream;
	assert(config.compute_size_ratio > 0);
	assert(config.integration_rate > 0);
	assert(config.volume_size.x > 0);
	assert(config.volume_resolution.x > 0);

	if (config.log_file != "") {
		logfilestream.open(config.log_file.c_str());
		logstream = &logfilestream;
	}
	if (config.input_file == "") {
		std::cerr << "No input found." << std::endl;
		print_arguments();
		exit(1);
	}

	// ========= READER INITIALIZATION  =========

	DepthReader * reader;

	if (is_file(config.input_file)) {
		reader = new RawDepthReader(config.input_file, config.fps,
				config.blocking_read);

	} else {
		reader = new SceneDepthReader(config.input_file, config.fps,
				config.blocking_read);
	}

	std::cout.precision(10);
	std::cerr.precision(10);

	float3 init_pose = config.initial_pos_factor * config.volume_size;
	const uint2 inputSize = reader->getinputSize();
	std::cerr << "input Size is = " << inputSize.x << "," << inputSize.y
			<< std::endl;

	//  =========  BASIC PARAMETERS  (input size / computation size )  =========

	const uint2 computationSize = make_uint2(
			inputSize.x / config.compute_size_ratio,
			inputSize.y / config.compute_size_ratio);
	float4 camera = reader->getK() / config.compute_size_ratio;

	if (config.camera_overrided)
		camera = config.camera / config.compute_size_ratio;
	//  =========  BASIC BUFFERS  (input / output )  =========

	// Construction Scene reader and input buffer
	uint16_t* inputDepth = (uint16_t*) malloc(
			sizeof(uint16_t) * inputSize.x * inputSize.y);
	uchar4* depthRender = (uchar4*) malloc(
			sizeof(uchar4) * computationSize.x * computationSize.y);
	uchar4* trackRender = (uchar4*) malloc(
			sizeof(uchar4) * computationSize.x * computationSize.y);
	uchar4* volumeRender = (uchar4*) malloc(
			sizeof(uchar4) * computationSize.x * computationSize.y);

	uint frame = 0;

	DenseSLAMSystem pipeline(
      Eigen::Vector2i(computationSize.x, computationSize.y), 
      Eigen::Vector3i::Constant(static_cast<int>(config.volume_resolution.x)),
			Eigen::Vector3f::Constant(config.volume_size.x), 
      Eigen::Vector3f(init_pose.x, init_pose.y, init_pose.z), 
      config.pyramid, config);
     
  bool bilateralfilter = false;
	std::chrono::time_point<std::chrono::steady_clock> timings[7];
	timings[0] = std::chrono::steady_clock::now();

	*logstream
			<< "frame\tacquisition\tpreprocessing\ttracking\tintegration\traycasting\trendering\tcomputation\ttotal    \tX          \tY          \tZ         \ttracked   \tintegrated"
			<< std::endl;
	logstream->setf(std::ios::fixed, std::ios::floatfield);

    while (reader->readNextDepthFrame(inputDepth)) {

		timings[1] = std::chrono::steady_clock::now();

		pipeline.preprocessing(inputDepth, 
        Eigen::Vector2i(inputSize.x, inputSize.y), config.bilateralFilter);

		timings[2] = std::chrono::steady_clock::now();

		bool tracked = pipeline.tracking(
        Eigen::Vector4f(camera.x, camera.y, camera.z, camera.w), 
        config.icp_threshold,
				config.tracking_rate, frame);


		timings[3] = std::chrono::steady_clock::now();

		Matrix4 pose = pipeline.getPose();

		float xt = pose.data[0].w - init_pose.x;
		float yt = pose.data[1].w - init_pose.y;
		float zt = pose.data[2].w - init_pose.z;


		bool integrated = pipeline.integration(
        Eigen::Vector4f(camera.x, camera.y, camera.z, camera.w), 
        config.integration_rate,
				config.mu, frame);

		timings[4] = std::chrono::steady_clock::now();

		bool raycast = pipeline.raycasting(
        Eigen::Vector4f(camera.x, camera.y, camera.z, camera.w), 
        config.mu, frame);

		timings[5] = std::chrono::steady_clock::now();

		pipeline.renderDepth(depthRender,   Eigen::Vector2i(computationSize.x, computationSize.y));
		pipeline.renderTrack(trackRender,   Eigen::Vector2i(computationSize.x, computationSize.y));
		pipeline.renderVolume(volumeRender, 
        Eigen::Vector2i(computationSize.x, computationSize.y), frame,
				config.rendering_rate, 
        Eigen::Vector4f(camera.x, camera.y, camera.z, camera.w), 
        0.75 * config.mu);

		timings[6] = std::chrono::steady_clock::now();

		*logstream << frame << "\t" 
      << std::chrono::duration<double>(timings[1] - timings[0]).count() << "\t" //  acquisition
      << std::chrono::duration<double>(timings[2] - timings[1]).count() << "\t"     //  preprocessing
      << std::chrono::duration<double>(timings[3] - timings[2]).count() << "\t"     //  tracking
      << std::chrono::duration<double>(timings[4] - timings[3]).count() << "\t"     //  integration
      << std::chrono::duration<double>(timings[5] - timings[4]).count() << "\t"     //  raycasting
      << std::chrono::duration<double>(timings[6] - timings[5]).count() << "\t"     //  rendering
      << std::chrono::duration<double>(timings[5] - timings[1]).count() << "\t"     //  computation
      << std::chrono::duration<double>(timings[6] - timings[0]).count() << "\t"     //  total
      << xt << "\t" << yt << "\t" << zt << "\t"     //  X,Y,Z
      << tracked << "        \t" << integrated // tracked and integrated flags
      << std::endl;

		frame++;
		timings[0] = std::chrono::steady_clock::now();
	}

    std::shared_ptr<se::Octree<FieldType> > map_ptr;
    pipeline.getMap(map_ptr);
    map_ptr->save("test.bin");
    
    // ==========     DUMP VOLUME      =========

  if (config.dump_volume_file != "") {
    auto s = std::chrono::steady_clock::now();
    pipeline.dump_volume(config.dump_volume_file);
    auto e = std::chrono::steady_clock::now();
    std::cout << "Mesh generated in " << (e - s).count() << " seconds" << std::endl;
  }

	//  =========  FREE BASIC BUFFERS  =========

	free(inputDepth);
	free(depthRender);
	free(trackRender);
	free(volumeRender);
	return 0;

}

