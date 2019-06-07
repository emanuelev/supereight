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
	assert(config.volume_size.x() > 0);
	assert(config.volume_resolution.x() > 0);

//	std::vector<Eigen::Vector3i>* updated_blocks;
//	std::vector<Eigen::Vector3i>* frontier_blocks;

	vec3i *updated_blocks;
	vec3i *frontier_blocks;
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

  Eigen::Vector3f init_pose = config.initial_pos_factor.cwiseProduct(config.volume_size);
	const uint2 inputSize = reader->getinputSize();
	std::cerr << "input Size is = " << inputSize.x << "," << inputSize.y
			<< std::endl;

	//  =========  BASIC PARAMETERS  (input size / computation size )  =========

	const uint2 computationSize = make_uint2(
			inputSize.x / config.compute_size_ratio,
			inputSize.y / config.compute_size_ratio);
  Eigen::Vector4f camera = reader->getK() / config.compute_size_ratio;

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
      config.volume_resolution, config.volume_size, 
      init_pose,
      config.pyramid, config);
     
	std::chrono::time_point<std::chrono::steady_clock> timings[7];
	timings[0] = std::chrono::steady_clock::now();

	*logstream
			<< "frame\tacquisition\tpreprocessing\ttracking\tintegration\traycasting\trendering\tcomputation\ttotal    \tX          \tY          \tZ         \ttracked   \tintegrated"
			<< std::endl;
	logstream->setf(std::ios::fixed, std::ios::floatfield);

    while (reader->readNextDepthFrame(inputDepth)) {

		bool tracked = false, integrated = false;

		timings[1] = std::chrono::steady_clock::now();

		pipeline.preprocessing(inputDepth, 
          Eigen::Vector2i(inputSize.x, inputSize.y), config.bilateral_filter);

		timings[2] = std::chrono::steady_clock::now();

		tracked = pipeline.tracking(camera, config.icp_threshold,
				config.tracking_rate, frame);


		timings[3] = std::chrono::steady_clock::now();

    Eigen::Matrix4f pose = pipeline.getPose();

		float xt = pose(0, 3) - init_pose.x();
		float yt = pose(1, 3) - init_pose.y();
		float zt = pose(2, 3) - init_pose.z();


		// Integrate only if tracking was successful or it is one of the first
		// 4 frames.
		if (tracked || (frame <=3)) {
			integrated = pipeline.integration(camera, config.integration_rate,
					config.mu, frame, updated_blocks, frontier_blocks);
		} else {
			integrated = false;
		}

		timings[4] = std::chrono::steady_clock::now();

		pipeline.raycasting(camera, config.mu, frame);

		timings[5] = std::chrono::steady_clock::now();

		pipeline.renderDepth( (unsigned char*)depthRender, Eigen::Vector2i(computationSize.x, computationSize.y));
		pipeline.renderTrack( (unsigned char*)trackRender, Eigen::Vector2i(computationSize.x, computationSize.y));
		pipeline.renderVolume((unsigned char*)volumeRender, 
        Eigen::Vector2i(computationSize.x, computationSize.y), frame,
				config.rendering_rate, camera, 0.75 * config.mu);

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

