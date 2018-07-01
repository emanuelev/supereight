/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include <iostream>
#include <stdexcept>
#include <vector_types.h>
#include <OpenNI.h>

const int2 imageSize = { 640, 480 };

class MyDepthFrameAllocator: public openni::VideoStream::FrameAllocator {
private:
	uint16_t *depth_buffers_;

public:
	MyDepthFrameAllocator(uint16_t *depth_buffers) :
			depth_buffers_(depth_buffers) {
	}
	void *allocateFrameBuffer(int) {
		return depth_buffers_;
	}
	void freeFrameBuffer(void *) {
	}
};

class MyColorFrameAllocator: public openni::VideoStream::FrameAllocator {
private:
	uchar3 *rgb_buffer_;

public:
	MyColorFrameAllocator(uchar3 *rgb_buffer) {
		rgb_buffer_ = rgb_buffer;
	}
	void *allocateFrameBuffer(int) {
		return rgb_buffer_;
	}
	void freeFrameBuffer(void *) {
	}
};

int main(int argc, char ** argv) {

	if (argc != 3) {
		std::cout << "bad arguments, two arguments are needed." << std::endl;
		exit(1);
	}

	uint16_t * depthImage = (uint16_t*) malloc(
			imageSize.x * imageSize.y * sizeof(uint16_t));
	uchar3 * rgbImage = (uchar3*) malloc(
			imageSize.x * imageSize.y * sizeof(uchar3));

	openni::Status rc = openni::STATUS_OK;
	openni::Device device;
	openni::VideoStream depth;
	openni::VideoStream rgb;
	openni::VideoFrameRef depthFrame;
	openni::VideoFrameRef colorFrame;

	rc = openni::OpenNI::initialize();
	if (rc != openni::STATUS_OK) {
		std::cout << "Bad initialize.";
		exit(1);
	}

	rc = device.open(argv[1]);
	if (rc != openni::STATUS_OK) {
		std::cout << "Bad input file.";
		exit(1);
	}

	if (device.getSensorInfo(openni::SENSOR_DEPTH) != NULL) {
		rc = depth.create(device, openni::SENSOR_DEPTH);

		if (rc != openni::STATUS_OK) {
			std::cout << "Couldn't create depth stream" << std::endl
					<< openni::OpenNI::getExtendedError() << std::endl;
			return 3;
		} else {
			rc = depth.start();
			if (rc != openni::STATUS_OK) {
				printf("Couldn't start depth stream:\n%s\n",
						openni::OpenNI::getExtendedError());
				depth.destroy();
				return 3;
			}
			depth.stop();

		}

		rc = rgb.create(device, openni::SENSOR_COLOR);

		if (rc != openni::STATUS_OK) {
			std::cout << "Couldn't create color stream" << std::endl
					<< openni::OpenNI::getExtendedError() << std::endl;
			return 3;
		}

	}

	openni::VideoMode depthMode = depth.getVideoMode();
	openni::VideoMode colorMode = rgb.getVideoMode();

	if (depthMode.getResolutionX() != imageSize.x
			|| depthMode.getResolutionY() != imageSize.y) {
		std::cout << "Incorrect depth resolution: "
				<< depthMode.getResolutionX() << " "
				<< depthMode.getResolutionY() << std::endl;
		return 3;
	}
	if (colorMode.getResolutionX() != imageSize.x
			|| colorMode.getResolutionY() != imageSize.y) {
		std::cout << "Incorrect rgb resolution: " << colorMode.getResolutionX()
				<< " " << colorMode.getResolutionY() << std::endl;
		return 3;
	}

	rgb.setMirroringEnabled(false);
	depth.setMirroringEnabled(false);

	device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);

	device.getPlaybackControl()->setRepeatEnabled(false);
	depthFrame.release();

	device.getPlaybackControl()->setSpeed(-1); // Set the playback in a manual mode i.e. read a frame whenever the application requests it

	// The allocators must survive this initialization function.
	MyDepthFrameAllocator *depthAlloc = new MyDepthFrameAllocator(depthImage);
	MyColorFrameAllocator *colorAlloc = new MyColorFrameAllocator(rgbImage);

	// Use allocators to have OpenNI write directly into our buffers
	depth.setFrameBuffersAllocator(depthAlloc);
	depth.start();

	rgb.setFrameBuffersAllocator(colorAlloc);
	rgb.start();

	FILE* pFile = fopen(argv[2], "wb");
	if (!pFile) {
		std::cout << "File opening failed : " << argv[2] << std::endl;
		exit(1);
	}

	for (int frame = 0;
			frame < device.getPlaybackControl()->getNumberOfFrames(depth);
			frame++) {

		std::cout << "\rCurrent frame :" << frame << "     " << std::flush;

		rc = depth.readFrame(&depthFrame);

		if (rc != openni::STATUS_OK) {
			std::cout << "Wait failed" << std::endl;
			exit(1);
		}

		rc = rgb.readFrame(&colorFrame);
		if (rc != openni::STATUS_OK) {
			std::cout << "Wait failed" << std::endl;
			exit(1);
		}

		if (depthFrame.getVideoMode().getPixelFormat()
				!= openni::PIXEL_FORMAT_DEPTH_1_MM
				&& depthFrame.getVideoMode().getPixelFormat()
						!= openni::PIXEL_FORMAT_DEPTH_100_UM) {
			std::cout << "Unexpected frame format" << std::endl;
			exit(1);
		}

		if (colorFrame.getVideoMode().getPixelFormat()
				!= openni::PIXEL_FORMAT_RGB888) {
			std::cout << "Unexpected frame format" << std::endl;
			exit(1);
		}

		int total = 0;

		total += fwrite(&(imageSize), sizeof(imageSize), 1, pFile);
		total += fwrite(depthImage, sizeof(uint16_t), imageSize.x * imageSize.y,
				pFile);
		total += fwrite(&(imageSize), sizeof(imageSize), 1, pFile);
		total += fwrite(rgbImage, sizeof(uchar3), imageSize.x * imageSize.y,
				pFile);

	}
	std::cout << std::endl;

	fclose(pFile);

	std::cout << "End of record : " << argv[2] << std::endl;

	free(depthImage);
	free(rgbImage);

	return 0;

}
