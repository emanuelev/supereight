/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#define EXTERNS TRUE
#include <se/DenseSLAMSystem.h>
#include <stdlib.h>
#include "interface.h"
#include <se/config.h>
#include <perfstats.h>
#include <PowerMonitor.h>

#include "ApplicationWindow.h"
#include "MainWindow.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <QMessageBox>
#include <QApplication>
#include <Eigen/Dense>

// #ifdef __APPLE__
// #include <GLUT/glut.h>
// #elif defined(WIN32)
// #define GLUT_NO_LIB_PRAGMA
// #include <glut.h>
// #else
// #include <GL/glut.h>
// #endif

using namespace std;
//The main application window
static ApplicationWindow *appWindow = NULL;

//We need to know abot the function that will process frames
extern int processAll(DepthReader *reader, bool process_frame, bool render_images,
		Configuration *config, bool reset = false);
extern DepthReader *createReader(Configuration *config, std::string filename =
		""

);
//We need to know where our kfusion object is so we can get required information
static DenseSLAMSystem **pipeline_pp;
static DepthReader **reader_pp;
static Configuration *config;

//force_render is passed to the GUI, and used to tell this file if a GUI event which requires rerendering of models to occur
//Typically this is when we rotate an image
static bool force_render = false;

#ifdef _OPENMP
static int num_threads=1;
static void setThreads() {
	omp_set_num_threads(num_threads);
}
#endif

static bool reset = false;

// The following control the view of the model if we aren't using camera track
static Sophus::SE3<float> rot;
static Sophus::SE3<float> trans;

//use_POV tells us to position the model according to the camera pose, setCameraView is passed to the QT and
//is called if we interact with the view button
static int use_POV = true;
void setCameraView(bool use_POV_) {
	use_POV = use_POV_;
}

// loopEnabled is used to control what we do when a scene finishes a pointer to it is passed to GUI to allow interaction
static bool loopEnabled = false;
void setLoopMode(bool value) {
	loopEnabled = value;
}
//Should we have a power_monitor in the main code we can
extern PowerMonitor *power_monitor;

// We can pass this to the QT and it will allow us to change features in the DenseSLAMSystem
static void newDenseSLAMSystem(bool reset_pose) {
  Eigen::Matrix4f init_pose = (*pipeline_pp)->getPose();

	if (*pipeline_pp)
		delete *pipeline_pp;
	if (!reset_pose) {
		*pipeline_pp = new DenseSLAMSystem(
				Eigen::Vector2i(640 / config->compute_size_ratio, 480 / config->compute_size_ratio),
				config->volume_resolution,
        config->volume_size, init_pose, config->pyramid, *config);
  }
	else {
    Eigen::Matrix<float, 6, 1> twist;
  twist << config->initial_pos_factor.x() * config->volume_size.x(),
					 config->initial_pos_factor.y() * config->volume_size.x(),
				   config->initial_pos_factor.z() * config->volume_size.x(), 0, 0, 0;
		trans = Sophus::SE3<float>::exp(twist);
		rot = Sophus::SE3<float>();
    Eigen::Vector3f init_pose = config->initial_pos_factor.cwiseProduct(config->volume_size);
		*pipeline_pp = new DenseSLAMSystem(
				Eigen::Vector2i(640 / config->compute_size_ratio,
						480 / config->compute_size_ratio),
				config->volume_resolution,
				config->volume_size,
        init_pose,
        config->pyramid, *config);
	}
	appWindow->viewers->setBufferSize(640 / config->compute_size_ratio,
			480 / config->compute_size_ratio);
	reset = true;
}
static void continueWithNewDenseSLAMSystem() {
	newDenseSLAMSystem(false);
}

//The GUI is passed this function and can call it to pause/start the camera or load a new scene
//This is used to control the readers.  They have 3 states
//CAMERA_RUNNING - we can read the next frame when we want
//CAMERA_PAUSED  - we have more data but have opted not to read it at the moment
//CAMERA_CLOSED  - we have more data and shouldn't try to accesss the camera.
//reader==NULL   - we don't even have a camera attached
//in the reader this is stored as 2 bool camera_open (i.e !CAMERA_CLOSED) camera_active = CAMERA_RUNNING
CameraState setEnableCamera(CameraState state, string input_file) {
	//float3 init_poseFactors = default_initial_pos_factor; /* FIXME */
	DepthReader *reader = *reader_pp;
	bool isLive = (state == CAMERA_LIVE) ? true : false;

	if (state == CAMERA_RUNNING || state == CAMERA_LIVE) {
		if (reader == NULL || (reader->camera_open == false)
				|| (state == CAMERA_LIVE)) {
			DepthReader *old_reader = NULL;
			string old_input_file = "";
			//Try to keep up a backup of the reader incase we need to go back to it
			if (reader) {
				//We can't open more than one openni reader and so we have to delete it
				//if the current reader is OpenNI, otherwise we will fail;
				if (reader->getType() == READER_OPENNI) {
					delete (reader);
					reader = NULL;
				}
				old_reader = reader;
				reader = NULL;
				old_input_file = config->input_file;
			}
			if (reader == NULL) {
				config->input_file = input_file;
				reader = createReader(config);

				appWindow->updateChoices();
				if (reader) {
					newDenseSLAMSystem(true);
					appWindow->setCameraFunction(&(reader->camera_active),
							(CameraState (*)(CameraState,
									std::string))&setEnableCamera);

}					else {
						bool camera_open = false;
						appWindow->setCameraFunction(&camera_open, (CameraState (*)(CameraState, std::string))&setEnableCamera);
					}
				*reader_pp = reader;
				if (reader == NULL) {
					if (input_file == "") {
						appWindow->errorBox("Unable to open a depth camera");
						isLive = false;
						config->input_file = old_input_file;
						*reader_pp = old_reader;
						if (reader != NULL) {
							reader = *reader_pp;
							reader->camera_active = false;
							appWindow->setCameraFunction(
									&(reader->camera_active),
									(CameraState (*)(CameraState,
											std::string))&setEnableCamera);}
						} else
						appWindow->errorBox("Unable to open specified file");

					} else {
						if(old_reader!=NULL)
						delete(old_reader);
						old_reader=NULL;
						Stats.reset();
						if((power_monitor!=NULL) && power_monitor->isActive())
						power_monitor->power_stats.reset();
					}
				}
			} else {

				reader->camera_active = true;
				reader->camera_open = true;
			}
		} else {
			if (reader) {
				if(state==CAMERA_PAUSED) {
					reader->camera_active=false;
				} else {
					reader->camera_active=false;
					reader->camera_open=false;
				}
			}

		}
	if (reader == NULL)
		return (CAMERA_CLOSED);

	if (reader->camera_open && reader->camera_active)
		return (isLive ? CAMERA_LIVE : CAMERA_RUNNING);
	if (reader->camera_open && !reader->camera_active)
		return (CAMERA_PAUSED);
	return (CAMERA_CLOSED);
}

//This function is passed to QT and is called whenever we aren't busy i.e in a constant loop
void qtIdle(void) {
	//This will set the view for rendering the model, either to the tracked camera view or the static view    
  Eigen::Matrix4f pose = (rot * trans).matrix();
	if (use_POV)
		(*pipeline_pp)->setViewPose(); //current position as found by track
	else
		(*pipeline_pp)->setViewPose(&pose);
	//If we are are reading a file then get a new frame and process it.
	if ((*reader_pp) && (*reader_pp)->camera_active) {
		int finished = processAll((*reader_pp), true, true, config, reset);
		if (finished) {
			if (loopEnabled) {
				newDenseSLAMSystem(true);
				(*reader_pp)->restart();
			} else {
				(*reader_pp)->camera_active = false;
				(*reader_pp)->camera_open = false;
			}
		} else {
			reset = false;
		}
	} else {
		//If we aren't reading 
		if ((*reader_pp == NULL) || !(*reader_pp)->camera_open
				|| !(*reader_pp)->camera_active)
			if (force_render) {
				processAll((*reader_pp), false, true, config, false);
			}
	}
	//refresh the gui

	appWindow->update(
			*reader_pp != NULL ? (*reader_pp)->getFrameNumber() + 1 : 0,
			((*reader_pp == NULL) || !((*reader_pp)->camera_active)) ?
					CAMERA_CLOSED :
					((*reader_pp)->camera_open ?
							((config->input_file == "") ?
									CAMERA_LIVE : CAMERA_RUNNING) :
							CAMERA_PAUSED));
}

//Should we want to dump data from the gui this function can be passed to the QT and will be called with a filename when needed
void dumpVolume() {
//Call the dump function
	std::string filename = appWindow->fileSaveSelector("Save volume", ".",
			"vol (*.vol);; All files (*.*)");
	if (filename != "")
		(*pipeline_pp)->dumpVolume(filename.c_str());
}
void dumpLog() {
	std::string filename = appWindow->fileSaveSelector("Save sequence log", ".",
			"*.log (*.log);; All files (*.*)");
	if (filename != "") {
		std::ofstream log_stream(filename.c_str());
		Stats.print_all_data(log_stream);
		log_stream.close();
	}
}
void dumpPowerLog() {
	std::string filename = appWindow->fileSaveSelector("Save power log", ".",
			"log (*.prpt);; All files (*.*)");
	if (filename != "" && power_monitor && power_monitor->isActive()) {
		std::ofstream log_stream(filename.c_str());
		power_monitor->power_stats.print_all_data(log_stream);
		log_stream.close();
	}
}

//This function is what sets up the GUI

void qtLinkKinectQt(int argc, char *argv[], DenseSLAMSystem **pipe_,
		DepthReader **depth_reder_, Configuration *config_, void *depth_render,
		void *track_render, void *volume_render, void *input_rgb) {
	pipeline_pp = pipe_;
	config = config_;
	reader_pp = depth_reder_;
  Eigen::Matrix<float, 6, 1> twist;
  twist << config->initial_pos_factor.x() * config->volume_size.x(),
					 config->initial_pos_factor.y() * config->volume_size.x(),
				   config->initial_pos_factor.z() * config->volume_size.x(), 0, 0, 0;
	trans = Sophus::SE3<float>::exp(twist);
	QApplication a(argc, argv);

	//Create a new ApplicationWindow (which holds everything)
	appWindow = new ApplicationWindow();

	//set the function we will call when we are idling (the process loop)
	appWindow->setIdleFunction(qtIdle);

	//Function to call to reset model (also enable reset button)
	appWindow->setResetFunction(&continueWithNewDenseSLAMSystem);

	//Function to call when we change our looping (also enable loop on file menu)
	appWindow->setLoopEnableFunction(&setLoopMode);

	//set function called when camera view of model or static view (enables button) (void function(bool use_POV))
	appWindow->setCameraViewFunction(&setCameraView);

	//rot controls the roation for the viewPose enables appropriate buttons on GUI
	//Rotate controls are disabled in this release
	appWindow->setRotPointer(&rot);
	appWindow->setRenderModelPointer(&force_render);
	appWindow->setFilenamePointer(&(config->input_file));

	//Function to call to dump volume model (also enable dump on file menu)
	appWindow->setDumpFunction("Save Volume", &dumpVolume);
	appWindow->setDumpFunction("Save Statistics log", &dumpLog);
	if (power_monitor && power_monitor->isActive())
		appWindow->setDumpFunction("Save power log ", &dumpPowerLog);

	//Fuction to control camera action, running, paused, closed or file to open, enables various options
	bool camera_active = false; //this is a bodge as we might not have a reader yet therefore camera must be off

	appWindow->setCameraFunction(
			(*reader_pp) ? &((*reader_pp)->camera_active) : &camera_active,
			(CameraState (*)(CameraState, std::string))&setEnableCamera);

			//This sets up the images but is pretty ugly and would be better stashed in DenseSLAMSystem

appWindow	->addButtonChoices("Compute Res",
			{ "640x480", "320x240", "160x120", "80x60" }, { 1, 2, 4, 8 },
			&(config->compute_size_ratio), continueWithNewDenseSLAMSystem);
	appWindow->addButtonChoices("Vol. Size", { "4.0mx4.0mx4.0m",
			"2.0mx2.0mx2.0m", "1.0mx1.0mx1.0m" }, { 4.0, 2.0, 1.0 },
			(float *) (&(config->volume_size.x())), continueWithNewDenseSLAMSystem);
	appWindow->addButtonChoices("Vol. Res", { "1024x1024x1024", "512x512x512",
			"256x256x256", "128x128x128", "64x64x64", "32x32x32" }, { 1024, 512,
			256, 128, 64, 32 }, (int *) &(config->volume_resolution.x()),
			continueWithNewDenseSLAMSystem);

	appWindow->addButtonChoices("ICP threshold", { "1e-4", "1e-5", "1e-6" }, {
			1e-4, 1e-5, 1e-6 }, (float *) &(config->icp_threshold));
	appWindow->addButtonChoices("Mu", { "0.005", "0.011", "0.022", "0.045",
			"0.090", "0.180", "0.360" }, { 0.005, 0.011, 0.022, 0.045, 0.09,
			0.18, 0.36 }, (float *) &(config->mu), continueWithNewDenseSLAMSystem);

	int cwidth = (
			((*reader_pp) == NULL) ? 640 : ((*reader_pp)->getinputSize()).x)
			/ config->compute_size_ratio;
	int cheight = (
			((*reader_pp) == NULL) ? 480 : ((*reader_pp)->getinputSize()).y)
			/ config->compute_size_ratio;
	int width =
			(((*reader_pp) == NULL) ? 640 : ((*reader_pp)->getinputSize()).x);
	int height = (
			((*reader_pp) == NULL) ? 480 : ((*reader_pp)->getinputSize()).y);

	FImage rgb_image = { width, height, GL_RGB, GL_UNSIGNED_BYTE, input_rgb };
	FImage depth_image =
			{ cwidth, cheight, GL_RGBA, GL_UNSIGNED_BYTE, depth_render };
	FImage track_image =
			{ cwidth, cheight, GL_RGBA, GL_UNSIGNED_BYTE, track_render };
	//FImage volume_image = { width, height, GL_RGB, GL_UNSIGNED_BYTE, volume_render};
	FImage volume_image = { config->render_volume_fullsize ? width : cwidth,
			config->render_volume_fullsize ? height : cheight, GL_RGBA,
			GL_UNSIGNED_BYTE, volume_render };
	appWindow->viewers->addViewer(rgb_image, (const char *) "RGB image",
			(const char *) "RGB representation of input, unused in processing",
			NULL, false);
	appWindow->viewers->addViewer(depth_image, (const char *) "Depth image",
			(const char *) "Depth image:\n  White too near\n  Black too far\n  Rainbow from near to far",
			NULL, true);
	appWindow->viewers->addViewer(track_image, (const char *) "Track Model",
			(const char *) "Track model:\n  Grey: OK\n  Black: no input\n  Red: not in image\n  Green: no correspondance\n  Blue: too far away\n  Yellow: wrong normal",
			NULL, true);
	appWindow->viewers->addViewer(volume_image, (const char *) "3D model",
			(const char *) "3D model", NULL, !(config->render_volume_fullsize));

	//The following enables the stats Viewer, the statsEnabled variable is to enable the removal of the capture phase
	//although this is currently not implemented (N.B clearly the variable wouldn't be local!!)
	bool statsEnabled = true;
	appWindow->viewers->addViewer(&Stats, (const char *) "Performance",
			(const char*) "Performance Statistics", &statsEnabled);
	appWindow->viewers->setStatEntry("Performance", { "X", "Y", "Z", "tracked",
			"integrated", "frame" }, false);
	//this is the default field used for calculating the frame rate    
	appWindow->setFrameRateField((char *) "computation");

	if (power_monitor != NULL && power_monitor->isActive()) {
		appWindow->viewers->addViewer(&(power_monitor->power_stats),
				(const char *) "Energy", (const char*) "Energy Statistics",
				&statsEnabled);
		appWindow->viewers->setStatEntry("Energy", { "Sample_time" }, false);
	}

	//There are a number of options  to set a range you can use either a slider a dial or a buttonChoice    
	appWindow->addButtonChoices("Track", 1, 5, &(config->tracking_rate));
	appWindow->addButtonChoices("Integrate", 1, 5, &(config->integration_rate));
	appWindow->addButtonChoices("Render", 1, 5, &(config->rendering_rate));

	//If we have built with openMP add a dropdown to set the number of threads
#ifdef _OPENMP
	num_threads= omp_get_max_threads();
	appWindow->addButtonChoices("Threads", 1, num_threads, &num_threads, setThreads);
#endif

	//For a checkbox you can use
	//bool fred
	//appWindow->addCheckBox("RGB overlay", &(fred));
	appWindow->addConditionalBreakpoint("tracked", COND_BOOL, 2);
	//Apologies for this but the followint tidies up if we have 3 viewers
	appWindow->viewers->reLayout();
	appWindow->setGeometry(0, 0, 1120, 960);
	appWindow->show();
	a.exec();
}
