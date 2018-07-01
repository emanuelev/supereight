/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef APPLICATIONWINDOW_H
#define APPLICATIONWINDOW_H

#include <QMainWindow>
#include <QSlider>
#include <QProgressBar>
#include <QDial>
#include <QDialog>
#include <QLineEdit>
#include <QFormLayout>
#include <QToolButton>
#include <initializer_list>
#include "MainWindow.h"
#include <string>
#include <initializer_list>
#include <string>
#ifdef _OPENMP
#include <omp.h>
#endif
enum VarType {
	INT, FLOAT
};
enum BreakpointType {
	COND_BOOL, COND_FGREATER, COND_FLESS, COND_FEQUAL
};
struct ConditionalBreakpoint {
	QString name;
	BreakpointType type;
	void *value;
};

class ButtonChoices: public QWidget {
	Q_OBJECT
public:
	ButtonChoices(QWidget *parent, const char* name, void *variable,
			VarType type, void (*callback)());
	QPushButton *button();
	void updateValue();
	QActionGroup *actionGroup;
private:
	QString *name;
	void *value;
	VarType type;
	void (*callback)();
	QWidget *parent;
	QPushButton *_button;

private slots:
	void changeButtonChoices(QAction *action);
};

class Slider {
public:
	QString *name;
	QSlider *slider;
	int *value;
};
class Dial {
public:
	QString *name;
	QDial *dial;
	int *value;
};

class CheckBox {
public:
	QCheckBox *checkbox;
	bool *value;
};

class ApplicationWindow: public QMainWindow {
	Q_OBJECT
	friend class Viewers;
public:
	enum FileMenuEnum {
		OPEN_FILE,
		OPEN_SCENE,
		USE_CAMERA,
		LOOP_SEQ,
		SAVE_LAYOUT,
		DUMP_VOLUME,
		EDIT_BREAKPOINTS,
		CLEAR_BREAKPOINTS,
		ADD_VIEWER,
		EXIT
	};

	struct DumpFunction {
		const char *label;
		void (*callback)();
	};

	ApplicationWindow();
	Viewers *viewers;

	void addSlider(const char *label, int min, int max, int *value);
	void addDial(const char *label, int min, int max, int *value);
	void addButtonChoices(const char *label, int min, int max, int *value,
			void (*callback)()= NULL);
	void addButtonChoices(const char *label, std::vector<const char *> entries,
			std::vector<int> values, int *variable, void (*callback)()=NULL);
	void addButtonChoices(const char *label, std::vector<const char *> entries,
			std::vector<float> values, float *variable,
			void (*callback)()=NULL);
	std::string fileOpenSelector(std::string title, std::string directory,
			std::string filter);
	std::string fileSaveSelector(std::string title, std::string directory,
			std::string filter);
	void errorBox(std::string message);
	void addCheckBox(const char *label, bool *value);
	void setIdleFunction(void (*_callback)());
	void setCameraFunction(bool *isOpen,
			CameraState (*_callback)(CameraState, std::string));
	void setDumpFunction(const char *label, void (*_callback)());
	void setBreakpoints(QString values);
	void addConditionalBreakpoint(QString label, BreakpointType type,
			int value);
	void addConditionalBreakpoint(QString label, BreakpointType type,
			double value);
	void setConditionalBreakpoint(QString label, int value);
	ConditionalBreakpoint *getConditionalBreakpoint(QString label);
	void setConditionalStart(int value);
	void setVolumeFunction(void (*_callback)(int, float, int));
	void setLoopEnableFunction(void (*_callback)(bool));
	void setCameraViewFunction(void (*_callback)(bool));
	void setCameraViewFunction(void (*_callback)(bool), bool initial);
	void setResetFunction(void (*_callback)());
	void setRenderModelPointer(bool *doRender);
	void setRotPointer(TooN::SE3<float> *rotVar);
	void setRenderTexturePointer(bool *value);
	void setFilenamePointer(std::string *filename);
	//void setShouldIntegratePointer(bool *value);
	void setTrackResolutionPointer(int *value);
	//void setIntegrateDelay(int *value);
	void setDepthResDelay(int *value);
	void setVolumeDelay(int *value);
	void setOpenMPThreadsPointer(int *value);
	void setParams(int width, int height, float volume, int voxLength);
	void setFrameRateField(char *value);
	void update(int frameID, CameraState state);
	void useOpenNI();
	void updateChoices();

protected slots:
	void keyPressEvent(QKeyEvent *event);
	void timerEvent(QTimerEvent *);
	void changeSlider();
	void changeDial();private slots:
	void cButtonPress();
	void lButtonPress();
	void rButtonPress();
	void uButtonPress();
	void dButtonPress();
	void povButtonPress();
	void cameraButtonPress();
	void cameraStepPress();
	void fileMenuSlot(QAction *action);
	void updatePower();
	void showEvent(QShowEvent * ev);
	void setFrameRateLabel(float rate, int frame);
	void restartFile();
	void startLive();
	void callDumpFunction(QAction *sender);
	void closeEvent(QCloseEvent *event);
	void changeCheckBox();
private:
	void showDebugWindow();

	QToolBar *cameraBar;
	QToolBar *simplifyBar;
	QStatusBar *_statusBar;
	QPushButton *depthResButton;
	QPushButton *volumePrecisionButton;
	QPushButton *volumeSizeButton;
	QPushButton *povButton;
	QPushButton *liveButton;
	QLabel *frameRateLabel;
	QProgressBar *progressBar;
	void setupOrientateToolBar();
	void setupSimplifyToolBar();

	QMenu *fileMenu;
	QTimer *timer;
	//QSlider *integrateDelay;
	QPushButton *cameraButton;
	QPushButton *threadsButton;
	TooN::SE3<float> *rot;
	QList<int> breakpoints;
	QList<ConditionalBreakpoint *> conditionalBreakpoints;
	int _conditionalStart;
	QList<Slider *> sliders;
	QList<CheckBox *> checkboxes;
	QList<Dial *> dials;
	QList<ButtonChoices *> buttonchoices;
	QList<DumpFunction *> dumpFunctions;

	int *trackResPtr;
	bool *render_texture;
	//bool *should_integrate;
	int *integration_rate;
	void (*callback)();
	void (*resetCallback)();
	CameraState (*cameraCallback)(CameraState, std::string);
	void (*volumeCallback)(int, float, int);
	void (*dumpCallback)(std::string);
	void (*loopCallback)(bool);
	void (*cameraViewCallback)(bool);
	bool *cameraOpen;
	bool requireUpdate;
	bool *forceRender;
	int *numThreads;
	int idleTimer;
	QString frameRateField;
	QTimer *powerTimer;
	PowerMonitor *powerMonitor;
	QString dirname;
	void readConfig(QString filename = "./.kfusion_kinectrc");
	void writeConfig(QString filename = "./.kfusion_kinectrc");
	CameraState setCameraState(CameraState state, std::string file);
	int frameNumber;
	bool singleStep;
	void setupParamsGUI();
	void enableMenuItem(FileMenuEnum entry);
	QToolBar *modelBar;
	std::string *video_file;
	void refreshCameraButtons(CameraState newState);
	void povButtonRefresh();
	CameraState _cameraState;
	bool holdProcessing;
};

class NoWrapDial: public QDial {
	Q_OBJECT
public:
	NoWrapDial(QWidget * parent = NULL) :
			QDial(parent) {
	connect(this, SIGNAL(actionTriggered(int)), this, SLOT(onAction(int)));
}
protected slots:
void onAction(int val) {
	static const int minDistance = 1;
	if (val == QAbstractSlider::SliderMove) {
		if (value() == maximum() && sliderPosition()<maximum()-minDistance) {
			this->setSliderPosition(maximum());
		} else if (value() == minimum() && sliderPosition()>minimum()+minDistance) {
			this->setSliderPosition(minimum());
		}
	}
}
};
class DebugWindow: public QDialog {
	Q_OBJECT
public:
	DebugWindow(ApplicationWindow *parent);
	~DebugWindow();
	void showForm();
	void addBoolTriRadioBox(const char *name, int value);
	void addLineEdit(const char *label, const char *value, const char *tooltip =
			NULL);private slots:
	void accept();
	void reject();

private:
	QVBoxLayout *vLayout;
	QFormLayout *fLayout;
	QLineEdit *nameLineEdit;
	ApplicationWindow *_appWindow;
};

#endif // APPLICATIONWINDOW_H
