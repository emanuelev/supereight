/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include <QMenuBar>
#include <QFile>
#include <QtWidgets>
#include <QAbstractButton>
#include <time.h>
#include "ApplicationWindow.h"
#include "MainWindow.h"
#include <perfstats.h>
#include <iostream>
#include <string>
#include <assert.h>
#include <unistd.h>

#ifdef __APPLE__
    #include <mach/clock.h>
    #include <mach/mach.h>
#endif

void ApplicationWindow::enableMenuItem(FileMenuEnum entry) {
	for (int i = 0; i < fileMenu->actions().length(); i++) {
		if (fileMenu->actions().at(i)->data().toInt() == entry)
			fileMenu->actions().at(i)->setVisible(true);
	}
}

ApplicationWindow::ApplicationWindow() {
	viewers = new Viewers(this);
	timer = new QTimer(this);
	powerMonitor = NULL;
	cameraButton = NULL;
	powerTimer = new QTimer(this);
	powerTimer->setInterval(200);
	powerTimer->stop();
	dirname = ".";
	_conditionalStart = 5;
	_cameraState = UNINITIALISED;
	connect(powerTimer, SIGNAL(timeout()), this, SLOT(updatePower()));
	requireUpdate = false;
	setCentralWidget(viewers);
	QAction *action;
	holdProcessing = false;
	fileMenu = menuBar()->addMenu(tr("&File"));

	action = fileMenu->addAction("&Open file");
	action->setData(OPEN_FILE);
	action->setVisible(false);

	action = fileMenu->addAction("Open scene");
	action->setData(OPEN_SCENE);
	action->setVisible(false);

	action = fileMenu->addAction("&Use Camera");
	action->setData(USE_CAMERA);
	action->setVisible(false);

	action = fileMenu->addAction("Edit breakpoints");
	action->setVisible(true);
	action->setData(EDIT_BREAKPOINTS);
	//perversly this forces any dump functions before loop sequence
	action = fileMenu->insertSeparator(action);
	action->setText("Dump");

	action = fileMenu->insertSeparator(action);
	action->setText("Debug");

	action = fileMenu->addAction("Clear breakpoints");
	action->setVisible(true);
	action->setData(CLEAR_BREAKPOINTS);

	action = fileMenu->addAction("Add new viewer");
	action->setData(ADD_VIEWER);
	action->setVisible(true);

	action = new QAction("Loop sequence", fileMenu);
	action->setCheckable(true);
	action->setChecked(false);
	action->setData(LOOP_SEQ);
	action->setVisible(false);
	fileMenu->addAction(action);
	action = fileMenu->insertSeparator(action);
	action->setText("Misc");

	action = fileMenu->addAction("&Save preferences");
	action->setVisible(true);
	action->setData(SAVE_LAYOUT);

	action = fileMenu->addAction("&Exit");
	action->setVisible(true);
	action->setData(EXIT);
	action = fileMenu->insertSeparator(action);
	action->setText("terminate");

	connect(fileMenu, SIGNAL(triggered(QAction*)), this, SLOT(fileMenuSlot(QAction*)));
	menuBar()->setHidden(false);
	setupOrientateToolBar();
	setupSimplifyToolBar();
	_statusBar = this->statusBar();
	frameRateLabel = new QLabel("Frame Rate:", this);
	progressBar = new QProgressBar(this);
	_statusBar->addWidget(frameRateLabel, 0);
	_statusBar->addWidget(progressBar, 1);
	progressBar->setVisible(false);
	callback = NULL;
	resetCallback = NULL;
	volumeCallback = NULL;
	loopCallback = NULL;
	rot = NULL;
	render_texture = NULL;
//    should_integrate = NULL;
	setFrameRateField(NULL);
	frameNumber = 0;
	singleStep = false;
	povButton = NULL;

	modelBar = new QToolBar(this);
	addToolBar(Qt::RightToolBarArea, modelBar);
	modelBar->setVisible(false);

	cameraBar = new QToolBar(this);
	addToolBar(Qt::TopToolBarArea, cameraBar);
	cameraBar->setVisible(false);

	simplifyBar = new QToolBar(this);
	addToolBar(Qt::TopToolBarArea, simplifyBar);
	simplifyBar->setVisible(false);

	addToolBar(Qt::TopToolBarArea, simplifyBar);
//    readConfig();
}

void ApplicationWindow::writeConfig(QString filename) {
	char tmp[256];
	QFile file(filename.toStdString().c_str());

	if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
		return;
	file.write(
			"#This file is used to configure the visual appearance and behaviour of kfusion");
	file.write("#It does NOT store values assotiated with specific models");
	file.write("\n#Default directory for storing scenes\n");
	sprintf(tmp, "data_dir \"%s\"\n", dirname.toStdString().c_str());
	file.write(tmp);
	file.write(
			"\n#Configuration for each viewer <id> <enabled> [<field> <enabled>]\n");
	for (int i = 0; i < viewers->getViewerCount(); i++) {
		file.write(viewers->getConfig(i).toStdString().c_str());
	}
	file.write(
			"\n#Define which statistic from the performance window is used to calculate frame rate for status bar\n");
	sprintf(tmp, "rate_statistic \"%s\"\n",
			frameRateField.toStdString().c_str());
	file.write(tmp);

	for (int i = 0; i < fileMenu->actions().length(); i++) {
		if (fileMenu->actions().at(i)->text().toLower() == "loop sequence") {
			file.write(
					"\n#enable_looping defines if we should loop at the end of the sequence: enable_looping <0 | 1>\n");
			sprintf(tmp, "enable_looping %d\n",
					fileMenu->actions().at(i)->isChecked() ? 1 : 0);
			file.write(tmp);
		}
	}
	file.write("\n#Stop at these frames: breakpoints [int] [int] [int].... \n");
	sprintf(tmp, "breakpoints");
	file.write(tmp);
	for (int i = 0; i < breakpoints.length(); i++) {
		sprintf(tmp, " %d", breakpoints.at(i));
		file.write(tmp);
	}
	file.write(
			"\n\n#Conditional breakpoints only become active after conditional_start frames \n");
	sprintf(tmp, "conditional_start %d\n", _conditionalStart);
	file.write(tmp);

	file.write(
			"\n#Conditional breakpoints [ <variable> COND_BOOL <value 0|1|2>] \n");
	sprintf(tmp, "conditional_breakpoints");
	file.write(tmp);
	for (int i = 0; i < conditionalBreakpoints.length(); i++) {
		if (conditionalBreakpoints.at(i)->type == COND_BOOL) {
			sprintf(tmp, " \"%s\" COND_BOOL %d",
					conditionalBreakpoints.at(i)->name.toStdString().c_str(),
					*((int *) (conditionalBreakpoints.at(i)->value)));
			file.write(tmp);
		}
	}
	sprintf(tmp, "\n");

	file.write(tmp);

}

void ApplicationWindow::readConfig(QString filename) {
	char tmp[1024];

	bool debug = false;
	QString word;
	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
		return;
	while (!file.atEnd()) {
		file.readLine(tmp, 1024);
		QString line(tmp);
		QStringList strings = line.split("\""); //divide by quote, should seperate out quoted strings
		QStringList words = strings.at(0).split(QRegExp("[,\\s]+"));
		strings.pop_front();
		if (words.at(0).toLower() == "data_dir") {
			dirname = strings.at(0);
			if (debug)
				std::cerr << "Default data dir for ONI files: "
						<< dirname.toStdString() << std::endl;

		}
		if (words.at(0).toLower() == "viewer") {
			int viewerID = words.at(1).toInt();
			QString title = strings.at(0);
			strings.pop_front();
			bool enabled = strings.at(0).toInt();
			strings.pop_front();
			viewers->addDefConfig(viewerID, title, enabled, strings);
			if (debug)
				std::cerr << "Viewer " << viewerID << " set to "
						<< title.toStdString() << " and "
						<< (enabled ? "enabled" : "disabled") << std::endl;
			continue;
		}
		if (words.at(0).toLower() == "rate_statistic") {
			frameRateField = strings.at(0);
			if (debug)
				std::cerr << "Statistic " << frameRateField.toStdString()
						<< " is used to calculate frame rate" << std::endl;

		}
		if (words.at(0).toLower() == "enable_looping") {
			QAction *loop = NULL;
			for (int i = 0; i < fileMenu->actions().length(); i++) {
				if (fileMenu->actions().at(i)->text().toLower()
						== "loop sequence")
					loop = fileMenu->actions().at(i);
			}
			if (loop) {
				if (words.at(1).toInt() == 1) {
					loop->setChecked(true);
				} else {
					loop->setChecked(false);
				}
				if (loopCallback)
					(loopCallback)(loop->isChecked());
			}
			if (debug)
				std::cerr << "Sequence looping  "
						<< (loop ? "enabled" : "disabled") << std::endl;
		}
		if (words.at(0).toLower() == "breakpoints") {
			for (int j = 1; j < words.length(); j++) {
				if (words.at(j) != "")
					breakpoints.push_back(words.at(j).toInt());
			}
		}
		if (words.at(0).toLower() == "conditional_start") {
			_conditionalStart = words.at(1).toInt();
		}
		if (words.at(0).toLower() == "conditional_breakpoints") {
			for (int j = 0; j < strings.length(); j = j + 2) {
				if (strings.at(j) != "") {
					words = strings.at(j + 1).split(QRegExp("[,\\s]+"));
					setConditionalBreakpoint(strings.at(j),
							words.at(2).toInt());

				}
			}
		}

	}

}

void ApplicationWindow::setFrameRateField(char *value) {
	frameRateField = value;
}
void ApplicationWindow::setFrameRateLabel(float value, int frame) {
	char tmp[256];
	sprintf(tmp, "%s frame rate: %0.2f frame: %d",
			frameRateField.toStdString().c_str(), value, frame);
	frameRateLabel->setText(tmp);
	frameRateLabel->update();
}

void ApplicationWindow::setupSimplifyToolBar() {

}

void ApplicationWindow::refreshCameraButtons(CameraState newState) {
	if (cameraButton) {
		if (newState == CAMERA_PAUSED || newState == CAMERA_CLOSED) {
			cameraButton->setChecked(false);
			cameraButton->setIcon(QIcon(":/icons/images/play.png"));
		} else {
			cameraButton->setChecked(true);
			cameraButton->setIcon(QIcon(":/icons/images/pause.png"));
			if (newState == CAMERA_LIVE) {
				liveButton->setChecked(true);
				liveButton->setIcon(QIcon(":/icons/images/live.png"));
			}
		}
		cameraButton->update();
	}
}
CameraState ApplicationWindow::setCameraState(CameraState state,
		std::string file = "") {
	static CameraState lastState = UNINITIALISED;

	if (cameraCallback) {
		CameraState newState = (cameraCallback)(state, file);
		if ((newState == CAMERA_RUNNING) || (newState == CAMERA_LIVE)) {
			if ((newState == CAMERA_LIVE)
					|| ((file == "")
							&& (liveButton->isChecked()
									|| (lastState == CAMERA_CLOSED)
									|| (lastState == UNINITIALISED)))) {
				liveButton->setChecked(true);
				liveButton->setIcon(QIcon(":/icons/images/live.png"));
			} else {
				liveButton->setChecked(false);
				liveButton->setIcon(QIcon(":/icons/images/film.png"));
			}
		}
		refreshCameraButtons(newState);
		lastState = newState;
		return (newState);
	}
	return (CAMERA_CLOSED);
}

std::string ApplicationWindow::fileSaveSelector(std::string title,
		std::string directory, std::string filter) {
	QString filename = QFileDialog::getSaveFileName(this, title.c_str(),
			directory.c_str(), filter.c_str());
	return (filename.toStdString());
}
std::string ApplicationWindow::fileOpenSelector(std::string title,
		std::string directory, std::string filter) {
	QString filename = QFileDialog::getOpenFileName(this, title.c_str(),
			directory.c_str(), filter.c_str());
	return (filename.toStdString());
}

void ApplicationWindow::errorBox(std::string message) {
	holdProcessing = true;
	QMessageBox::critical(this, "Error", message.c_str());
	holdProcessing = false;
}

void ApplicationWindow::fileMenuSlot(QAction *action) {
	int data = action->data().toInt();
	switch (data) {
	case OPEN_FILE: {
		setCameraState(CAMERA_PAUSED, "");

		QString filename = QFileDialog::getOpenFileName(this,
				"Select a file to open...", dirname,
				tr("ONI, RAW and SCF (*.oni *.raw *.scf);; All files (*.*)"));
		if (filename != "") {
			setCameraState(CAMERA_CLOSED, "");
			dirname = (QFileInfo(filename)).path();
			setCameraState(CAMERA_RUNNING, filename.toStdString());
		}
		break;
	}
	case OPEN_SCENE: {
		setCameraState(CAMERA_PAUSED, "");
		QString filename = QFileDialog::getExistingDirectory(this,
				"Select scene directory...", dirname,
				QFileDialog::ShowDirsOnly);
		if (filename != "") {
			setCameraState(CAMERA_CLOSED, "");
			dirname = (QFileInfo(filename)).path();
			setCameraState(CAMERA_RUNNING, filename.toStdString());
		}
		break;
	}
	case USE_CAMERA: {
		if (cameraCallback) {
			//setCameraState(CAMERA_CLOSED, "");
			setCameraState(CAMERA_LIVE, "");
		}
	}
		break;
	case CLEAR_BREAKPOINTS: {
		breakpoints.clear();
		for (int i = 0; i < conditionalBreakpoints.length(); i++) {
			if (conditionalBreakpoints.at(i)->type == COND_BOOL)
				*((int *) (conditionalBreakpoints.at(i)->value)) = 2;
		}
		break;
	}

	case EDIT_BREAKPOINTS: {
		showDebugWindow();
		//errorBox("Edititing breakpoints not yet implemented, You can add/remove in .kfusion_kinectrc this feature will be included this weekend");
		break;
	}

	case ADD_VIEWER: {
		int viewerID = viewers->getViewerCount();
		QStringList strings;
		viewers->addDefConfig(viewerID, "Performance", true, strings);
	}
	case LOOP_SEQ: {
		if (loopCallback)
			(loopCallback)(action->isChecked());
	}
		break;
	case SAVE_LAYOUT: {
		writeConfig();
	}
		break;
	case DUMP_VOLUME: {
		if (dumpCallback)
			(dumpCallback)("breakpoint.vol");

	}
		break;

	case EXIT:
		exit(0);
	default:
		break;
	}

	if (data >= 1000 && data < (1000 + dumpFunctions.length())) {
		(dumpFunctions.at(data - 1000))->callback();
	}
}

void ApplicationWindow::setupOrientateToolBar() {
}

void ApplicationWindow::setIdleFunction(void (*_callback)()) {
	callback = _callback;
	idleTimer = startTimer(0);
}
void ApplicationWindow::setDumpFunction(const char *label,
		void (*_callback)()) {
	for (int i = 0; i < fileMenu->actions().length(); i++) {
		if ((fileMenu->actions().at(i)->isSeparator())
				&& fileMenu->actions().at(i)->text() == "Dump") {
			QAction *action = new QAction(label, this);
			QAction *tmp = fileMenu->actions().at(
					i + 1 + dumpFunctions.length());
			action->setData(1000 + dumpFunctions.length());
			fileMenu->insertAction(tmp, action);
			if (dumpFunctions.length() == 0) {
				QAction *tmpAction = fileMenu->insertSeparator(tmp);
				tmpAction->setText("more");
			}
			action->setVisible(true);
			DumpFunction *dumpFunction = new DumpFunction;
			dumpFunction->label = label;
			dumpFunction->callback = _callback;
			dumpFunctions.push_back(dumpFunction);
			break;
		}
	}
}
void ApplicationWindow::callDumpFunction(QAction *sender) {
	int req = sender->data().toInt();

}
void ApplicationWindow::setFilenamePointer(std::string *filename) {
	video_file = filename;
}
void ApplicationWindow::restartFile() {
	setCameraState(CAMERA_CLOSED, "");
	setCameraState(CAMERA_RUNNING, *video_file);
}

void ApplicationWindow::setResetFunction(void (*_callback)()) {
	resetCallback = _callback;
	QPushButton *clearButton = new QPushButton(this);
	clearButton->setIcon(QIcon(":/icons/images/clear24.png"));
	clearButton->setIconSize(QSize(24, 24));
	clearButton->setToolTip("Clear the reconstructed model");
	modelBar->addWidget(clearButton);
	connect(clearButton, SIGNAL(clicked()), this, SLOT(cButtonPress()));
	modelBar->setVisible(true);

}
void ApplicationWindow::setVolumeFunction(void (*_callback)(int, float, int)) {
	volumeCallback = _callback;
}
void ApplicationWindow::setLoopEnableFunction(void (*_callback)(bool)) {
	loopCallback = _callback;
	enableMenuItem(LOOP_SEQ);

}
void ApplicationWindow::setCameraViewFunction(void (*_callback)(bool),
		bool initial) {
	setCameraViewFunction(_callback);
	povButton->setChecked(initial);
	povButtonRefresh();
}

void ApplicationWindow::setCameraViewFunction(void (*_callback)(bool)) {
	cameraViewCallback = _callback;
	povButton = new QPushButton(this);
	povButton->setIcon(QIcon(":/icons/images/camera.png"));
	povButton->setToolTip(
			"Toggle view point between camera, dynamic, and fixed, at sequence start point");
	povButton->setCheckable(true);
	povButton->setIconSize(QSize(24, 24));
	modelBar->addWidget(povButton);
	connect(povButton, SIGNAL(clicked()), this, SLOT(povButtonPress()));
	modelBar->setVisible(true);
	povButton->setChecked(true);
}
void ApplicationWindow::startLive() {
	bool isChecked = (((QToolButton *) QObject::sender())->isChecked());
	if (isChecked) {
		//setCameraState(CAMERA_CLOSED, "");
		CameraState state = setCameraState(CAMERA_LIVE, "");
		if (state != CAMERA_LIVE)
			(((QToolButton *) QObject::sender())->setChecked(false));

	} else {
		setCameraState(CAMERA_CLOSED, "");
		((QToolButton *) QObject::sender())->setIcon(
				QIcon(":/icons/images/film.png"));
	}
}

void ApplicationWindow::setCameraFunction(bool *isOpen,
		CameraState (*_callback)(CameraState, std::string)) {
	cameraOpen = isOpen;
	cameraCallback = _callback;
	if (cameraButton == NULL) {

		liveButton = new QPushButton(this);
		liveButton->setCheckable(true);
		liveButton->setToolTip(
				"Toggle between live camera and recorded sequence");
		liveButton->setIcon(QIcon(":/icons/images/film.png"));
		cameraBar->addWidget(liveButton);
		connect(liveButton, SIGNAL(clicked()), this, SLOT(startLive()));

		QPushButton *restartButton = new QPushButton(this);
		restartButton->setCheckable(false);
		restartButton->setIcon(QIcon(":/icons/images/restart.png"));
		restartButton->setToolTip(
				"Return to start of recorded sequence, and reset model");
		cameraBar->addWidget(restartButton);
		connect(restartButton, SIGNAL(clicked()), this, SLOT(restartFile()));

		cameraButton = new QPushButton(this);
		cameraButton->setCheckable(true);
		cameraButton->setChecked(false);
		cameraButton->setToolTip("Start or pause running of input stream");
		cameraButton->setIcon(QIcon(":/icons/images/play.png"));
		cameraBar->addWidget(cameraButton);
		connect(cameraButton, SIGNAL(clicked()), this,
				SLOT(cameraButtonPress()));

		QPushButton *stepButton = new QPushButton(this);
		stepButton->setIcon(QIcon(":/icons/images/step.png"));
		stepButton->setToolTip("Single step the input stream");
		cameraBar->addWidget(stepButton);
		connect(stepButton, SIGNAL(clicked()), this, SLOT(cameraStepPress()));
		cameraBar->setVisible(true);
	}

	cameraButton->setChecked(*cameraOpen);
	enableMenuItem(USE_CAMERA);
	enableMenuItem(OPEN_FILE);
	enableMenuItem(OPEN_SCENE);
}

void ApplicationWindow::setRotPointer(TooN::SE3<float> *rotVar) {
	rot = rotVar;

	QPushButton *rightButton, *leftButton, *upButton, *downButton;

	leftButton = new QPushButton(this);
	leftButton->setToolTip(
			"Rotate model anticlockwise around y axis, after enabling fixed view ");
	leftButton->setIcon(QIcon(":/icons/images/rotccw24.png"));
	leftButton->setIconSize(QSize(24, 24));

	modelBar->addWidget(leftButton);
	connect(leftButton, SIGNAL(clicked()), this, SLOT(lButtonPress()));

	rightButton = new QPushButton(this);
	//rightButton->setText("&R");
	rightButton->setIcon(QIcon(":/icons/images/rotcw24.png"));
	rightButton->setToolTip(
			"Rotate model clockwise around y axis, after enabling fixed view ");
	rightButton->setIconSize(QSize(24, 24));
	modelBar->addWidget(rightButton);
	connect(rightButton, SIGNAL(clicked()), this, SLOT(rButtonPress()));

	upButton = new QPushButton(this);
	//upButton->setText("&U");
	modelBar->addWidget(upButton);
	upButton->setIcon(QIcon(":/icons/images/rotd24.png"));
	upButton->setToolTip(
			"Rotate model anticlockwise around x axis, after enabling fixed view ");
	upButton->setIconSize(QSize(24, 24));
	connect(upButton, SIGNAL(clicked()), this, SLOT(uButtonPress()));

	downButton = new QPushButton(this);
//    downButton->setText("&D");
	downButton->setIcon(QIcon(":/icons/images/rotu24.png"));
	downButton->setIconSize(QSize(24, 24));
	downButton->setToolTip(
			"Rotate model clockwise around x axis, after enabling fixed view ");
	modelBar->addWidget(downButton);
	connect(downButton, SIGNAL(clicked()), this, SLOT(dButtonPress()));
	modelBar->setVisible(true);
}

void ApplicationWindow::setRenderModelPointer(bool *doRender) {

	forceRender = doRender;

}

void ApplicationWindow::setTrackResolutionPointer(int *ptr) {
	trackResPtr = ptr;
}

void ApplicationWindow::addCheckBox(const char *label, bool *value) {
	CheckBox *container = new CheckBox;
	container->value = value;
	container->checkbox = new QCheckBox(label, this);
	container->checkbox->setChecked(*value);

	connect(container->checkbox, SIGNAL( clicked(bool) ), this, SLOT(changeCheckBox()));
	simplifyBar->addWidget(container->checkbox);
	simplifyBar->setVisible(true);
	checkboxes.push_back(container);
}

void ApplicationWindow::changeCheckBox() {
	for (int i = 0; i < checkboxes.length(); i++) {
		if (checkboxes.at(i)->checkbox == QObject::sender()) {
			*(checkboxes.at(i)->value) =
					((QCheckBox *) QObject::sender())->isChecked();
			break;
		}
	}
}

ButtonChoices::ButtonChoices(QWidget* parent, const char* name, void *variable,
		VarType type, void (*callback)()) {
	this->name = new QString(name);
	this->value = variable;
	this->_button = new QPushButton(name, parent);
	this->callback = callback;
	this->type = type;
	this->parent = parent;
	QMenu *menu = new QMenu(this->_button);
	this->actionGroup = new QActionGroup(this);
	_button->setMenu(menu);
connect(menu, SIGNAL(triggered(QAction *)), this, SLOT(changeButtonChoices(QAction*)));
}

void ButtonChoices::updateValue() {
QString lab = *name;
lab += QString(": ");

for (int i = 0; i < (_button->menu()->actions()).length(); i++) {
	if (type == INT) {
		if (*((int*) value)
				== _button->menu()->actions().at(i)->data().toInt()) {
			_button->menu()->actions().at(i)->setChecked(true);
			lab += _button->menu()->actions().at(i)->text();
			_button->setText(lab);
			return;
		}
	}
	if (type == FLOAT) {
		if (*((float*) value)
				== _button->menu()->actions().at(i)->data().toFloat()) {
			_button->menu()->actions().at(i)->setChecked(true);
			lab += _button->menu()->actions().at(i)->text();
			_button->setText(lab);
			return;
		}
	}
}
char first[256];
char *tmp = first;
strcpy(first,
		(char*) ((_button->menu()->actions().at(0)->text().toStdString()).c_str()));
QString newEntry = QString::number(
		type == INT ? *((int *) value) : *((float*) value));
QString num = newEntry;
bool duplicate = false;
while (*tmp) {
	if (!((*tmp >= 48 && *tmp <= 57) || *tmp == 46)) {
		newEntry += *tmp;
		if (*tmp == 'x' || *tmp == 'X' || *tmp == ',') {
			if (*(tmp + 1) == 32) {
				tmp++;
				newEntry += *tmp;
			}
			newEntry += num;
		}
	}
	tmp++;
}
QAction *res = new QAction(newEntry.toStdString().c_str(), this);
_button->setText(lab + newEntry);
res->setData(type == INT ? *((int *) value) : *((float*) value));
res->setCheckable(true);
res->setChecked(true);
_button->menu()->addAction(res);
actionGroup->addAction(res);
}

void ApplicationWindow::addButtonChoices(const char *label,
	std::vector<const char *> entries, std::vector<int> values, int *variable,
	void (*callback)()) {
ButtonChoices *container = new ButtonChoices(this, label, variable, INT,
		callback);
QMenu *menu = container->button()->menu();

QActionGroup *group = container->actionGroup;
for (int i = 0; i < entries.size(); i++) {
	QAction *res = new QAction(entries[i], this);
	res->setData(values[i]);
	res->setCheckable(true);
	menu->addAction(res);
	group->addAction(res);
}
container->updateValue();
simplifyBar->addWidget(container->button());
buttonchoices.push_back(container);
}

void ApplicationWindow::addButtonChoices(const char *label,
	std::vector<const char *> entries, std::vector<float> values,
	float *variable, void (*callback)()) {
assert(values.size() == entries.size());
ButtonChoices *container = new ButtonChoices(this, label, variable, FLOAT,
		callback);

QMenu *menu = container->button()->menu();
QActionGroup *group = container->actionGroup;
for (int i = 0; i < entries.size(); i++) {
	QAction *res = new QAction(entries[i], this);
	res->setData(values[i]);
	res->setCheckable(true);
	menu->addAction(res);
	group->addAction(res);
}
container->updateValue();
simplifyBar->addWidget(container->button());
buttonchoices.push_back(container);
}

void ApplicationWindow::addButtonChoices(const char *label, int min, int max,
	int *variable, void (*callback)()) {
char tmp[50];
ButtonChoices *container = new ButtonChoices(this, label, variable, INT,
		callback);
QMenu *menu = container->button()->menu();
QActionGroup *group = container->actionGroup;
for (int i = min; i <= max; i++) {
	sprintf(tmp, "%d", i);
	QAction *res = new QAction(tmp, this);
	res->setData(i);
	res->setCheckable(true);
	menu->addAction(res);
	group->addAction(res);
}
container->updateValue();
simplifyBar->setVisible(true);
simplifyBar->addWidget(container->button());
buttonchoices.push_back(container);
}
QPushButton *ButtonChoices::button() {
return (_button);
}
void ButtonChoices::changeButtonChoices(QAction *action) {
char tmp[256];
if (type == INT)
	*((int *) (value)) = action->data().toInt();
else
	*((float *) (value)) = action->data().toFloat();

sprintf(tmp, "%s: %s", name->toStdString().c_str(),
		action->text().toStdString().c_str());
_button->setText(tmp);
if (callback)
	callback();
}

void ApplicationWindow::addSlider(const char *label, int min, int max,
	int *value) {
Slider *container = new Slider;
container->slider = new QSlider(Qt::Horizontal, this);
container->slider->setTickPosition(QSlider::TicksBelow);
container->name = new QString(label);
container->value = value;
QLabel *qlabel = new QLabel(label);

container->slider->setRange(min, max);
container->slider->setTickInterval(1);
container->slider->setValue(*value);
simplifyBar->addWidget(qlabel);
simplifyBar->addWidget(container->slider);

connect(container->slider, SIGNAL( valueChanged(int) ), this, SLOT(changeSlider()));
simplifyBar->setVisible(true);
sliders.push_back(container);

}
void ApplicationWindow::changeSlider() {
for (int i = 0; i < sliders.length(); i++) {
	if (sliders.at(i)->slider == QObject::sender()) {
		*(sliders.at(i)->value) = ((QSlider *) QObject::sender())->value();
		break;
	}
}
}
void ApplicationWindow::addDial(const char *label, int min, int max,
	int *value) {
Dial *container = new Dial;
container->dial = new NoWrapDial(this);

container->name = new QString(label);
container->value = value;
QLabel *qlabel = new QLabel(label);
container->dial->setMaximumWidth(40);
container->dial->setMaximumHeight(40);
container->dial->setNotchesVisible(true);
container->dial->setWrapping(0);

container->dial->setRange(min, max);
container->dial->setValue(*value);
simplifyBar->addWidget(qlabel);
simplifyBar->addWidget(container->dial);

connect(container->dial, SIGNAL( valueChanged(int) ), this, SLOT(changeDial()));
simplifyBar->setVisible(true);
dials.push_back(container);
}
void ApplicationWindow::changeDial() {
for (int i = 0; i < dials.length(); i++) {
	if (dials.at(i)->dial == QObject::sender()) {
		*(dials.at(i)->value) = ((QDial *) QObject::sender())->value();
		break;
	}
}
}

void ApplicationWindow::setDepthResDelay(int *) {
}
void ApplicationWindow::setVolumeDelay(int *) {
}

void ApplicationWindow::keyPressEvent(QKeyEvent *event) {

//std::cerr <<"Key pressed " <<  event->key() << std::endl;
switch (event->key()) {
case Qt::Key_Left:
	lButtonPress();
	break;
case Qt::Key_Right:
	rButtonPress();
	break;
case Qt::Key_Up:
	uButtonPress();
	break;
case Qt::Key_Down:
	dButtonPress();
	break;
case 'T':
	if (render_texture) {
		*render_texture = !(*render_texture);
		std::cerr << "should be rendering texture" << std::endl;
	}
	break;
	//case 'I':
	//  if(should_integrate){
	//      *should_integrate = !(*should_integrate);           
	//  }
	// break;
case 'Q':
	exit(0);
	break;
case 'C':
	if (resetCallback) {
		(resetCallback)();
		std::cerr << "Clear pressed" << std::endl;
	}
	break;
}
}
void ApplicationWindow::timerEvent(QTimerEvent *) {

// This should run the callback whenever we have finished all events in our queue  

if ((*cameraOpen || requireUpdate) && !viewers->holdProcessing()
		&& !holdProcessing) {
	if (callback != NULL) {
		(callback)();
	}
} else {
	usleep(16666); // limits cylicng to 60 Hz if we aren't doing anything
}
requireUpdate = false;
}
void ApplicationWindow::cameraStepPress() {
std::string file = "";
if (_cameraState == CAMERA_CLOSED)
	file = *video_file;

if (cameraButton->isChecked()) {
	setCameraState(CAMERA_PAUSED, file);
} else {
	setCameraState(CAMERA_RUNNING, file);
	singleStep = true;
}
}

void ApplicationWindow::cameraButtonPress() {
bool isChecked = (((QToolButton *) QObject::sender())->isChecked());
bool wasChecked = !isChecked;
CameraState targetState = isChecked ? CAMERA_RUNNING : CAMERA_PAUSED;
if (cameraCallback) {
	CameraState newState;
	if (_cameraState != CAMERA_CLOSED)
		newState = setCameraState(targetState, "");
	else
		newState = setCameraState(targetState, *video_file);
	refreshCameraButtons(newState);
	if (((QCheckBox *) QObject::sender())->isChecked() == false) {
		powerTimer->start();
	} else
		powerTimer->stop();
}
isChecked = (((QToolButton *) QObject::sender())->isChecked());
if (wasChecked != isChecked)
	requireUpdate = true;
}

void ApplicationWindow::cButtonPress() {
if (resetCallback)
	(resetCallback)();
}
void ApplicationWindow::lButtonPress() {
if (rot)
	*rot = TooN::SE3<float>(TooN::makeVector(0.0, 0, 0, 0, 0.1, 0)) * (*rot);
requireUpdate = true;
*forceRender = true;
if (povButton) {
	povButton->setChecked(false);
	povButtonPress();
}
}
void ApplicationWindow::rButtonPress() {
if (rot) {
	*rot = TooN::SE3<float>(TooN::makeVector(0.0, 0, 0, 0, -0.1, 0)) * (*rot);
	requireUpdate = true;
	*forceRender = true;
	if (povButton) {
		povButton->setChecked(false);
		povButtonPress();
	}
}
}
void ApplicationWindow::uButtonPress() {
if (rot) {
	*rot *= TooN::SE3<float>(TooN::makeVector(0.0, 0, 0, -0.1, 0, 0));
	requireUpdate = true;
	*forceRender = true;
	if (povButton) {
		povButton->setChecked(false);
		povButtonPress();
	}
}
}
void ApplicationWindow::dButtonPress() {
if (rot) {
	*rot *= TooN::SE3<float>(TooN::makeVector(0.0, 0, 0, 0.1, 0, 0));
	requireUpdate = true;
	*forceRender = true;
	if (povButton) {
		povButton->setChecked(false);
		povButtonPress();
	}
}
}
void ApplicationWindow::povButtonRefresh() {
if (povButton->isChecked())
	povButton->setIcon(QIcon(":/icons/images/camera.png"));
else
	povButton->setIcon(QIcon(":/icons/images/tripod.png"));
}
void ApplicationWindow::povButtonPress() {
if (cameraViewCallback) {
	cameraViewCallback(povButton->isChecked());
	povButtonRefresh();
}
}

void ApplicationWindow::updatePower() {
//Put code for idle power here
}
void ApplicationWindow::showEvent(QShowEvent *) {
readConfig();
}

void ApplicationWindow::setBreakpoints(QString values) {
breakpoints.clear();
QStringList words = values.split(QRegExp("[\\s,]+"));
for (int i = 0; i < words.length(); i++) {
	if (words.at(i) != "")
		breakpoints.push_back(words.at(i).toInt());
}
}

void ApplicationWindow::useOpenNI() {

}
void ApplicationWindow::closeEvent(QCloseEvent *event) {
exit(0);
}

void ApplicationWindow::updateChoices() {
for (int i = 0; i < buttonchoices.size(); i++) {
	buttonchoices.at(i)->updateValue();
}
}
void ApplicationWindow::addConditionalBreakpoint(QString label,
	BreakpointType type, int value) {
ConditionalBreakpoint *bp = new (ConditionalBreakpoint);
bp->name = label;
bp->type = type;
bp->value = new int(value);
conditionalBreakpoints.push_back(bp);
}
void ApplicationWindow::addConditionalBreakpoint(QString label,
	BreakpointType type, double value) {
ConditionalBreakpoint *bp = new (ConditionalBreakpoint);
bp->name = label;
bp->type = type;
bp->value = new double(value);
conditionalBreakpoints.push_back(bp);
}
void ApplicationWindow::setConditionalBreakpoint(QString label, int value) {
bool found = false;
for (int i = 0; i < conditionalBreakpoints.length(); i++) {
	if (conditionalBreakpoints.at(i)->name == label) {
		*((int *) (conditionalBreakpoints.at(i)->value)) = value;
		found = true;
	}
}
if (found == false) {
	addConditionalBreakpoint(label, COND_BOOL, value);
}
}

ConditionalBreakpoint *ApplicationWindow::getConditionalBreakpoint(
	QString label) {
for (int i = 0; i < conditionalBreakpoints.count(); i++) {
	if (conditionalBreakpoints.at(i)->name == label)
		std::cerr << label.toStdString() << " "
				<< *((int *) conditionalBreakpoints.at(i)->value) << std::endl;
	return (conditionalBreakpoints.at(i));
}
return (NULL);
}

void ApplicationWindow::update(int fid, CameraState cameraState) {
static double lastUpdateTime = 0;
static double cumulativeTime = 0.0;
static int numSamples = 0;
static PerfStats *stats = NULL;
QMessageBox *messageBox = NULL;
double now;
#ifdef __APPLE__
    clock_serv_t cclock;
    mach_timespec_t clockData;
#else
    struct timespec clockData;
#endif
double elapsed;
bool forceLabel = false;
if (stats == NULL) {
	//This should get the stats that contain the required data
	stats = viewers->getStats((char *) frameRateField.toStdString().c_str());
}
if (cameraState != _cameraState) {
	_cameraState = cameraState;
	refreshCameraButtons(cameraState);
}
//frameNumber++;

if (singleStep) {
	setCameraState(CAMERA_PAUSED);
	singleStep = false;
}
viewers->updateImages();
if (fid > _conditionalStart)
	for (int i = 0; i < conditionalBreakpoints.length(); i++) {
		if (conditionalBreakpoints.at(i)->type == COND_BOOL) {
			char message[256];
			int value = stats->getLastData(
					conditionalBreakpoints.at(i)->name.toStdString().c_str());
			if (value == *((int *) (conditionalBreakpoints.at(i)->value))) {
				setCameraState(CAMERA_PAUSED);
				sprintf(message,
						"Conditional breakpoint hit: stopped at frame %d\n%s == %s",
						fid,
						conditionalBreakpoints.at(i)->name.toStdString().c_str(),
						value == 1 ? "TRUE" : "FALSE");
				messageBox = new QMessageBox(QMessageBox::Information,
						"Breakpoint", message, QMessageBox::Ok);
			}

		}
	}
for (int i = 0; i < breakpoints.length(); i++) {
	if (breakpoints.at(i) == fid) {
		char message[128];
		setCameraState(CAMERA_PAUSED);
		forceLabel = true;
		sprintf(message, "Breakpoint hit: stopped at frame %d", fid);
		messageBox = new QMessageBox(QMessageBox::Information, "Breakpoint",
				message, QMessageBox::Ok);
	}
}

if (stats != NULL) {
	elapsed = stats->getLastData(frameRateField.toStdString().c_str());
	cumulativeTime = cumulativeTime + elapsed;
	numSamples++;

#ifdef __APPLE__
		host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &cclock);
		clock_get_time(cclock, &clockData);
		mach_port_deallocate(mach_task_self(), cclock);
#else
		clock_gettime(CLOCK_MONOTONIC, &clockData);
#endif

	now = clockData.tv_sec + clockData.tv_nsec / 1000000000.0;

	if ((now - lastUpdateTime > 1.0) || (forceLabel)) {
		if (cumulativeTime > 0) {
			double rate = numSamples / cumulativeTime;
			setFrameRateLabel(rate, fid);
		}
		numSamples = 0;
		cumulativeTime = 0;
		lastUpdateTime = now;
	}
}

if (messageBox) {
	messageBox->setWindowFlags(Qt::WindowStaysOnTopHint);
	messageBox->exec();
	delete (messageBox);
}

}

DebugWindow::DebugWindow(ApplicationWindow *parent) {
_appWindow = parent;
vLayout = new QVBoxLayout();
fLayout = new QFormLayout();
vLayout->addLayout(fLayout);

QDialogButtonBox *buttonBox = new QDialogButtonBox(
		QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
vLayout->addWidget(buttonBox);
connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
}

void DebugWindow::reject() {
this->destroy();
}
void DebugWindow::accept() {

QLayoutItem *lo = fLayout->itemAt(0, QFormLayout::ItemRole::FieldRole);
_appWindow->setBreakpoints(((QLineEdit *) lo->widget())->text());
lo = fLayout->itemAt(1, QFormLayout::ItemRole::FieldRole);

_appWindow->setConditionalStart(((QLineEdit *) lo->widget())->text().toInt());

for (int i = 2; i < fLayout->rowCount(); i++) {
	//this should give us the button boxes layout		
	lo = fLayout->itemAt(i, QFormLayout::ItemRole::LabelRole);
	QString label = ((QLabel *) lo->widget())->text();
	lo = fLayout->itemAt(i, QFormLayout::ItemRole::FieldRole); //			
	QPushButton *button = (QPushButton *) (lo->layout()->itemAt(0)->widget());
	_appWindow->setConditionalBreakpoint(label, button->group()->checkedId());
}
this->destroy();
}

DebugWindow::~DebugWindow() {
delete fLayout;
delete vLayout;
}
void DebugWindow::showForm() {
this->setLayout(vLayout);
this->setWindowFlags(Qt::WindowStaysOnTopHint);
this->setModal(true);
this->show();
}
void DebugWindow::addBoolTriRadioBox(const char *label, int value) {
QString tooltip = "Break when " + QString(label) + " is ";

QPushButton *l0 = new QPushButton("False", this);
l0->setToolTip(tooltip + "false");
QPushButton *l1 = new QPushButton("True", this);
l1->setToolTip(tooltip + "true");
QPushButton *lu = new QPushButton("Disabled", this);
lu->setToolTip("Disable conditional breakpoint");
QButtonGroup *lg = new QButtonGroup(this);
l1->setFocusPolicy(Qt::NoFocus);
lu->setFocusPolicy(Qt::NoFocus);
l0->setFocusPolicy(Qt::NoFocus);
l0->setCheckable(true);
l1->setCheckable(true);
lu->setCheckable(true);

QHBoxLayout* hlay = new QHBoxLayout();
hlay->addWidget(l0);
hlay->addWidget(l1);
hlay->addWidget(lu);
lg->addButton(l0, 0);
lg->addButton(l1, 1);
lg->addButton(lu, 2);

switch (value) {
case 0:
	l0->setChecked(true);
	break;
case 1:
	l1->setChecked(true);
	break;
default:
	lu->setChecked(true);
}
fLayout->addRow(label, hlay);

}
void DebugWindow::addLineEdit(const char *label, const char *value,
	const char *tooltip) {
nameLineEdit = new QLineEdit(this);
if (value != NULL)
	nameLineEdit->setText(value);
if (tooltip != NULL)
	nameLineEdit->setToolTip(tooltip);
fLayout->addRow(label, nameLineEdit);
}
void ApplicationWindow::setConditionalStart(int value) {
_conditionalStart = value;
}
;
void ApplicationWindow::showDebugWindow() {

DebugWindow *form = new DebugWindow(this);
QString values = "";
for (int i = 0; i < breakpoints.length(); i++) {
	values = values + (i == 0 ? "" : " ");
	values = values + QString::number(breakpoints.at(i));
}

form->addLineEdit("frame", values.toStdString().c_str(),
		"Space seperated list of frames to break at");
form->addLineEdit("Condtional start",
		QString::number(_conditionalStart).toStdString().c_str(),
		"Conditionals will only start after the nth frame");
for (int i = 0; i < conditionalBreakpoints.length(); i++) {
	if ((conditionalBreakpoints.at(i))->type == COND_BOOL) {
		form->addBoolTriRadioBox(
				(conditionalBreakpoints.at(i))->name.toStdString().c_str(),
				*((int *) conditionalBreakpoints.at(i)->value));
	}
}

//layout->addRow("tracked",  );
//form->setLayout(formLayout);
form->setWindowTitle("Break at");

form->showForm();

}
