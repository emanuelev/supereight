/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "MainWindow.h"
#include "qcustomplot.h"
#include <QtWidgets>
#include <iostream>

Viewer::Viewer(Stream *stream, int id, QWidget *parent) {
	opengl = NULL;
	label = NULL;
	checkbox = NULL;
	plot = NULL;
	optionButton = NULL;
	holdProcessing = NULL;
	_id = id;
	_container = (Viewers *) parent;
	layout = new QGridLayout();
	layout->setSpacing(0);
	layout->setContentsMargins(0, 0, 0, 0);
	setStream(stream);
	this->setLayout(layout);
	_stream = stream;
	allowMixedPlot = IGNORE;
	mixedPlot = false;
}
int Viewer::getID() {
	return (_id);
}
QSize Viewer::sizeHint() const {
	return QSize(320, 240);
}

QString Viewer::getOptions() {
	QString ret = "";
	if (!optionButton || !optionButton->menu())
		return (ret);

	for (int i = 0; i < optionButton->menu()->actions().length(); i++) {
		if (optionButton->menu()->actions().at(i)->isSeparator())
			continue;
		ret = ret + " \"" + optionButton->menu()->actions().at(i)->text()
				+ "\" ";
		ret = ret
				+ (optionButton->menu()->actions().at(i)->isChecked() ?
						"1" : "0");
	}
	return (ret);
}
void Viewer::setActive(bool value) {

	if (checkbox && checkbox->isEnabled() && checkbox->isCheckable()) {

		if (_stream->active) {
			checkbox->setChecked(value);
			*(_stream->active) = value;
		} else {
			//checkbox->setChecked(true);
			//checkbox->setEnabled(false);
		}

	}

}

void Viewer::setStreamImage(Stream *stream) {
	static int count = 0;
	if (opengl)
		delete (opengl);
	if (plot)
		delete (plot);
	if (checkbox)
		delete (checkbox);
	if (optionButton)
		delete (optionButton);
	plot = NULL;
	label = NULL;
	opengl = NULL;
	checkbox = NULL;
	optionButton = NULL;
	this->image = stream->image;
	opengl = new OpenGLWidget(stream->image, this);
	//label = new QLabel(QString(title), this );
	if (stream->active != NULL) {
		checkbox = new QCheckBox(stream->title, this);
		connect(checkbox, SIGNAL( clicked(bool) ), this, SLOT(checkBoxChanged()));
		checkbox->setChecked(true);
		checkbox->setStyleSheet(
				"QLabel { background-color : white; color : blue; }");

	} else {
		label = new QLabel(stream->title, opengl);
		label->setAlignment(Qt::AlignCenter);
		label->setStyleSheet(
				"QLabel { background-color : black; color : gray;\
            border-style: outset;\
            border-width: 2px;\
            border-color: black; }");
	}

	opengl->setToolTip(stream->help);
	layout->addWidget(opengl, 0, 0, 1, 2);
	if (checkbox) {
		layout->addWidget(checkbox, 1, 0, 1, 1);
		;
	}
	count++;
}
void Viewer::plotWaveSelectionChanged() {

	if (allowMixedPlot == IGNORE)
		return;

	QAction *sender = (QAction *) QObject::sender();
	int seenChecked = -1;
	int section = -1;
	bool isMixed = false;
	for (int i = 0; i < optionButton->menu()->actions().length(); i++) {
		QAction * action = optionButton->menu()->actions().at(i);
		if (action->isSeparator()) {
			section++;
			continue;
		}
		if (!(action->isCheckable()) || action->isChecked()) {
			if (seenChecked == -1)
				seenChecked = section;
			else {
				if (seenChecked != section) {
					isMixed = true;
					if (allowMixedPlot == WARNING) {
						*holdProcessing = true;
						QMessageBox::StandardButton reply;
						QMessageBox *messageBox =
								new QMessageBox(QMessageBox::Warning,
										"Mixed Graph",
										"Should adding this dataset would cause a mixed type graph?\n\nCancel: to abort adding the new tracen\nNo: to remove old traces\nYes: to create mixed graph\n",
										QMessageBox::Yes | QMessageBox::No
												| QMessageBox::Cancel);
						messageBox->setDetailedText("");
						messageBox->setWindowFlags(Qt::WindowStaysOnTopHint);
						reply =
								(QMessageBox::StandardButton) messageBox->exec();
						delete (messageBox);

						switch (reply) {
						case QMessageBox::Cancel:
							sender->setChecked(false);
							break;
						case QMessageBox::Yes:
							allowMixedPlot = ALLOW;
							plot->yAxis->setLabel(" ");
							break;
						case QMessageBox::No:
							for (int j = 0; j < i; j++) {
								allowMixedPlot = IGNORE;
								QAction * action =
										optionButton->menu()->actions().at(j);
								if (action->isSeparator()) {
									section++;
									continue;
								}
								action->setChecked(false);
							}
							plot->yAxis->setLabel("");
							allowMixedPlot = WARNING;
						}
						*holdProcessing = false;
						return;

					}
				}
			}
		}
	}
	mixedPlot = isMixed;
	if (!mixedPlot) {
		allowMixedPlot = WARNING;
		plot->yAxis->setLabel("");
	} else {
		plot->yAxis->setLabel(" ");
	}

}
void Viewer::setOptionMenu() {
	if (_stream->statistics != NULL) {
		PerfStats *stats = _stream->statistics;
		QMenu *optionMenu = NULL;
		int i = 0;
		int colorRIndex[16] = { 0, 255, 0, 0, 255, 255, 0, 128, 0, 0, 128, 128,
				0, 128, 255, 64 };
		int colorGIndex[16] = { 0, 0, 255, 0, 255, 0, 255, 0, 128, 0, 128, 0, 0,
				64, 128, 255 };
		int colorBIndex[16] = { 0, 0, 0, 255, 0, 255, 255, 0, 0, 128, 0, 128,
				128, 255, 64, 128 };

		PerfStats::Type lastType = PerfStats::UNDEFINED;
		for (std::map<int, std::string>::const_iterator kt =
				stats->order.begin(); kt != stats->order.end(); kt++) {
			std::map<std::string, PerfStats::Stats>::const_iterator it =
					stats->stats.find(kt->second);

			if (optionMenu == NULL) {
				optionMenu = new QMenu(optionButton);
				optionButton->setMenu(optionMenu);
			}
			int spot = (i * 2) + 1;
			int line = i * 2;

			QAction *action = new QAction(it->first.c_str(), this);
			optionMenu->addAction(action);
			action->setData(i);
			action->setCheckable(true);
			action->setChecked(true);

			if (it->second.type != lastType) {
				QAction *tmpAction = optionMenu->insertSeparator(action);
				tmpAction->setText("Different Type");
				lastType = it->second.type;
			}

			connect(action, SIGNAL(changed()), this,
					SLOT(plotWaveSelectionChanged()));

			plot->addGraph();
			int r = colorRIndex[i % 16];
			int g = colorGIndex[i % 16];
			int b = colorBIndex[i % 16];
			plot->graph(line)->setName(it->first.c_str());
			plot->graph(line)->setPen(QPen(QColor(r, g, b)));
			plot->graph(line)->setAntialiasedFill(false);

			plot->addGraph();
			plot->graph(spot)->setPen(QPen(QColor(r, g, b)));
			plot->graph(spot)->setLineStyle(QCPGraph::lsNone);
			plot->graph(spot)->setScatterStyle(QCPScatterStyle::ssDisc);
			//std::cerr << "Spot element:" << fred <<  "name: " << plot->graph(line)->name().toStdString() << " elementCount: "<<  plot->legend->itemCount()   <<std::endl;

			plot->legend->removeItem(plot->legend->itemCount() - 1);
			i++;
		}
	}
	updateEnableStates();
}

void Viewer::setStreamStats(Stream *stream) {
	if (opengl)
		delete (opengl);
	if (checkbox)
		delete (checkbox);
	if (plot)
		delete (plot);
	if (optionButton)
		delete (optionButton);
	plot = NULL;
	label = NULL;
	opengl = NULL;
	checkbox = NULL;
	optionButton = NULL;
	lastTime = -1;
	plot = new QCustomPlot(this);
	checkbox = new QCheckBox(stream->title, this);
	optionButton = new QPushButton("Data", this);
	setOptionMenu();

	plot->xAxis->setAutoTickStep(false);
	plot->xAxis->setTickStep(2);
	plot->xAxis->setTickLabels(false);
	//plot->yAxis->setLabel("Time (s)");
	plot->axisRect()->setupFullAxesBox();
	//plot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignBottom);
	plot->plotLayout()->addElement(1, 0, plot->legend);
	plot->legend->setBorderPen(Qt::NoPen);
	plot->plotLayout()->setRowStretchFactor(1, 0.01);
	plot->plotLayout()->setRowStretchFactor(0, 1);
	plot->legend->setVisible(true);
	plot->legend->setFont(QFont("Helvetica", 9));
	plot->legend->setRowSpacing(0);

	//connect(label, SIGNAL( clicked(bool) ), this, SLOT(checkBoxChanged()));
	checkbox->setChecked(true);
	checkbox->setStyleSheet(
			"QLabel { background-color : white; color : blue; }");

	layout->addWidget(plot, 0, 0, 1, 2);
	layout->addWidget(checkbox, 1, 0, 1, 1);
	layout->addWidget(optionButton, 1, 1, 1, 1);
}

void Viewer::checkBoxChanged() {
	if (((QCheckBox *) QObject::sender())->isChecked())
		_container->resumeStream(_stream->id);
	else
		_container->pauseStream(_stream->id);
}

int Viewer::getStreamID() {
	return (_stream->id);
}
Stream* Viewer::getStream() {
	return (_stream);
}

bool Viewer::getActive() {

	if (checkbox && checkbox->isCheckable())
		return (checkbox->isChecked());
	else
		return (true);
}
void Viewer::setZoom(float zoomX, float zoomY) {
	opengl->setZoom(zoomX, zoomY);
}
void Viewer::setBufferSize(int width, int height) {

	image.x = width;
	image.y = height;
	layout->removeWidget(opengl);
	delete opengl;
	opengl = new OpenGLWidget(image, this);
	opengl->setToolTip(getStream()->help);
	label = new QLabel(getStream()->title, opengl);
	label->setAlignment(Qt::AlignCenter);
	label->setStyleSheet(
			"QLabel { background-color : black; color : gray;\
        border-style: outset;\
        border-width: 2px;\
        border-color: black; }");
	layout->addWidget(opengl, 0, 0, 1, 2);
}

void Viewer::pause() {
	if (_stream->active) {
		if (*(_stream->active))
			*(_stream->active) = false;
		checkbox->setChecked(false);
	}
}
void Viewer::resume() {
	if (_stream->active) {
		if (*(_stream->active) == false)
			*(_stream->active) = true;
		checkbox->setChecked(true);
	}
}

void Viewer::setStream(Stream *stream) {
	_stream = stream;
	if (stream->statistics == NULL)
		setStreamImage(stream);
	else
		setStreamStats(stream);
}

Viewer::~Viewer() {
	if (opengl)
		delete (opengl);
	if (layout)
		delete (layout);
}
void Viewer::update() {

	static double lastPointKey = 0;
	double min;
	double max;
	int xpoints = 100;

	if (opengl) {
		opengl->updateGL();
	} else if (plot) {
		if (plot->yAxis->label() == "") {
			min = 9e99;
			max = -9e99;
		} else {
			max = plot->yAxis->range().upper / 1.1;
			min = plot->yAxis->range().lower / 1.1;
			if (min > 0)
				min = 0;

		}
		int numStreams = plot->legend->itemCount();
		if (checkbox && checkbox->isChecked()) {
			if (optionButton->menu() == NULL) {
				setOptionMenu();
			}
			double key = lastPointKey + 1;
			int currentViewed = 0;
			double localMin = 9e10;
			double localMax = -9e10;
			if (optionButton->menu() != NULL) {
				int j = -1;
				for (int i = 0; i < optionButton->menu()->actions().length();
						i++) {
					if ((optionButton->menu()->actions().at(i)->isSeparator()))
						continue;

					j++;
					int line = j * 2;
					int spot = line + 1;
					if (optionButton->menu()->actions().at(i)->isChecked()) {
						if (plot->graph(j * 2)->visible() == false) {
							plot->graph(j * 2)->setVisible(true);
							plot->graph(2 * j)->addToLegend();
						}
						QString name =
								optionButton->menu()->actions().at(i)->text();

						_stream->statistics->getSampleTime(
								name.toStdString().c_str());
						double value = _stream->statistics->getLastData(
								name.toStdString().c_str());

						if (_stream->statistics->getType(
								name.toStdString().c_str())
								== PerfStats::FRAME) {
							int frame = floor(value + 0.1);
							if (frame >= lastFrame)
								value = frame - lastFrame;
							else
								value = 0;
							lastFrame = frame;
						}
						//if we haven't labelled the axis then we should add a label
						if (plot->yAxis->label() == "") {
							switch (_stream->statistics->getType(
									name.toStdString().c_str())) {
							case PerfStats::FRAME:
								plot->yAxis->setLabel("Frame (change)");
								break;
							case PerfStats::TIME:
								plot->yAxis->setLabel("Time (s)");
								break;
							case PerfStats::COUNT:
								plot->yAxis->setLabel("Count");
								break;
							case PerfStats::PERCENTAGE:
								plot->yAxis->setLabel("Percentage");
								break;
							case PerfStats::POWER:
								plot->yAxis->setLabel("Power (W)");
								break;
							case PerfStats::VOLTAGE:
								plot->yAxis->setLabel("Voltage (V)");
								break;
							case PerfStats::CURRENT:
								plot->yAxis->setLabel("current (A)");
								break;
							case PerfStats::FREQUENCY:
								plot->yAxis->setLabel("Frequency (Hz)");
								break;
							case PerfStats::DISTANCE:
								plot->yAxis->setLabel("Distance (m)");
								break;
							default:
								plot->yAxis->setLabel(" ");
								break;
							}
							plot->yAxis->setRange(0, 1e-9);
						}
						/*
						 * FOR AN ENERGY PLOT TRY THIS
						 if(_stream->statistics->getType(name.toStdString().c_str()) == PerfStats::POWER) {
						 double samplePeriod = _stream->statistics->getSampleTime(name.toStdString().c_str());
						 value = value * samplePeriod;
						 }
						 */
						if (value < localMin)
							localMin = value;
						if (value > localMax)
							localMax = value;

						plot->graph(line)->addData(key, value);
						plot->graph(spot)->clearData();
						plot->graph(spot)->addData(key, value);
						plot->graph(line)->removeDataBefore(key - xpoints);
						currentViewed++;
					} else {
						plot->graph(2 * j)->removeFromLegend();
						plot->graph(j * 2)->setVisible(false);

					}
				}
				if (currentViewed != numStreams) {
					min = localMin;
					max = localMax;
					numStreams = currentViewed;
				} else {
					if (localMax > plot->yAxis->range().upper)
						max = localMax;
					if (localMin < min)
						min = localMin;
				}

				if (localMin < 1e10 && localMax > -1e10) {
					lastPointKey = key;
					plot->xAxis->setRange(key + 0.25, xpoints, Qt::AlignRight);
					plot->yAxis->setRange((0 > min) ? min * 1.1 : 0, max * 1.1);
					plot->replot();
				}
			}
		}
	}
}
void Viewer::contextMenuEvent(QContextMenuEvent *event) {
	//std::cerr << "streamId: " << _stream->id <<std::endl;
	QMenu *menu = _container->streamMenu;
	menu->actions().at(_stream->id)->setChecked(true);
	menu->popup(QPoint(event->globalX(), event->globalY()));
	_container->setCurrentViewer(this);

}

void Viewer::mousePressEvent(QMouseEvent *) {
}
void Viewer::setHoldVariable(bool *variable) {
	holdProcessing = variable;
}

Viewers::Viewers(QWidget *)

{
	streamMenu = new QMenu();
	streamMenuActions = new QActionGroup(this);
	connect(streamMenu, SIGNAL(triggered(QAction*)), this, SLOT(menuAction(QAction*)));

	layout = new QGridLayout();
	_holdProcessing = false;
	setLayout(layout);
}
Viewers::~Viewers() {
	//delete ui;
}
bool Viewers::holdProcessing() {
	return (_holdProcessing);
}
void Viewers::pauseStream(int id) {
	if (streams.at(id)->active) {
		*(streams.at(id)->active) = false;
		for (int i = 0; i < viewers.count(); i++)
			if (viewers.at(i)->getStreamID() == id)
				(viewers.at(i))->setActive(false);
	}
}

void Viewers::resumeStream(int id) {
	if (streams.at(id)->active) {
		*(streams.at(id)->active) = false;
		for (int i = 0; i < viewers.count(); i++) {

			int streamID = viewers.at(i)->getStreamID();
			if (streamID == id)
				(viewers.at(i))->setActive(true);
		}
	}
}
void Viewers::setZoomViewer(int id, float zoomX, float zoomY) {
	viewers.at(id)->setZoom(zoomX, zoomY);
}
void Viewers::setZoom(float zoom) {
	setZoom(zoom, zoom);
}

void Viewers::setZoom(float zoomX, float zoomY) {
	for (int i = 0; i < viewers.count(); i++) {
		int streamID = viewers.at(i)->getStreamID();
		if (streams.at(streamID)->allowScaling)
			setZoomViewer(i, zoomX, zoomY);

	}
}
void Viewers::setBufferSize(int width, int height) {
	for (int i = 0; i < viewers.count(); i++) {
		int streamID = viewers.at(i)->getStreamID();
		if (streams.at(streamID)->allowScaling) {
			viewers.at(i)->setBufferSize(width, height);
		}
	}
}
void Viewers::setCurrentViewer(Viewer *viewer) {
	_currentViewer = viewer;

}
bool Viewers::isStreamActive(int id) {
	for (int i = 0; i < viewers.count(); i++) {
		if ((viewers.at(i))->getStreamID() == id)
			return (viewers.at(i))->getActive();
	}
	return (false);
}

void Viewers::menuAction(QAction *action) {
	int lid = layout->indexOf(_currentViewer);
	int row, col, rowSpan, colSpan, id;
	layout->getItemPosition(lid, &row, &col, &rowSpan, &colSpan);
	//std::cerr <<"replacing Viewer itemAt "<<  row<< " " <<col << "\n";
	//std::cerr <<"Dimension " << (int)(_currentViewer->sizePolicy().controlType()) << " " << "\n";
	// std::cerr <<"New Dimension " <<(_currentViewer->size()).width() << " " << (_currentViewer->size()).height() << "\n";
	_currentViewer->setStream(streams.at(action->data().toInt()));

	for (int i = 0; i < streams.count(); i++) {
		if (streams.at(i)->active) {
			_currentViewer->setActive(true);
			if (isStreamActive(i))
				*(streams.at(i)->active) = true;
			else
				*(streams.at(i)->active) = false;
		} else {
			_currentViewer->setActive(true);
		}
	}
}

void Viewers::addViewer(PerfStats *stats, const char *title, const char *help,
		bool *active) {
	Stream *stream = new Stream;
	stream->help = (char *) help;
	stream->title = (char *) title;
	stream->statistics = stats;
	stream->id = streams.count();
	stream->active = active;
	stream->allowScaling = false;
	if (active == NULL) {
		//stream->active = new bool;
		//*(stream->active)=true;
	}
	QAction *action = new QAction(title, this);
	action->setData(streams.count());
	action->setCheckable(true);
	streams.append(stream);
	streamMenuActions->addAction(action);
	streamMenu->addAction(action);
	addViewer(stream);
}
void Viewers::addDefConfig(int viewerID, QString name, bool enabled,
		QStringList visible) {
	if (viewerID < viewers.length()) {
		Viewer *viewer = viewers.at(viewerID);
		if (QString(streams.at(viewer->getStreamID())->title) != name) {
			for (int i = 0; i < streams.length(); i++) {
				if (QString(streams.at(i)->title) == name) {
					viewer->setStream(streams.at(i));
				}
			}
		}
		viewer->setActive(enabled);
		viewer->setEnableStates(visible);
	} else {

		for (int i = 0; i < streams.length(); i++) {
			if (QString(streams.at(i)->title) == name) {
				Viewer *viewer = new Viewer(streams.at(i), viewers.length(),
						this);
				viewer->setHoldVariable(&_holdProcessing);
				viewers.append(viewer);
				viewer->setStream(streams.at(i));
				layout->addWidget(viewer, layout->rowCount(), 0, 1, 3);
				addDefConfig(viewerID, name, enabled, visible);

			}
		}
	}

}

void Viewers::setStatEntry(QString title, std::vector<const char *> entries,
		bool enabled) {
	QStringList visible;
	for (int i = 0; i < entries.size(); i++) {
		visible.push_back(entries[i]);
		visible.push_back(enabled ? "1" : "0");
	}
	for (int i = 0; i < viewers.length(); i++) {
		Stream *stream = streams.at(viewers.at(i)->getStreamID());
		if (stream->title == title) {
			viewers.at(i)->setEnableStates(visible);
		}
	}
}
QString Viewers::getConfig(int viewerID) {
	char tmp[250];
	QString ret = "";
	Stream *stream = streams.at(viewers.at(viewerID)->getStreamID());
	sprintf(tmp, "viewer %d \"%s\" %d", viewerID, stream->title,
			viewers.at(viewerID)->getActive() ? 1 : 0);
	ret = tmp;
	if (stream->statistics)
		ret = ret + viewers.at(viewerID)->getOptions();
	ret = ret + "\n";
	return (ret);
}
int Viewers::getViewerCount() {
	return (viewers.length());
}

void Viewers::addViewer(Stream *stream) {
	static int xPos = 0, yPos = 0;
	int length = viewers.length();
	if (stream->statistics == NULL) {
		if (length < 5) {
			if (stream->active)
				*(stream->active) = true;
			Viewer *viewer = new Viewer(stream, length, this);
			viewer->setHoldVariable(&_holdProcessing);
			viewers.append(viewer);
			int xSpan = 1;
			int ySpan = 1;

			int dim = (int) floor(sqrt(length));
			dim = (dim == 0) ? 1 : dim;
			if (xPos == 2 && yPos == 0) {
				xSpan = 2;
				ySpan = 2;
			}
			layout->addWidget(viewer, yPos, xPos, ySpan, xSpan);
			if (xPos <= yPos) {
				xPos++;
				if (xPos > yPos) {
					yPos = 0;
				}
			} else {
				yPos++;
				xPos = 0;
			}
		} else {
			if (stream->active)
				*(stream->active) = false;
		}
	} else {
		//statistics windows are added extended across the bottom
		if (stream->active)
			*(stream->active) = true;
		Viewer *viewer = new Viewer(stream, length, this);
		viewer->setHoldVariable(&_holdProcessing);
		viewers.append(viewer);
		layout->addWidget(viewer,
				layout->rowCount() > 2 ? layout->rowCount() : 2, 0, 1, 4);
	}
}
void Viewers::addViewer(FImage image, const char *title, const char *help,
		bool *active) {
	addViewer(image, title, help, active, false);
}

void Viewers::addViewer(FImage image, const char *title, const char *help,
		bool *active, bool allowScale) {
	//printf("Added image %s width:%d height:%d format: %d type:%d\n",title, image.x, image.y, image.format, image.type);
	Stream *stream = new Stream;
	stream->help = (char *) help;
	stream->title = (char *) title;
	stream->image = image;
	stream->id = streams.count();
	stream->statistics = NULL;
	stream->active = active;
	stream->allowScaling = allowScale;

	QAction *action = new QAction(title, this);
	action->setData(streams.count());
	action->setCheckable(true);
	streams.append(stream);
	streamMenuActions->addAction(action);
	streamMenu->addAction(action);
	addViewer(stream);
}
void Viewers::reLayout() {
	int numGraphics = 0;
	int maxRow = layout->rowCount();
	QLayoutItem *item;
	QWidget **widgets;
	widgets = (QWidget **) malloc(sizeof(QWidget *) * viewers.size());
	for (int i = 0; i < viewers.size(); ++i) {
		int streamId = (viewers.at(i))->getStreamID();
		if ((streams.at(streamId))->statistics == NULL)
			numGraphics++;

	}
	if (numGraphics == 3) {
		item = layout->itemAtPosition(1, 0);
		QWidget *bigWidget = item->widget();

		if (bigWidget) {
			layout->removeWidget(bigWidget);
		}

		item = layout->itemAtPosition(0, 1);
		QWidget *widget = item->widget();
		if (widget) {
			layout->removeWidget(widget);
		}

		layout->addWidget(widget, 1, 0, 1, 1);

		layout->addWidget(bigWidget, 0, 1, 2, 2);

		for (int row = 2; row < layout->rowCount(); row++) {
			item = layout->itemAtPosition(row, 0);
			widget = item->widget();
			layout->removeWidget(widget);
			layout->addWidget(widget, row, 0, 1, 3);

		}
	}
	if (numGraphics == 4) {
		//OLD          New Layout
		// 0 1         0 3 3
		// 2 3         1 3 3		
		// 4 4         2 4 4
		// 5 5         5 5 5
		widgets[0] = layout->itemAtPosition(0, 0)->widget();
		widgets[1] = layout->itemAtPosition(0, 1)->widget();
		widgets[2] = layout->itemAtPosition(1, 0)->widget();
		widgets[3] = layout->itemAtPosition(1, 1)->widget();
		for (int i = 0; i < 4; i++) {
			layout->removeWidget(widgets[0]);
		}
		layout->addWidget(widgets[0], 0, 0, 1, 1);
		layout->addWidget(widgets[1], 1, 0, 1, 1);
		layout->addWidget(widgets[2], 2, 0, 1, 1);
		layout->addWidget(widgets[3], 0, 1, 2, 2);

		if (maxRow >= 2) {
			//this will be the  move viewer 4 into position
			item = layout->itemAtPosition(2, 0);
			QWidget *widget = item->widget();
			layout->removeWidget(widget);
			layout->addWidget(widget, 2, 1, 1, 2);
		}

		int maxRow = layout->rowCount() - 1;
		for (int row = layout->rowCount() - 1; row > 2; row--) {
			item = layout->itemAtPosition(row, 0);
			QWidget *widget = item->widget();
			layout->removeWidget(widget);
			layout->addWidget(widget, row + 1, 0, 1, 3);
		}
	}
	for (int i = 0; i < layout->columnCount(); i++)
		layout->setColumnStretch(i, 0);

}
void Viewers::updateImages() {
	for (int i = 0; i < viewers.size(); ++i) {
		(viewers.at(i))->update();
	}
}

PerfStats *Viewers::getStats(char *field) {
	for (int i = 0; i < streams.length(); i++) {
		if (streams.at(i)->statistics) {
			double elapsed = streams.at(i)->statistics->getSampleTime(field);
			if (elapsed > 0)
				return (streams.at(i)->statistics);
		}
	}
	return (NULL);
}
void Viewer::updateEnableStates() {
	if (optionButton && optionButton->menu()) {
		allowMixedPlot = IGNORE;
		int section = -1;
		int seenEnabled = -1;
		bool isMixed = false;
		for (int i = 0; i < optionButton->menu()->actions().length(); i++) {
			if (optionButton->menu()->actions().at(i)->isSeparator()) {
				section++;
				continue;
			}
			bool enable = true;
			optionButton->menu()->actions().at(i)->setChecked(true);
			for (int di = 0; di < cfgDisable.length(); di++) {
				if (cfgDisable.at(di).toLower()
						== optionButton->menu()->actions().at(i)->text().toLower()) {
					enable = false;
					break;
				}
			}
			if (enable && seenEnabled != section) {
				if (seenEnabled == -1)
					seenEnabled = section;
				else
					isMixed = true;
			}
			optionButton->menu()->actions().at(i)->setChecked(enable);
		}
		if (isMixed) {
			mixedPlot = true;
			if (plot)
				plot->yAxis->setLabel(" ");
		} else {
			allowMixedPlot = WARNING;
		}
	}
}

void Viewer::setEnableStates(QStringList elements) {
	cfgDisable.clear();
	for (int i = 0; i < elements.length(); i = i + 2) {
		if (elements.at(i + 1).toInt() == 0) {
			cfgDisable.push_back(elements.at(i));
		}
	}
	// this will be run when we reset so
	updateEnableStates();
}

