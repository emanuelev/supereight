/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QLabel>
#include <QAction>
#include <QCheckBox>
#include <QSignalMapper>
#include <perfstats.h>
#include <PowerMonitor.h>
#include "OpenGLWidget.h"

#include <QGridLayout>
#include <TooN/se3.h>
namespace Ui {
class Viewers;
}
class Viewers;
class QCustomPlot;
struct Stream {
	FImage image;
	PerfStats *statistics;
	char *title;
	char *help;
	bool *active;
	int id;
	bool allowScaling;
};
typedef struct Stream Stream;
enum CameraState {
	CAMERA_RUNNING, CAMERA_CLOSED, CAMERA_PAUSED, CAMERA_LIVE, UNINITIALISED
};

class Viewer: public QWidget {
	Q_OBJECT
public:

	Viewer(Stream *stream, int id, QWidget *parent);
	~Viewer();
	void setStream(Stream *stream);
	Stream *getStream();
	int getStreamID();
	void setActive(bool value);
	bool getActive();
	int getID();
	void pause();
	void resume();
	void update();
	void setZoom(float zoomX, float zoomY);
	void setBufferSize(int width, int height);
	void setEnableStates(QStringList elements);
	QString getOptions();
	QCustomPlot *plot;
	QSize sizeHint() const;
	void setHoldVariable(bool *variable);

	signals:
	// define mouseClick signal
	void mouseClickEvent();protected slots:
	void mousePressEvent(QMouseEvent * event);
	void contextMenuEvent(QContextMenuEvent *event);
	void checkBoxChanged();
	void plotWaveSelectionChanged();
private:
	enum MixType {
		ALLOW, IGNORE, WARNING
	};
	FImage image;
	OpenGLWidget *opengl;
	MixType allowMixedPlot;
	bool mixedPlot;
	QCheckBox *checkbox;
	QLabel *label;
	QPushButton *optionButton;
	QWidget *parent;
	QGridLayout *layout;
	QFrame *frame;
	int _id;
	bool *holdProcessing;
	Viewers *_container;
	Stream *_stream;
	void setStreamImage(Stream *stream);
	void setStreamStats(Stream *stream);
	void setOptionMenu();
	double lastTime;
	QStringList cfgDisable;
	void updateEnableStates();
	int lastFrame;
};

class Viewers: public QWidget { //QMainWindow{
	Q_OBJECT
	friend class Viewer;
public:
	Viewers(QWidget *parent = 0);
	~Viewers();
	void updateImages();
	void setIdleFunction(void (*_callback)());
	void setZoomViewer(int id, float zoomX, float zoomY);
	void setBufferSize(int width, int height);
	void addViewer(FImage image, const char *title, const char *help,
			bool *active);
	void addViewer(FImage image, const char *title, const char *help,
			bool *active, bool allowScale);
	void addViewer(PerfStats *stats, const char *title, const char *help,
			bool *active);
	void setCurrentViewer(Viewer *viewer);
	void pauseStream(int id);
	void resumeStream(int id);
	void setZoom(float zoomX, float zoomY);
	void setZoom(float zoom);
	void addDefConfig(int viewer, QString name, bool enabled,
			QStringList visible);
	void setStatEntry(QString title, std::vector<const char *> entries,
			bool enabled);
	QString getConfig(int viewerID);
	int getViewerCount();
	PerfStats *getStats(char *);
	QMenu *streamMenu;
	void reLayout();
	bool holdProcessing();protected slots:
	void menuAction( QAction *action);

private:
	QActionGroup *streamMenuActions;
	QList<Viewer *> viewers;
	QList<Stream *> streams;
	void addViewer(Stream *);
	QGridLayout *layout;
	Viewer *_currentViewer;
	bool isStreamActive(int id);
	bool _holdProcessing;

};

#endif // MAINWINDOW_H
