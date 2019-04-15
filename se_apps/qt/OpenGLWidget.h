/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef OPENGLWIDGET_H
#define OPENGLWIDGET_H

#include <QtOpenGL/QGLWidget>
struct FImage {
	int x;
	int y;
	int format;
	int type;
	void *data;
};
typedef FImage FImage;

class OpenGLWidget: public QGLWidget {
	Q_OBJECT
public:
	explicit OpenGLWidget(QWidget *parent = 0, int width = 640,
			int height = 480);
	OpenGLWidget(FImage image, QWidget *parent = 0);
	void setCallback(void (*_callback)());
	void setZoom(float zoomX, float zoomY);
	void setImage(FImage image);
	void setBufferSize(int width, int height);
	void updateImage();signals:
protected:
	void initializeGL();
	void paintGL();
	void resizeGL(int width, int height);
//void mousePressEvent(QMouseEvent *event);
//void mouseMoveEvent(QMouseEvent *event);

private:
	QWidget *_parent;
	void makeImage();
	void draw();
	QPoint lastPos;
	void *pixelData;
	int pixelFormat;
	int pixelType;
	float zoomX;
	float zoomY;
	int xRot;
	int yRot;
	int zRot;
	bool _3d;
	GLubyte* chkImage;
	void (*callback)();
	bool video;
	int m_nWidth, m_nHeight;
	int m_nCurrWidth, m_nCurrHeight;public slots:

};

#endif // OPENGLWIDGET_H
