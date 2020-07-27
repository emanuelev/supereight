/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include <QtWidgets>
#include <QtOpenGL>
#include "OpenGLWidget.h"
#include "MainWindow.h"
#include <iostream>
OpenGLWidget::OpenGLWidget(FImage image, QWidget *parent) :
		QGLWidget(QGLFormat(QGL::SampleBuffers), parent) {
	callback = NULL;
	video = false;
	xRot = 0;
	yRot = 0;
	zRot = 0;
	zoomX = 1;
	zoomY = 1;
	m_nWidth = image.x;
	m_nHeight = image.y;
	_3d = false;
	m_nCurrWidth = m_nWidth;
	m_nCurrHeight = m_nHeight;
	setImage(image);
}
void OpenGLWidget::setBufferSize(int width, int height) {
	m_nWidth = width;
	m_nHeight = height;
	//glOrtho(0, m_nWidth, 0, m_nHeight, 0, 1);
	//glViewport(0, 0, m_nWidth, m_nHeight);
}
OpenGLWidget::OpenGLWidget(QWidget *parent, int width, int height) :
		QGLWidget(QGLFormat(QGL::SampleBuffers), parent) {
	callback = NULL;
	video = false;
	xRot = 0;
	yRot = 0;
	zRot = 0;
	zoomX = 1.0;
	zoomY = 1.0;
	m_nWidth = width;
	m_nHeight = height;
	_3d = false;
	m_nCurrWidth = m_nWidth;
	m_nCurrHeight = m_nHeight;
	_parent = parent;
}
void OpenGLWidget::setCallback(void (*_callback)()) {
	callback = _callback;
}
void OpenGLWidget::initializeGL() {
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, m_nWidth, 0, m_nHeight, 0, 1);
	glMatrixMode (GL_MODELVIEW);
	glShadeModel (GL_FLAT);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

}

void OpenGLWidget::paintGL() {

	glClear (GL_COLOR_BUFFER_BIT);
	glPixelZoom(m_nCurrWidth / (double) m_nWidth,
			-(m_nCurrHeight / ((double) m_nHeight)));
	glRasterPos2i(0, m_nHeight); //m_nHeight *( m_nCurrHeight / ((double) m_nHeight) ));
	glDrawPixels(m_nWidth, m_nHeight, pixelFormat, pixelType, pixelData);
}

void OpenGLWidget::resizeGL(int width, int height) {
	glViewport(0, 0, width, height);
	m_nCurrWidth = width;
	m_nCurrHeight = height;
	glPixelZoom(m_nCurrWidth / (double) m_nWidth,
			-m_nCurrHeight / (double) m_nHeight);

	//std::cerr << width << "x" << height << "  ----- " <<  m_nWidth  << "x"<<m_nHeight << "zoom " << (m_nCurrWidth / (double) m_nWidth)<<"\n";
}

void OpenGLWidget::updateImage() {

}
void OpenGLWidget::setZoom(float x, float y) {
	zoomX = x;
	zoomY = y;
	std::cerr << "CHANGED ZOOM\n";
}

void OpenGLWidget::setImage(FImage image) {
	::glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	::glPixelStorei(GL_UNPACK_ROW_LENGTH, image.x);
	pixelData = (void *) image.data;
	pixelFormat = image.format;
	pixelType = image.type;

	m_nWidth = image.x;
	m_nHeight = image.y;
	video = true;
}

