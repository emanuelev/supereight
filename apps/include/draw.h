/*

 Copyright (c) 2011-2013 Gerhard Reitmayr, TU Graz

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */
#ifdef __APPLE__
    #include <GLUT/glut.h>
#else
    #include <GL/glut.h>
#endif

#define WIDTH 1024
#define HEIGHT 768

template<typename T> struct gl;

template<> struct gl<unsigned char> {
	static const int format = GL_LUMINANCE;
	static const int type = GL_UNSIGNED_BYTE;
};

template<> struct gl<uchar3> {
	static const int format = GL_RGB;
	static const int type = GL_UNSIGNED_BYTE;
};

template<> struct gl<uchar4> {
	static const int format = GL_RGBA;
	static const int type = GL_UNSIGNED_BYTE;
};

template<> struct gl<float> {
	static const int format = GL_LUMINANCE;
	static const int type = GL_FLOAT;
};
template<> struct gl<uint16_t> {
	static const int format = GL_LUMINANCE;
	static const int type = GL_UNSIGNED_SHORT;
};
template<> struct gl<float3> {
	static const int format = GL_RGB;
	static const int type = GL_FLOAT;
};

template<typename T>
void drawit(T* scene, uint2 size) {
	static uint2 lastsize = { 0, 0 };
	char * t = (char*) "toto";
	int g = 1;
	if (lastsize.x != size.x || lastsize.y != size.y) {
		lastsize = size;
		glutInit(&g, &t);
		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

		glutInitWindowPosition(100, 100);
		glutInitWindowSize(size.x, size.y);
		glutCreateWindow(" ");
	}

	glClear(GL_COLOR_BUFFER_BIT);
	glRasterPos2i(-1, 1);
	glPixelZoom(1, -1);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glPixelStorei(GL_UNPACK_ROW_LENGTH, size.x);
	glDrawPixels(size.x, size.y, gl<T>::format, gl<T>::type, scene);
	glutSwapBuffers();

}
template<typename A, typename B, typename C, typename D, typename E>
void drawthem(A* scene1, B* scene2, C* scene3, D* scene4, E*, 
        uint2 size_s1, uint2 size_s2, uint2 size_s3, uint2 size_s4) {
	static uint2 lastsize = { 0, 0 };
	char * t = (char*) "toto";
	int g = 1;
	if (lastsize.x != size_s2.x || lastsize.y != size_s2.y) {
		lastsize = size_s2;
		glutInit(&g, &t);
		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
		glutInitWindowSize(320 * 2, 240 * 2);
		// glutInitWindowPosition(100, 100);

		glutCreateWindow("Kfusion Display");
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		glMatrixMode(GL_PROJECTION);

		gluOrtho2D(0.0, (GLfloat) 640, 0.0, (GLfloat) 480);
		glMatrixMode(GL_MODELVIEW);

	}
	glClear(GL_COLOR_BUFFER_BIT);

	glRasterPos2i(0, 480);
	glPixelZoom(320.0 / size_s1.x, -240.0 / size_s1.y);
	glDrawPixels(size_s1.x, size_s1.y, gl<A>::format, gl<A>::type, scene1);
	glRasterPos2i(320, 480);
	glPixelZoom(320.0 / size_s2.x, -240.0 / size_s2.y);
	glDrawPixels(size_s2.x, size_s2.y, gl<B>::format, gl<B>::type, scene2);
	glRasterPos2i(0, 240);
	glPixelZoom(320.0 / size_s3.x, -240.0 / size_s3.y);
	glDrawPixels(size_s3.x, size_s3.y, gl<C>::format, gl<C>::type, scene3);
	glRasterPos2i(320, 240);
	glPixelZoom(320.0 / size_s4.x, -240.0 / size_s4.y);
	glDrawPixels(size_s4.x, size_s4.y, gl<D>::format, gl<D>::type, scene4);
	glutSwapBuffers();

}

