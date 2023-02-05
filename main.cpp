#include <GL/glut.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "interpolation.h"
#define M_PI 3.1415926



std::vector<Eigen::Vector3d> ctrl_pnts;	// positions of control points
std::vector<Eigen::Vector3d> positions; // positions of points on a curve between two control points
Eigen::Vector3d cur_pos;	//current position
std::vector<QuaternionS> ctrl_oriens;	// orientations of control points
std::vector<QuaternionS> orientations;	
QuaternionS cur_orien(1.0, 0, 0, 0);	// orientations of points on a curve between two control points
double dt = 0.01;
int theta = 0;
int cur_start = 0;	// index of start of current curve segment
int inter_type, orien_type;	// type of interpolation for translation and orientation
int num_ctrl_pnts = 4;	// number of control points

void displayTeapot()
{
	// clear buffer
	glClearColor(1.0, 0.941, 0.961, 1.0);
	glClearDepth(1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// render state
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);

	// enable lighting
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	// light source attributes
	GLfloat LightAmbient[] = { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightDiffuse[] = { 0.3f, 0.3f, 0.3f, 1.0f };
	GLfloat LightSpecular[] = { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightPosition[] = { 5.0f, 5.0f, 5.0f, 1.0f };

	glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);

	// surface material attributes
	GLfloat material_Ka[] = { 1.0f, 0.713f, 0.757f, 1.0f };
	GLfloat material_Kd[] = { 0.43f, 0.47f, 0.54f, 1.0f };
	GLfloat material_Ks[] = { 0.33f, 0.33f, 0.52f, 1.0f };
	GLfloat material_Ke[] = { 0.1f , 0.0f , 0.1f , 1.0f };
	GLfloat material_Se = 10;

	glMaterialfv(GL_FRONT, GL_AMBIENT, material_Ka);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, material_Kd);
	glMaterialfv(GL_FRONT, GL_SPECULAR, material_Ks);
	glMaterialfv(GL_FRONT, GL_EMISSION, material_Ke);
	glMaterialf(GL_FRONT, GL_SHININESS, material_Se);

	// modelview matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(cur_pos(0), cur_pos(1), cur_pos(2));
	Eigen::Vector4d cur_ori = cur_orien.toAxisAngle();
	glRotatef(cur_ori(0) * 180 / M_PI, cur_ori(1), cur_ori(2), cur_ori(3));
	//Eigen::Vector4d theta_axis = cur_orien.toAxisAngle();
	if (orien_type == 1)
	{
		glRotatef(theta, 1, 1, 1);
	}
	else
	{
		Eigen::Vector4d theta_axis = cur_orien.toAxisAngle();
		glRotatef(theta_axis(0), theta_axis(1), theta_axis(2), theta_axis(3));
	}
	// render objects
	glutSolidTeapot(0.5);

	// disable lighting
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);

	// swap back and front buffers
	glutSwapBuffers();
}


void translateAndRotate(int i)
{
	// when on a curve between two points
	if (i < positions.size() - 1)
	{
		cur_pos = positions[i];
		cur_orien = orientations[i];
		glutPostRedisplay();
		glutTimerFunc(10, translateAndRotate, i + 1);
	}
	// when on the start of a curve
	else
	{
		// interpolate translations and rotations
		if (cur_start < ctrl_pnts.size() - 3)
		{
			positions = getPosition(ctrl_pnts[cur_start], ctrl_pnts[cur_start + 1], ctrl_pnts[cur_start + 2], ctrl_pnts[cur_start + 3], 0, 1, dt, inter_type);
			orientations = SLerp(ctrl_oriens[cur_start + 1], ctrl_oriens[cur_start + 2], dt);
			cur_start++;
			glutTimerFunc(10, translateAndRotate, 0);
		}
		else if (cur_start == ctrl_pnts.size() - 3)
		{
			positions = getPosition(ctrl_pnts[cur_start], ctrl_pnts[cur_start + 1], ctrl_pnts[cur_start + 2], 2 * ctrl_pnts[cur_start + 2] - ctrl_pnts[cur_start + 1], 0, 1, dt, inter_type);
			orientations = SLerp(ctrl_oriens[cur_start + 1], ctrl_oriens[cur_start + 2], dt);
			cur_start++;
			glutTimerFunc(10, translateAndRotate, 0);
		}
		else
		{
			positions = getPosition(2 * ctrl_pnts[0] - ctrl_pnts[1], ctrl_pnts[0], ctrl_pnts[1], ctrl_pnts[2], 0, 1, dt, inter_type);
			orientations = SLerp(ctrl_oriens[0], ctrl_oriens[1], dt);
			cur_start = 0;
			glutTimerFunc(10, translateAndRotate, 0);
		}
	}
	theta += 1;
	if (theta == 360)
	{
		theta = 0;
	}
}

void reshape(int w, int h) {
	// viewport
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	// projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, (GLfloat)w / (GLfloat)h, 1.0, 2000.0);
}


int main(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

	// for test, if we don't want to input the control points in person
	std::cout << "please input the method of interpolation (0 for CatmullRom and 1 for B): ";
	std::cin >> inter_type;
	std::cout << "please input the method of orientation expression (0 for fixed angles and 1 for quaternion): ";
	std::cin >> orien_type;
	std::cout << "please input dt: ";
	std::cin >> dt;

	// set positions of control points
	num_ctrl_pnts = 4;
	//Eigen::Vector3d p0 (0, 0, 0), p1(1, 0.5, 0), p2 (2, 0.866, 0), p3(3, 1, 0);
	Eigen::Vector3d p0(0, -2.0, -15), p1(1.2, -1.0, -5.0), p2(-1, 1.5, -5.0), p3(0, -1.0, -10.0), p4(-1.0, -1.0, -4.0), p5(0.5, 1.0, -5.0);
	ctrl_pnts.push_back(p0);
	ctrl_pnts.push_back(p1);
	ctrl_pnts.push_back(p2);
	ctrl_pnts.push_back(p3);
	ctrl_pnts.push_back(p4);
	ctrl_pnts.push_back(p5);
	
	// set fixed angles
	//QuaternionS q0(Eigen::Vector3d(1, 1, 1), 0), q1(Eigen::Vector3d(1, 1, 1), 45), q2(Eigen::Vector3d(1, 1, 1), 90 * M_PI / 180), q3(Eigen::Vector3d(1, 1, 1), 135 * M_PI / 180), q4(Eigen::Vector3d(1, 1, 1), 180 * M_PI / 180), q5(Eigen::Vector3d(1, 1, 1), 225 * M_PI / 180);
	QuaternionS q0, q1, q2, q3, q4, q5;
	q0 = q0.fromFixedXYZ(0, 0, 0);
	q1 = q1.fromFixedXYZ(90 * M_PI / 180, 0, 0);
	q2 = q2.fromFixedXYZ(90 * M_PI / 180, 90 * M_PI / 180, 0);
	q3 = q3.fromFixedXYZ(0, 0, 90 * M_PI / 180);
	q4 = q4.fromFixedXYZ(90 * M_PI / 180, 90 * M_PI / 180, 0);
	q5 = q5.fromFixedXYZ(90 * M_PI / 180, 90 * M_PI / 180, 0);
	ctrl_oriens.push_back(q0);
	ctrl_oriens.push_back(q1);
	ctrl_oriens.push_back(q2);
	ctrl_oriens.push_back(q3);
	ctrl_oriens.push_back(q4);
	ctrl_oriens.push_back(q5);

	/* if we want to set the control points in person, we can use the following codes
	std::cout << "please input num of control points (at least 4): ";
	std::cin >> num_ctrl_pnts;
	if (num_ctrl_pnts < 4)
	{
		num_ctrl_pnts = 4;
	}
	for (int i = 0; i < num_ctrl_pnts; i++)
	{
		double x, y, z;
		std::cout << "please input position of control point " << i << ": ";
		std::cin >> x >> y >> z;
		Eigen::Vector3d p;
		p << x, y, z;
		ctrl_pnts.push_back(p);
	}

	for (int i = 0; i < num_ctrl_pnts; i++)
	{
		if (orien_type == 0)
		{
			double alpha, beta, gamma;
			std::cout << "please input orientation at control point " << i << " (RPY, degree): ";
			std::cin >> alpha >> beta >> gamma;
			QuaternionS q;
			q.fromFixedXYZ(alpha * M_PI / 180, beta * M_PI / 180, gamma * M_PI / 180);
			ctrl_oriens.push_back(q);
		}
		else
		{
			double w, x, y, z;
			std::cout << "please input orientation at control point " << i << " (w + ix + jy + kz): ";
			std::cin >> w >> x >> y >> z;
			QuaternionS q(w, x, y, z);
			ctrl_oriens.push_back(q);
		}
	}
	*/

	// interpolate translation and rotation
	positions = getPosition(2 * ctrl_pnts[0] - ctrl_pnts[1], ctrl_pnts[0], ctrl_pnts[1], ctrl_pnts[2], 0, 1, dt, inter_type);
	orientations = SLerp(ctrl_oriens[0], ctrl_oriens[1], dt);

	// set the window size and create a window
	glutInitWindowSize(900, 900);
	glutCreateWindow("MainWindow");
	// set the display function
	glutDisplayFunc(displayTeapot);
	// set the reshape function, control the perspective
	glutReshapeFunc(reshape);
	// set the timer
	glutTimerFunc((int)dt*1000, translateAndRotate, 0);
	// start the loop of the window
	glutMainLoop();
	return 0;
}

