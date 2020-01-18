#include "LidarRender.h"

#include <cmath>
#include <limits>

#include <GL/glut.h>

namespace lidar
{
LidarRender::LidarRender(int win_width, int win_height, std::string win_title, 
    float disp_limits_x_range, float disp_limits_y_range) :
    disp_x_range(disp_limits_x_range), 
    disp_y_range(disp_limits_y_range)
{
    int argc = 0;
    glutInit(&argc, NULL);
    glutInitDisplayMode(GLUT_SINGLE);
    glutInitWindowSize(win_width, win_height);
    glutInitWindowPosition(0, 0);
    glutCreateWindow(win_title.c_str());
}

LidarRender::~LidarRender()
{
}

void LidarRender::setBackground(float r, float g, float b, float a)
{
    glClearColor(r, g, b, a);
    glClear(GL_COLOR_BUFFER_BIT);
}

void LidarRender::drawPoints(const std::vector<PointXY> &pts, float r, float g, float b,
    float pt_size)
{
    glColor3f(r, g, b);
    glPointSize(pt_size);
    glBegin(GL_POINTS);
    for (PointXY p : pts)
    {
        glVertex2f(p.x / this->disp_x_range, p.y / this->disp_y_range);
    }
    glEnd();
}

void LidarRender::drawBoundingPolygon(std::vector<PointXY> &vertices, float r,
    float g, float b, float line_width)
{
    glColor3f(r, g, b);
    glLineWidth(line_width);
    glBegin(GL_LINES);
    for (int i = 0; i < vertices.size(); i++)
    {
        PointXY v1 = vertices[i];
        PointXY v2 = vertices[(i + 1) % vertices.size()];
        glVertex2f(v1.x / this->disp_x_range, v1.y / this->disp_y_range);
        glVertex2f(v2.x / this->disp_x_range, v2.y / this->disp_y_range);
    }
    glEnd();
}

void LidarRender::flushToDisplay()
{
    glFlush();
}

} // namespace Lidar

int main(int argc, char** argv)
{
    using namespace lidar;
    LidarRender lr(800, 600, "point clustering simulation", 5000, 5000);
    RPLidar l(256000, "/dev/ttyUSB0");

    lr.setBackground(1, 1, 1, 1);
    while (1)
    {
        lr.drawPoints(l.scan_frame(100), 0, 0, 0, 3);
        lr.flushToDisplay();
    }
}
