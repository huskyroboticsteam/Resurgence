#include "LidarRender.h"

#include <cmath>
#include <limits>
#include <iostream>

#include <GL/glut.h>

int main(int argc, char** argv)
{
    using namespace Lidar;
    LidarRender lr(800, 600, "hello");
    std::set<std::shared_ptr<PointXY>> pts;
    for (int i = 0; i < 10; i++)
    {
        PointXY p;
        p.x = i;
        p.y = i;
        pts.insert(std::make_shared<PointXY>(p));
    }
    while (1)
    {
        lr.setBackground(0, 0, 1, 1);
        lr.drawBoundingPolygon(std::vector<std::pair<float, float>>(), 0, 0, 0);
        lr.drawPoints(pts, 1, 0, 0);
        lr.flushToDisplay();
    }
}

namespace Lidar
{
LidarRender::LidarRender(int win_width, int win_height, std::string win_title)
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

void LidarRender::drawPoints(std::set<std::shared_ptr<PointXY>> pts, float r, float g, float b)
{
    float xmin = std::numeric_limits<float>::infinity();
    float xmax = -xmin;
    float ymin = xmin;
    float ymax = -xmin;
    for (std::shared_ptr<PointXY> p: pts)
    {
        xmin = fmin(p->x, xmin);
        xmax = fmax(p->x, xmax);
        ymin = fmin(p->y, ymin);
        ymax = fmax(p->y, ymax);        
    }

    glColor3f(r, g, b);
    glBegin(GL_POINTS);
    for (std::shared_ptr<PointXY> p: pts)
    {
        glVertex2f((p->x - xmin) / (xmax - xmin), (p->y - ymin) / (ymax - ymin));
    }
    glEnd();
}

void LidarRender::drawBoundingPolygon(std::vector<std::pair<float, float>> vertices, float r,
    float g, float b)
{
    glColor3f(r, g, b);
    glBegin(GL_POLYGON);
    glVertex2f(0, 0);
    glVertex2f(0, 0.5);
    glVertex2f(0.5, 0);
    glEnd();
}

void LidarRender::flushToDisplay()
{
    glFlush();
}

} // namespace Lidar
