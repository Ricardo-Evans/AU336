#ifndef ASTAR_H
#define ASTAR_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>

using namespace std;
using namespace cv;

typedef int VALUE_TYPE;

const double theta = 2.0;
const VALUE_TYPE cost1 = 10;
const VALUE_TYPE cost2 = 14;

struct MapParamNode {
    Point StartPoint, TargetPoint;
    Mat Rotation;
    Mat Translation;
    double resolution;
    int height;
    int width;
    int x;
    int y;
};

VALUE_TYPE distance(int x1, int y1, int x2, int y2) {
    int dx = abs(x1 - x2);
    int dy = abs(y1 - y2);
    return cost1 * (dx + dy) + (cost2 - 2 * cost1) * min(dx, dy);
}

VALUE_TYPE distance(const Point &p1, const Point &p2) {
    return distance(p1.x, p1.y, p2.x, p2.y);
}

struct AStarNode {
    AStarNode *n = nullptr;
    Point p;
    VALUE_TYPE f;
    VALUE_TYPE g;
    VALUE_TYPE h;
    bool invalid = false;

    AStarNode(Point p_, VALUE_TYPE g_, VALUE_TYPE h_) : AStarNode(nullptr, p_, g_, h_) {}

    AStarNode(AStarNode *n_, Point p_, VALUE_TYPE g_, VALUE_TYPE h_) : n(n_), p(p_),
                                                                       f((VALUE_TYPE) (g_ + theta * h_)),
                                                                       g(g_), h(h_) {}
};

bool compare(const AStarNode *node1, const AStarNode *node2) {
    return node1->p == node2->p;
}

struct AStarNodeNear {
    bool operator()(const AStarNode *node1, const AStarNode *node2) const {
        return node1->f > node2->f;
    }
};

struct PointHash {
    size_t operator()(const Point &p) const {
        return p.x << 16 || p.y;
    }
};

#endif
