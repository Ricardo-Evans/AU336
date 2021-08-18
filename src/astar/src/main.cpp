#include <iostream>
#include <vector>
#include <list>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cctype>
#include "Astar.h"

using namespace cv;
using namespace std;
MapParamNode MapParam;
Mat mat;
int width = -1;
int height = -1;
const int swell = 7;
bool mapReady = false;
bool startReady = false;
bool endReady = false;
bool needPlan = false;

Point startPoint{-1, -1};
Point endPoint{-1, -1};

void World2MapGrid(MapParamNode &MapParam, Point2d &src_point, Point &dst_point) {
    Mat P_src = Mat(Vec2d(src_point.x, src_point.y), CV_64FC1);
    Mat P_dst = MapParam.Rotation.inv() * (P_src - MapParam.Translation);

    dst_point.x = round(P_dst.at<double>(0, 0));
    dst_point.y = MapParam.height - 1 - round(P_dst.at<double>(1, 0));
}

void MapGrid2world(MapParamNode &MapParam, Point &src_point, Point2d &dst_point) {
    Mat P_src = Mat(Vec2d(src_point.x, MapParam.height - 1 - src_point.y), CV_64FC1);

    Mat P_dst = MapParam.Rotation * P_src + MapParam.Translation;

    dst_point.x = P_dst.at<double>(0, 0);
    dst_point.y = P_dst.at<double>(1, 0);
}

void swellMap(Mat &mat, int x, int y) {
    for (int i = -swell; i <= swell; ++i) {
        for (int j = -swell; j <= swell; ++j) {
            if (i == 0 && j == 0) continue;
            int x_ = x + i;
            int y_ = y + j;
            if (x_ < 0 || x_ >= width || y_ < 0 || y_ >= height) continue;
            if (mat.at<uint8_t>(x_, y_) == 0XFF) mat.at<uint8_t>(x_, y_) = 0X01;
        }
    }
}

void initializeMap(Mat &mat) {
    width = mat.rows;
    height = mat.cols;
    for (int i = 0; i < width; ++i) {
        for (int j = 0; j < height; ++j) {
            uint8_t value = mat.at<uint8_t>(i, j);
            if (value == 0X00) swellMap(mat, i, j);
            mat.at<uint8_t>(i, j) = value;
        }
    }
}

void MapCallback(const nav_msgs::OccupancyGrid &msg) {

    // Get the parameters of map
    MapParam.resolution = msg.info.resolution;
    MapParam.height = msg.info.height;
    MapParam.width = msg.info.width;
    // The origin of the MapGrid is on the bottom left corner of the map
    MapParam.x = msg.info.origin.position.x;
    MapParam.y = msg.info.origin.position.y;


    // Calculate the pose of map with respect to the world of rviz
    double roll, pitch, yaw;
    geometry_msgs::Quaternion q = msg.info.origin.orientation;
    tf::Quaternion quat(q.x, q.y, q.z, q.w); // x, y, z, w
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    double theta = yaw;

    //从rviz上所给定的起点和终点坐标是真实世界坐标系下的位置，需要转化为地图坐标下的表示
    //MapParam.Rotation MapParam.Translation 用于该变换
    MapParam.Rotation = Mat::zeros(2, 2, CV_64FC1);
    MapParam.Rotation.at<double>(0, 0) = MapParam.resolution * cos(theta);
    MapParam.Rotation.at<double>(0, 1) = MapParam.resolution * sin(-theta);
    MapParam.Rotation.at<double>(1, 0) = MapParam.resolution * sin(theta);
    MapParam.Rotation.at<double>(1, 1) = MapParam.resolution * cos(theta);
    MapParam.Translation = Mat(Vec2d(MapParam.x, MapParam.y), CV_64FC1);

    cout << "Map:" << endl;
    cout << "MapParam.height:" << MapParam.height << endl;
    cout << "MapParam.width:" << MapParam.width << endl;

    Mat Map(MapParam.height, MapParam.width, CV_8UC1);
    int GridFlag;
    for (int i = 0; i < MapParam.height; i++) {
        for (int j = 0; j < MapParam.width; j++) {
            GridFlag = msg.data[i * MapParam.width + j];
            GridFlag = (GridFlag < 0) ? 100 : GridFlag; // set Unknown to 0
            Map.at<uchar>(j, MapParam.height - i - 1) = 255 - round(GridFlag * 255.0 / 100.0);
        }
    }
    initializeMap(Map);
    mat = Map;
    mapReady = true;
    needPlan = true;
}

void StartPointCallback(const geometry_msgs::PoseWithCovarianceStamped &msg) {
    Point2d src_point = Point2d(msg.pose.pose.position.x, msg.pose.pose.position.y);
    World2MapGrid(MapParam, src_point, MapParam.StartPoint);
    cout << "StartPoint:" << MapParam.StartPoint << endl;
    startReady = true;
    needPlan = true;
}

void TargetPointCallback(const geometry_msgs::PoseStamped &msg) {
    Point2d src_point = Point2d(msg.pose.position.x, msg.pose.position.y);
    World2MapGrid(MapParam, src_point, MapParam.TargetPoint);
    int p = mat.at<uint8_t>(MapParam.TargetPoint.x, MapParam.TargetPoint.y);
    cout << "flag:" << p << endl;
    MapGrid2world(MapParam, MapParam.TargetPoint, src_point);
    cout << "TargetPoint world:" << src_point << endl;
    cout << "TargetPoint:" << MapParam.TargetPoint << endl;
    endReady = true;
    needPlan = true;
}

void PathGrid2world(MapParamNode &MapParam, vector <Point> &PathList, nav_msgs::Path &plan_path) {
    for (int i = 0; i < PathList.size(); i++) {
        Point2d dst_point;
        MapGrid2world(MapParam, PathList[i], dst_point);
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = dst_point.x;
        pose_stamped.pose.position.y = dst_point.y;
        pose_stamped.pose.position.z = 0;
        pose_stamped.pose.orientation.w = 1.0;
        plan_path.poses.push_back(pose_stamped);
    }
}

void PathGrid2world(MapParamNode &MapParam, list <Point> &PathList, nav_msgs::Path &plan_path) {
    for (auto &p:PathList) {
        Point2d dst_point;
        MapGrid2world(MapParam, p, dst_point);
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = dst_point.x;
        pose_stamped.pose.position.y = dst_point.y;
        pose_stamped.pose.position.z = 0;
        pose_stamped.pose.orientation.w = 1.0;
        plan_path.poses.push_back(pose_stamped);
    }
}

bool isValid(const Point &start, const Point &end) {
    return mat.at<uint8_t>(start.x, start.y) == 0XFF && mat.at<uint8_t>(end.x, end.y) == 0XFF;
}

list <Point> plan(Point start, Point end, list<AStarNode *> &nodes) {
    list <Point> path;
    if (!isValid(start, end)) {
        cout << "invalid start or end" << endl;
        return move(path);
    }
    cout << "start plan from " << start << " to " << end << endl;
    priority_queue < AStarNode * , deque < AStarNode * >, AStarNodeNear > queue;
    unordered_set <Point, PointHash> closeSet;
    int startX = start.x;
    int startY = start.y;
    int endX = end.x;
    int endY = end.y;
    AStarNode *node = new AStarNode(start, 0, distance(start, end));
    nodes.push_back(node);
    queue.push(node);
    while (!queue.empty()) {
        node = queue.top();
        queue.pop();
        if (node->invalid) continue;
        if (node->p == end) {
            while (node != nullptr) {
                path.push_front(node->p);
                node = node->n;
            }
            cout << "plan end, total steps: " << path.size() - 1 << endl;
            return move(path);
        }
        if (closeSet.count(node->p) != 0) continue;
        closeSet.insert(node->p);
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;
                int x = node->p.x + dx;
                int y = node->p.y + dy;
                VALUE_TYPE g = node->g;
                if (dx != 0 && dy != 0) g += cost2;
                else g += cost1;
                Point p{x, y};
                if (x < 0 || x >= width || y < 0 || y >= height) continue;
                if (mat.at<uint8_t>(x, y) != 0xFF) continue;
                if (closeSet.count(p) != 0) continue;
                AStarNode *n = new AStarNode(node, p, g, distance(p, end));
                nodes.push_back(n);
                queue.push(n);
            }
        }
    }
    cout << "cannot plan a path" << endl;
    return move(path);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "astar");
    ros::NodeHandle n;

    geometry_msgs::PointStamped astar_step;

    // Subscribe topics
    ros::Subscriber Map_sub = n.subscribe("map", 10, MapCallback);
    ros::Subscriber StarPoint_sub = n.subscribe("move_base/NavfnROS/Astar/initialpose", 10, StartPointCallback);
    ros::Subscriber TargetPoint_sub = n.subscribe("move_base/NavfnROS/Astar/target", 10, TargetPointCallback);

    // Publisher topics
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("move_base/NavfnROS/nav_path", 10);
    ros::Rate loop_rate(20);

    while (ros::ok()) {
        if (needPlan && mapReady && startReady && endReady) {
            Point currentStart = MapParam.StartPoint;
            Point currentEnd = MapParam.TargetPoint;
            if (startPoint != currentStart || endPoint != currentEnd) {
                startPoint = currentStart;
                endPoint = currentEnd;
                list < AStarNode * > nodes;
                double startTime = ros::Time::now().toSec();
                list <Point> path = plan(startPoint, endPoint, nodes);
                double endTime = ros::Time::now().toSec();
                cout << "time usage: " << endTime - startTime << endl;
                for (auto node:nodes) delete node;
                nav_msgs::Path plan_path;
                PathGrid2world(MapParam, path, plan_path);
                path_pub.publish(plan_path);
            }
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
