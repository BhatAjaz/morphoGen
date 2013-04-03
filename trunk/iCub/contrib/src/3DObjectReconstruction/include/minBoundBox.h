#include <string>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <boost/thread/thread.hpp>
#include "pcl/common/common_headers.h"
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>

#include "helpers.h"

class MinBoundBox
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    void retrieveEdges(const yarp::sig::Vector &vx, const yarp::sig::Vector &vy, const yarp::sig::Vector &vz, std::vector<yarp::sig::Vector> &edges);
    void retrieveEdges2(const yarp::sig::Vector &vx, const yarp::sig::Vector &vy, const yarp::sig::Vector &vz, const std::vector<yarp::sig::Vector> &edges1, std::vector<yarp::sig::Vector> &edges);
    void eulerAngles(const std::vector<yarp::sig::Vector> &edges1,const std::vector<yarp::sig::Vector> &edges2,const std::vector<yarp::sig::Vector> &crossProduct,yarp::sig::Vector &alpha,yarp::sig::Vector &beta,yarp::sig::Vector &gamma);
    void buildRotMat3(const double &alpha,const double &beta,const double &gamma,yarp::sig::Matrix &rot);
    void buildRotMat2(const double &theta,yarp::sig::Matrix &rot);
    void findRotation(const yarp::sig::Matrix &xyz_i, yarp::sig::Matrix &rot2);
    void findRotation2D(const yarp::sig::Matrix &xy, yarp::sig::Matrix &rot2);
    void assignCorners(const yarp::sig::Matrix &minmax, yarp::sig::Matrix &cornerpoints);

    public:

    MinBoundBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void retrieveMinimumBox(yarp::sig::Matrix &cornerpoints, yarp::sig::Matrix &rotmat);
};

