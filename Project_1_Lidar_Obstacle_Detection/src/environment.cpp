/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <unordered_set>

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

//streaming multiple point cloud files
void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    bool showInputCloud = false;
    bool showFilteredCloud = true;
    bool showVanBox = false;
    bool useCustomFunctions = true;
    float distanceTol = 0.2; // in cm.
    int minClusterSize = 50;
    int maxClusterSize = 2000;
    std::vector<Color> colors = {Color(0, 1, 1), Color(1, 0.92, 0.016), Color(0, 0, 1)};

    if (showInputCloud)
    {
        renderPointCloud(viewer, inputCloud, "inputCloud");
        return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.1f, Eigen::Vector4f(-20.0f, -5.0f, -2.0f, 1), Eigen::Vector4f(20.0f, 8.0f, 2.0f, 1));
    if (showFilteredCloud)
        renderPointCloud(viewer, filterCloud, "filterCloud");

    // render box for seperating the van points
    int clusterId = 0;
    renderVanBox(viewer, clusterId, -1.5, -1.7, -1, 2.6, 1.7, 0.4, showVanBox);
    clusterId++;

    // seperate the point cloud into planes - road and obstacles
    ProcessPointClouds<pcl::PointXYZI> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud;
    if (!useCustomFunctions)
    {
        segmentCloud = pointProcessor.SegmentPlane(filterCloud, 100, 0.2); // can use .SegmentPlane because it's on the stack
    }
    else
    {
        std::cout << "Segmenting plane using RANSAC" << std::endl;
        segmentCloud = pointProcessor.SegmentPlaneRansac(filterCloud, 100, 0.2);
    }

    renderPointCloud(viewer, segmentCloud.first, "obstacleCloud", Color(1, 0, 0)); // obstacle cloud
    renderPointCloud(viewer, segmentCloud.second, "roadCloud", Color(0, 1, 0));    // road cloud

    // cluster the obstacles to differentiate between various objects
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters;
    if (!useCustomFunctions)
    {
        std::cout << "Performing PCL Clustering" << std::endl;
        cloudClusters = pointProcessor.Clustering(segmentCloud.first, distanceTol, minClusterSize, maxClusterSize);
    }
    else
    {
        std::cout << "Performing Euclidean Clustering" << std::endl;
        cloudClusters = pointProcessor.EuclideanClustering(segmentCloud.first, distanceTol, minClusterSize, maxClusterSize);
    }

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        //std::cout << "cluster " << clusterId << ": color: " << clusterId % colors.size() << " size ";
        // pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstacleCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId, Color(1, 1, 1), 0.7);

        ++clusterId;
    }
}

// reads a single lidar file and creates bounding boxes for obstacles, segments the road
void cityBlockStatic(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.1f, Eigen::Vector4f(-20.0f, -5.0f, -2.0f, 1), Eigen::Vector4f(20.0f, 8.0f, 2.0f, 1));
    //renderPointCloud(viewer,inputCloud,"inputCloud");
    //renderPointCloud(viewer, filterCloud, "filterCloud");

    // render box for seperating the van points
    int clusterId = 0;
    renderVanBox(viewer, clusterId, -1.5, -1.7, -1, 2.6, 1.7, 0.4, false);
    clusterId++;

    // seperate the point cloud into planes - road and obstacles
    ProcessPointClouds<pcl::PointXYZI> pointProcessor;                                                                                                       // on the stack
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor.SegmentPlane(filterCloud, 100, 0.2); // can use .SgementPlane because it's on the stack
    segmentCloud = pointProcessor.SegmentPlane(filterCloud, 100, 0.2);                                                                                       // can use .SegmentPlane because it's on the stack

    // std::unordered_set<int> inliers = pointProcessor.Ransac3DHelper(filterCloud, 100, 0.2);
    // cout << "\nAll elements : ";
    // std::unordered_set<int>::iterator itr;
    // for (itr = inliers.begin(); itr != inliers.end(); itr++)
    //     cout << (*itr) << endl;
    // cout << "\nAll elements : " << endl;

    renderPointCloud(viewer, segmentCloud.first, "obstacleCloud", Color(1, 0, 0)); // obstacle cloud
    renderPointCloud(viewer, segmentCloud.second, "roadCloud", Color(0, 1, 0));    // road cloud

    // cluster the obstacles to differentiate between various objects
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor.EuclideanClustering(segmentCloud.first, 0.2, 50, 2000);
    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 0.2, 50, 2000);
    std::vector<Color> colors = {Color(0, 1, 1), Color(1, 0.92, 0.016), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {

        //std::cout << "cluster " << clusterId << ": color: " << clusterId % colors.size() << " size ";
        // pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstacleCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId, Color(1, 1, 1), 0.7);

        ++clusterId;
    }
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    bool renderPoints = true;
    bool renderClusters = true;
    bool renderBoxes = true;

    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    double groundSlope = 0;
    Lidar *lidar = new Lidar(cars, groundSlope);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = lidar->scan();
    //renderRays(viewer, lidar->position, pointCloud);  // renders the laser rays from the lidar scanner
    //renderPointCloud(viewer, pointCloud, "pointCloud"); // renders the points in the point cloud viewer in white color

    // TODO:: Create point processor
    // // on the stack
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    // // as pointer on the heap
    // ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(pointCloud, 100, 0.2); // can use .SgementPlane because it's on the stack
    // // if on heap as pointer
    //std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(pointCloud, 100, 0.2);

    if (renderPoints)
    {
        renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));   // obstacle cloud
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0)); // road cloud
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.5, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        if (renderClusters)
        {
            std::cout << "cluster size ";
            pointProcessor.numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
        }

        if (renderBoxes)
        {
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer, box, clusterId, colors[clusterId], 0.7);
        }

        ++clusterId;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv)
{
    bool showSimpleHighway = false;
    bool showCityBlockStatic = false;
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

    if (showSimpleHighway)
        simpleHighway(viewer);
    else
    {
        if (showCityBlockStatic)
        {
            cityBlockStatic(viewer);
            while (!viewer->wasStopped())
            {
                viewer->spinOnce();
            }
        }
        else
        {
            ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
            std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
            auto streamIterator = stream.begin();
            pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

            while (!viewer->wasStopped())
            {
                // Clear viewer
                viewer->removeAllPointClouds();
                viewer->removeAllShapes();

                // Load pcd and run obstacle detection process
                inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
                cityBlock(viewer, pointProcessorI, inputCloudI);

                streamIterator++;
                if (streamIterator == stream.end())
                    streamIterator = stream.begin();

                viewer->spinOnce();
            }
        }
    }
}
