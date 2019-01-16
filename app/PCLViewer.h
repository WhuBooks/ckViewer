//
// Created by books on 2018/5/7.
//

#ifndef CKVIEWER_PCLVIEWER_H
#define CKVIEWER_PCLVIEWER_H

#include <cstdlib>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <numeric>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <LcmType/LcmHandler.h>
#include <LcmType/Draw_t.hpp>

#include <tinyxml2.h>
#include <CloudHandler.h>

pcl::visualization::Camera left_button_camera;
typedef pcl::PlanarPolygon<pcl::PointXYZ>::Ptr PolygonXYZPtr;
typedef LcmHandler<ckLcmType::Draw_t>::Ptr DrawHandlerPtr;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,void* viewer_void)
{
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
    if (event.getKeySym() == "r" && event.keyDown())
    {
    }
}

void mouseEventOccurred(const pcl::visualization::MouseEvent &event,void* viewer_void)
{
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);

    if (event.getButton() == pcl::visualization::MouseEvent::LeftButton)
    {
        if (event.getType() == pcl::visualization::MouseEvent::MouseButtonPress)
        {
            viewer->getCameraParameters(left_button_camera);
        }
        else if (event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
        {
            viewer->setCameraParameters(left_button_camera);
        }
    }
    else if(event.getButton()== pcl::visualization::MouseEvent::RightButton)
    {

    }
    else
    {

    }

}

//PolygonXYZPtr BuildCar(double width,double length,double cen_x,double cen_y)
//{
//    Viewer::CloudXYZPtr cloud_car(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointXYZ car1, car2, car3, car4;
//    car1.x = -width / 2.0f + cen_x;
//    car1.y = -length / 2.0f + cen_y;
//    car1.z = 0.0f;
//
//    car2.x = width / 2.0f + cen_x;
//    car2.y = -length / 2.0f + cen_y;
//    car2.z = 0.0f;
//
//    car3.x = width / 2.0f + cen_x;
//    car3.y = length / 2.0f + cen_y;
//    car3.z = 0.0f;
//
//    car4.x = -width / 2.0f + cen_x;
//    car4.y = length / 2.0f + cen_y;
//    car4.z = 0.0f;
//
//    cloud_car->points.push_back(car1);
//    cloud_car->points.push_back(car2);
//    cloud_car->points.push_back(car3);
//    cloud_car->points.push_back(car4);
//    cloud_car->points.push_back(car1);
//    cloud_car->points.push_back(car3);
//    cloud_car->points.push_back(car2);
//    cloud_car->points.push_back(car4);
//
//    PolygonXYZPtr polygon(new pcl::PlanarPolygon<pcl::PointXYZ>);
//    polygon->setContour(*cloud_car);
//
//    return polygon;
//}

Viewer::CloudXYZPtr BuildCar(double width,double length,double cen_x,double cen_y)
{
    Viewer::CloudXYZPtr cloud_car(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ car1, car2, car3, car4;
    car1.x = -width / 2.0f - cen_x;
    car1.y = -length / 2.0f - cen_y;
    car1.z = 0.0f;

    car2.x = width / 2.0f - cen_x;
    car2.y = -length / 2.0f - cen_y;
    car2.z = 0.0f;

    car3.x = width / 2.0f - cen_x;
    car3.y = length / 2.0f - cen_y;
    car3.z = 0.0f;

    car4.x = -width / 2.0f - cen_x;
    car4.y = length / 2.0f - cen_y;
    car4.z = 0.0f;

    pcl::PointXYZ car_cen(0.0f,0.0f,0.0f);

    cloud_car->push_back(car1);
    cloud_car->push_back(car2);
    cloud_car->push_back(car2);
    cloud_car->push_back(car3);
    cloud_car->push_back(car3);
    cloud_car->push_back(car4);
    cloud_car->push_back(car4);
    cloud_car->push_back(car1);

    cloud_car->push_back(car1);
    cloud_car->push_back(car_cen);
    cloud_car->push_back(car2);
    cloud_car->push_back(car_cen);
    cloud_car->push_back(car3);
    cloud_car->push_back(car_cen);
    cloud_car->push_back(car4);
    cloud_car->push_back(car_cen);

    return cloud_car;
}

Viewer::CloudXYZPtr BuildGrid(int half_row,int half_col,double row_res,double col_res)
{
    Viewer::CloudXYZPtr cloud_grid(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=-half_row;i<=half_row;i++)
    {
        double x = half_col * col_res;
        double y = i * row_res;
        cloud_grid->push_back(pcl::PointXYZ(-x, y, 0.0f));
        cloud_grid->push_back(pcl::PointXYZ(x, y, 0.0f));
    }
    for(int i=-half_col;i<=half_col;i++)
    {
        double x=i*col_res;
        double y=half_row*row_res;
        cloud_grid->push_back(pcl::PointXYZ(x, y, 0.0f));
        cloud_grid->push_back(pcl::PointXYZ(x, -y, 0.0f));
    }
    return cloud_grid;
}





#endif //CKVIEWER_PCLVIEWER_H
