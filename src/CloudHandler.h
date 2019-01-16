//
// Created by books on 2018/5/6.
//

#ifndef CKVIEWER_CLOUDHANDLER_H
#define CKVIEWER_CLOUDHANDLER_H

#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <numeric>
#include <mutex>

#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>

namespace Viewer
{
    typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudXYZPtr;
    typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudXYZRGBPtr;
    typedef std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>> PtVec;

    class CloudHandler
    {
        struct Cloud
        {
            std::string name;
            CloudXYZRGBPtr data;
            int r; int g; int b;

            Cloud(std::string _name,int _r,int _g,int _b)
                    :data(new pcl::PointCloud<pcl::PointXYZRGB>)
            {
                name=_name;
                r=_r;
                g=_g;
                b=_b;
            };

            CloudXYZRGBPtr Convert(CloudXYZPtr cloud)
            {
                CloudXYZRGBPtr update_data(new pcl::PointCloud<pcl::PointXYZRGB>);
                for(const pcl::PointXYZ &pt : cloud->points)
                {
                    pcl::PointXYZRGB pt_rgb(r,g,b);
                    pt_rgb.x=pt.x; pt_rgb.y=pt.y; pt_rgb.z=pt.z;
                    update_data->push_back(pt_rgb);
                }
                return update_data;
            };

            CloudXYZRGBPtr Convert(const PtVec &vec)
            {
                CloudXYZRGBPtr update_data(new pcl::PointCloud<pcl::PointXYZRGB>);
                for(const Eigen::Vector2d &pt : vec)
                {
                    pcl::PointXYZRGB pt_rgb(r,g,b);
                    pt_rgb.x=pt(0); pt_rgb.y=pt(1); pt_rgb.z=0.0f;
                    update_data->push_back(pt_rgb);
                }
                return update_data;
            };
        };

    public:
        CloudHandler()= default;

        ~CloudHandler()= default;

        void Add(std::string name,int r,int g, int b);

        void Update(std::string name,CloudXYZPtr cloud);

        void Update(std::string name,const PtVec &vec);

        CloudXYZRGBPtr GetData();

    private:
        std::vector<Cloud> m_cloud_vec;
        std::mutex m_update_mutex;

    };
    typedef std::shared_ptr<CloudHandler> CloudHandlerPtr;

}


#endif //CKVIEWER_CLOUDHANDLER_H
