//
// Created by books on 2018/5/6.
//

#include "CloudHandler.h"

namespace Viewer
{

    void CloudHandler::Add(std::string name, int r, int g, int b)
    {
        m_cloud_vec.emplace_back(Cloud(name,r,g,b));
    }

    void CloudHandler::Update(std::string name, CloudXYZPtr cloud)
    {
        for (Cloud &tmp : m_cloud_vec)
        {
            if (tmp.name == name)
            {
                CloudXYZRGBPtr cloud_conv=tmp.Convert(cloud);

                std::unique_lock<std::mutex> lock(m_update_mutex);
                tmp.data.swap(cloud_conv);
                lock.unlock();

                return;
            }
        }
    }

    void CloudHandler::Update(std::string name,const PtVec &vec)
    {
        for (Cloud &tmp : m_cloud_vec)
        {
            if (tmp.name == name)
            {
                CloudXYZRGBPtr cloud_conv=tmp.Convert(vec);

                std::unique_lock<std::mutex> lock(m_update_mutex);
//                std::cout<<"Update Cloud ~ "<<name<<std::endl;
                tmp.data.swap(cloud_conv);
                lock.unlock();

                return;
            }
        }
    }

    CloudXYZRGBPtr CloudHandler::GetData()
    {
        CloudXYZRGBPtr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        std::unique_lock<std::mutex> lock(m_update_mutex);
        for(const Cloud &tmp : m_cloud_vec)
        {
            for(const pcl::PointXYZRGB &pt : tmp.data->points)
            {
                cloud->push_back(pt);
            }
        }
        lock.unlock();

        return cloud;
    }
}