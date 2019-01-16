//
// Created by books on 2018/5/6.
//
#include "PCLViewer.h"

int main()
{
    std::string config_file="../config/ViewerConfig.xml";

    /// grid parameters
    int grid_half_rows = 25;
    int grid_half_cols = 25;
    double grid_row_resolution = 1;
    double grid_col_resolution = 1;

    /// car parameters
    double car_width = 2.6;
    double car_length = 4.5;
    double car_center_x = 0.0;
    double car_center_y = 0.0;

    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError error=doc.LoadFile(config_file.c_str());
    if (error != tinyxml2::XMLError::XML_SUCCESS)
        return -1;
    tinyxml2::XMLElement *root = doc.RootElement();
    std::cout << root->Name() << std::endl;

    if (root->NoChildren())
        return -1;

    /// read grid parameters
    tinyxml2::XMLElement * tmp = root->FirstChildElement("Geometry");
    tinyxml2::XMLElement * gridrow = tmp->FirstChildElement("Grid_Half_Row");
    gridrow->QueryIntText(&grid_half_rows);
    std::cout << "Grid Rows = " << 2*grid_half_rows << "\t";
    tinyxml2::XMLElement * gridcol = tmp->FirstChildElement("Grid_Half_Col");
    gridcol->QueryIntText(&grid_half_cols);
    std::cout << "Grid Cols = " << 2 * grid_half_cols << std::endl;
    tinyxml2::XMLElement * gridrowres = tmp->FirstChildElement("Grid_Res_Row");
    gridrowres->QueryDoubleText(&grid_row_resolution);
    std::cout << "Grid Row Resolution = " << grid_row_resolution << "\t";
    tinyxml2::XMLElement * gridcolres = tmp->FirstChildElement("Grid_Res_Col");
    gridcolres->QueryDoubleText(&grid_col_resolution);
    std::cout << "Grid Col Resolution = " << grid_col_resolution << std::endl;

    /// read car parameters
    tinyxml2::XMLElement * carwidth = tmp->FirstChildElement("Car_Width");
    carwidth->QueryDoubleText(&car_width);
    std::cout << "Car Width = " << car_width << "\t";
    tinyxml2::XMLElement * carlength = tmp->FirstChildElement("Car_Length");
    carlength->QueryDoubleText(&car_length);
    std::cout << "Car Length = " << car_length << "\t";
    tinyxml2::XMLElement * carx = tmp->FirstChildElement("Car_Center_X");
    carx->QueryDoubleText(&car_center_x);
    std::cout << "Car Center X = " << car_center_x << "\t";
    tinyxml2::XMLElement * cary = tmp->FirstChildElement("Car_Center_Y");
    cary->QueryDoubleText(&car_center_y);
    std::cout << "Car Center Y = " << car_center_y << std::endl;

    /// read modules
    Viewer::CloudHandlerPtr cloud_handler(new Viewer::CloudHandler());
    tinyxml2::XMLElement *lcm = root->FirstChildElement("Lcm");
    tinyxml2::XMLElement *module = lcm->FirstChildElement("Module");
    while (true)
    {
        if (module && !module->NoChildren())
        {
            tinyxml2::XMLElement *net_ele = module->FirstChildElement("Net");
            tinyxml2::XMLElement *chan_ele = module->FirstChildElement("Channel");
            tinyxml2::XMLElement *r = module->FirstChildElement("color_r");
            tinyxml2::XMLElement *g = module->FirstChildElement("color_g");
            tinyxml2::XMLElement *b = module->FirstChildElement("color_b");

            const char *namestr = module->Attribute("Name");
            const char *netstr = net_ele->GetText();
            const char *chanstr = chan_ele->GetText();
            unsigned int color_r = 0, color_g = 0, color_b = 0;
            r->QueryUnsignedText(&color_r);
            g->QueryUnsignedText(&color_g);
            b->QueryUnsignedText(&color_b);

            std::string module_name,module_net,module_channel;
            if (namestr)
                module_name = std::string(namestr);
            if (netstr)
                module_net = std::string(netstr);
            if (chanstr)
                module_channel = std::string(chanstr);

            cloud_handler->Add(std::string(module_name),color_r,color_g,color_b);

            std::thread th([&](){
                std::string name=module_name;
                std::string net=module_net;
                std::string chann=module_channel;
                DrawHandlerPtr handler(new LcmHandler<ckLcmType::Draw_t>);
                handler->SetNet(net);
                handler->SetChannel(chann);
                int res=handler->InitialListen();
                if(res==0)
                    return;
                while(true)
                {
                    ckLcmType::Draw_t draw_msg=handler->GetData();
                    Viewer::PtVec vec;
                    for(int i=0;i<draw_msg.ptnum;i++)
                        vec.emplace_back(Eigen::Vector2d(draw_msg.x[i],draw_msg.y[i]));
                    cloud_handler->Update(name,vec);
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            });
            th.detach();

            std::cout << "module = " << module_name << ";\t";
            std::cout << "net = " << module_net << ";\t";
            std::cout << "channel = " << module_channel << ";\t";
            std::cout << "(r,g,b) = (" << color_r << "," << color_g << "," << color_b << ")" << std::endl;
        }
        else
            break;
        module = module->NextSiblingElement("Module");
    }
    std::cout<<"Initialize All The Module Done."<<std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    /// set pcl viewer parameters
    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("viewer"));
    visualizer->setBackgroundColor(0,0,0);
    visualizer->initCameraParameters();
    visualizer->setCameraPosition(0.0, 0.0, 30.0, 0.0, 0.0, 0.0, 0);
    visualizer->registerKeyboardCallback(keyboardEventOccurred, (void*)visualizer.get());
    visualizer->registerMouseCallback(mouseEventOccurred, (void*)visualizer.get());

    /// add car model
    Viewer::CloudXYZPtr cloud_car=BuildCar(car_width,car_length,car_center_x,car_center_y);
    for(int i=0;i<cloud_car->size();i=i+2)
    {
        visualizer->addLine(cloud_car->points[i], cloud_car->points[i + 1], 1, 1, 0, "car"+std::to_string(i));
    }

    /// add grid line
    Viewer::CloudXYZPtr cloud_grid=BuildGrid(grid_half_rows,grid_half_cols,grid_row_resolution,grid_col_resolution);
    for(int i=0;i<cloud_grid->size();i=i+2)
    {
        visualizer->addLine(cloud_grid->points[i], cloud_grid->points[i + 1], 0.5, 0.5, 0.5, "grid"+std::to_string(i));
    }

    visualizer->setFullScreen(false);

    Viewer::CloudXYZRGBPtr cloud_all(new pcl::PointCloud<pcl::PointXYZRGB>);
    visualizer->addPointCloud(cloud_all,"cloud");
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,"cloud");

    while(!visualizer->wasStopped())
    {
         cloud_all=cloud_handler->GetData();
         visualizer->updatePointCloud(cloud_all,"cloud");

         visualizer->spinOnce(50);
         std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return 1;
}