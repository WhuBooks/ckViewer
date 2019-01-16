#define _CRTDBG_MAP_ALLOC

//#include <stdlib.h>
//#include <crtdbg.h>

#include <cstdlib>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <numeric>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <LcmType/LcmHandler.h>
#include <LcmType/Draw_t.hpp>

#include <tinyxml2.h>

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr;
typedef LcmHandler<ckLcmType::Draw_t> DrawHandler;

static int grid_half_rows = 25;
static int grid_half_cols = 25;
static double grid_row_resolution = 1;
static double grid_col_resolution = 1;

static double car_width = 2.6;
static double car_length = 4.5;
static double car_center_x = 0.0;
static double car_center_y = 0.0;

class Configure
{
public:
	std::string name;
	std::string net;
	std::string channel;
	int8_t r;
	int8_t g;
	int8_t b;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr;
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb;
	std::thread *pthread;
	LcmHandler<ckLcmType::Draw_t> * handler;
	void Start()
	{
		handler = new LcmHandler<ckLcmType::Draw_t>();
		handler->SetNet(net);
		handler->SetChannel(channel);
		handler->InitialListen();
		pthread =new std::thread(&Configure::UpdatePointCloud,this);
	}
	void Stop();

	void UpdatePointCloud()
	{
		ckLcmType::Draw_t message;
		while (1)
		{
			handler->GetData(message);
			std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> > buffer;
			//if (flag)
			{
				//point_cloud_ptr->clear();
				for (int i = 0; i < message.ptnum; i++)
				{
					pcl::PointXYZRGB pt;
					pt.x = message.x[i];
					pt.y = message.y[i];
					pt.z = 0;
					pt.r = r;
					pt.g = g;
					pt.b = b;
					buffer.push_back(pt);
					//point_cloud_ptr->points.push_back(pt);
				}
				point_cloud_ptr->clear();
				point_cloud_ptr->points.swap(buffer);
				std::cout << name << " update" << std::endl;
			}
			//else
				//std::cout << name << " stable" << std::endl;
		}
	}
};

pcl::visualization::Camera left_button_camera;
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
	//if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
	//	event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
	//{
	//	std::cout << "Left mouse button released at position (" << event.getX() << ", " << event.getY() << ")" << std::endl;
	//}

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

}

void PPUpdatePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptcloud)
{
	pcl::PointXYZRGB pt;
	pt.x = rand() % 10;
	pt.y = rand() % 10;
	pt.z = 0;
	pt.r = rand() % 255;
	pt.g = rand() % 255;
	pt.b = rand() % 255;
	ptcloud->points.push_back(pt);
}

void GetXYZRGBCloudFromLcm(cloudptr ptcloud,const ckLcmType::Draw_t &mes,
	int8_t r=100,int8_t g=100,int8_t b=100)
{
	for (int i = 0; i < mes.ptnum; i++)
	{
		pcl::PointXYZRGB pt;
		pt.x = mes.x[i];
		pt.y = mes.y[i];
		pt.z = 0;
		pt.r = r;
		pt.g = g;
		pt.b = b;
		ptcloud->points.push_back(pt);
	}
}

void WriteConfig()
{
	const char* declaration = "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>";
	tinyxml2::XMLDocument doc;
	doc.Parse(declaration);

	tinyxml2::XMLElement* root = doc.NewElement("ViewerConfig");
	doc.InsertEndChild(root);

	tinyxml2::XMLElement * geo = doc.NewElement("Geometry");
	root->InsertEndChild(geo);
	tinyxml2::XMLElement * tmp1 = doc.NewElement("Grid_Half_Row");
	tmp1->SetText(25);
	geo->InsertEndChild(tmp1);
	tinyxml2::XMLElement * tmp2 = doc.NewElement("Grid_Half_Col");
	tmp2->SetText(25);
	geo->InsertEndChild(tmp2);
	tinyxml2::XMLElement * tmp3 = doc.NewElement("Grid_Res_Row");
	tmp3->SetText(0.5);
	geo->InsertEndChild(tmp3);
	tinyxml2::XMLElement * tmp4 = doc.NewElement("Grid_Res_Col");
	tmp4->SetText(0.5);
	geo->InsertEndChild(tmp4);
	tinyxml2::XMLElement * tmp5 = doc.NewElement("Car_Width");
	tmp5->SetText(2.6);
	geo->InsertEndChild(tmp5);
	tinyxml2::XMLElement * tmp6 = doc.NewElement("Car_Length");
	tmp6->SetText(4.5);
	geo->InsertEndChild(tmp6);
	tinyxml2::XMLElement * tmp7 = doc.NewElement("Car_Center_X");
	tmp7->SetText(0.0);
	geo->InsertEndChild(tmp7);
	tinyxml2::XMLElement * tmp8 = doc.NewElement("Car_Center_Y");
	tmp8->SetText(0.0);
	geo->InsertEndChild(tmp8);

	tinyxml2::XMLElement * ele1 = doc.NewElement("Lcm");

	tinyxml2::XMLElement *module = doc.NewElement("Module");
	//module->SetText("Example");
	module->SetAttribute("Name", "Example");
	ele1->InsertEndChild(module);

	tinyxml2::XMLElement * net = doc.NewElement("Net");
	net->SetText("");
	module->InsertEndChild(net);
	tinyxml2::XMLElement * chan = doc.NewElement("Channel");
	chan->SetText("CKDRAW");
	module->InsertEndChild(chan);
	tinyxml2::XMLElement *r = doc.NewElement("color_r");
	r->SetText(0);
	module->InsertEndChild(r);
	tinyxml2::XMLElement *g = doc.NewElement("color_g");
	g->SetText(0);
	module->InsertEndChild(g);
	tinyxml2::XMLElement *b = doc.NewElement("color_b");
	b->SetText(0);
	module->InsertEndChild(b);

	root->InsertEndChild(ele1);
	doc.SaveFile("ViewerConfig.xml");
}

bool ReadConfig(std::vector<Configure> &vcon)
{
	tinyxml2::XMLDocument doc;
	tinyxml2::XMLError error=doc.LoadFile("ViewerConfig.xml");
	if (error != tinyxml2::XMLError::XML_SUCCESS)
		return false;
	tinyxml2::XMLElement *root = doc.RootElement();
	std::cout << root->Name() << std::endl;

	if (root->NoChildren())
	{
		return false;
	}

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
	
	tinyxml2::XMLElement *lcm = root->FirstChildElement("Lcm");
	tinyxml2::XMLElement *module = lcm->FirstChildElement("Module");
	while (true)
	{
		if (module&&!module->NoChildren())
		{
			Configure con;

			con.point_cloud_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
			con.rgb = pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(con.point_cloud_ptr);

			tinyxml2::XMLElement *net = module->FirstChildElement("Net");
			tinyxml2::XMLElement *chan = module->FirstChildElement("Channel");
			tinyxml2::XMLElement *r = module->FirstChildElement("color_r");
			tinyxml2::XMLElement *g = module->FirstChildElement("color_g");
			tinyxml2::XMLElement *b = module->FirstChildElement("color_b");
			
			const char * namestr = module->Attribute("Name");
			const char * netstr = net->GetText();
			const char * chanstr = chan->GetText();
			unsigned int color_r = 0, color_g = 0, color_b = 0;
			r->QueryUnsignedText(&color_r);
			g->QueryUnsignedText(&color_g);
			b->QueryUnsignedText(&color_b);
			
			if (namestr)
				con.name = namestr;
			if (netstr)
				con.net = netstr;
			if (chanstr)
				con.channel = std::string(chanstr);
			con.r = static_cast<int8_t>(color_r);
			con.g = static_cast<int8_t>(color_g);
			con.b = static_cast<int8_t>(color_b);
			//con.Start();
			vcon.push_back(con);
			std::cout << "module = " << con.name << ";\t";
			std::cout << "net = " << con.net << ";\t";
			std::cout << "channel = " << con.channel << ";\t";
			std::cout << "(r,g,b) = (" << color_r << "," << color_g << "," << color_b << ")" << std::endl;
		}
		else
			return true;
		module = module->NextSiblingElement("Module");
	}

}

int main(int argc, char** argv)
{
	//WriteConfig();
	//return 0;
	std::vector<Configure> vcon;
	std::vector<Configure>::iterator iter;
	ReadConfig(vcon);
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	for (iter = vcon.begin(); iter != vcon.end(); iter++)
	{
		viewer->addPointCloud<pcl::PointXYZRGB>(iter->point_cloud_ptr, iter->rgb, iter->name);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, iter->name);
		iter->Start();
	}

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
	//viewer->addPointCloud<pcl::PointXYZRGB>(point_cloud_ptr, rgb, "sample cloud");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	

	//viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0.0, 0.0, 30.0, 0.0, 0.0, 0.0, 0);
	//viewer->setCameraClipDistances(0.0, 50.0);
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)viewer.get());
	viewer->registerMouseCallback(mouseEventOccurred, (void*)viewer.get());

	//add car
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr car_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointXYZRGB car1, car2, car3, car4;
	car1.x = -car_width / 2 + car_center_x;
	car1.y = -car_length / 2 + car_center_y;
	car1.z = 0; car1.r = 255; car1.g = 255; car1.b = 0;

	car2.x = car_width / 2 + car_center_x;
	car2.y = -car_length / 2 + car_center_y;
	car2.z = 0; car2.r = 255; car2.g = 255; car2.b = 0;

	car3.x = car_width / 2 + car_center_x;
	car3.y = car_length / 2 + car_center_y;
	car3.z = 0; car3.r = 255; car3.g = 255; car3.b = 0;

	car4.x = -car_width / 2 + car_center_x;
	car4.y = car_length / 2 + car_center_y;
	car4.z = 0; car4.r = 255; car4.g = 255; car4.b = 0;

	car_ptr->points.push_back(car1);
	car_ptr->points.push_back(car2);
	car_ptr->points.push_back(car3);
	car_ptr->points.push_back(car4);
	car_ptr->points.push_back(car1);
	car_ptr->points.push_back(car3);
	car_ptr->points.push_back(car2);
	car_ptr->points.push_back(car4);

	pcl::PlanarPolygon<pcl::PointXYZRGB> car_polygon;
	car_polygon.setContour(*car_ptr);
	viewer->addPolygon(car_polygon,255,255,0, "car");

	//add grid
	for (int i = -grid_half_rows; i <= grid_half_rows; i++)
	{
		double y = i*grid_row_resolution;
		double x1 = -grid_half_cols*grid_col_resolution;
		double x2 =  grid_half_cols*grid_col_resolution;
		pcl::PointXYZRGB p1, p2;
		p1.x = x1; p1.y = y; p1.z = 0; p1.r = 255; p1.g = 255; p1.b = 255;
		p2.x = x2; p2.y = y; p2.z = 0; p2.r = 255; p2.g = 255; p2.b = 255;
		std::string lineid = std::to_string(i) + "row";
		viewer->addLine(p1, p2,lineid);
	}
	for (int i = -grid_half_cols; i <= grid_half_cols; i++)
	{
		double x = i*grid_col_resolution;
		double y1 = -grid_half_rows*grid_row_resolution;
		double y2 = grid_half_rows*grid_row_resolution;
		pcl::PointXYZRGB p1, p2;
		p1.x = x; p1.y = y1; p1.z = 0; p1.r = 255; p1.g = 255; p1.b = 255;
		p2.x = x; p2.y = y2; p2.z = 0; p2.r = 255; p2.g = 255; p2.b = 255;
		std::string lineid = std::to_string(i) + "col";
		viewer->addLine(p1, p2, lineid);
	}

	//std::vector<std::string> vchan = { "CKLOC", "CKMAP", "CKLIDAR", "CKCAMERA" };
	//std::vector<DrawHandler *> vphandler;
	//std::vector<DrawHandler *>::iterator iter;
	//for (const Configure &con : vcon)
	//{
	//	DrawHandler * phandler = new DrawHandler();
	//	phandler->SetNet(con.net);
	//	phandler->SetChannel(con.channel);
	//	phandler->InitialListen();
	//	std::cout << "Start listen message from : " << con.name << std::endl;
	//	vphandler.push_back(phandler);
	//}

	while (!viewer->wasStopped())
	{
		//point_cloud_ptr->clear();
		int i = 0;
		for (iter = vcon.begin(); iter != vcon.end(); iter++)
		{
			//iter->UpdatePointCloud();
			viewer->updatePointCloud<pcl::PointXYZRGB>(iter->point_cloud_ptr, iter->rgb, iter->name);
		}

		viewer->spinOnce(50);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100));
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	_CrtDumpMemoryLeaks();
}