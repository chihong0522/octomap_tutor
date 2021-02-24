/*************************************************************************
	> File Name: src/pcd2octomap.cpp
	> Author: Gao Xiang
	> Mail: gaoxiang12@mails.tsinghua.edu.cn
	> Created Time: 2015年12月12日 星期六 15时51分45秒
 ************************************************************************/

#include <iostream>
#include <utility>

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//octomap 
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include "../include/pugixml-1.11/src/pugixml.hpp"
using namespace std;

const string scenenn_xml_path = "/home/chihung/SceneNN-dataset/scenenn/025/025.xml";
const string nyu_xml_path = "/home/chihung/SceneNN-dataset/scenenn/nyu_color.xml";

map <string, map<string, string>> parse_scene_nn_xml() {
    map <string, map<string, string>> XmlContent;

    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(scenenn_xml_path.c_str());
    if (!result)
        return XmlContent;

    pugi::xml_node label_node = doc.child("annotation");

    for (pugi::xml_node label = label_node.first_child(); label; label = label.next_sibling())
    {
        map <string, string> labelContent;
        for (pugi::xml_attribute attr = label.first_attribute(); attr; attr = attr.next_attribute())
        {
            labelContent[static_cast<string>(attr.name())] = static_cast<string>(attr.value());
        }
        XmlContent[labelContent["color"]] = labelContent;
    }
    return XmlContent;
}

map <string, map<string, string>> parse_nyu_xml() {
    map <string, map<string, string>> XmlContent;

    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(nyu_xml_path.c_str());
    if (!result)
        return XmlContent;

    pugi::xml_node label_node = doc.child("colormap");

    for (pugi::xml_node label = label_node.first_child(); label; label = label.next_sibling())
    {
        map <string, string> labelContent;
        for (pugi::xml_attribute attr = label.first_attribute(); attr; attr = attr.next_attribute())
        {
            labelContent[static_cast<string>(attr.name())] = static_cast<string>(attr.value());
        }
        XmlContent[labelContent["color"]] = labelContent;
    }

//    cout << XmlContent["215 179 119"]["color"] << endl;
    return XmlContent;
}


int main( int argc, char** argv )
{
    if (argc != 3)
    {
        cout<<"Usage: pcd2colorOctomap <input_file> <output_file>"<<endl;
        return -1;
    }

    map <string, map<string, string>> scene_nn_xml = parse_scene_nn_xml();
    map <string, map<string, string>> nyu_color_xml = parse_nyu_xml();

    string input_file = argv[1], output_file = argv[2];
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    pcl::io::loadPCDFile<pcl::PointXYZRGBA> ( input_file, cloud );

    cout<<"point cloud loaded, piont size = "<<cloud.points.size()<<endl;

    //声明octomap变量
    cout<<"copy data into octomap..."<<endl;
    // 创建带颜色的八叉树对象，参数为分辨率，这里设成了0.05
    octomap::ColorOcTree tree( 0.02 );

    for (auto p:cloud.points)
    {
        // 将点云里的点插入到octomap中
        tree.updateNode( octomap::point3d(p.x, p.y, p.z), true );
    }

    // 设置颜色
    for (auto p:cloud.points)
    {
        vector<long> voxelColorRGB1 = {static_cast<long>(p.r),static_cast<long>(p.g),static_cast<long>(p.b)};
        tree.integrateNodeColor( p.x, p.y, p.z, p.r, p.g, p.b );
    }

    // 更新octomap
    tree.updateInnerOccupancy();
    // 存储octomap, 注意要存成.ot文件而非.bt文件
    tree.write( output_file );
    cout<<"done."<<endl;

    return 0;
}



