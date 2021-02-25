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

#include<cstring>
#include<vector>

//octomap 
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include "../include/pugixml-1.11/src/pugixml.hpp"
using namespace std;

double resolution = 0.01;

map <string, map<string, string>> parse_scene_nn_xml(string scenenn_xml_path) {
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

map <string, map<string, string>> parse_nyu_xml(string nyu_xml_path) {
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
        XmlContent[labelContent["text"]] = labelContent;
    }

//    cout << XmlContent["215 179 119"]["color"] << endl;
    return XmlContent;
}

vector<string> splitString(const string& str,const string& pattern)
{
    vector<string> result;
    //string::size_type型別，left：左邊界位置  right：右邊界位置
    string::size_type left, right;

    right = str.find(pattern);
    left = 0;

    while(right != string::npos)
    {
        //以免字串首部就是分割依據，壓入長度為0的字串
        if(right-left)
        {
            //壓入起始位置為left，長度為（right-left）的字串
            result.push_back(str.substr(left, right-left));
        }
        left = right + pattern.size();   //右邊界右移分割依據的長度，作為新的左邊界
        right = str.find(pattern, left);   //從left這個位置開始find
    }

    //退出迴圈時，左邊界不是最後一個元素
    if(left != str.length())
    {
        result.push_back(str.substr(left));
    }

    return result;
}


int main( int argc, char** argv )
{
    if (argc != 4)
    {
        cout<<"Usage: pcd2colorOctomap <input_file> <output_file> <scene_id>"<<endl;
        return -1;
    }
    string input_file = argv[1], output_file = argv[2], scene_id = argv[3];

    string scenenn_xml_path = "/home/chihung/SceneNN-dataset/scenenn/";
    string nyu_xml_path = "/home/chihung/semantic_map_ws/src/semantic_cloud/include/multitask_refinenet/nyu_color.xml";
    scenenn_xml_path =scenenn_xml_path + scene_id +"/"+scene_id+".xml" ;

    map <string, map<string, string>> scene_nn_xml = parse_scene_nn_xml(scenenn_xml_path);
    map <string, map<string, string>> nyu_color_xml = parse_nyu_xml(nyu_xml_path);

    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    pcl::io::loadPCDFile<pcl::PointXYZRGBA> ( input_file, cloud );

    cout<<"point cloud loaded, piont size = "<<cloud.points.size()<<endl;

    //声明octomap变量
    cout<<"copy data into octomap..."<<endl;
    // 创建带颜色的八叉树对象，参数为分辨率，这里设成了0.05
    octomap::ColorOcTree tree( resolution );

    for (auto p:cloud.points)
    {
        // 将点云里的点插入到octomap中
        tree.updateNode( octomap::point3d(p.x, p.y, p.z), true );
    }

    // 设置颜色
    for (auto p:cloud.points)
    {
//        vector<long> voxelColorRGB1 = {static_cast<long>(p.r),static_cast<long>(p.g),static_cast<long>(p.b)};
        string colorIndex = to_string(p.r) + " " + to_string(p.g) + " " + to_string(p.b);
        map<string, string>scenenn_data = scene_nn_xml[colorIndex];

        string nyu_class_name;
        string scene_class_name = scenenn_data["nyu_class"];

        if (scene_class_name.length()==0 or scene_class_name=="prop" or scene_class_name=="structure")
            nyu_class_name = "otherprop";

        if (scene_class_name=="fridge")
            nyu_class_name="refridgerator";

        if (nyu_class_name.length()==0)
            nyu_class_name = scenenn_data["nyu_class"];

        vector<string> nyu_colors = splitString(nyu_color_xml[nyu_class_name]["sem_map_color"], " ");
//        if (nyu_class_name=="door") {
//            cout<< "";
//        }
        uint8_t b = atoi(nyu_colors[0].c_str());
        uint8_t g = atoi(nyu_colors[1].c_str());
        uint8_t r = atoi(nyu_colors[2].c_str());

        p.b = b;
        p.g = g;
        p.r = r;
        tree.integrateNodeColor( p.x, p.y, p.z, r, g, b );
    }

    // 更新octomap
    tree.updateInnerOccupancy();
    // 存储octomap, 注意要存成.ot文件而非.bt文件
    tree.write( output_file );
    cout<<"done."<<endl;

    return 0;
}




