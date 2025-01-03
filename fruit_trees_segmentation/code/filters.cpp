void visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> single_color(cloud, "z");
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, "x");
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, "y");
    viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "example");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "example");


}



    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>("../4hao.pcd", *cloud);
    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(10); 
    sor.setStddevMulThresh(0.6); 
    sor.setNegative(false); 
    sor.filter(*cloud_filtered); 
    std::cerr << "Cloud  filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;
    pcl::io::savePCDFileASCII("../filtered4hao.pcd", *cloud_filtered);
    pcl::visualization::PCLVisualizer viewer("viewer");
    int v1;
    int v2;
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer.addText("v1", 10, 10, 20, 1, 0, 0, "viewport_v1", v1);
    viewer.addText("v2", 10, 10, 20, 0, 1, 0, "viewport_v2", v2);
    viewer.addCoordinateSystem(0.5);  
    viewer.setBackgroundColor(0,0,0);   
    viewer.addPointCloud(cloud, "cloud", v1);
    viewer.addPointCloud(cloud_filtered, "cloud_filtered", v2);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud_filtered, "z");
    viewer.addPointCloud<pcl::PointXYZ>(cloud_filtered, fildColor, "v2");
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

