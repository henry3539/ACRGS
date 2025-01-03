if (expanded_inliers1->indices.size() > 0)
{
    pcl::ExtractIndices<pcl::PointXYZ> extract0;
    extract0.setInputCloud(cloudt1);
    extract0.setIndices(expanded_inliers1);
    extract0.setNegative(false);
    extract0.filter(*cloudt2);
    expanded_inliers1->indices.clear();

    pcl::getMinMax3D(*cloudt2, mint1, maxt1);
    centert1.x = (mint1.x + maxt1.x) / 2;
    centert1.y = (mint1.y + maxt1.y) / 2;
    centert1.z = (mint1.z + maxt1.z) / 2;


    Eigen::Vector4f vec1(centert0.x - centert1.x, centert0.y - centert1.y, centert0.z - centert1.z, 0);//方向向量

    double d2 = vec1.dot(vec1);
    double d = sqrt(d2);


    pcl::PointIndices::Ptr inliers1(new pcl::PointIndices());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudye(new pcl::PointCloud<pcl::PointXYZ>), cloudye1(new pcl::PointCloud<pcl::PointXYZ>), cloudye2(new pcl::PointCloud<pcl::PointXYZ>);

    for (int pan = 0; pan < 15; pan++)
    {
        pcl::PointXYZ cs;
        pcl::PointXYZ cs1;
        pcl::PointIndices::Ptr inliers2(new pcl::PointIndices());
        centert1 = centert0;
        centert0.x = centert0.x + vec1.x() / 2;
        centert0.y = centert0.y + vec1.y() / 2;
        centert0.z = centert0.z + vec1.z() / 2;

        cs.x = centert0.x;
        cs.y = centert0.y;
        cs.z = centert0.z - (2 * juxingk.z);

        for (size_t i = 0; i < cloud_cylinder->points.size(); i++)
            if ((cloud_cylinder->points[i].x < centert0.x + juxingk.x && cloud_cylinder->points[i].x >= centert0.x - juxingk.x) &&
                (cloud_cylinder->points[i].y < centert0.y + juxingk.y && cloud_cylinder->points[i].y >= centert0.y - juxingk.y) &&
                (cloud_cylinder->points[i].z < centert0.z + juxingk.z && cloud_cylinder->points[i].z >= centert0.z - juxingk.z)
                )
            {
                inliers1->indices.push_back(i);
            }
        cout << "number1:" << inliers1->indices.size() << endl;

        for (size_t i = 0; i < cloud_cylinder->points.size(); i++)
            if ((cloud_cylinder->points[i].x < cs.x + juxingk.x && cloud_cylinder->points[i].x >= cs.x - juxingk.x) &&
                (cloud_cylinder->points[i].y < cs.y + juxingk.y && cloud_cylinder->points[i].y >= cs.y - juxingk.y) &&
                (cloud_cylinder->points[i].z < cs.z + juxingk.z && cloud_cylinder->points[i].z >= cs.z - juxingk.z)
                )
            {
                inliers2->indices.push_back(i);
            }
        cout << "number2:" << inliers2->indices.size() << endl;
        if (inliers1->indices.size() > 1)
            if (inliers2->indices.size() > (inliers1->indices.size() * 0.5))
                pan = 15;
        inliers2->indices.clear();



        if (inliers1->indices.size() > 0 && pan < 15)
        {
            pcl::ExtractIndices<pcl::PointXYZ> extract0;
            extract0.setInputCloud(cloud_cylinder);
            extract0.setIndices(inliers1);
            extract0.setNegative(false);
            extract0.filter(*cloudye1);
            extract0.setNegative(true);
            extract0.filter(*cloud_cylinder);
            *cloud_cluster = *cloud_cluster + *cloudye1;
            inliers1->indices.clear();
            pcl::PointXYZ mintye1, maxtye1;
            pcl::getMinMax3D(*cloudye1, mintye1, maxtye1);
            cenye1.x = (mintye1.x + maxtye1.x) / 2;
            cenye1.y = (mintye1.y + maxtye1.y) / 2;
            cenye1.z = (mintye1.z + maxtye1.z) / 2;
            Eigen::Vector4f vec2(cenye1.x - centert1.x, cenye1.y - centert1.y, cenye1.z - centert1.z, 0);//方向向量

            double yd2 = vec2.dot(vec2);
            double yd = sqrt(yd2);

            vec1.x() = vec2.x() * d / yd;
            vec1.y() = vec2.y() * d / yd;
            vec1.z() = vec2.z() * d / yd;
        }
    }
}

