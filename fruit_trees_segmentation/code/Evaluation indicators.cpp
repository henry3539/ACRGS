    pcl::PointCloud<pcl::PointXYZ>::Ptr predictedCloud1(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("E:/0.PCL/DATA/Apple_tree/all_tree/" + br1.str(), *predictedCloud1) == -1) {
        PCL_ERROR("Couldn't read file branch.pcd\n");
        return -1;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr groundTruth1(new pcl::PointCloud<pcl::PointXYZ>);
    std::stringstream  br5;
    br5 << "t" << ttt << "/True_value/branch.pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("E:/0.PCL/DATA/Apple_tree/all_tree/" + br5.str(), *groundTruth1) == -1) {
        PCL_ERROR("Couldn't read file branch.pcd\n");
        return -1;
    }
    double precisionb1, recallb1;
    int truePositive1 = 0;
    int falsePositive1 = 0;
    int falseNegative1 = 0;
    for (const auto& predictedPoint : *predictedCloud1) {
        bool isTruePositive = false;
        for (const auto& truthPoint : *groundTruth1) {
            if (predictedPoint.x == truthPoint.x &&
                predictedPoint.y == truthPoint.y &&
                predictedPoint.z == truthPoint.z)
            {
                truePositive1++;
                isTruePositive = true;
                break;
            }
        }
        if (!isTruePositive) {
            falsePositive1++;
        }
    }
    falseNegative1 = groundTruth1->size() - truePositive1;
    precisionb1 = static_cast<double>(truePositive1) / (truePositive1 + falsePositive1);
    recallb1 = static_cast<double>(truePositive1) / (truePositive1 + falseNegative1);
    std::cout << "branch data " << std::endl;
    std::cout << "Precision: " << precisionb1 << std::endl;
    std::cout << "Recall: " << recallb1 << std::endl;
    double F1 = 2 * precisionb1 * recallb1 / (precisionb1 + recallb1);
    std::cout << "F1: " << F1 << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr predictedCloud2(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("E:/0.PCL/DATA/Apple_tree/all_tree/" + br2.str(), *predictedCloud2) == -1) {
        PCL_ERROR("Couldn't read file branch.pcd\n");
        return -1;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr groundTruth2(new pcl::PointCloud<pcl::PointXYZ>);
    std::stringstream  br6;
    br6 << "t" << ttt << "/True_value/trunk.pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("E:/0.PCL/DATA/Apple_tree/all_tree/" + br6.str(), *groundTruth2) == -1) {
        PCL_ERROR("Couldn't read file branch.pcd\n");
        return -1;
    }
    double precisiont1, recallt1;
    int truePositive2 = 0;
    int falsePositive2 = 0;
    int falseNegative2 = 0;
    for (const auto& predictedPoint : *predictedCloud2) {
        bool isTruePositive = false;
        for (const auto& truthPoint : *groundTruth2) {
            if (predictedPoint.x == truthPoint.x &&
                predictedPoint.y == truthPoint.y &&
                predictedPoint.z == truthPoint.z)
            {
                truePositive2++;
                isTruePositive = true;
                break;
            }
        }
        if (!isTruePositive) {
            falsePositive2++;
        }
    }
    falseNegative2 = groundTruth2->size() - truePositive2;
    precisiont1 = static_cast<double>(truePositive2) / (truePositive2 + falsePositive2);
    recallt1 = static_cast<double>(truePositive2) / (truePositive2 + falseNegative2);
    std::cout << "trunk data " << std::endl;
    std::cout << "Precision: " << precisiont1 << std::endl;
    std::cout << "Recall: " << recallt1 << std::endl;
    double F2 = 2 * precisiont1 * recallt1 / (precisiont1 + recallt1);
    std::cout << "F1: " << F2 << std::endl;
    double precision, recall;
    precision = static_cast<double>(truePositive1 + truePositive2) / (truePositive1 + falsePositive1 + truePositive2 + falsePositive2);
    recall = static_cast<double>(truePositive1 + truePositive2) / (truePositive1 + falseNegative1 + truePositive2 + falseNegative2);
    std::cout << "all_tree data " << std::endl;
    std::cout << "Precision: " << precision << std::endl;
    std::cout << "Recall: " << recall << std::endl;
    double F3 = 2 * precision * recall / (precision + recall);
    std::cout << "F1: " << F3 << std::endl;
    float param1 = precisionb1;
    float param2 = recallb1;
    float param3 = F1;
    Eigen::Vector3f currentRow1(param1, param2, param3);
    data.push_back(currentRow1);
    float param4 = precisiont1;
    float param5 = recallt1;
    float param6 = F2;
    Eigen::Vector3f currentRow2(param4, param5, param6);
    data.push_back(currentRow2);
    float param7 = precision;
    float param8 = recall;
    float param9 = F3;
    Eigen::Vector3f currentRow3(param7, param8, param9);
    data.push_back(currentRow3);

    for (size_t i = 0; i < data.size(); ++i) {
    std::cout
        << data[i][0] << " " << data[i][1] << " " << data[i][2] << "\n";
    }
