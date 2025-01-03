
	for (int j = 7; j < 8; j++)
	{
		std::stringstream s, s1;
		s <<  "t" << j << ".las";
		s1 <<  "t" << j << ".pcd";
		std::ifstream ifs("E:/0.PCL/DATA/Apple_tree/daiseg/" + s.str(), std::ios::in | std::ios::binary); 
		cout << endl;

		cout << "sb1";
		liblas::ReaderFactory f;
		liblas::Reader reader = f.CreateWithStream(ifs); 
		cout << "sb2";
		unsigned long int nbPoints = reader.GetHeader().GetPointRecordsCount();

		pcl::PointCloud<pcl::PointXYZI> cloud;
		cloud.width = nbPoints;
		cloud.height = 1;
		cloud.is_dense = false;
		cloud.points.resize(cloud.width * cloud.height);

		int i = 0;
		while (reader.ReadNextPoint())
		{
			cloud.points[i].x = (reader.GetPoint().GetX());
			cloud.points[i].y = (reader.GetPoint().GetY());
			cloud.points[i].z = (reader.GetPoint().GetZ());
			cloud.points[i].intensity = (float)(reader.GetPoint().GetIntensity());
			i++;
		}

		pcl::io::savePCDFileASCII("E:/0.PCL/DATA/Apple_tree/daiseg/" + s1.str(), cloud);
	}
