#include <direct_map_localization/grid_map.hpp>

// Class Constructor
GridMap::GridMap(ros::NodeHandle& nh):
	visPublisher(nh.advertise<visualization_msgs::Marker>("/vis_map", 1)){
	nh.param("map_name", map_name, std::string("/home/gent/catkin_ws/src/flightmare/flightrender/pointcloud_data/leonardo.ply"));
	nh.param("grid_name", grid_name, std::string("/home/gent/catkin_ws/src/drone_localization/direct_map_localization/data/leonardo.bin"));
	nh.param("grid_resolution", m_res, 0.1);
	nh.param("max_x", m_max_x, 0.2);
	nh.param("max_y", m_max_y, 10.2);
	nh.param("max_z", m_max_z, 3.0);
	nh.param("min_x", m_min_x, -20.0);
	nh.param("min_y", m_min_y, -0.2);
	nh.param("min_z", m_min_z, -0.2);
	nh.param("border_extension", m_ext, 2.0);
	nh.param("max_nn_threads", max_nn_threads, 4);
	nh.param("publish_debug", debug_flag, true);

	m_max_x += m_ext;
	m_max_y += m_ext;
	m_max_z += m_ext;

	m_min_x -= m_ext;
	m_min_y -= m_ext;
	m_min_z -= m_ext;

	pcl::io::loadPLYFile(map_name, pc);
	kdtree.setInputCloud(pc.makeShared());

	if(!loadGrid()){
		computeGrid();
		saveGrid();
	} else{
		// Wait a While for Map Publication
		ros::Duration(0.5).sleep();
	}

	if(debug_flag){
		publishMap();
	}
}

// Class GridMap
GridMap::~GridMap(){
	delete m_grid;
}

void GridMap::saveGrid(){
	FILE* ff;
		
	// Open File
	ff = fopen(grid_name.c_str(), "wb");
	if(ff == NULL){
		ROS_ERROR("[GRID MAP]: Error opening file");
		return;
	}
	
	fwrite(&m_gridSize, sizeof(int), 1, ff);
	fwrite(&m_gridSize_x, sizeof(int), 1, ff);
	fwrite(&m_gridSize_y, sizeof(int), 1, ff);
	fwrite(&m_gridSize_z, sizeof(int), 1, ff);
	
	// Write Grid Cells
	fwrite(m_grid, sizeof(uint16_t), m_gridSize, ff);
	
	// Close File
	fclose(ff);

	ROS_INFO("[GRID MAP]: Grid Saved!");
}

bool GridMap::loadGrid(){
	FILE* ff;
	
	// Open File
	ff = fopen(grid_name.c_str(), "rb");
	if(ff == NULL){
		ROS_WARN("[GRID MAP]: Grid not Found - Start Generation");
		return false;
	}
		
	fread(&m_gridSize, sizeof(int), 1, ff);
	fread(&m_gridSize_x, sizeof(int), 1, ff);
	fread(&m_gridSize_y, sizeof(int), 1, ff);
	fread(&m_gridSize_z, sizeof(int), 1, ff);

	m_gridStep_y = m_gridSize_x;
	m_gridStep_z = m_gridSize_x*m_gridSize_y;
	m_grid = new uint16_t[m_gridSize];
	fread(m_grid, sizeof(uint16_t), m_gridSize, ff);		
	fclose(ff);
	
	ROS_INFO("[GRID MAP]: Grid Loaded From File");
	return true;
}

double GridMap::getDistance(double px, double py, double pz){
	if(isIntoMap(px, py, pz)){
		uint64_t ii = point2grid(px, py, pz); 
		return (double)m_grid[ii]*0.01;
	}

	return 0.0;
}

TrilinearParameters GridMap::getTrilinearParameters(double px, double py, double pz){
	TrilinearParameters tp;
	if(isIntoMap(px, py, pz)){
		uint64_t ii = point2grid(px, py, pz); 

		double c000 = (double)m_grid[ii]*0.01; 
		double c001 = (double)m_grid[ii + m_gridStep_z]*0.01; 
		double c010 = (double)m_grid[ii + m_gridStep_y]*0.01; 
		double c011 = (double)m_grid[ii + m_gridStep_y + m_gridStep_z]*0.01; 
		double c100 = (double)m_grid[ii + 1]*0.01; 
		double c101 = (double)m_grid[ii + 1 + m_gridStep_z]*0.01; 
		double c110 = (double)m_grid[ii + 1 + m_gridStep_y]*0.01; 
		double c111 = (double)m_grid[ii + 1 + m_gridStep_y + m_gridStep_z]*0.01; 

		double div = -1.0/(m_res*m_res*m_res);
		double x0 = ((int)(px/m_res))*m_res;
		double x1 = x0 + m_res;
		double y0 = ((int)(py/m_res))*m_res;
		double y1 = y0 + m_res;
		double z0 = ((int)(pz/m_res))*m_res;
		double z1 = z0 + m_res;

		tp.p1 = (-c000*x1*y1*z1 + c001*x1*y1*z0 + c010*x1*y0*z1 - c011*x1*y0*z0 + c100*x0*y1*z1 - c101*x0*y1*z0 - c110*x0*y0*z1 + c111*x0*y0*z0)*div;
		tp.p2 = (c000*y1*z1 - c001*y1*z0 - c010*y0*z1 + c011*y0*z0 - c100*y1*z1 + c101*y1*z0 + c110*y0*z1 - c111*y0*z0)*div;
		tp.p3 = (c000*x1*z1 - c001*x1*z0 - c010*x1*z1 + c011*x1*z0 - c100*x0*z1 + c101*x0*z0 + c110*x0*z1 - c111*x0*z0)*div;
		tp.p4 = (c000*x1*y1 - c001*x1*y1 - c010*x1*y0 + c011*x1*y0 - c100*x0*y1 + c101*x0*y1 + c110*x0*y0 - c111*x0*y0)*div;
		tp.p5 = (-c000*z1 + c001*z0 + c010*z1 - c011*z0 + c100*z1 - c101*z0 - c110*z1 + c111*z0)*div;
		tp.p6 = (-c000*y1 + c001*y1 + c010*y0 - c011*y0 + c100*y1 - c101*y1 - c110*y0 + c111*y0)*div;
		tp.p7 = (-c000*x1 + c001*x1 + c010*x1 - c011*x1 + c100*x0 - c101*x0 - c110*x0 + c111*x0)*div;
		tp.p8 = (c000 - c001 - c010 + c011 - c100 + c101 + c110 - c111)*div;
	}

	return tp;
}

void GridMap::computeGrid(){
	// Alloc the 3D Grid
	m_gridSize_x = (int)((m_max_x - m_min_x)/m_res);
	m_gridSize_y = (int)((m_max_y - m_min_y)/m_res); 
	m_gridSize_z = (int)((m_max_z - m_min_z)/m_res);
	m_gridSize   = m_gridSize_x*m_gridSize_y*m_gridSize_z;
	m_gridStep_y = m_gridSize_x;
	m_gridStep_z = m_gridSize_x*m_gridSize_y;
	m_grid = new uint16_t[m_gridSize];

	// Compute the Distance to the Closest Point of the Grid
	std::vector<std::thread> threads;
	int dii = m_gridSize_z/max_nn_threads;
	int res = m_gridSize_z%max_nn_threads;

	for(int ii = 0; ii < max_nn_threads; ii++){
		if(ii == max_nn_threads-1){
			threads.emplace_back(&GridMap::computePartialGrid, this, ii*dii, (ii+1)*dii + res, kdtree);
		} else{
			threads.emplace_back(&GridMap::computePartialGrid, this, ii*dii, (ii+1)*dii, kdtree);
		}
	}

	for(int ii = 0; ii < max_nn_threads; ii++){
		threads[ii].join();
	}
}

void GridMap::computePartialGrid(int init_z, int end_z, pcl::KdTreeFLANN<pcl::PointXYZ> kt){
	for(int iz = init_z; iz < end_z; iz++){
		pcl::PointXYZ searchPoint;
		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);

		ROS_INFO("[LOCALIZER]: Processing z = %d out of %d", iz, end_z);
		for(int iy = 0; iy < m_gridSize_y; iy++){
			for(int ix = 0; ix < m_gridSize_x; ix++){
				searchPoint.x = ix*m_res + m_min_x;
				searchPoint.y = iy*m_res + m_min_y;
				searchPoint.z = iz*m_res + m_min_z;
				int index = ix + iy*m_gridStep_y + iz*m_gridStep_z;

				if(kt.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
					m_grid[index] = (uint16_t)(sqrt(pointNKNSquaredDistance[0])*100.0);
				else
					m_grid[index] = 0;
			}
		}
	}
}

inline int GridMap::point2grid(const float &x, const float &y, const float &z){
	return (int)((x - m_min_x)/m_res) + (int)((y - m_min_y)/m_res)*m_gridStep_y + (int)((z - m_min_z)/m_res)*m_gridStep_z;
}

inline bool GridMap::isIntoMap(double x, double y, double z){
	return (x >= m_min_x && y >= m_min_y && z >= m_min_z && x < m_max_x && y < m_max_y && z < m_max_z);
}

void GridMap::publishMap(){
	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();
	marker.ns = "Map";
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.w = 1.0;
	marker.id += 1;
	marker.type = visualization_msgs::Marker::SPHERE_LIST;
	marker.scale.x = 0.03;
	marker.scale.y = 0.03;
	marker.scale.z = 0.03;
	marker.color.b = 1.0;
	marker.color.r = 0.0;
	marker.color.a = 1.0;

	geometry_msgs::Point p;
	for(pcl::PointCloud<pcl::PointXYZ>::const_iterator iter = pc.begin();
			iter != pc.end(); iter++){

		p.x = iter->x;
		p.y = iter->y;
		p.z = iter->z;

		marker.points.push_back(p);
	}

	visPublisher.publish(marker);
}