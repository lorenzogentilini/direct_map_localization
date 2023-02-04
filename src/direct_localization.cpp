#include <direct_map_localization/direct_localization.hpp>

// Class Constructor
Localizer::Localizer(ros::NodeHandle& nh):
	#ifdef DEPTH
	data_sub(nh.subscribe("/d400/depth/image_rect_raw", 1, &Localizer::imageCallback, this)),
	#endif
	#ifdef CLOUD
	data_sub(nh.subscribe("/d400/depth/color/points", 1, &Localizer::cloudCallback, this)),
	#endif

	#ifdef REALSENSE
	odom_sub(nh.subscribe("/vi_odom", 1, &Localizer::odometryCallback, this)),
	#endif
	#ifdef MAVROS
	odom_sub(nh.subscribe("/mavros/local_position/pose", 1, &Localizer::odometryCallback, this)),
	#endif

	fiducial_sub(nh.subscribe("/fiducial_odom", 1, &Localizer::fiducialCallback, this)),
	pc_aligned_pub(nh.advertise<sensor_msgs::PointCloud2>("/debug_pc_aligned", 1)),
	result_pub(nh.advertise<std_msgs::Float64MultiArray>("/opt_result", 1)),
	exec_timer(nh.createTimer(ros::Duration(0.4), &Localizer::exec, this)){

	nh.param("camera_cx", cx, 320.3732299804687);
	nh.param("camera_cy", cy, 240.4962158203125);
	nh.param("camera_fx", fx, 386.1335144042969);
	nh.param("camera_fy", fy, 386.1335144042969);
	nh.param("px_sensor", ps_x, 0.05);
	nh.param("py_sensor", ps_y, 0.1);
	nh.param("pz_sensor", ps_z, 0.1);
	nh.param("qx_sensor", qs_x, -0.5);
	nh.param("qy_sensor", qs_y, 0.5);
	nh.param("qz_sensor", qs_z, -0.5);
	nh.param("qw_sensor", qs_w, 0.5);
	nh.param("publish_debug", debug_flag, true);

	nh.param("external_initial_guess", initial_guess, true);
	nh.param("px_guess", px_g, -16.5);
	nh.param("py_guess", py_g, 4.5);
	nh.param("pz_guess", pz_g, 0.1);
	nh.param("qx_guess", qx_g, 0.0);
	nh.param("qy_guess", qy_g, 0.0);
	nh.param("qz_guess", qz_g, 0.0);
	nh.param("qw_guess", qw_g, 1.0);

	nh.param("odom_data_sync", odom_sync, false);
	nh.param("min_sensor_r", min_r, 0.3);
	nh.param("max_sensor_r", max_r, 5.0);
	nh.param("cloud_rescaling_res", ds_r, 0.1);

	nh.param("max_x", m_max_x, 0.2);
	nh.param("max_y", m_max_y, 10.2);
	nh.param("max_z", m_max_z, 3.0);
	nh.param("min_x", m_min_x, -20.0);
	nh.param("min_y", m_min_y, -0.2);
	nh.param("min_z", m_min_z, -0.2);

	nh.param("normals_k_search", k_search, 40);
	nh.param("cluster_min_dim", min_dim, 100);
	nh.param("cluster_neighbors", nn_neighbors, 20);
	nh.param("cluster_smooth_thr", smooth_thr, 5.0);
	nh.param("cluster_curv_thr", curv_thr, 1.0);
	nh.param("minimum_clusters_n", min_clusters, 2);
	nh.param("min_nn_point", min_nn_point, 300);
	nh.param("use_only_clusters", use_only_clusters, false);

	nh.param("correctness_factor", correctness_factor, 10.0);
	nh.param("relocalization_factor", relocalization_factor, 1000.0);
	nh.param("n_relocalization_samples", n_rel_samples, 10);
	nh.param("sampling_factor_x", sampling_factor_x, 1.0);
	nh.param("sampling_factor_y", sampling_factor_y, 1.0);
	nh.param("sampling_factor_z", sampling_factor_z, 1.0);
	nh.param("sampling_factor_w", sampling_factor_w, 1.0);

	nh.param("optimization_bounds_xyz", bb, 1.0);
	nh.param("optimization_bounds_yaw", bb_yy, 0.5);

	nh.param("max_roll", mm_rr, 0.15);
	nh.param("max_pitch", mm_pp, 0.15);
	nh.param("loss_param", loss_param, 0.1);
	nh.param("cost_bias", bias, 1.0);

	nh.param("use_lowpass_filter", use_lp, true);
	nh.param("lowpass_cutoff_ff", ff_pp, 1.0);

	nh.param("use_cost_bound", use_cost_bound, false);
	nh.param("cost_bound", cost_bound, 1.0e2);
	nh.param("points_factor", points_factor, 200.0);
	nh.param("nn_points_bound", nn_points_bound, 1500);

	_map = new GridMap(nh);

	lp_xx = new LowPassFilter(ff_pp);
	lp_yy = new LowPassFilter(ff_pp);
	lp_zz = new LowPassFilter(ff_pp);
	lp_ww = new LowPassFilter(ff_pp);

	// Zero Quantities Initialization
	tf2::Vector3 zero_translation(0.0, 0.0, 0.0);
	tf2::Quaternion zero_quaternion(0.0, 0.0, 0.0, 1.0);
	map_toOdom_transformation.setOrigin(zero_translation);
	map_toOdom_transformation.setRotation(zero_quaternion);
	odom_toBody_transformation.setOrigin(zero_translation);
	odom_toBody_transformation.setRotation(zero_quaternion);

	sensorBody_trasformation << 1-2*pow(qs_y, 2)-2*pow(qs_z, 2), 2*qs_x*qs_y-2*qs_w*qs_z, 2*qs_x*qs_z+2*qs_w*qs_y, ps_x,
								2*qs_x*qs_y+2*qs_w*qs_z, 1-2*pow(qs_x, 2)-2*pow(qs_z, 2), 2*qs_y*qs_z-2*qs_w*qs_x, ps_y,
								2*qs_x*qs_z-2*qs_w*qs_y, 2*qs_y*qs_z+2*qs_w*qs_x, 1-2*pow(qs_x, 2)-2*pow(qs_y, 2), ps_z,
								0, 0, 0, 1;

	if(initial_guess){
		// Set Initial Guess
		tf2::Vector3 guess_translation(px_g, py_g, pz_g);
		tf2::Quaternion guess_quaternion(qx_g, qy_g, qz_g, qw_g);
		map_toOdom_transformation.setOrigin(guess_translation);
		map_toOdom_transformation.setRotation(guess_quaternion);

		double roll, pitch, yaw;
		tf2::Matrix3x3 m(guess_quaternion);
		m.getRPY(roll, pitch, yaw);

		lp_xx->setInitialCondition(px_g);
		lp_yy->setInitialCondition(py_g);
		lp_zz->setInitialCondition(pz_g);
		lp_ww->setInitialCondition(yaw);

		publishTF();
	}
}

// Class Destructor
Localizer::~Localizer(){
	delete _map;
}

void Localizer::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pc_msg){
	if(buffer.size() == 0 || !new_odom_received || !initial_guess){
		return;
	}

	pcl::PointCloud<pcl::PointXYZ> pc;
	pcl::fromROSMsg(*pc_msg, pc);

	uint opt_idx = 0;
	if(odom_sync){
		// Search For Corresponding ODOM Transformation
		double opt_dt = INFINITY;
		uint ii = 0;
		for(std::vector<Transform>::iterator it = buffer.begin(); it != buffer.end(); it++){
			double dt = abs(((*it).tt - pc_msg->header.stamp).toSec());
			if(dt < opt_dt){
				opt_dt = dt;
				opt_idx = ii;
			}

			ii++;
		}
	} else{
		opt_idx = buffer.size()-1;
	}

	odom_toBody_transformation = buffer[opt_idx].tf;

	double r,p,y;
	tf2::Quaternion q = odom_toBody_transformation.getRotation();
	tf2::Matrix3x3 m(q);
	m.getRPY(r,p,y);

	if(abs(r) >= mm_rr || abs(p) >= mm_pp){
		return;
	}
	
	insertCloud(pc);
}

void Localizer::imageCallback(const sensor_msgs::Image::ConstPtr& img){
	if(buffer.size() == 0 || !new_odom_received || !initial_guess){
		return;
	}

	cv_bridge::CvImagePtr cv_ptr;
	sensor_msgs::ImagePtr msg;

	try{
		cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_16UC1);
	} catch(cv_bridge::Exception& e){
		ROS_ERROR("[LOCALIZER]: cv_bridge exception: %s", e.what());
		return;
	}

	uint opt_idx = 0;
	if(odom_sync){
		// Search For Corresponding ODOM Transformation
		double opt_dt = INFINITY;
		uint ii = 0;
		for(std::vector<Transform>::iterator it = buffer.begin(); it != buffer.end(); it++){
			double dt = abs(((*it).tt - img->header.stamp).toSec());
			if(dt < opt_dt){
				opt_dt = dt;
				opt_idx = ii;
			}

			ii++;
		}
	} else{
		opt_idx = buffer.size()-1;
	}

	odom_toBody_transformation = buffer[opt_idx].tf;

	// Convert Depth to Pointcloud
	pcl::PointCloud<pcl::PointXYZ> pc;
	for(uint mm = 0; mm < cv_ptr->image.rows; mm++){
		for(int nn = 0; nn < cv_ptr->image.cols; nn++){
			pcl::PointXYZ pp;

			pp.z = (float)cv_ptr->image.at<uint16_t>(mm, nn)/1000.f;
			pp.x = (nn - cx)*pp.z/fx;
			pp.y = (mm - cy)*pp.z/fy;
			pc.points.push_back(pp);
		}
	}

	double r,p,y;
	tf2::Quaternion q = odom_toBody_transformation.getRotation();
	tf2::Matrix3x3 m(q);
	m.getRPY(r,p,y);

	if(abs(r) >= mm_rr || abs(p) >= mm_pp){
		return;
	}

	insertCloud(pc);
}

void Localizer::insertCloud(pcl::PointCloud<pcl::PointXYZ> pc){
	pc_avaliable = false;

	// Cut Cloud at Maximum and Minimum Distance
	pcl::PassThrough<pcl::PointXYZ> pt;
	pt.setInputCloud(pc.makeShared());
	pt.setFilterFieldName("z");
	pt.setFilterLimits(min_r, max_r);
	pt.filter(pc);

	// Downsampling & Filtering
	pcl::VoxelGrid<pcl::PointXYZ> ds;
	ds.setInputCloud(pc.makeShared());
	ds.setLeafSize(ds_r, ds_r, ds_r);
	ds.filter(pc);

	// Convert PC in Body Frame - Adjust Roll & Pitch
	tf2::Quaternion q = odom_toBody_transformation.getRotation();
	tf2::Matrix3x3 m(q);

	double r, p, y;
	m.getRPY(r,p,y);
	q.setRPY(r, p, 0.0);

	Eigen::Matrix<double, 4, 4> odom_toBody;
	odom_toBody << 1-(2*q.y()*q.y())-(2*q.z()*q.z()), 2*q.x()*q.y()-2*q.w()*q.z(), 2*q.x()*q.z()+2*q.w()*q.y(), 0.0,
				   2*q.x()*q.y()+2*q.w()*q.z(), 1-(2*q.x()*q.x())-(2*q.z()*q.z()), 2*q.y()*q.z()-2*q.w()*q.x(), 0.0,
				   2*q.x()*q.z()-2*q.w()*q.y(), 2*q.y()*q.z()+2*q.w()*q.x(), 1-(2*q.x()*q.x())-(2*q.y()*q.y()), 0.0,
				   0.0, 0.0, 0.0, 1.0;

	Eigen::Matrix<double, 4, 4> pc_tranformation = odom_toBody*sensorBody_trasformation;
	pcl::transformPointCloud(pc, pc, pc_tranformation);

	// Filter out Ceiling
	pcl::PointCloud<pcl::PointXYZ> pc_rotated;
	pcl::PointCloud<pcl::PointXYZ> insert_cloud;
	
	tf2::Transform body_toMap_transformation = map_toOdom_transformation*odom_toBody_transformation;
	tf2::Vector3 body_toMap_translation = body_toMap_transformation.getOrigin();
	tf2::Quaternion body_toMap_rotation = body_toMap_transformation.getRotation();

	tf2::Matrix3x3 mm(body_toMap_rotation);
	mm.getRPY(r,p,y);
	q.setRPY(0.0, 0.0, y);

	Eigen::Matrix<double, 4, 4> body_toMap;
	body_toMap << 1-(2*q.y()*q.y())-(2*q.z()*q.z()), 2*q.x()*q.y()-2*q.w()*q.z(), 2*q.x()*q.z()+2*q.w()*q.y(), body_toMap_translation.x(),
					2*q.x()*q.y()+2*q.w()*q.z(), 1-(2*q.x()*q.x())-(2*q.z()*q.z()), 2*q.y()*q.z()-2*q.w()*q.x(), body_toMap_translation.y(),
					2*q.x()*q.z()-2*q.w()*q.y(), 2*q.y()*q.z()+2*q.w()*q.x(), 1-(2*q.x()*q.x())-(2*q.y()*q.y()), body_toMap_translation.z(),
					0.0, 0.0, 0.0, 1.0;

	pcl::transformPointCloud(pc, pc_rotated, body_toMap);
	
	pcl::PointCloud<pcl::PointXYZ>::const_iterator iter_pc = pc.begin();
	for(pcl::PointCloud<pcl::PointXYZ>::const_iterator iter = pc_rotated.begin(); iter != pc_rotated.end(); iter++){

		if(iter->z < (m_max_z-0.2)){
			insert_cloud.push_back(*iter_pc);
		}

		iter_pc++;
	}

	// Check minimum point number
	if(insert_cloud.size() < min_nn_point){
		return;
	}
	
	// Cluster in Planes
	pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;

	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(insert_cloud.makeShared());
	normal_estimator.setKSearch(k_search);
	normal_estimator.compute(*normals);

	reg.setMinClusterSize(min_dim);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(nn_neighbors); 
	reg.setInputCloud(insert_cloud.makeShared());
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(smooth_thr*M_PI/180.0);
	reg.setCurvatureThreshold(curv_thr);

	std::vector<pcl::PointIndices> clusters;
	reg.extract(clusters);

	if(clusters.size () < min_clusters){
		return;
	}

	if(use_only_clusters){
		for(uint ii = 0; ii < clusters.size(); ii++){
			for(uint jj = 0; jj < clusters[ii].indices.size(); jj++){
				pc_final_.push_back(insert_cloud[clusters[ii].indices[jj]]);
			}
		}
	} else{
		pc_final_ = insert_cloud;
	}

	// Setting Up Current Initial Condition
	x_init = body_toMap_translation.x();
	y_init = body_toMap_translation.y();
	z_init = body_toMap_translation.z();
	yy_init = y;

	pc_avaliable = true;
}

void Localizer::exec(const ros::TimerEvent& e){
	if(buffer.size() == 0 || !new_odom_received || !initial_guess || !pc_avaliable){
		return;
	}

	if(needed_relocalization){
		needed_relocalization = false;
		relocalize();

		new_odom_received = false;
		pc_avaliable = false;

		return;
	}

	double x[4] = {x_init, y_init, z_init, yy_init};

	ceres::Problem problem;
	problem.AddParameterBlock(x, 4);
	pcl::PointCloud<pcl::PointXYZ> pc_final = pcl::PointCloud<pcl::PointXYZ>(pc_final_);
	for(pcl::PointCloud<pcl::PointXYZ>::const_iterator iter = pc_final.begin();
		iter != pc_final.end(); iter++){

		ceres::CostFunction* costFunction = new CostFunction(iter->x, iter->y, iter->z, _map, bias);
		problem.AddResidualBlock(costFunction, new ceres::CauchyLoss(loss_param), x);
	}
	
	// Set Bounds
	problem.SetParameterLowerBound(x, 0, x_init - bb);
	problem.SetParameterLowerBound(x, 1, y_init - bb);
	problem.SetParameterLowerBound(x, 2, z_init - bb/2);
	problem.SetParameterLowerBound(x, 3, yy_init - bb_yy);
	problem.SetParameterUpperBound(x, 0, x_init + bb);
	problem.SetParameterUpperBound(x, 1, y_init + bb);
	problem.SetParameterUpperBound(x, 2, z_init + bb/2);
	problem.SetParameterUpperBound(x, 3, yy_init + bb_yy);

	// Run the Solver
	ceres::Solver::Options options;
	ceres::Solver::Summary summary;
	options.minimizer_progress_to_stdout = false;
	options.linear_solver_type = ceres::DENSE_QR;
	ceres::Solve(options, &problem, &summary);

	ROS_INFO_COND(debug_flag & summary.IsSolutionUsable(), "[LOCALIZER]: Solution Usable");
	ROS_INFO_COND(debug_flag & !summary.IsSolutionUsable(), "[LOCALIZER]: Solution NOT Usable !!!!!");

	// Compute the Result Covariance
	ceres::Covariance::Options covariance_options;
	ceres::Covariance covariance(covariance_options);

	std::vector<std::pair<const double*, const double*>> covariance_blocks;
	covariance_blocks.push_back(std::make_pair(x, x));
	if(!covariance.Compute(covariance_blocks, &problem)){
		new_odom_received = false;
		pc_avaliable = false;

		return;
	}

	double covariance_xx[4*4];
	covariance.GetCovarianceBlock(x, x, covariance_xx);

	// Compute Covariance Determinant
	Eigen::Matrix<double, 4, 4> CO;
	CO << covariance_xx[0],  covariance_xx[1],  covariance_xx[2],  covariance_xx[3],
		  covariance_xx[4],  covariance_xx[5],  covariance_xx[6],  covariance_xx[7],
		  covariance_xx[8],  covariance_xx[9],  covariance_xx[10], covariance_xx[11],
		  covariance_xx[12], covariance_xx[13], covariance_xx[14], covariance_xx[15];

	double dd = CO.determinant();
	if(first_opt_circle){
		dd_base = dd;
		first_opt_circle = false;
	}

	if(dd > dd_base*relocalization_factor){
		// Save Normalized Covariance Entries
		double nn = covariance_xx[0] + covariance_xx[5] + covariance_xx[10] + covariance_xx[15];
		xx_n = covariance_xx[0]/nn;
		yy_n = covariance_xx[5]/nn;
		zz_n = covariance_xx[10]/nn;
		ww_n = covariance_xx[15]/nn;

		needed_relocalization = true;
		ROS_ERROR("[LOCALIZER]: RELOCALIZATION");

		new_odom_received = false;
		pc_avaliable = false;

		return;

	} else if(dd > dd_base*correctness_factor){
		ROS_WARN("[LOCALIZER]: JUMPING ITERATION");

		new_odom_received = false;
		pc_avaliable = false;

		return;
	}

	if(use_cost_bound){
		ROS_INFO_COND(debug_flag, "[LOCALIZER]: Final Cost %f With Points %ld", summary.final_cost, pc_final.size());

		double bound_factor = (pc_final.size() - nn_points_bound)/points_factor;
		bound_factor = bound_factor < 0.0 ? 0.0 : bound_factor;

		double actual_bound = cost_bound + bound_factor;
		if(summary.final_cost > actual_bound){
			return;
		}
	}

	if(debug_flag){
		// Publish Result
		std_msgs::Float64MultiArray result;
		for(uint ii = 0; ii < 4; ii++)
			result.data.push_back(x[ii]);
		for(uint ii = 0; ii < 16; ii++)
			result.data.push_back(covariance_xx[ii]);
		result_pub.publish(result);
	}

	// Update Current Transformation
	tf2::Vector3 body_toMap_translation_updated(x[0], x[1], x[2]);
	tf2::Quaternion body_toMap_rotation_updated;

	tf2::Quaternion odom_toBody_rotation = odom_toBody_transformation.getRotation();
	tf2::Matrix3x3 mm(odom_toBody_rotation);

	double r, p, y;
	mm.getRPY(r,p,y);

	body_toMap_rotation_updated.setRPY(r, p, x[3]);

	tf2::Transform body_toMap_transformation_updated(body_toMap_rotation_updated, body_toMap_translation_updated);
	map_toOdom_transformation = body_toMap_transformation_updated*odom_toBody_transformation.inverse();
	publishTF();

	if(debug_flag){
		// Publish PC in Map Coordinates for Debug
		sensor_msgs::PointCloud2 pc_msg;
		tf2::Quaternion q;
		q.setRPY(0.0, 0.0, x[3]);

		Eigen::Matrix<double, 4, 4> map_toBody;
		map_toBody << 1-(2*q.y()*q.y())-(2*q.z()*q.z()), 2*q.x()*q.y()-2*q.w()*q.z(), 2*q.x()*q.z()+2*q.w()*q.y(), body_toMap_translation_updated.x(),
					  2*q.x()*q.y()+2*q.w()*q.z(), 1-(2*q.x()*q.x())-(2*q.z()*q.z()), 2*q.y()*q.z()-2*q.w()*q.x(), body_toMap_translation_updated.y(),
				      2*q.x()*q.z()-2*q.w()*q.y(), 2*q.y()*q.z()+2*q.w()*q.x(), 1-(2*q.x()*q.x())-(2*q.y()*q.y()), body_toMap_translation_updated.z(),
				      0.0, 0.0, 0.0, 1.0;

		pcl::transformPointCloud(pc_final, pc_final, map_toBody);
		pcl::toROSMsg(pc_final, pc_msg);
		pc_msg.header.frame_id = "map";
		pc_aligned_pub.publish(pc_msg);

		ROS_INFO("[LOCALIZER]: X: %f Y: %f Z: %f YAW: %f", x[0], x[1], x[2], x[3]);
	}

	new_odom_received = false;
	pc_avaliable = false;
}

void Localizer::relocalize(){
	std::vector<double> dds;
	std::vector<tf2::Transform> tts;

	double ss_factor_x = xx_n*sampling_factor_x;
	double ss_factor_y = yy_n*sampling_factor_y;
	double ss_factor_z = zz_n*sampling_factor_z;
	double ss_factor_w = ww_n*sampling_factor_w;
	for(uint ii = 0; ii < n_rel_samples; ii++){
		double dx = (((double)std::rand()/(double)RAND_MAX)*2*ss_factor_x) - ss_factor_x;
		double dy = (((double)std::rand()/(double)RAND_MAX)*2*ss_factor_y) - ss_factor_y;
		double dz = (((double)std::rand()/(double)RAND_MAX)*2*ss_factor_z) - ss_factor_z;
		double dw = (((double)std::rand()/(double)RAND_MAX)*2*ss_factor_w) - ss_factor_w;

		double x_init_nn = x_init+dx;
		double y_init_nn = y_init+dy;
		double z_init_nn = z_init+dz;

		checkBounds(x_init_nn, y_init_nn, z_init_nn);
		double x[4] = {x_init_nn, y_init_nn, z_init_nn, yy_init+dw};

		ceres::Problem problem;
		problem.AddParameterBlock(x, 4);
		pcl::PointCloud<pcl::PointXYZ> pc_final = pcl::PointCloud<pcl::PointXYZ>(pc_final_);
		for(pcl::PointCloud<pcl::PointXYZ>::const_iterator iter = pc_final.begin();
			iter != pc_final.end(); iter++){

			ceres::CostFunction* costFunction = new CostFunction(iter->x, iter->y, iter->z, _map, bias);
			problem.AddResidualBlock(costFunction, new ceres::CauchyLoss(loss_param), x);
		}
		
		// Set Bounds
		problem.SetParameterLowerBound(x, 0, m_min_x);
		problem.SetParameterLowerBound(x, 1, m_min_y);
		problem.SetParameterLowerBound(x, 2, m_min_z);
		problem.SetParameterUpperBound(x, 0, m_max_x);
		problem.SetParameterUpperBound(x, 1, m_max_y);
		problem.SetParameterUpperBound(x, 2, m_max_z);

		// Run the Solver
		ceres::Solver::Options options;
		ceres::Solver::Summary summary;
		options.minimizer_progress_to_stdout = false;
		options.linear_solver_type = ceres::DENSE_QR;
		ceres::Solve(options, &problem, &summary);

		// Compute the Result Covariance
		ceres::Covariance::Options covariance_options;
		ceres::Covariance covariance(covariance_options);

		std::vector<std::pair<const double*, const double*>> covariance_blocks;
		covariance_blocks.push_back(std::make_pair(x, x));
		if(!covariance.Compute(covariance_blocks, &problem)){
			continue;
		}

		double covariance_xx[4*4];
		covariance.GetCovarianceBlock(x, x, covariance_xx);

		// Compute Covariance Determinant
		Eigen::Matrix<double, 4, 4> CO;
		CO << covariance_xx[0],  covariance_xx[1],  covariance_xx[2],  covariance_xx[3],
			  covariance_xx[4],  covariance_xx[5],  covariance_xx[6],  covariance_xx[7],
			  covariance_xx[8],  covariance_xx[9],  covariance_xx[10], covariance_xx[11],
			  covariance_xx[12], covariance_xx[13], covariance_xx[14], covariance_xx[15];

		double dd = CO.determinant();

		if(dd > dd_base*correctness_factor){
			continue;
		}

		if(use_cost_bound){
			ROS_INFO_COND(debug_flag, "[LOCALIZER]: Final Cost %f With Points %ld", summary.final_cost, pc_final.size());

			double bound_factor = (pc_final.size() - nn_points_bound)/points_factor;
			bound_factor = bound_factor < 0.0 ? 0.0 : bound_factor;

			double actual_bound = cost_bound + bound_factor;
			if(summary.final_cost > actual_bound){
				continue;
			}
		}

		// Update Current Transformation
		tf2::Vector3 body_toMap_translation_updated(x[0], x[1], x[2]);
		tf2::Quaternion body_toMap_rotation_updated;

		tf2::Quaternion odom_toBody_rotation = odom_toBody_transformation.getRotation();
		tf2::Matrix3x3 mm(odom_toBody_rotation);

		double r, p, y;
		mm.getRPY(r,p,y);

		body_toMap_rotation_updated.setRPY(r, p, x[3]);

		tf2::Transform body_toMap_transformation_updated(body_toMap_rotation_updated, body_toMap_translation_updated);
		tf2::Transform map_toOdom_transformation_updated = body_toMap_transformation_updated*odom_toBody_transformation.inverse();

		if(dd < dd_base){
			map_toOdom_transformation = map_toOdom_transformation_updated;
			//publishTF();

			ROS_INFO("[LOCALIZER]: Relocalization Done Perfectly!!");
			ROS_INFO_COND(debug_flag, "[LOCALIZER]: X: %f Y: %f Z: %f", x[0], x[1], x[2]);
			return;
		}

		dds.push_back(dd);
		tts.push_back(map_toOdom_transformation_updated);
	}

	if(dds.size() == 0){
		ROS_ERROR("[LOCALIZER]: Relocalization Failed!!");
		needed_relocalization = true;
		return;
	}

	double min_dd = INFINITY;
	uint min_idx = 0;
	for(uint ii = 0; ii < dds.size(); ii++){
		if(dds[ii] < min_dd){
			min_dd = dds[ii];
			min_idx = ii;
		}
	}

	map_toOdom_transformation = tts[min_idx];
	//publishTF();

	tf2::Vector3 tt = tts[min_idx].getOrigin();
	ROS_INFO("[LOCALIZER]: Relocalization Done!!");
	ROS_INFO_COND(debug_flag, "[LOCALIZER]: X: %f Y: %f Z: %f", tt.x(), tt.y(), tt.z());
}

void Localizer::checkBounds(double& dx, double& dy, double& dz){
	if(dx > m_max_x){
		dx = m_max_x;
	}

	if(dy > m_max_y){
		dy = m_max_y;
	}

	if(dz > m_max_z){
		dz = m_max_z;
	}

	if(dx < m_min_x){
		dx = m_min_x;
	}

	if(dy < m_min_y){
		dy = m_min_y;
	}

	if(dz < m_min_z){
		dz = m_min_z;
	}
}

#ifdef REALSENSE
void Localizer::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
	tf2::Vector3 odomBody_translation(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	tf2::Quaternion odomBody_quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	tf2::Transform odomBody_transformation(odomBody_quaternion, odomBody_translation);
#endif
#ifdef MAVROS
void Localizer::odometryCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	tf2::Vector3 odomBody_translation(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
	tf2::Quaternion odomBody_quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
	tf2::Transform odomBody_transformation(odomBody_quaternion, odomBody_translation);
#endif
	Transform tf;
	tf.tt = msg->header.stamp;
	tf.tf = odomBody_transformation;

	if(buffer.size() < MAX_DIM){
		buffer.push_back(tf);
	} else{
		buffer.erase(buffer.begin());
		buffer.push_back(tf);
	}

	new_odom_received = true;
}

void Localizer::fiducialCallback(const nav_msgs::Odometry::ConstPtr& msg){
	if(buffer.size() == 0){
		return;
	}

	initial_guess = false;

	tf2::Vector3 body_toMap_translation(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	tf2::Quaternion body_toMap_quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	tf2::Transform body_toMap_transformation(body_toMap_quaternion, body_toMap_translation);

	map_toOdom_transformation = body_toMap_transformation*((buffer.back().tf).inverse());

	tf2::Vector3 guess_translation = map_toOdom_transformation.getOrigin();
	tf2::Quaternion guess_quaternion = map_toOdom_transformation.getRotation();

	tf2::Matrix3x3 m(guess_quaternion);

	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	lp_xx->setInitialCondition(guess_translation.x());
	lp_yy->setInitialCondition(guess_translation.y());
	lp_zz->setInitialCondition(guess_translation.z());
	lp_ww->setInitialCondition(yaw);

	publishTF();

	initial_guess = true;
}

void Localizer::publishTF(){
	tf2::Vector3 map_toOdom_translation = map_toOdom_transformation.getOrigin();
	tf2::Quaternion map_toOdom_rotation = map_toOdom_transformation.getRotation();

	if(use_lp && initial_guess){
		map_toOdom_translation.setX(lp_xx->update(map_toOdom_translation.x()));
		map_toOdom_translation.setY(lp_yy->update(map_toOdom_translation.y()));
		map_toOdom_translation.setZ(lp_zz->update(map_toOdom_translation.z()));

		double roll, pitch, yaw;
		tf2::Matrix3x3 m(map_toOdom_rotation);
		m.getRPY(roll, pitch, yaw);

		yaw = lp_ww->update(yaw);
		map_toOdom_rotation.setRPY(roll, pitch, yaw);

		map_toOdom_transformation.setOrigin(map_toOdom_translation);
		map_toOdom_transformation.setRotation(map_toOdom_rotation);
	}

  	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "map";
	transformStamped.child_frame_id = "odom";
	transformStamped.transform.translation.x = map_toOdom_translation.x();
	transformStamped.transform.translation.y = map_toOdom_translation.y();
	transformStamped.transform.translation.z = map_toOdom_translation.z();
	transformStamped.transform.rotation.x = map_toOdom_rotation.x();
	transformStamped.transform.rotation.y = map_toOdom_rotation.y();
	transformStamped.transform.rotation.z = map_toOdom_rotation.z();
	transformStamped.transform.rotation.w = map_toOdom_rotation.w();

	br.sendTransform(transformStamped);
}