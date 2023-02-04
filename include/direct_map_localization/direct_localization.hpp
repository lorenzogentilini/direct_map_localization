#include <direct_map_localization/grid_map.hpp>
#include <direct_map_localization/common.hpp>

//#define DEPTH
#define CLOUD

//#define REALSENSE
#define MAVROS

#define MAX_DIM 15

typedef struct{
	ros::Time tt;
	tf2::Transform tf;
} Transform;

class LowPassFilter{
    public:
    double A, B, C;
    double x = 0;
    ros::Time previousTime = ros::Time::now();

    LowPassFilter(double cutoff_ff){
        double cutoff_w = cutoff_ff*2*M_PI;
        A = -cutoff_w;
        B = cutoff_w;
        C = 1.0;
    };

    void setInitialCondition(double x_){
        previousTime = ros::Time::now();
        x = x_/C;
    };

    double update(double u){
        double dt = (ros::Time::now() - previousTime).toSec();
        previousTime = ros::Time::now();

        x = x + (A*x + B*u)*dt;

        return C*x;
    };
};

class Localizer{
	public:
	// Class Constructor
	Localizer(ros::NodeHandle& nh);

	// Class Destructor
	~Localizer();

	private:
	double ps_x, ps_y, ps_z, qs_x, qs_y, qs_z, qs_w;
	double cx, cy, fx, fy;
	double px_g, py_g, pz_g, qx_g, qy_g, qz_g, qw_g;
	double min_r, max_r, ds_r;
	double m_max_x, m_max_y, m_max_z, m_min_x, m_min_y, m_min_z;
	int k_search, min_dim, nn_neighbors, min_clusters, n_rel_samples;
	double smooth_thr, curv_thr;
	double sampling_factor_x, sampling_factor_y, sampling_factor_z, sampling_factor_w;
	bool debug_flag, initial_guess, odom_sync, use_only_clusters, use_lp;

	bool use_cost_bound;
	double cost_bound;

	bool new_odom_received = false;
	bool pc_avaliable = false;
	bool first_opt_circle = true;
	bool needed_relocalization = false;

	double dd_base, correctness_factor, relocalization_factor;
	double xx_n, yy_n, zz_n, ww_n;
	double bb, bb_yy, ff_pp;
	double mm_rr, mm_pp, loss_param;

	double x_init, y_init, z_init, yy_init;
	int min_nn_point;

	double bias;
	int nn_points_bound;
	double points_factor;

	Eigen::Matrix<double, 4, 4> sensorBody_trasformation;

	ros::Subscriber data_sub;
	ros::Subscriber odom_sub;
	ros::Subscriber fiducial_sub;
	ros::Publisher pc_aligned_pub;
	ros::Publisher result_pub;
	ros::Timer exec_timer;

	tf2::Transform map_toOdom_transformation;
	tf2::Transform odom_toBody_transformation;
	tf2_ros::TransformBroadcaster br;

	GridMap* _map;
	pcl::PointCloud<pcl::PointXYZ> pc_final_;
	std::vector<Transform> buffer; 

	LowPassFilter* lp_xx;
	LowPassFilter* lp_yy;
	LowPassFilter* lp_zz;
	LowPassFilter* lp_ww;

	void exec(const ros::TimerEvent& e);
	void relocalize();
	void checkBounds(double& dx, double& dy, double& dz);
	void insertCloud(pcl::PointCloud<pcl::PointXYZ> pc);
	void imageCallback(const sensor_msgs::Image::ConstPtr& img);
	void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pc_msg);
	void fiducialCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void publishTF();

	#ifdef REALSENSE
	void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
	#endif
	#ifdef MAVROS
	void odometryCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	#endif
};


class CostFunction: public ceres::SizedCostFunction<1, 4>{
	public:
	CostFunction(double px, double py, double pz, GridMap* map, double bias, double ww = 1.0):
			_px(px), _py(py), _pz(pz), _map(map), _bias(bias), _ww(ww){;}

	virtual ~CostFunction(){;}

	virtual bool Evaluate(double const* const* parameters,
						  double* residuals,
						  double** jacobian) const{

		double tx = parameters[0][0];
		double ty = parameters[0][1];
		double tz = parameters[0][2];
		double ry = parameters[0][3];

		// Rotate and Translate the Point
		double nx = cos(ry)*_px - sin(ry)*_py + tx;
		double ny = sin(ry)*_px + cos(ry)*_py + ty;
		double nz = _pz + tz;

		// Compute the Residual
		TrilinearParameters tp = _map->getTrilinearParameters(nx, ny, nz);
		residuals[0] = _bias + _ww*(tp.p1 + tp.p2*nx + tp.p3*ny + tp.p4*nz + tp.p5*nx*ny + tp.p6*nx*nz + tp.p7*ny*nz + tp.p8*nx*ny*nz);

		// Compute the Jacobian, if Required
		if(jacobian != NULL && jacobian[0] != NULL){
			double dxy = _py*cos(ry) + _px*sin(ry);
			double dyy = _px*cos(ry) - _py*sin(ry);
			jacobian[0][0] = _ww*(tp.p2 + tp.p6*nz + tp.p5*ny + tp.p8*nz*ny);
			jacobian[0][1] = _ww*(tp.p3 + tp.p7*nz + tp.p5*nx + tp.p8*nz*nx);
			jacobian[0][2] = _ww*(tp.p4 + tp.p6*nx + tp.p7*ny + tp.p8*nx*ny);
			jacobian[0][3] = _ww*(tp.p3*dyy - tp.p2*dxy + tp.p5*dyy*nx - tp.p5*dxy*ny - tp.p6*nz*dxy + tp.p7*nz*dyy + tp.p8*nz*dyy*nx - tp.p8*nz*dxy*ny);
		}

		return true;
	}

	private:
	// Attributes
	double _px, _py, _pz, _ww, _bias;
	GridMap* _map;
};
