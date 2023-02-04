#include <direct_map_localization/common.hpp>

class TrilinearParameters{
  public:
  TrilinearParameters(){
    p1 = p2 = p3 = p4 = p5 = p6 = p7 = p8 = 0.0;
  };

  inline double interpolate(double x, double y, double z){
    return p1 + p2*x + p3*y + p4*z + p5*x*y + p6*x*z + p7*y*z + p8*x*y*z;
  };

  double p1, p2, p3, p4, p5, p6, p7, p8;
};

class GridMap{
  public:
  // Class Constructor
  GridMap(ros::NodeHandle& nh);

  // Class Destructor
  ~GridMap();

  double getDistance(double px, double py, double pz);
  TrilinearParameters getTrilinearParameters(double px, double py, double pz);

  private:
  std::string map_name;
  std::string grid_name;
  bool debug_flag;

  int m_gridSize_x, m_gridSize_y, m_gridSize_z, m_gridSize;
  int m_gridStep_y, m_gridStep_z;
  double m_max_x, m_max_y, m_max_z, m_min_x, m_min_y, m_min_z, m_res, m_ext;
  uint16_t *m_grid;

	int max_nn_threads;

  pcl::PointCloud<pcl::PointXYZ> pc;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  ros::Publisher visPublisher;

  void publishMap();
  void computeGrid();
  void computePartialGrid(int init_z, int end_z, pcl::KdTreeFLANN<pcl::PointXYZ> kt);
  bool loadGrid();
  void saveGrid();
  inline int point2grid(const float &x, const float &y, const float &z);
  inline bool isIntoMap(double x, double y, double z);
};