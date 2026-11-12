/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef VIO_H_
#define VIO_H_

#include "voxel_map.h"
#include "feature.h"
#include <opencv2/imgproc/imgproc_c.h>
#include <pcl/filters/voxel_grid.h>
#include <set>
#include <unordered_set>
#include <vikit/math_utils.h>
#include <vikit/robust_cost.h>
#include <vikit/vision.h>
#include <vikit/pinhole_camera.h>

struct SubSparseMap
{
  vector<float> propa_errors;
  vector<float> errors;
  vector<vector<float>> warp_patch;
  vector<int> search_levels;
  vector<VisualPoint *> voxel_points;
  vector<double> inv_expo_list;
  vector<pointWithVar> add_from_voxel_map;

  SubSparseMap()
  {
    propa_errors.reserve(SIZE_LARGE);
    errors.reserve(SIZE_LARGE);
    warp_patch.reserve(SIZE_LARGE);
    search_levels.reserve(SIZE_LARGE);
    voxel_points.reserve(SIZE_LARGE);
    inv_expo_list.reserve(SIZE_LARGE);
    add_from_voxel_map.reserve(SIZE_SMALL);
  };

  void reset()
  {
    propa_errors.clear();
    errors.clear();
    warp_patch.clear();
    search_levels.clear();
    voxel_points.clear();
    inv_expo_list.clear();
    add_from_voxel_map.clear();
  }
};

class Warp
{
public:
  Matrix2d A_cur_ref;
  int search_level;
  Warp(int level, Matrix2d warp_matrix) : search_level(level), A_cur_ref(warp_matrix) {}
  ~Warp() {}
};

class VOXEL_POINTS
{
public:
  std::vector<VisualPoint *> voxel_points;
  int count;
  VOXEL_POINTS(int num) : count(num) {}
  ~VOXEL_POINTS() 
  { 
    for (VisualPoint* vp : voxel_points) 
    {
      if (vp != nullptr) { delete vp; vp = nullptr; }
    }
  }
};

// A photometric patch sampled at equal angular steps on the viewing sphere.
// Each ray is intersected with the local LiDAR plane and stored in world
// coordinates, so any calibrated camera can observe the same map patch.
struct SphericalPatchPoint
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  V3D center_w;
  VOXEL_LOCATION map_key = VOXEL_LOCATION(0, 0, 0);
  int reference_camera_id = 0;
  int reference_frame_id = 0;
  V3D reference_origin_w;
  V3D normal_w;
  std::vector<V3D> patch_world;
  std::vector<float> intensity;
  std::vector<std::vector<V3D>> coarse_world;
  std::vector<std::vector<float>> coarse_intensity;
  V3D geometry_origin_w;
  std::vector<double> range_variance;
  std::vector<std::vector<double>> coarse_range_variance;
  bool normalized_brightness = false;
  double reference_brightness_gain = 1.0, reference_brightness_bias = 0.0;
};

struct SphericalVisualMap
{
  std::unordered_map<VOXEL_LOCATION, std::shared_ptr<SphericalPatchPoint>> voxels;
  std::deque<VOXEL_LOCATION> insertion_order;
  size_t max_points = 15000;
};

struct SphericalObservation
{
  std::shared_ptr<SphericalPatchPoint> point;
  double score = 0.0;
  double weight = 1.0;
  int spatial_cell = -1;
  MD(6, 6) posterior_information = MD(6, 6)::Zero();
  Eigen::Matrix<double,6,2> brightness_coupling = Eigen::Matrix<double,6,2>::Zero();
  Eigen::Matrix2d brightness_information = Eigen::Matrix2d::Zero();
};

// A fixed support set for trial evaluation. No trial may refit brightness,
// change weights, or drop an inconvenient point to manufacture a lower cost.
struct SphericalCostPatch
{
  std::shared_ptr<const SphericalPatchPoint> point;
  int level = 0;
  double gain = 1.0, bias = 0.0, weight = 1.0;
  std::vector<double> sample_weights;
  double global_gain = 1.0, global_bias = 0.0;
  Eigen::Vector2d local_brightness = Eigen::Vector2d::Zero();
};

class VIOManager
{
public:
  int grid_size;
  vk::AbstractCamera *cam;
  vk::PinholeCamera *pinhole_cam;
  StatesGroup *state;
  StatesGroup *state_propagat;
  M3D Rli, Rci, Rcl, Rcw, Jdphi_dR, Jdp_dt, Jdp_dR;
  V3D Pli, Pci, Pcl, Pcw;
  vector<int> grid_num;
  vector<int> map_index;
  vector<int> border_flag;
  vector<int> update_flag;
  vector<float> map_dist;
  vector<float> scan_value;
  vector<float> patch_buffer;
  bool normal_en, inverse_composition_en, exposure_estimate_en, raycast_en, has_ref_patch_cache;
  bool ncc_en = false, colmap_output_en = false;

  int width, height, grid_n_width, grid_n_height, length;
  int camera_id = 0;
  double image_resize_factor;
  double fx, fy, cx, cy;
  int patch_pyrimid_level, patch_size, patch_size_total, patch_size_half, border, warp_len;
  int max_iterations, total_points;

  double img_point_cov, outlier_threshold, ncc_thre;
  
  SubSparseMap *visual_submap;
  std::vector<std::vector<V3D>> rays_with_sample_points;

  double compute_jacobian_time, update_ekf_time;
  double ave_total = 0;
  // double ave_build_residual_time = 0;
  // double ave_ekf_time = 0;

  int frame_count = 0;
  bool plot_flag;

  Matrix<double, DIM_STATE, DIM_STATE> G, H_T_H;
  MatrixXd K, H_sub_inv;

  ofstream fout_camera, fout_colmap;
  unordered_map<VOXEL_LOCATION, VOXEL_POINTS *> feat_map;
  std::shared_ptr<SphericalVisualMap> spherical_map;
  std::vector<SphericalObservation> spherical_candidates;
  std::vector<SphericalObservation> spherical_selected;
  cv::Mat spherical_gray;
  std::vector<cv::Mat> spherical_pyramid;
  bool spherical_plane_anchor_en = false;
  bool spherical_geometry_support_en = false;
  const std::vector<PointToPlane> *spherical_plane_associations = nullptr;
  int spherical_geometry_fine_rejections = 0;
  int spherical_geometry_coarse_rejections = 0;
  bool spherical_shared_brightness_en = false;
  bool spherical_local_brightness_en = false;
  bool spherical_bounded_brightness_en = false;
  int spherical_brightness_fit_rejections = 0, spherical_brightness_bound_fits = 0;
  std::unordered_map<const SphericalPatchPoint *,Eigen::Vector2d> spherical_fitted_local_brightness;
  double spherical_brightness_gain = 1.0, spherical_brightness_bias = 0.0;
  double spherical_brightness_prior_gain = 1.0, spherical_brightness_prior_bias = 0.0;
  double spherical_fitted_brightness_gain = 1.0, spherical_fitted_brightness_bias = 0.0;
  bool spherical_cache_information = false;
  MD(6,6) spherical_selection_factor = MD(6,6)::Zero();
  int spherical_frame_id = 0;
  unordered_map<VOXEL_LOCATION, int> sub_feat_map; 
  unordered_map<int, Warp *> warp_map;
  vector<VisualPoint *> retrieve_voxel_points;
  vector<pointWithVar> append_voxel_points;
  FramePtr new_frame_;
  cv::Mat img_cp, img_rgb, img_test;

  enum CellType
  {
    TYPE_MAP = 1,
    TYPE_POINTCLOUD,
    TYPE_UNKNOWN
  };

  VIOManager();
  ~VIOManager();
  void updateStateInverse(cv::Mat img, int level);
  void updateState(cv::Mat img, int level);
  void processFrame(cv::Mat &img, vector<pointWithVar> &pg, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &feat_map, double img_time);
  void prepareSphericalFrame(const cv::Mat &img, const vector<pointWithVar> &points,
                             const V3D &weak_world, bool information_selection, int max_probes,
                             double candidate_min_zncc = -1.0, int pyramid_levels = 1,
                             bool effective_information = false, bool gain_only = false,
                             const MD(6, 6) *posterior_factor = nullptr,
                             const std::vector<std::shared_ptr<SphericalPatchPoint>> *temporal_seeds = nullptr);
  void selectSphericalObservations(size_t budget, bool spatial_diversity = false);
  void linearizeSpherical(MD(6, 6) &information, MD(6, 1) &gradient,
                          double &cost, int &valid_patches, bool per_patch_affine,
                          double min_zncc, bool marginalize_affine = false, int level = 0,
                          bool camera_affine = false, bool gain_only = false,
                          std::vector<SphericalCostPatch> *fixed_support = nullptr);
  bool evaluateSphericalCost(const StatesGroup &pose, const std::vector<SphericalCostPatch> &support,
                             double &cost, bool fitted_brightness = false) const;
  void insertSphericalMapPoints(const vector<pointWithVar> &points, int budget, int max_probes);
  int refineSphericalGeometry(const vector<pointWithVar> &points, int max_probes);
  int refreshSphericalReferences(int budget, double min_zncc = 0.9);
  void retrieveFromVisualSparseMap(cv::Mat img, vector<pointWithVar> &pg, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  void generateVisualMapPoints(cv::Mat img, vector<pointWithVar> &pg);
  void setImuToLidarExtrinsic(const V3D &transl, const M3D &rot);
  void setLidarToCameraExtrinsic(vector<double> &R, vector<double> &P);
  void initializeVIO();
  void getImagePatch(cv::Mat img, V2D pc, float *patch_tmp, int level);
  void computeProjectionJacobian(V3D p, MD(2, 3) & J);
  void computeJacobianAndUpdateEKF(cv::Mat img);
  void resetGrid();
  void updateVisualMapPoints(cv::Mat img);
  void getWarpMatrixAffine(const vk::AbstractCamera &cam, const Vector2d &px_ref, const Vector3d &f_ref, const double depth_ref, const SE3 &T_cur_ref,
                           const int level_ref, 
                           const int pyramid_level, const int halfpatch_size, Matrix2d &A_cur_ref);
  void getWarpMatrixAffineHomography(const vk::AbstractCamera &cam, const V2D &px_ref,
                                     const V3D &xyz_ref, const V3D &normal_ref, const SE3 &T_cur_ref, const int level_ref, Matrix2d &A_cur_ref);
  void warpAffine(const Matrix2d &A_cur_ref, const cv::Mat &img_ref, const Vector2d &px_ref, const int level_ref, const int search_level,
                  const int pyramid_level, const int halfpatch_size, float *patch);
  void insertPointIntoVoxelMap(VisualPoint *pt_new);
  void plotTrackedPoints();
  void updateFrameState(StatesGroup state);
  void projectPatchFromRefToCur(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  void updateReferencePatch(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  void precomputeReferencePatches(int level);
  void dumpDataForColmap();
  double calculateNCC(float *ref_patch, float *cur_patch, int patch_size);
  int getBestSearchLevel(const Matrix2d &A_cur_ref, const int max_level);
  V3F getInterpolatedPixel(cv::Mat img, V2D pc);
  
  // void resetRvizDisplay();
  // deque<VisualPoint *> map_cur_frame;
  // deque<VisualPoint *> sub_map_ray;
  // deque<VisualPoint *> sub_map_ray_fov;
  // deque<VisualPoint *> visual_sub_map_cur;
  // deque<VisualPoint *> visual_converged_point;
  // std::vector<std::vector<V3D>> sample_points;

  // PointCloudXYZI::Ptr pg_down;
  // pcl::VoxelGrid<PointType> downSizeFilter;
};
typedef std::shared_ptr<VIOManager> VIOManagerPtr;

#endif // VIO_H_
