#include <gpg/cloud_camera.h>
#include <gpg/candidates_generator.h>
#include <gpg/config_file.h>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <thread>

namespace py = pybind11;

std::vector<Grasp> grasp_generation(const Eigen::MatrixXf &pc, const int num_samples, const bool show_grasp, const std::string gripper_config_file) {
    CandidatesGenerator::Parameters generator_params;
    HandSearch::Parameters hand_search_params;

    // Read parameters from configuration file.
    ConfigFile config_file(gripper_config_file);
    double finger_width = config_file.getValueOfKey<double>("finger_width", 0.01);
    double hand_outer_diameter = config_file.getValueOfKey<double>("hand_outer_diameter", 0.12);
    double hand_depth = config_file.getValueOfKey<double>("hand_depth", 0.06);
    double hand_height = config_file.getValueOfKey<double>("hand_height", 0.02);
    double init_bite = config_file.getValueOfKey<double>("init_bite", 0.01);
    int max_grasp_num = config_file.getValueOfKey<int>("max_grasp_num", 0);
    // Read hand geometry parameters.
    hand_search_params.finger_width_ = finger_width;
    hand_search_params.hand_outer_diameter_ = hand_outer_diameter;
    hand_search_params.hand_depth_ = hand_depth;
    hand_search_params.hand_height_ = hand_height;
    hand_search_params.init_bite_ = init_bite;

    // Read local hand search parameters.
    hand_search_params.nn_radius_frames_ = 0.01;
    hand_search_params.num_orientations_ = 8;
    hand_search_params.num_samples_ = num_samples;
    hand_search_params.num_threads_ = std::thread::hardware_concurrency();
    hand_search_params.rotation_axis_ = 2; // cannot be changed

    generator_params.num_samples_ = hand_search_params.num_samples_;
    generator_params.num_threads_ = hand_search_params.num_threads_;
    generator_params.max_grasp_num_ = max_grasp_num;
    // Read plotting parameters.
    generator_params.plot_grasps_ = show_grasp;
    generator_params.plot_normals_ = false;

    // Read preprocessing parameters
    generator_params.remove_statistical_outliers_ = true;
    generator_params.voxelize_ = true;
    generator_params.workspace_ = std::vector<double>{-10.0, 10.0, -10.0, 10.0, -10.0, 10.0};

    Eigen::Vector3d view_point_;           ///< (input) view point of the camera onto the point cloud
    CloudCamera *cloud_camera_;            ///< stores point cloud with (optional) camera information and surface normals
    CandidatesGenerator *candidates_generator_;

    Eigen::Matrix3Xd view_points(3, 1);
    view_points << 0.0, 0.0, 0.0;
    PointCloudRGBA::Ptr cloud(new PointCloudRGBA);
    pcl::PointXYZRGBA p;
    long r = pc.rows();
    for (int i = 0; i < r; ++i) {
        p.x = pc(i, 0);
        p.y = pc(i, 1);
        p.z = pc(i, 2);
        cloud->push_back(p);
    }

    cloud_camera_ = new CloudCamera(cloud, 0, view_points);

    // detect grasps in the point cloud
    candidates_generator_ = new CandidatesGenerator(generator_params, hand_search_params);

    // Preprocess the point cloud: voxelization, removing statistical outliers, workspace filtering.
    candidates_generator_->preprocessPointCloud(*cloud_camera_);

    std::vector<Grasp> grasps = candidates_generator_->generateGraspCandidates(*cloud_camera_);
    return grasps;
}

std::vector<Grasp> generate_grasps(Eigen::MatrixXf &pc, int num_samples, bool show_grasp, std::string gripper_config_file) {
    Eigen::MatrixXf ret;
    // seed the random number generator
    std::srand(std::time(nullptr));
    std::vector<Grasp> grasps = grasp_generation(pc, num_samples, show_grasp, gripper_config_file);
    return grasps;
}

PYBIND11_MODULE(pygpg, m) {
    m.doc() = "python gpg";
    m.def("generate_grasps", &generate_grasps);

    py::class_<Grasp>(m, "Grasp")
            .def(py::init<>())
            .def("get_grasp_bottom", &Grasp::getGraspBottom, "get grasp bottom")
            .def("get_grasp_top", &Grasp::getGraspTop, "get grasp top")
            .def("get_grasp_surface", &Grasp::getGraspSurface, "get grasp surface")
            .def("get_grasp_approach", &Grasp::getApproach, "get grasp approach")
            .def("get_grasp_binormal", &Grasp::getBinormal, "get grasp binormal")
            .def("get_grasp_axis", &Grasp::getAxis, "get grasp axis")
            .def("get_grasp_width", &Grasp::getGraspWidth, "get grasp width")
      ;
}
