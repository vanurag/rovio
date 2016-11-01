#include "rovio/OptimizeRovio.hpp"


RovioOptimizer::RovioOptimizer(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::string& bagFileName, const std::string& groundTruthTopic,
                               const std::string& rovioCam0Topic, const std::string& rovioCam1Topic, const std::string& rovioImuTopic)
  : nh_(nh), nh_private_(nh_private), rovioFilter_(new mtFilter),
    ground_truth_topic_(groundTruthTopic),
    rovio_cam0_topic_(rovioCam0Topic), rovio_cam1_topic_(rovioCam1Topic), rovio_imu_topic_(rovioImuTopic)
{
  // ROVIO
  // Filter
  std::string rootdir = ros::package::getPath("rovio"); // Leaks memory
  filter_config = rootdir + "/cfg/rovio.info";
  optimized_filter_config = rootdir + "/cfg/rovio_optimized.info";
  nh_private_.param("filter_config", filter_config, filter_config);
  nh_private_.param("optimized_filter_config", optimized_filter_config, optimized_filter_config);
  if (nh_private_.hasParam("filter_config")) {
    nh_private_.getParam("filter_config", filter_config);
  }
  if (nh_private_.hasParam("optimized_filter_config")) {
    nh_private_.getParam("optimized_filter_config", optimized_filter_config);
  }
  // Filter
  rovioFilter_->readFromInfo(filter_config);
  // Force the camera calibration paths to the ones from ROS parameters.
  for (unsigned int camID = 0; camID < ROVIO_NCAM; ++camID) {
    std::string camera_config;
    if (nh_private_.getParam("camera" + std::to_string(camID)
                              + "_config", camera_config)) {
      rovioFilter_->cameraCalibrationFile_[camID] = camera_config;
    }
  }
  rovioFilter_->refreshProperties();
  // Node
  rovioNode_ = new rovio::RovioNode<mtFilter>(nh_, nh_private_, rovioFilter_, false);
  rovioNode_->makeTest();

  // BAG contents
  rovio_cam0_msg_ = nullptr;
  rovio_cam1_msg_ = nullptr;
  ground_truth_msg_ = nullptr;

  bag_.open(bagFileName, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(ground_truth_topic_);
  topics.push_back(rovio_cam0_topic_);
  topics.push_back(rovio_cam1_topic_);
  topics.push_back(rovio_imu_topic_);

  bag_view_.addQuery(bag_, rosbag::TopicQuery(topics));
  current_bag_pos_ = bag_view_.begin();
  num_associations = 0;

  latest_groundtruth_time_ = ros::Time(0);
}

void printPTree(boost::property_tree::ptree const& pt)
{
  using boost::property_tree::ptree;
  for (ptree::const_iterator it = pt.begin(); it != pt.end(); ++it) {
      std::cout << it->first << ": " << it->second.get_value<std::string>() << std::endl;
      printPTree(it->second);
  }
}

// NLOPT Objective function
double testNloptFunc(unsigned n, const double* x, double* grad, void* my_func_data) {

  // Get ROVIO Optimizer member
  RovioOptimizer* optimizer = (RovioOptimizer*)my_func_data;

  // current set of parameters
  double score = std::numeric_limits<double>::infinity();
  std::cout << "Current Parameters: " << std::endl;
  for (int i = 0; i < n; ++i) {
    std::cout << x[i] << ", " << std::endl;
  }
  std::cout << std::endl;


  // Update ROVIO parameters
  //std::cout << "check: " << optimizer->rovioNode_->mpFilter_->mPrediction_.prenoiP_ << std::endl;
  boost::property_tree::ptree pt;
  //optimizer->rovioNode_->mpFilter_->buildPropertyTree(pt);
  //std::cout << "p tree from build: " << std::endl;
  //printPTree(pt);
  //std::cout << "p tree from info file: " << std::endl;
  boost::property_tree::info_parser::read_info(optimizer->filter_config, pt);
  //printPTree(ptD);

  pt.put("Prediction.PredictionNoise.pos_0", x[0]);
  pt.put("Prediction.PredictionNoise.pos_1", x[1]);
  pt.put("Prediction.PredictionNoise.pos_2", x[2]);
  pt.put("Prediction.PredictionNoise.vel_0", x[3]);
  pt.put("Prediction.PredictionNoise.vel_1", x[4]);
  pt.put("Prediction.PredictionNoise.vel_2", x[5]);
  pt.put("Prediction.PredictionNoise.acb_0", x[6]);
  pt.put("Prediction.PredictionNoise.acb_1", x[7]);
  pt.put("Prediction.PredictionNoise.acb_2", x[8]);
  pt.put("Prediction.PredictionNoise.gyb_0", x[9]);
  pt.put("Prediction.PredictionNoise.gyb_1", x[10]);
  pt.put("Prediction.PredictionNoise.gyb_2", x[11]);
  pt.put("Prediction.PredictionNoise.vep", x[12]);
  pt.put("Prediction.PredictionNoise.att_0", x[13]);
  pt.put("Prediction.PredictionNoise.att_1", x[14]);
  pt.put("Prediction.PredictionNoise.att_2", x[15]);
  pt.put("Prediction.PredictionNoise.vea", x[16]);
  pt.put("Prediction.PredictionNoise.dep", x[17]);
  pt.put("Prediction.PredictionNoise.nor", x[18]);

  //std::cout << "access: " << ptD.get<double>("Prediction.PredictionNoise.pos_0") << std::endl;
  optimizer->rovioNode_->mpFilter_->readPropertyTree(pt);
  optimizer->rovioNode_->mpFilter_->refreshProperties();
  for(typename std::unordered_map<std::string,LWF::PropertyHandler*>::iterator it=optimizer->rovioNode_->mpFilter_->subHandlers_.begin();
      it != optimizer->rovioNode_->mpFilter_->subHandlers_.end(); ++it){
    it->second->refreshProperties();
  }


  // Run ROVIO on all datasets multiple times
  optimizer->current_bag_pos_ = optimizer->bag_view_.begin();
  optimizer->num_associations = 0;

  // RESET Rovio: Doesn't work as expected...
//  optimizer->rovioNode_->mpFilter_->reset(0.0);
//  optimizer->rovioNode_->isInitialized_ = false;

  for (; optimizer->current_bag_pos_ != optimizer->bag_view_.end(); ++optimizer->current_bag_pos_) {
    rosbag::MessageInstance& message_view = *optimizer->current_bag_pos_;

    // Read ROVIO Cam0 message
    if (message_view.getTopic() == optimizer->rovio_cam0_topic_) {
      optimizer->rovio_cam0_msg_ = message_view.instantiate<sensor_msgs::Image>();
      if(optimizer->rovio_cam0_msg_ == nullptr) {
        std::cout << "Wrong type on topic " << optimizer->rovio_cam0_topic_
            << ", expected sensor_msgs::Image" << std::endl;
        exit(1);
      }
      // callback
      optimizer->rovioNode_->imgCallback0(optimizer->rovio_cam0_msg_);
    }

    // Read ROVIO Cam1 message
    if (message_view.getTopic() == optimizer->rovio_cam1_topic_ && ROVIO_NCAM > 1) {
      optimizer->rovio_cam1_msg_ = message_view.instantiate<sensor_msgs::Image>();
      if(optimizer->rovio_cam1_msg_ == nullptr) {
        std::cout << "Wrong type on topic " << optimizer->rovio_cam1_topic_
            << ", expected sensor_msgs::Image" << std::endl;
        exit(1);
      }
      // callback
      optimizer->rovioNode_->imgCallback1(optimizer->rovio_cam1_msg_);
    }

    // Read ROVIO IMU message
    if (message_view.getTopic() == optimizer->rovio_imu_topic_) {
      optimizer->rovio_imu_msg_ = message_view.instantiate<sensor_msgs::Imu>();
      if(optimizer->rovio_imu_msg_ == nullptr) {
        std::cout << "Wrong type on topic " << optimizer->rovio_imu_topic_
            << ", expected sensor_msgs::Imu" << std::endl;
        exit(1);
      }
      // callback
      optimizer->rovioNode_->imuCallback(optimizer->rovio_imu_msg_);
    }

    // Read ground truth message
    if (message_view.getTopic() == optimizer->ground_truth_topic_) {
      optimizer->ground_truth_msg_ = message_view.instantiate<geometry_msgs::TransformStamped>();
      if(optimizer->ground_truth_msg_ == nullptr) {
        std::cout << "Wrong type on topic " << optimizer->ground_truth_topic_
            << ", expected geometry_msgs::TransformStamped" << std::endl;
        exit(1);
      }
      optimizer->latest_groundtruth_time_ = optimizer->ground_truth_msg_->header.stamp;
      optimizer->rovioNode_->groundtruthCallback(optimizer->ground_truth_msg_);
    }

    // Ealuate score
    optimizer->latest_estimate_time_ = ros::Time(optimizer->rovioNode_->mpFilter_->safe_.t_);
    ros::Duration delta(optimizer->latest_groundtruth_time_ - optimizer->latest_estimate_time_);
    if (abs(delta.toSec()) < 0.05) {  // valid association
      optimizer->num_associations += 1;
      //std::cout << "found association" << std::endl << std::endl;
      if (score == std::numeric_limits<double>::infinity()) {
        score = optimizer->evaluationCriterion();
        //std::cout << "score init: " << score << std::endl;
      } else {
        score = score + optimizer->evaluationCriterion();
        //std::cout << "score: " << score << std::endl;
      }
    }
  }

  // Evaluate performance (RMS error in cm)
  if (optimizer->num_associations > 0) score = 100*sqrt(score/optimizer->num_associations);
  std::cout << "ROVIO performance: " << score << std::endl;

  // Delete and Re-create ROVIO node
  std::cout << "Resetting ROVIO..." << std::endl;
  if (optimizer->rovioNode_ != NULL) delete optimizer->rovioNode_;
  if (optimizer->rovioFilter_ != NULL) optimizer->rovioFilter_.reset();
  // Filter
  optimizer->rovioFilter_ = std::make_shared<mtFilter>();
  optimizer->rovioFilter_->readFromInfo(optimizer->filter_config);
  // Force the camera calibration paths to the ones from ROS parameters.
  for (unsigned int camID = 0; camID < ROVIO_NCAM; ++camID) {
    std::string camera_config;
    if (optimizer->nh_private_.getParam("camera" + std::to_string(camID)
                                        + "_config", camera_config)) {
      optimizer->rovioFilter_->cameraCalibrationFile_[camID] = camera_config;
    }
  }
  optimizer->rovioFilter_->refreshProperties();
  optimizer->rovioNode_ = new rovio::RovioNode<mtFilter>(optimizer->nh_, optimizer->nh_private_, optimizer->rovioFilter_, false);
  std::cout << std::endl << std::endl;

  return score;
}

// Evaluate performance and return performance measure
double RovioOptimizer::evaluationCriterion() {

  rovio::RovioNode<mtFilter>::mtState& state = rovioNode_->mpFilter_->safe_.state_;
  // qIM (from groundtruth) = qIV*qVM
  V3D tIM_mocap = rovioNode_->poseUpdateMeas_.pos() - rovioNode_->poseUpdateMeas_.att().inverted().rotate(rovioNode_->mpPoseUpdate_->get_qVM(state).rotate(rovioNode_->mpPoseUpdate_->get_MrMV(state)));
  QPD qIM_mocap = rovioNode_->poseUpdateMeas_.att().inverted()*rovioNode_->mpPoseUpdate_->get_qVM(state);
//  std::cout << "mocap pos: " << tIM_mocap << std::endl;
  tf_t_.setOrigin(tf::Vector3(tIM_mocap(0), tIM_mocap(1), tIM_mocap(2)));
  tf::Quaternion tf_q1(qIM_mocap.x(), qIM_mocap.y(), qIM_mocap.z(), qIM_mocap.w());
  tf_t_.setRotation(tf_q1);
  br_.sendTransform(tf::StampedTransform(tf_t_, ros::Time::now(), "world", "mocap"));

  // qIM (from rovio) = qIW*qWM
  V3D tIM_vio = rovioNode_->mpPoseUpdate_->get_qWI(state).inverted().rotate(state.WrWM()) + rovioNode_->mpPoseUpdate_->get_IrIW(state);
  QPD qIM_vio = rovioNode_->mpPoseUpdate_->get_qWI(state).inverted()*state.qWM();
//  std::cout << "vio pos: " << tIM_vio << std::endl;

  tf_t_.setOrigin(tf::Vector3(tIM_vio(0), tIM_vio(1), tIM_vio(2)));
  tf::Quaternion tf_q2(qIM_vio.x(), qIM_vio.y(), qIM_vio.z(), qIM_vio.w());
  tf_t_.setRotation(tf_q2);
  br_.sendTransform(tf::StampedTransform(tf_t_, ros::Time::now(), "world", "vio"));


//  std::cout << "mocap pos: " << tIM_mocap << std::endl;
  return (tIM_mocap-tIM_vio).norm();
}

// ROVIO optimization routine using NLOPT
void RovioOptimizer::optimizeRovioUsingNlopt() {
  // bounds
  double lb[19] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  double ub[19] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};

  nlopt_opt opt;
  //opt = nlopt_create(NLOPT_LN_BOBYQA, 17);
  opt = nlopt_create(NLOPT_GN_ISRES, 19);
  nlopt_set_lower_bounds(opt, lb);
  nlopt_set_upper_bounds(opt, ub);
  nlopt_set_min_objective(opt, testNloptFunc, this);
  //nlopt_set_ftol_abs(opt, 0.0001);
  nlopt_set_maxtime(opt, 60*15);    //less than 15 hours

  // initial values
  double x[19] = { 1e-4,1e-4,1e-4,4e-5,4e-5,4e-5,1e-8,1e-8,1e-8,3.8e-7,3.8e-7,3.8e-7,1e-8,7.6e-7,7.6e-7,7.6e-7,1e-8,0.0001,0.00001};

  double minf; /* the minimum objective value, upon return */

  nlopt_result res = nlopt_optimize(opt, x, &minf);

  if (res < 0) {
    std::cout << "NLOPT failed!" << std::endl;
    std::cout << "Return Code: " << res << std::endl;
  } else {
    std::cout << "Found minimum with best score: " << minf << std::endl;
    rovioNode_->mpFilter_->writeToInfo(optimized_filter_config);
    std::cout << "Saved optimal parameters to: " << optimized_filter_config << std::endl;
  }

  nlopt_destroy(opt);

}

int main(int argc, char** argv){
  ros::init(argc, argv, "rovio_optimizer");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string bagfile, groundtruth_topic, cam0_topic, cam1_topic, imu_topic;
  nh_private.getParam("bagfile", bagfile);
  nh_private.getParam("groundtruth_topic", groundtruth_topic);
  nh_private.getParam("cam0_topic", cam0_topic);
  nh_private.getParam("cam1_topic", cam1_topic);
  nh_private.getParam("imu_topic", imu_topic);

  RovioOptimizer optimizer(nh, nh_private, bagfile, groundtruth_topic, cam0_topic, cam1_topic, imu_topic);

  optimizer.optimizeRovioUsingNlopt();

  return 0;
}
