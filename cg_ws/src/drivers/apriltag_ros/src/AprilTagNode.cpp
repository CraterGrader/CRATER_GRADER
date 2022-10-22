#include <AprilTagNode.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>


#include <tagStandard41h12.h>
#include <Eigen/Dense>

// create and delete functions for default tags
#define TAG_CREATE(name) { #name, tag##name##_create },
#define TAG_DESTROY(name) { #name, tag##name##_destroy },

const std::map<std::string, apriltag_family_t *(*)(void)> AprilTagNode::tag_create =
{
    TAG_CREATE(Standard41h12)
};

const std::map<std::string, void (*)(apriltag_family_t*)> AprilTagNode::tag_destroy =
{
    TAG_DESTROY(Standard41h12)
};

AprilTagNode::AprilTagNode(rclcpp::NodeOptions options)
  : Node("apriltag", "apriltag", options.use_intra_process_comms(true)),
    // parameter
    td(apriltag_detector_create()),
    tag_family(declare_parameter<std::string>("family", "36h11")),
    tag_edge_size(declare_parameter<double>("size", 2.0)),
    max_hamming(declare_parameter<int>("max_hamming", 0)),
    z_up(declare_parameter<bool>("z_up", false)),
    // topics
    pub_tf(create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100))),
    pub_detections(create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("detections", rclcpp::QoS(1)))
{
    td->quad_decimate = declare_parameter<float>("decimate", 1.0);
    td->quad_sigma =    declare_parameter<float>("blur", 0.0);
    td->nthreads =      declare_parameter<int>("threads", 1);
    td->debug =         declare_parameter<int>("debug", false);
    td->refine_edges =  declare_parameter<int>("refine-edges", true);

    // get tag names, IDs and sizes
    const auto ids = declare_parameter<std::vector<int64_t>>("tag_ids", std::vector<int64_t>{});
    const auto frames = declare_parameter<std::vector<std::string>>("tag_frames", std::vector<std::string>{});

    if(!frames.empty()) {
        if(ids.size()!=frames.size()) {
            throw std::runtime_error("Number of tag ids ("+std::to_string(ids.size())+") and frames ("+std::to_string(frames.size())+") mismatch!");
        }
        for(size_t i = 0; i<ids.size(); i++) { tag_frames[ids[i]] = frames[i]; }
    }

    const auto sizes = declare_parameter<std::vector<double>>("tag_sizes", std::vector<double>{});
    if(!sizes.empty()) {
        // use tag specific size
        if(ids.size()!=sizes.size()) {
            throw std::runtime_error("Number of tag ids ("+std::to_string(ids.size())+") and sizes ("+std::to_string(sizes.size())+") mismatch!");
        }
        for(size_t i = 0; i<ids.size(); i++) { tag_sizes[ids[i]] = sizes[i]; }
    }

    if(tag_create.count(tag_family)) {
        tf = tag_create.at(tag_family)();
        apriltag_detector_add_family(td, tf);
    }
    else {
        throw std::runtime_error("Unsupported tag family: "+tag_family);
    }
    sun_cam = this->create_subscription<sensor_msgs::msg::Image>("/image_raw", 30, std::bind(&AprilTagNode::onCamera, this, std::placeholders::_1));
    K << 535.40740544, 0., 944.90571238, 0., 536.25014079, 487.50675226, 0., 0., 1.;
}

AprilTagNode::~AprilTagNode() {
    apriltag_detector_destroy(td);
    tag_destroy.at(tag_family)(tf);
}

//void AprilTagNode::onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci) {
void AprilTagNode::onCamera(const sensor_msgs::msg::Image & msg_img) {
    // convert to 8bit monochrome image
    const cv::Mat img_uint8 = cv_bridge::toCvCopy(msg_img, "mono8")->image;

    image_u8_t im = {
        .width = img_uint8.cols,
        .height = img_uint8.rows,
        .stride = img_uint8.cols,
        .buf = img_uint8.data
    };

    // detect tags
    zarray_t* detections = apriltag_detector_detect(td, &im);

    apriltag_msgs::msg::AprilTagDetectionArray msg_detections;
    msg_detections.header = msg_img.header;

    tf2_msgs::msg::TFMessage tfs;

    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        // ignore untracked tags
        if(!tag_frames.empty() && !tag_frames.count(det->id)) { continue; }

        // reject detections with more corrected bits than allowed
        if(det->hamming>max_hamming) { continue; }

        // detection
        apriltag_msgs::msg::AprilTagDetection msg_detection;
        msg_detection.family = std::string(det->family->name);
        msg_detection.id = det->id;
        msg_detection.hamming = det->hamming;
        msg_detection.decision_margin = det->decision_margin;
        msg_detection.centre.x = det->c[0];
        msg_detection.centre.y = det->c[1];
        std::memcpy(msg_detection.corners.data(), det->p, sizeof(double)*8);
        std::memcpy(msg_detection.homography.data(), det->H->data, sizeof(double)*9);
        msg_detections.detections.push_back(msg_detection);

        // 3D orientation and position
        geometry_msgs::msg::TransformStamped tf;
        tf.header = msg_img.header;
        // set child frame name by generic tag name or configured tag name
        tf.child_frame_id = tag_frames.count(det->id) ? tag_frames.at(det->id) : std::string(det->family->name)+":"+std::to_string(det->id);
        getPose(*(det->H), tf.transform, tag_sizes.count(det->id) ? tag_sizes.at(det->id) : tag_edge_size);

        tfs.transforms.push_back(tf);
    }

    pub_detections->publish(msg_detections);
    pub_tf->publish(tfs);

    apriltag_detections_destroy(detections);
}

void AprilTagNode::getPose(const matd_t& H, geometry_msgs::msg::Transform& t, const double size) const {

    const Eigen::Map<const Mat3>Hm(H.data);

    // compute extrinsic camera parameter
    // https://dsp.stackexchange.com/a/2737/31703
    // H = K * T  =>  T = K^(-1) * H
    const Mat3 T = K.inverse() * Hm / Hm(2,2);
    Mat3 R;
    R.col(0) = T.col(0).normalized();
    R.col(1) = T.col(1).normalized();
    R.col(2) = R.col(0).cross(R.col(1));

    if(z_up) {
        // rotate by half rotation about x-axis
        R.col(1) *= -1;
        R.col(2) *= -1;
    }

    // the corner coordinates of the tag in the canonical frame are (+/-1, +/-1)
    // hence the scale is half of the edge size
    const Eigen::Vector3d tt = T.rightCols<1>() / ((T.col(0).norm() + T.col(0).norm())/2.0) * (size/2.0);

    const Eigen::Quaterniond q(R);

    t.translation.x = tt.x();
    t.translation.y = tt.y();
    t.translation.z = tt.z();
    t.rotation.w = q.w();
    t.rotation.x = q.x();
    t.rotation.y = q.y();
    t.rotation.z = q.z();
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(AprilTagNode)
