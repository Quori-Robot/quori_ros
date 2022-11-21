/* Adapted from gazebo_ros_video plugin to display on existing mesh
 * Desc: Video plugin for displaying ROS image topics on Ogre textures
 * Original Author: Piyush Khandelwal
 * Original Date: 26 July 2013
 */

//#define TESTING

#include <gazebo/gazebo.hh>
#include <gazebo/rendering/Visual.hh>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <Ogre.h>
#include <opencv2/opencv.hpp>

using namespace gazebo;
using namespace rendering;

class VideoVisual : public Visual
{
 private:
  Ogre::TexturePtr texture_;
  int height_;
  int width_;

 public:
  VideoVisual(const std::string &name, VisualPtr visual,
	      int height, int width) : 
    Visual(name, visual), height_(height), width_(width)
  {
    std::string vname = visual->Name();
    std::string mname = vname+"_Material", tname = vname+"_Texture";

    texture_ = Ogre::TextureManager::getSingleton().createManual(
	tname, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D, width_, height_, 0, Ogre::PF_BYTE_BGRA,
        Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

    Ogre::MaterialPtr material =
      Ogre::MaterialManager::getSingleton().create(mname, "General");
    material->getTechnique(0)->getPass(0)->createTextureUnitState(tname);
    visual->SetMaterial(mname);
    material->setReceiveShadows(false);
  }

  virtual ~VideoVisual() {};

  void render(const cv::Mat& image)
  {
    // Fix image size if necessary
    const cv::Mat* image_ptr = &image;
    cv::Mat converted_image;
    if (image_ptr->rows != height_ || image_ptr->cols != width_) {
      cv::resize(*image_ptr, converted_image, cv::Size(width_, height_));
      image_ptr = &converted_image;
    }

    // Get the pixel buffer
    Ogre::HardwarePixelBufferSharedPtr pixelBuffer = texture_->getBuffer();

    // Lock the pixel buffer and get a pixel box
    pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
    const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
    uint8_t* pDest = static_cast<uint8_t*>(pixelBox.data);

    memcpy(pDest, image_ptr->data, height_ * width_ * 4);

    // Unlock the pixel buffer
    pixelBuffer->unlock();
  }
};

class FacePlugin : public VisualPlugin
{
 private:
  // Pointer to the link's VideoVisual
  boost::shared_ptr<VideoVisual> video_visual_;

  // Pointer to the update event connection
  event::ConnectionPtr update_connection_;

  Ogre::TexturePtr texture_;
  int height_;
  int width_;

  cv_bridge::CvImagePtr image_;
  boost::mutex m_image_;
  bool new_image_available_;

  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  ros::NodeHandle* rosnode_;
  
  // ROS Stuff
  ros::Subscriber camera_subscriber_;
  std::string robot_namespace_;
  std::string topic_name_;
  
  ros::CallbackQueue queue_;
  boost::thread callback_queue_thread_;

 public:
  FacePlugin();
  ~FacePlugin();
  void Load(VisualPtr visual, sdf::ElementPtr _sdf);
  void processImage(const sensor_msgs::ImageConstPtr &msg);

 private:
  void queueThread();
  void setupROS();
  void updateChild();
#ifdef TESTING
  unsigned int count_;
  void setImage(std::string fname);
#else
  void initImage(int width, int height);
#endif
};

void FacePlugin::queueThread(void)
{
  static const double timeout = 0.01;
  while (rosnode_->ok()) {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void FacePlugin::setupROS(void)
{
  // Initialize the ROS node for the gazebo client if necessary
  if (!ros::isInitialized()) {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
  }
  std::string gazebo_source =
    (ros::this_node::getName() == "/gazebo_client") ? "gzclient" : "gzserver";
  rosnode_ = new ros::NodeHandle(robot_namespace_);

  // Subscribe to the image topic
  ros::SubscribeOptions so =
    ros::SubscribeOptions::create<sensor_msgs::Image>(topic_name_, 1,
			      boost::bind(&FacePlugin::processImage, this, _1),
			      ros::VoidPtr(), &queue_);
  camera_subscriber_ = rosnode_->subscribe(so);

  new_image_available_ = false;

  callback_queue_thread_ = boost::thread(boost::bind(&FacePlugin::queueThread,
						     this));

  update_connection_ =
    event::Events::ConnectPreRender(boost::bind(&FacePlugin::updateChild, this));

  ROS_INFO_NAMED("video", "Face (%s, ns = %s) has started",
		 gazebo_source.c_str(), robot_namespace_.c_str());
}

#ifdef TESTING
void FacePlugin::setImage(std::string fname)
{
  std::string path="~/quori_files/quori_ros/src/quori_description/models/face_model/materials/textures/";  
  cv::Mat image = cv::imread(path + fname, cv::IMREAD_COLOR);
  std::vector<cv::Mat> channels;
  cv::split(image, channels);
  cv::Mat alpha = channels.at(0) + channels.at(1) + channels.at(2);
  channels.push_back(alpha);
  cv::merge(channels, this->image_->image);

  this->new_image_available_ = true;
}
#else
// Set up an initial white face
void FacePlugin::initImage(int width, int height)
{
  cv::Mat white(width, height, CV_8UC4, cv::Scalar(255, 255, 255));
  this->image_->image = white;
  this->new_image_available_ = true;
}
#endif

// Update the controller
void FacePlugin::updateChild(void)
{
  boost::mutex::scoped_lock scoped_lock(m_image_);
  if (this->new_image_available_) {
    video_visual_->render(this->image_->image);
  }
  this->new_image_available_ = false;
#ifdef TESTING
  this->setImage(this->count_%3 == 0 ? "dog.jpg" : 
		 this->count_%3 == 1 ? "dog2.jpg" : "dog3.jpg");
  this->count_++;
#endif
}

void FacePlugin::processImage(const sensor_msgs::ImageConstPtr &msg)
{
  // Get a reference to the image from the image message pointer
  boost::mutex::scoped_lock scoped_lock(m_image_);
  // We get image with alpha channel as it allows memcpy onto ogre texture
  image_ = cv_bridge::toCvCopy(msg, "bgra8");
  new_image_available_ = true;
}

FacePlugin::FacePlugin() :
#ifdef TESTING
  count_(0),
#endif
  image_(cv_bridge::CvImagePtr(new cv_bridge::CvImage())) {}

FacePlugin::~FacePlugin()
{
  update_connection_.reset();

  // Custom Callback Queue
  queue_.clear();
  queue_.disable();
  rosnode_->shutdown();
  callback_queue_thread_.join();

  delete rosnode_;
}

void FacePlugin::Load(VisualPtr visual, sdf::ElementPtr sdf)
{
#ifdef TESTING
  std::cerr << "LOAD FacePlugin! " << visual->Name() << " " << sdf->GetParent()->GetName() << std::endl;
#endif

  sdf::ElementPtr p_sdf;
  p_sdf = (sdf->HasElement("sdf") ? sdf->GetElement("sdf") : sdf);
  robot_namespace_ = (!p_sdf->HasElement("robotNamespace") ? "" :
		      p_sdf->GetElement("robotNamespace")->Get<std::string>());
  if (!p_sdf->HasElement("robotNamespace")) {
    ROS_WARN_NAMED("video", "Face plugin missing <robotNamespace>, "
		   "defaults to \"%s\".", robot_namespace_.c_str());
  }

  topic_name_ = (!p_sdf->HasElement("topicName") ? "face_plugin_image" :
		 p_sdf->GetElement("topicName")->Get<std::string>());
  if (!p_sdf->HasElement("topicName")) {
    ROS_WARN_NAMED("video", "Face Plugin (ns = %s) missing <topicName>, "
		   "defaults to \"%s\".", robot_namespace_.c_str(), topic_name_.c_str());
  }
  int height = (!p_sdf->HasElement("height") ? 240 :
		p_sdf->GetElement("height")->Get<int>());
  if (!p_sdf->HasElement("height")) {
    ROS_WARN_NAMED("video", "Face Plugin (ns = %s) missing <height>, "
		   "defaults to %i.", robot_namespace_.c_str(), height);
  }

  int width = (!p_sdf->HasElement("width") ? 320 :
	       p_sdf->GetElement("width")->Get<int>()); 
  if (!p_sdf->HasElement("width")) {
    ROS_WARN_NAMED("video", "Face Plugin (ns = %s) missing <width>, "
		   "defaults to %i", robot_namespace_.c_str(), width);
  }

  std::string name = robot_namespace_ + "_visual";
  video_visual_.reset(new VideoVisual(name, visual, height, width));

  setupROS();

#ifdef TESTING
  this->count_ = 0;
  this->setImage("dog.jpg");
#else
  initImage(width, height);
#endif
}

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(FacePlugin)

