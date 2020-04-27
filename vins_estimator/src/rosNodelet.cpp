#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"
#include "utility/tic_toc.h"

#include <boost/thread.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "depth_generation/depth_camera_manager.h"
#include "vins/FlattenImages.h"


namespace vins_nodelet_pkg
{
    class VinsNodeletClass : public nodelet::Nodelet
    {
        public:
            VinsNodeletClass() {}
        private:
            message_filters::Subscriber<sensor_msgs::Image> * image_sub_l;
            message_filters::Subscriber<sensor_msgs::Image> * image_sub_r;

            DepthCamManager * cam_manager = nullptr;
            virtual void onInit()
            {
                auto n = getNodeHandle();
                auto private_n = getPrivateNodeHandle();
                std::string config_file;
                private_n.getParam("config_file", config_file);
                
                std::cout << "config file is " << config_file << '\n';
                readParameters(config_file);
                estimator.setParameter();
                
                if (ENABLE_DEPTH) {
                    cam_manager = new DepthCamManager(n, &(estimator.featureTracker.fisheys_undists[0]));
                    cam_manager -> init_with_extrinsic(estimator.ric[0], estimator.tic[0], estimator.ric[1], estimator.tic[1]);
                    estimator.depth_cam_manager = cam_manager;
                }
            #ifdef EIGEN_DONT_PARALLELIZE
                ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
            #endif

                ROS_WARN("waiting for image and imu...");

                registerPub(n);

                sub_imu = n.subscribe(IMU_TOPIC, 2000, &VinsNodeletClass::imu_callback, this);
                sub_feature = n.subscribe("/feature_tracker/feature", 2000, &VinsNodeletClass::feature_callback, this);
                sub_restart = n.subscribe("/vins_restart", 100, &VinsNodeletClass::restart_callback, this);

                if(!FISHEYE_EXTERNAL_FLATTEN) {
                    image_sub_l = new message_filters::Subscriber<sensor_msgs::Image> (n, IMAGE0_TOPIC, 100);
                    image_sub_r = new message_filters::Subscriber<sensor_msgs::Image> (n, IMAGE1_TOPIC, 100);
                    sync = new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> (*image_sub_l, *image_sub_r, 10);
                    sync->registerCallback(boost::bind(&VinsNodeletClass::img_callback, this, _1, _2));
                } else {
                    ROS_INFO("Will directly receive flattened images");
                    flatten_sub = n.subscribe("/vins_estimator/flattened_raw", 2000, &VinsNodeletClass::flatten_callback, this);
                }
            }

            void flatten_callback(const vins::FlattenImagesConstPtr & flattend_raw) {
                ROS_INFO("Recevied flattened images");
                TicToc tic;
                vector<cv::Mat> up_cameras, down_cameras;
                for (auto & cam : flattend_raw->up_cams) {
                    auto img = getImageFromMsg(cam);
                    up_cameras.push_back(img->image);
                }

                for (auto & cam : flattend_raw->down_cams) {
                    auto img = getImageFromMsg(cam);
                    down_cameras.push_back(img->image);
                }
                double decode_time = tic.toc();
                TicToc tic_input;
                ROS_INFO("Will track %ld %ld", up_cameras.size(), down_cameras.size());
                estimator.inputImage(flattend_raw->header.stamp.toSec(), cv::Mat(), cv::Mat(), up_cameras, down_cameras);

                ROS_INFO("Decode: %fms. Input Image: %fms", decode_time, tic_input.toc());
            }

            void img_callback(const sensor_msgs::ImageConstPtr &img1_msg, const sensor_msgs::ImageConstPtr &img2_msg)
            {
                auto img1 = getImageFromMsg(img1_msg);
                auto img2 = getImageFromMsg(img2_msg);
                estimator.inputImage(img1_msg->header.stamp.toSec(), img1->image, img2->image);
            }

            cv_bridge::CvImageConstPtr getImageFromMsg(const sensor_msgs::Image &img_msg)
            {
                cv_bridge::CvImageConstPtr ptr;
                if (img_msg.encoding == "8UC1")
                {
                    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
                } else {
                    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
                }
                return ptr;
            }
            
            cv_bridge::CvImageConstPtr getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
            {
                cv_bridge::CvImageConstPtr ptr;
                std::cout << img_msg->encoding << std::endl;
                if (img_msg->encoding == "8UC1")
                {
                    sensor_msgs::Image img;
                    img.header = img_msg->header;
                    img.height = img_msg->height;
                    img.width = img_msg->width;
                    img.is_bigendian = img_msg->is_bigendian;
                    img.step = img_msg->step;
                    img.data = img_msg->data;
                    img.encoding = "mono8";
                    ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
                }
                else
                {
                    if (FISHEYE) {
                        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
                    } else {
                        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);        
                    }
                }
                return ptr;
            }

            void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
            {
                double t = imu_msg->header.stamp.toSec();
                double dx = imu_msg->linear_acceleration.x;
                double dy = imu_msg->linear_acceleration.y;
                double dz = imu_msg->linear_acceleration.z;
                double rx = imu_msg->angular_velocity.x;
                double ry = imu_msg->angular_velocity.y;
                double rz = imu_msg->angular_velocity.z;
                Vector3d acc(dx, dy, dz);
                Vector3d gyr(rx, ry, rz);
                estimator.inputIMU(t, acc, gyr);

                // test, should be deleted
                if (! last_time_initialized)
                {
                    last_time = ros::Time::now().toSec();
                    last_time_initialized = true;
                }
                else
                {
                    double now_time = ros::Time::now().toSec();
                    if (now_time - last_time > 3)
                        ros::shutdown();
                    last_time = now_time;
                }
                // test end
            }

            void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
            {
                FeatureFrame featureFrame;
                for (unsigned int i = 0; i < feature_msg->points.size(); i++)
                {
                    int feature_id = feature_msg->channels[0].values[i];
                    int camera_id = feature_msg->channels[1].values[i];
                    double x = feature_msg->points[i].x;
                    double y = feature_msg->points[i].y;
                    double z = feature_msg->points[i].z;
                    double p_u = feature_msg->channels[2].values[i];
                    double p_v = feature_msg->channels[3].values[i];
                    double velocity_x = feature_msg->channels[4].values[i];
                    double velocity_y = feature_msg->channels[5].values[i];
                    if(feature_msg->channels.size() > 5)
                    {
                        double gx = feature_msg->channels[6].values[i];
                        double gy = feature_msg->channels[7].values[i];
                        double gz = feature_msg->channels[8].values[i];
                        pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
                        //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
                    }
                    ROS_ASSERT(z == 1);
                    TrackFeatureNoId xyz_uv_velocity;
                    xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y, 0;
                    featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
                }
                double t = feature_msg->header.stamp.toSec();
                estimator.inputFeature(t, featureFrame);
                return;
            }

            void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
            {
                if (restart_msg->data == true)
                {
                    ROS_WARN("restart the estimator!");
                    estimator.clearState();
                    estimator.setParameter();
                }
                return;
            }

            Estimator estimator;

            ros::Subscriber sub_imu;
            ros::Subscriber sub_feature;
            ros::Subscriber sub_restart;
            ros::Subscriber flatten_sub;

            message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> * sync;
            double last_time;
            bool last_time_initialized;
    };
    PLUGINLIB_EXPORT_CLASS(vins_nodelet_pkg::VinsNodeletClass, nodelet::Nodelet);

}
