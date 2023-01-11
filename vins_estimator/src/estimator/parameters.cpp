/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "parameters.h"

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;
double THRES_OUTLIER;
double triangulate_max_err = 0.5;

double IMU_FREQ;
double IMAGE_FREQ;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string OUTPUT_FOLDER;
std::string IMU_TOPIC;
int ROW, WIDTH;
int SHOW_WIDTH;
double TD;
int NUM_OF_CAM;
int STEREO;
int FISHEYE;
double FISHEYE_FOV;
int enable_up_top;
int enable_down_top;
int enable_up_side;
int enable_down_side;
int enable_rear_side;

int USE_VXWORKS;
double depth_estimate_baseline;

int USE_IMU;
int USE_GPU;
int ENABLE_DOWNSAMPLE;
int PUB_RECTIFY;
int USE_ORB;
Eigen::Matrix3d rectify_R_left;
Eigen::Matrix3d rectify_R_right;
map<int, Eigen::Vector3d> pts_gt;
std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
std::string COMP_IMAGE0_TOPIC, COMP_IMAGE1_TOPIC;
int IS_COMP_IMAGES;
std::string FISHEYE_MASK;
std::vector<std::string> CAM_NAMES;
std::string depth_config;
int MAX_CNT;
int TOP_PTS_CNT;
int SIDE_PTS_CNT;
int MAX_SOLVE_CNT;
int RGB_DEPTH_CLOUD;
int ENABLE_DEPTH;
int ENABLE_PERF_OUTPUT;
int MIN_DIST;
double F_THRESHOLD;
int SHOW_TRACK;
int FLOW_BACK;

int WARN_IMU_DURATION;
int PUB_FLATTEN;
int FLATTEN_COLOR;
int PUB_FLATTEN_FREQ;

std::string configPath;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(std::string config_file)
{
    /*
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;          
    }
    fclose(fh);*/

    cv::FileStorage fsSettings;
    try {
        fsSettings.open(config_file.c_str(), cv::FileStorage::READ);
    } catch(cv::Exception ex) {
        std::cerr << "ERROR:" << ex.what() << " Can't open config file" << std::endl;
    }
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["image0_topic"] >> IMAGE0_TOPIC;
    fsSettings["image1_topic"] >> IMAGE1_TOPIC;

    fsSettings["compressed_image0_topic"] >> COMP_IMAGE0_TOPIC;
    fsSettings["compressed_image1_topic"] >> COMP_IMAGE1_TOPIC;
    IS_COMP_IMAGES = fsSettings["is_compressed_images"];
    MAX_CNT = fsSettings["max_cnt"];
    TOP_PTS_CNT = fsSettings["top_cnt"];
    SIDE_PTS_CNT = fsSettings["side_cnt"];
    MAX_SOLVE_CNT = fsSettings["max_solve_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    USE_ORB = fsSettings["use_orb"];

    SHOW_TRACK = fsSettings["show_track"];
    FLOW_BACK = fsSettings["flow_back"];
    RGB_DEPTH_CLOUD = fsSettings["rgb_depth_cloud"];
    ENABLE_DEPTH = fsSettings["enable_depth"];
    ENABLE_DOWNSAMPLE = fsSettings["enable_downsample"];
    THRES_OUTLIER = fsSettings["thres_outlier"];
    triangulate_max_err = fsSettings["tri_max_err"];
    USE_GPU = fsSettings["use_gpu"];
#ifndef USE_CUDA
        if (USE_GPU) {
            std::cerr << "Must set USE_CUDA on in CMake to enable cuda!!!" << std::endl;
            exit(-1);
        }
#endif
    FISHEYE = fsSettings["is_fisheye"];
    FISHEYE_FOV = fsSettings["fisheye_fov"];
    USE_VXWORKS = fsSettings["use_vxworks"];
    enable_up_top = fsSettings["enable_up_top"];
    enable_up_side = fsSettings["enable_up_side"];
    enable_down_top = fsSettings["enable_down_top"];
    enable_down_side = fsSettings["enable_down_side"];
    enable_rear_side = fsSettings["enable_rear_side"];
    depth_estimate_baseline = fsSettings["depth_estimate_baseline"];
    ENABLE_PERF_OUTPUT = fsSettings["enable_perf_output"];

    IMU_FREQ = fsSettings["imu_freq"];
    IMAGE_FREQ = fsSettings["image_freq"];
    WARN_IMU_DURATION = fsSettings["warn_imu_duration"];
    PUB_FLATTEN = fsSettings["pub_flatten"];
    FLATTEN_COLOR = fsSettings["flatten_color"];
    USE_IMU = fsSettings["imu"];
    PUB_FLATTEN_FREQ = fsSettings["pub_flatten_freq"];
    if (PUB_FLATTEN_FREQ == 0) {
        PUB_FLATTEN_FREQ = 10;
    }

    printf("USE_IMU: %d\n", USE_IMU);
    if(USE_IMU)
    {
        fsSettings["imu_topic"] >> IMU_TOPIC;
        printf("IMU_TOPIC: %s\n", IMU_TOPIC.c_str());
        ACC_N = fsSettings["acc_n"];
        ACC_W = fsSettings["acc_w"];
        GYR_N = fsSettings["gyr_n"];
        GYR_W = fsSettings["gyr_w"];
        G.z() = fsSettings["g_norm"];
    }

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    fsSettings["output_path"] >> OUTPUT_FOLDER;
    VINS_RESULT_PATH = OUTPUT_FOLDER + "/vio.csv";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    RIC.resize(2);
    TIC.resize(2);

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC[0] = Eigen::Matrix3d::Identity();
        TIC[0] = Eigen::Vector3d::Zero();
        EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        Eigen::Matrix4d T;

        // configuration of drone1
        T <<  9.9976326633205825e-01, -1.5962615960314536e-02,
       -1.4785336815826064e-02, -5.7474075887206502e-02,
       1.5950154713398050e-02, 9.9987232699006656e-01,
       -9.6035622974547665e-04, -1.7874191735684002e-04,
       1.4798778925052431e-02, 7.2430047139055266e-04,
       9.9989022974082242e-01, 1.1611346883283556e-01, 0, 0, 0, 1;
        
        // configuration of drone4
    //     T << 9.9994248539804909e-01, -9.7708368424183085e-03,
    //    4.4222893812111020e-03, -4.7186307264815902e-02,
    //    9.7416125923359253e-03, 9.9993088420061405e-01,
    //    6.5823860323645355e-03, 5.1646305096511771e-03,
    //    -4.4862991511014499e-03, -6.5389272191290378e-03,
    //    9.9996855728105261e-01, 1.1799412663586026e-01, 0., 0., 0., 1;
        
        // configuration of fisheye
    //     T << 9.9970152931501111e-01, -2.4334070146456496e-02,
    //    -2.1691738831616180e-03, 2.4271285208706174e-02,
    //    2.4313045731037753e-02, 9.9966167038360143e-01,
    //    -9.2423250942725247e-03, 4.6146656309142874e-03,
    //    2.3933433745542093e-03, 9.1868273073508668e-03,
    //    9.9995493604037800e-01, 8.9835134676070530e-02, 0., 0., 0., 1;
        
        RIC[0] = T.block<3, 3>(0, 0);
        TIC[0] = T.block<3, 1>(0, 3);
    } 

    NUM_OF_CAM = fsSettings["num_of_cam"];
    printf("camera number %d\n", NUM_OF_CAM);

    if(NUM_OF_CAM != 1 && NUM_OF_CAM != 2)
    {
        printf("num_of_cam should be 1 or 2\n");
        assert(0);
    }


    int pn = config_file.find_last_of('/');
    configPath = config_file.substr(0, pn);


    depth_config = configPath + "/" +((std::string) fsSettings["depth_config"]);

    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    CAM_NAMES.resize(2);

    CAM_NAMES[0] = cam0Path;

    if(NUM_OF_CAM == 2)
    {
        STEREO = 1;
        std::string cam1Calib;
        fsSettings["cam1_calib"] >> cam1Calib;
        std::string cam1Path = configPath + "/" + cam1Calib; 
        
        CAM_NAMES[1] = cam1Path;      

        Eigen::Matrix4d T;

        // configuration of drone1
        T << 9.9983528585161952e-01, 1.2295587859875078e-02,
       1.3349894575998694e-02, -5.3122369629771571e-02,
       1.2527691710654654e-02, -9.9976935435548442e-01,
       -1.7444054345837460e-02, -3.6008274027051416e-02,
       1.3132330578118318e-02, 1.7608424426899433e-02,
       -9.9975871353181467e-01, -3.8557581591239801e-02, 0, 0, 0, 1;

        // configuration of drone4
    //     T <<  9.9993458422028059e-01, -9.2496221080246958e-03,
    //    -6.7284300600721821e-03, -5.4555597696550183e-02,
    //    -9.2920134443754167e-03, -9.9993700371876737e-01,
    //    -6.2965927361157614e-03, 1.0322653996469110e-02,
    //    -6.6697650906226563e-03, 6.3587015021700838e-03,
    //    -9.9995753967298207e-01, -3.8302293717411999e-02, 0., 0., 0., 1;
        

        // configuration of fisheye
    //     T << 9.9985740148420199e-01, 1.3221990047149310e-02,
    //    1.0505031006729837e-02, 2.1768721164581610e-02,
    //    1.3173059971592354e-02, -9.9990212239267229e-01,
    //    4.7133985207440873e-03, 2.1485901422100902e-03,
    //    1.0566323307759515e-02, -4.5743429936555748e-03,
    //    -9.9993371190191072e-01, -2.4828776478011370e-02, 0., 0., 0., 1;
        
        RIC[1] = T.block<3, 3>(0, 0);
        TIC[1] = T.block<3, 1>(0, 3);
        fsSettings["publish_rectify"] >> PUB_RECTIFY;
    }

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROW = fsSettings["image_height"];
    WIDTH = fsSettings["image_width"];
    SHOW_WIDTH = fsSettings["show_width"];
    ROS_INFO("ROW: %d COL: %d ", ROW, WIDTH);

    if(!USE_IMU)
    {
        ESTIMATE_EXTRINSIC = 0;
        ESTIMATE_TD = 0;
        printf("no imu, fix extrinsic param; no time offset calibration\n");
    }
    if(PUB_RECTIFY)
    {
        cv::Mat rectify_left;
        cv::Mat rectify_right;
        fsSettings["cam0_rectify"] >> rectify_left;
        fsSettings["cam1_rectify"] >> rectify_right;
        cv::cv2eigen(rectify_left, rectify_R_left);
        cv::cv2eigen(rectify_right, rectify_R_right);

    }

    fsSettings.release();
}
