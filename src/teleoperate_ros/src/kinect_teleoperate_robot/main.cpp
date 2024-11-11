// For kinect camera driver
#include <k4arecord/playback.h>
#include <k4a/k4a.h>

// For kinect body tracking and render
#include <k4abt.h>
#include <BodyTrackingHelpers.h>
#include <Utilities.h>
#include <Window3dWrapper.h>

// STL
#include <chrono>
#include <thread>
#include <mutex>
#include <array>
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <functional>
#include <limits>
#include <atomic>

// For mujoco render
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <lcm/lcm-cpp.hpp>
// #include "unitree_h1_lcm.hpp"



//----for high_level commond-----//
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

static const std::string kTopicArmSDK = "rt/arm_sdk";
constexpr float kPi = 3.141592654;
constexpr float kPi_2 = 1.57079632;


// For math tool
#include "math_tool.hpp"
// For start or end state detect
#include "StartEndPoseDetector.hpp"
// For retargeting function from the skeleton joint angles to the robot motor joint angles.
#include "jointRetargeting.hpp"

#include <Eigen/Geometry>
#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <std_msgs/Float32MultiArray.h>
// #include </home/wang/Desktop/Unitree_H1/kinect_teleoperate_ros/src/teleoperate_ros/src/kinect_teleoperate_robot/msg/Rotation.msg>


using namespace std::chrono;

bool s_isRunning = true;

#define Control_G1 false
#define Control_H1 true
#define Real_Control true    // control real unitree robot in reality
#define Enable_Torso false    // enable torso rotation angle mapping. Test function, open with caution!
#define Enable_Hand  false    // enable hand opening and closing status detection. Test function, open with caution!
#define EchoFrequency false   // Whether to display the running frequency of each thread

// #define NETWORK_INTERFACE "enp3s0"  // define wangtao network interface
const std::string NETWORK_INTERFACE = "enp3s0";  // define wangtao network interface


#if Real_Control
// SDK include files for unitree robot real control
// Please refer to https://github.com/unitreerobotics/unitree_sdk2
#endif

// kinect body tracking skeleton joint angle
// reference: https://learn.microsoft.com/en-us/azure/kinect-dk/body-joints
// ':=' means that the item on the left hand side is being defined to be what is on the right hand side.
// sc:=spine chest, ls:=left shoulder, le:=left elbow, rs:=right shoulder, re:=right elbow, lh:=left hand, rh:=right hand
// _r:=roll, _p:=pitch, _y:=yaw, _a:=angle
static float sc_r, sc_p, sc_y, ls_r, ls_p, ls_y, le_r, le_p, le_y, rs_r, rs_p, rs_y, re_r, re_p, re_y, lh_a, rh_a;

struct hardware_control_signal {
    float left_shoulder_roll = 0.0;
    float left_shoulder_pitch = 0.0;
    float left_shoulder_yaw = 0.0;
    float right_shoulder_roll = 0.0;
    float right_shoulder_pitch = 0.0;
    float right_shoulder_yaw = 0.0;
    float left_elbow_yaw = 0.0;
    float right_elbow_yaw = 0.0;
};

struct Rotation {
    Eigen::Vector3d axis; // 旋转轴 (例如 (1,0,0) 表示 X 轴)
    double angle;         // 旋转角度（弧度）
};

enum JointIndex {
  // Right leg
  kRightHipYaw = 8,
  kRightHipRoll = 0,
  kRightHipPitch = 1,
  kRightKnee = 2,
  kRightAnkle = 11,

  // Left leg
  kLeftHipYaw = 7,
  kLeftHipRoll = 3,
  kLeftHipPitch = 4,
  kLeftKnee = 5,
  kLeftAnkle = 10,

  kWaistYaw = 6,
  kNotUsedJoint = 9,

  // Right arm
  kRightShoulderPitch = 12,
  kRightShoulderRoll = 13,
  kRightShoulderYaw = 14,
  kRightElbow = 15,

  // Left arm
  kLeftShoulderPitch = 16,
  kLeftShoulderRoll = 17,
  kLeftShoulderYaw = 18,
  kLeftElbow = 19
};

std::array<float, 8> init_pos{};
std::array<float, 8> current_jpos_des{};
// std::array<float, 8> current_jpos_des{0.f,  kPi_2,  0.f, kPi_2,
//                                 0.f, -kPi_2,  0.f, kPi_2 };//for reset

// std::array<float, 8> target_pos = { 0.f,  kPi_2,  0.f, kPi_2,
//                                 0.f, -kPi_2,  0.f, kPi_2 };

std::array<float, 8> target_pos{};
/*-------for contain last pos info------------*/
std::array<float, 8> last_pos{};


std::array<JointIndex, 9> arm_joints = {
JointIndex::kLeftShoulderPitch,  JointIndex::kLeftShoulderRoll,
JointIndex::kLeftShoulderYaw,    JointIndex::kLeftElbow,
JointIndex::kRightShoulderPitch, JointIndex::kRightShoulderRoll,
JointIndex::kRightShoulderYaw,   JointIndex::kRightElbow, JointIndex::kWaistYaw};

float weight = 0.f;
float weight_rate = 0.2f;

float kp = 80.f;
float kd = 2.0f;
float dq = 0.f;
float tau_ff = 0.f;

float control_dt = 0.01f;
float max_joint_velocity = 0.5f;

float delta_weight = weight_rate * control_dt;
float max_joint_delta = max_joint_velocity * control_dt;
auto sleep_time =
std::chrono::milliseconds(static_cast<int>(control_dt / 0.001f));

void printArray(const std::array<float, 9>& arr) {
    for (const auto& val : arr) {
        std::cout << val << " ";
    }
    std::cout << std::endl;  
}

bool is_arm_lifted_up = false;

// For control real robot G1
#if Control_G1
hardware_control_signal G1_hardware_signal;
#endif

// For control real robot H1
#if Control_H1
hardware_control_signal H1_hardware_signal;
#endif

/*************************************************Kinect Render, display human skeleton joint tracking*********************************************/

Visualization::Layout3d s_layoutMode = Visualization::Layout3d::OnlyMainView;
bool s_visualizeJointFrame = false;
k4abt_frame_t globalBodyFrameForSkeleton = nullptr;

void PrintUsage()
{
    printf("\n");
    printf(" Basic Navigation:\n\n");
    printf(" Rotate: Rotate the camera by moving the mouse while holding mouse left button\n");
    printf(" Pan: Translate the scene by holding Ctrl key and drag the scene with mouse left button\n");
    printf(" Zoom in/out: Move closer/farther away from the scene center by scrolling the mouse scroll wheel\n");
    printf(" Select Center: Center the scene based on a detected joint by right clicking the joint with mouse\n");
    printf("\n");
    printf(" Key Shortcuts\n\n");
    printf(" ESC: quit\n");
    printf(" h: help\n");
    printf(" b: body visualization mode\n");
    printf(" k: 3d window layout\n");
    printf("\n");
}

int64_t ProcessKey(void* /*context*/, int key)
{
    // https://www.glfw.org/docs/latest/group__keys.html
    switch (key)
    {
        // Quit
    case GLFW_KEY_ESCAPE:
        s_isRunning = false;
        break;
    case GLFW_KEY_K:
        s_layoutMode = (Visualization::Layout3d)(((int)s_layoutMode + 1) % (int)Visualization::Layout3d::Count);
        break;
    case GLFW_KEY_B:
        s_visualizeJointFrame = !s_visualizeJointFrame;
        break;
    case GLFW_KEY_H:
        PrintUsage();
        break;
    }
    return 1;
}

int64_t CloseCallback(void* /*context*/)
{
    s_isRunning = false;
    return 1;
}

void renderSkeletonAndPointCloud(k4abt_frame_t bodyFrame, Window3dWrapper &kinectRenderWindow, int depthWidth, int depthHeight, int step = 2) {
    uint32_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);
    float minDistance = std::numeric_limits<float>::max();
    uint32_t closestBodyIndex = 0;

    if (numBodies == 0) {
        return;
    }
    
    for (uint32_t i = 0; i < numBodies; i++) {
        k4abt_body_t body;
        if (k4abt_frame_get_body_skeleton(bodyFrame, i, &body.skeleton) != K4A_RESULT_SUCCEEDED) {
            std::cerr << "Get skeleton from body frame failed!" << std::endl;
            continue;
        }

        k4abt_joint_t spineChestJoint = body.skeleton.joints[K4ABT_JOINT_SPINE_CHEST];
        float distance = spineChestJoint.position.xyz.z;

        if (distance < minDistance) {
            minDistance = distance;
            closestBodyIndex = i;
        }
    }

    // Obtain original capture that generates the body tracking result
    k4a_capture_t originalCapture = k4abt_frame_get_capture(bodyFrame);
    k4a_image_t depthImage = k4a_capture_get_depth_image(originalCapture);
    std::vector<Color> pointCloudColors(depthWidth * depthHeight, { 1.f, 1.f, 1.f, 1.f });
    // Read body index map and assign colors
    k4a_image_t bodyIndexMap = k4abt_frame_get_body_index_map(bodyFrame);
    const uint8_t* bodyIndexMapBuffer = k4a_image_get_buffer(bodyIndexMap);
    for (int i = 0; i < depthWidth * depthHeight; i = i+step) {
        uint8_t bodyIndex = bodyIndexMapBuffer[i];
        if (bodyIndex == closestBodyIndex) {
            uint32_t bodyId = k4abt_frame_get_body_id(bodyFrame, bodyIndex);
            pointCloudColors[i] = g_bodyColors[bodyId % g_bodyColors.size()];
        }
    }
    k4a_image_release(bodyIndexMap);
    kinectRenderWindow.UpdatePointClouds(depthImage, pointCloudColors);//add parameter 'int step' to accelerate point cloud render
    k4a_capture_release(originalCapture);
    k4a_image_release(depthImage);


    kinectRenderWindow.CleanJointsAndBones();
    k4abt_body_t body;
    VERIFY(k4abt_frame_get_body_skeleton(bodyFrame, closestBodyIndex, &body.skeleton), "Get skeleton from body frame failed!");

    // Assign the correct color based on the body id
    Color color = g_bodyColors[ 1 % g_bodyColors.size()];
    color.a = 0.4f;
    Color lowConfidenceColor = g_bodyColors[ 6 % g_bodyColors.size()];
    lowConfidenceColor.a = 0.3f;

    // Visualize joints
    for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++) {
        if (body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM) {
            const k4a_float3_t& jointPosition = body.skeleton.joints[joint].position;
            const k4a_quaternion_t& jointOrientation = body.skeleton.joints[joint].orientation;

            kinectRenderWindow.AddJoint(
                jointPosition,
                jointOrientation,
                body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM ? color : lowConfidenceColor);
        }
    }

    // Visualize bones
    for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++) {
        k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
        k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;

        if (body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW &&
            body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW) {
            bool confidentBone = body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
                                 body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM;
            const k4a_float3_t& joint1Position = body.skeleton.joints[joint1].position;
            const k4a_float3_t& joint2Position = body.skeleton.joints[joint2].position;

            kinectRenderWindow.AddBone(joint1Position, joint2Position, confidentBone ? color : lowConfidenceColor);
        }
    }
}

void KinectRender_loop(k4a_calibration_t sensorCalibration) {
    std::cout<<"Kinect Render loop start..."<<std::endl;
    std::cout<<"Please press key [s] to start or [e] end the contorl loop   in kinect winndow..."<<std::endl;
    std::cout<<"If want init pose please in kinect window press Ese!..."<<std::endl;
    Window3dWrapper kinectRenderWindow;
    kinectRenderWindow.Create("Kinect Render", sensorCalibration);
    kinectRenderWindow.SetCloseCallback(CloseCallback);
    kinectRenderWindow.SetKeyCallback(ProcessKey);
    int depthWidth = sensorCalibration.depth_camera_calibration.resolution_width;
    int depthHeight = sensorCalibration.depth_camera_calibration.resolution_height;

    time_point<high_resolution_clock> KinectRender_start;
    while (s_isRunning) {
        if (globalBodyFrameForSkeleton != nullptr) {
            renderSkeletonAndPointCloud(globalBodyFrameForSkeleton, kinectRenderWindow, depthWidth, depthHeight);
            k4abt_frame_release(globalBodyFrameForSkeleton);
            globalBodyFrameForSkeleton = nullptr;

            kinectRenderWindow.SetLayout3d(s_layoutMode);
            kinectRenderWindow.SetJointFrameVisualization(s_visualizeJointFrame);
            kinectRenderWindow.Render();

            #if EchoFrequency
            time_point<high_resolution_clock> KinectRender_end = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(KinectRender_end - KinectRender_start).count();
            KinectRender_start = KinectRender_end;
            double frequency = 1e6 / duration;
            std::cout << "Kinect Render loop: " << frequency << " Hz" << std::endl;
            #endif
            // std::cout << "position: " << spineChestJoint.position.xyz << ---- << std::endl;

        } 
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    kinectRenderWindow.Delete();
}

/*************************************************Simulation Unitree Robot Control************************************************/

mjModel* m = nullptr;               // MuJoCo model
mjData*  d = nullptr;               // MuJoCo data

GLFWwindow* mujocoRenderWindow;
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);
  glfwGetCursorPos(window, &lastx, &lasty);
}
// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  int width, height;
  glfwGetWindowSize(window, &width, &height);

  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}
// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

GLFWwindow * InitWindow() {
    if (!glfwInit())
        mju_error("can not initialize GLFW");
    GLFWwindow* window = glfwCreateWindow(1280, 960, "Mujoco Render", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
    return window;
}
void initCamera(mjvCamera* camera) {
    camera->lookat[0] = 0.0; 
    camera->lookat[1] = 0.0; 
    camera->lookat[2] = 0.5; 

    camera->distance = 3.0;

    camera->azimuth = 0;  
    camera->elevation = -30;
}

void DrawOneFrame(GLFWwindow* window) {
    // get framebuffer viewport
    glfwMakeContextCurrent(window);
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    glfwSwapBuffers(window);
    glfwPollEvents();
}

void MujocoRender_loop() {
    std::cout<<"Mujoco Render loop start..."<<std::endl;
    mujocoRenderWindow = InitWindow();
    initCamera(&cam);
    DrawOneFrame(mujocoRenderWindow);

    time_point<high_resolution_clock> mujocoRender_start;
    while (s_isRunning) {
        DrawOneFrame(mujocoRenderWindow);

        #if EchoFrequency
        time_point<high_resolution_clock> mujocoRender_end = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(mujocoRender_end - mujocoRender_start).count();
        double frequency = 1e6 / duration;
        mujocoRender_start = mujocoRender_end;
        std::cout << "Mujoco Render loop: " << frequency << " Hz" << std::endl;
        #endif
    }

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    glfwDestroyWindow(mujocoRenderWindow);
    mj_deleteData(d);
    mj_deleteModel(m);
}

/*****************************************************Real Unitree Robot Control*****************************************************************/
#if Real_Control

#if Control_G1
// Replace there with the Unitree SDK control code
// Send your motor joint value by G1_hardware_signal
// Please refer to https://github.com/unitreerobotics/unitree_sdk2
#endif

#if Control_H1
// Replace there with the Unitree SDK control code
// Send your motor joint value by H1_hardware_signal
// Please refer to https://github.com/unitreerobotics/unitree_sdk2
#endif

#endif
            Eigen::Matrix3d calculateFinalRotationMatrix(const std::vector<Rotation>& rotations) {
                Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity(); // 初始化为单位矩阵
                for (const auto& rot : rotations) {
                    // 使用旋转轴和角度构造单个旋转矩阵
                    Eigen::Matrix3d single_rotation = Eigen::AngleAxisd(rot.angle, rot.axis).toRotationMatrix();
                    
                    // 累积旋转矩阵
                    rotation_matrix *= single_rotation;
                }
                return rotation_matrix;
            }
/*****************************************************Control Smooth and Transfer*****************************************************************/
void Control_loop(std::shared_ptr<unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>> arm_sdk_publisher,
    unitree_go::msg::dds_::LowCmd_ &msg, ros::Publisher& rotation_pub_left, ros::Publisher& rotation_pub_right, std::atomic<bool>& is_active) {
// void Control_loop() {
    std::cout<<"control loop start..."<<std::endl;
    int left_shoulder_roll_joint_id = mj_name2id(m, mjOBJ_ACTUATOR, "left_shoulder_roll_joint");
    int left_shoulder_pitch_joint_id = mj_name2id(m, mjOBJ_ACTUATOR, "left_shoulder_pitch_joint");
    int left_shoulder_yaw_joint_id = mj_name2id(m, mjOBJ_ACTUATOR, "left_shoulder_yaw_joint");
    int right_shoulder_roll_joint_id = mj_name2id(m, mjOBJ_ACTUATOR, "right_shoulder_roll_joint");
    int right_shoulder_pitch_joint_id = mj_name2id(m, mjOBJ_ACTUATOR, "right_shoulder_pitch_joint");
    int right_shoulder_yaw_joint_id = mj_name2id(m, mjOBJ_ACTUATOR, "right_shoulder_yaw_joint");
    #if Control_H1
    int left_elbow_pitch_joint_id = mj_name2id(m, mjOBJ_ACTUATOR, "left_elbow_joint");
    int right_elbow_pitch_joint_id = mj_name2id(m, mjOBJ_ACTUATOR, "right_elbow_joint");
    #elif Control_G1
    int left_elbow_pitch_joint_id = mj_name2id(m, mjOBJ_ACTUATOR, "left_elbow_pitch_joint");
    int right_elbow_pitch_joint_id = mj_name2id(m, mjOBJ_ACTUATOR, "right_elbow_pitch_joint");
    #endif
    #if Enable_Torso
    int torso_joint_id = mj_name2id(m, mjOBJ_ACTUATOR, "torso_joint");
    #endif
    
    mj_step(m, d); // For starting render mujoco——

    MovingAverageFilter ls_r_filter,ls_p_filter,ls_y_filter,
                        rs_r_filter,rs_p_filter,rs_y_filter,
                        le_y_filter,re_y_filter,sc_p_filter;

    StartEndPoseDetector pose_detector;



    time_point<high_resolution_clock> ctrl_start;
    while (s_isRunning) {
        double left_shoulder_roll = LS_mappingCameraRoll2RobotRadians(ls_r);
        double left_shoulder_pitch = LS_mappingCameraPitch2RobotRadians(ls_p);
        double left_shoulder_yaw = LS_mappingCameraYaw2RobotRadians(ls_y);
        double right_shoulder_roll = RS_mappingCameraRoll2RobotRadians(rs_r);
        double right_shoulder_pitch = RS_mappingCameraPitch2RobotRadians(rs_p);
        double right_shoulder_yaw = RS_mappingCameraYaw2RobotRadians(rs_y);
        double left_elbow_yaw = LE_mappingCameraYaw2RobotRadians(le_y);
        double right_elbow_yaw = RE_mappingCameraYaw2RobotRadians(re_y);

        #if Enable_Torso
        double spine_chest_torso = SC_mappingCameraPitch2RobotTorso(sc_p);
        #endif

        // bool start = pose_detector.isStartEndPose(left_shoulder_roll, left_shoulder_pitch, left_shoulder_yaw,
        //                                           right_shoulder_roll, right_shoulder_pitch, right_shoulder_yaw,
        //                                           left_elbow_yaw, right_elbow_yaw);

        // bool start = is_active;


        // std::cout << "left_shoulder_roll: " << left_shoulder_roll << std::endl;
        // std::cout << "left_shoulder_pitch: " << left_shoulder_pitch << std::endl;
        // std::cout << "left_shoulder_yaw: " << left_shoulder_yaw << std::endl;
        // std::cout << "left_elbow_yaw: " << left_elbow_yaw << std::endl;

        // float max_roll = std::numeric_limits<float>::lowest();
        // float min_roll = std::numeric_limits<float>::max();
        // float max_pitch = std::numeric_limits<float>::lowest();
        // float min_pitch = std::numeric_limits<float>::max();
        // float max_yaw = std::numeric_limits<float>::lowest();
        // float min_yaw = std::numeric_limits<float>::max();
        // float max_elbow_yaw = std::numeric_limits<float>::lowest();
        // float min_elbow_yaw = std::numeric_limits<float>::max();

        // if (right_shoulder_roll > max_roll) max_roll = right_shoulder_roll;
        // if (right_shoulder_roll < min_roll) min_roll = right_shoulder_roll;
        // if (right_shoulder_pitch > max_pitch) max_pitch = right_shoulder_pitch;
        // if (right_shoulder_pitch < min_pitch) min_pitch = right_shoulder_pitch;
        // if (right_shoulder_yaw > max_yaw) max_yaw = right_shoulder_yaw;
        // if (right_shoulder_yaw < min_yaw) min_yaw = right_shoulder_yaw;
        // if (right_elbow_yaw > max_elbow_yaw) max_elbow_yaw = right_elbow_yaw;
        // if (right_elbow_yaw < min_elbow_yaw) min_elbow_yaw = right_elbow_yaw;

        // std::cout << "Max right_shoulder_roll: " << max_roll << std::endl;
        // std::cout << "Min right_shoulder_roll: " << min_roll << std::endl;
        // std::cout << "Max right_shoulder_pitch: " << max_pitch << std::endl;
        // std::cout << "Min right_shoulder_pitch: " << min_pitch << std::endl;
        // std::cout << "Max right_shoulder_yaw: " << max_yaw << std::endl;
        // std::cout << "Min right_shoulder_yaw: " << min_yaw << std::endl;
        // std::cout << "Max right_elbow_yaw: " << max_elbow_yaw << std::endl;
        // std::cout << "Min right_elbow_yaw: " << min_elbow_yaw << std::endl;

        // //=====================================================================//
        // float max_roll = std::numeric_limits<float>::lowest();
        // float min_roll = std::numeric_limits<float>::max();
        // float max_pitch = std::numeric_limits<float>::lowest();
        // float min_pitch = std::numeric_limits<float>::max();
        // float max_yaw = std::numeric_limits<float>::lowest();
        // float min_yaw = std::numeric_limits<float>::max();
        // float max_elbow_yaw = std::numeric_limits<float>::lowest();
        // float min_elbow_yaw = std::numeric_limits<float>::max();

        // if (left_shoulder_roll > max_roll) max_roll = left_shoulder_roll;
        // if (left_shoulder_roll < min_roll) min_roll = left_shoulder_roll;
        // if (left_shoulder_pitch > max_pitch) max_pitch = left_shoulder_pitch;
        // if (left_shoulder_pitch < min_pitch) min_pitch = left_shoulder_pitch;
        // if (left_shoulder_yaw > max_yaw) max_yaw = left_shoulder_yaw;
        // if (left_shoulder_yaw < min_yaw) min_yaw = left_shoulder_yaw;
        // if (left_elbow_yaw > max_elbow_yaw) max_elbow_yaw = left_elbow_yaw;
        // if (left_elbow_yaw < min_elbow_yaw) min_elbow_yaw = left_elbow_yaw;

        // std::cout << "Max left_shoulder_roll: " << max_roll << std::endl;
        // std::cout << "Min left_shoulder_roll: " << min_roll << std::endl;
        // std::cout << "Max left_shoulder_pitch: " << max_pitch << std::endl;
        // std::cout << "Min left_shoulder_pitch: " << min_pitch << std::endl;
        // std::cout << "Max left_shoulder_yaw: " << max_yaw << std::endl;
        // std::cout << "Min left_shoulder_yaw: " << min_yaw << std::endl;
        // std::cout << "Max left_elbow_yaw: " << max_elbow_yaw << std::endl;
        // std::cout << "Min left_elbow_yaw: " << min_elbow_yaw << std::endl;        

        // std::cout << "right_shoulder_roll: " << right_shoulder_roll << std::endl;
        // std::cout << "right_shoulder_pitch: " << right_shoulder_pitch << std::endl;
        // std::cout << "right_shoulder_yaw: " << right_shoulder_yaw << std::endl;
        // std::cout << "right_elbow_yaw: " << right_elbow_yaw << std::endl;

        if(is_active.load())
        {
            // smoothing-----------math_tool//
            left_shoulder_roll = ls_r_filter.update(left_shoulder_roll);
            left_shoulder_pitch = ls_p_filter.update(left_shoulder_pitch);
            left_shoulder_yaw = ls_y_filter.update(left_shoulder_yaw);
            right_shoulder_roll = rs_r_filter.update(right_shoulder_roll);
            right_shoulder_pitch = rs_p_filter.update(right_shoulder_pitch);
            right_shoulder_yaw = rs_y_filter.update(right_shoulder_yaw);
            left_elbow_yaw = le_y_filter.update(left_elbow_yaw);
            right_elbow_yaw = re_y_filter.update(right_elbow_yaw);
            #if Enable_Torso
            spine_chest_torso = sc_p_filter.update(spine_chest_torso);
            #endif

            mj_step1(m, d);
            // The coordinate system of the robot joints is as follows: the x-axis (roll) points forward, the y-axis (pitch) points left, 
            // and the z-axis (yaw) points upward. 
            // Test to get the rotation order, because the coordinate system definition of the Kinect camera is still a mystery
            // Camera:   z(yaw)      x(roll)   y(pitch) 
            //             |           |          |
            //             ∨           ∨          ∨
            // Robot :  y(pitch)     z(yaw)    x(roll)
            d->ctrl[left_shoulder_pitch_joint_id] = left_shoulder_yaw;
            d->ctrl[left_shoulder_roll_joint_id] = left_shoulder_pitch;
            d->ctrl[left_shoulder_yaw_joint_id] = left_shoulder_roll;
            d->ctrl[right_shoulder_pitch_joint_id] = right_shoulder_yaw;
            d->ctrl[right_shoulder_roll_joint_id] = right_shoulder_pitch;
            d->ctrl[right_shoulder_yaw_joint_id] = right_shoulder_roll;
            d->ctrl[left_elbow_pitch_joint_id] = left_elbow_yaw;
            d->ctrl[right_elbow_pitch_joint_id] = right_elbow_yaw;

            // printf("aaaaa\n");
            // lcm.publish("EXAMPLE_CHANNEL", &msg);

            #if Enable_Torso
            d->ctrl[torso_joint_id] = spine_chest_torso;
            #endif
            mj_step2(m, d);

            #if Real_Control
            #if Control_H1
            H1_hardware_signal.left_shoulder_pitch = static_cast<float>(left_shoulder_yaw);
            H1_hardware_signal.left_shoulder_roll = static_cast<float>(left_shoulder_pitch);
            H1_hardware_signal.left_shoulder_yaw = static_cast<float>(left_shoulder_roll);
            H1_hardware_signal.right_shoulder_pitch = static_cast<float>(right_shoulder_yaw);
            H1_hardware_signal.right_shoulder_roll = static_cast<float>(right_shoulder_pitch);
            H1_hardware_signal.right_shoulder_yaw = static_cast<float>(right_shoulder_roll);
            H1_hardware_signal.left_elbow_yaw = static_cast<float>(left_elbow_yaw);
            H1_hardware_signal.right_elbow_yaw = static_cast<float>(right_elbow_yaw);
            #elif Control_G1
            G1_hardware_signal.left_shoulder_pitch = left_shoulder_yaw;
            G1_hardware_signal.left_shoulder_roll = left_shoulder_roll;
            G1_hardware_signal.left_shoulder_yaw = left_shoulder_pitch;
            G1_hardware_signal.right_shoulder_pitch = right_shoulder_yaw;
            G1_hardware_signal.right_shoulder_roll = right_shoulder_pitch;
            G1_hardware_signal.right_shoulder_yaw = right_shoulder_roll;
            G1_hardware_signal.left_elbow_yaw = left_elbow_yaw;
            G1_hardware_signal.right_elbow_yaw = right_elbow_yaw;
            #endif
            #endif

            // Target Pos Set and  envalue;
            target_pos = {
                H1_hardware_signal.left_shoulder_pitch,
                H1_hardware_signal.left_shoulder_roll,
                H1_hardware_signal.left_shoulder_yaw,
                H1_hardware_signal.left_elbow_yaw,
                H1_hardware_signal.right_shoulder_pitch,
                H1_hardware_signal.right_shoulder_roll,
                H1_hardware_signal.right_shoulder_yaw,
                H1_hardware_signal.right_elbow_yaw}; 


            std::vector<Rotation> rotations_left = {
                    {Eigen::Vector3d(1, 0, 0), 0.43633}, // 绕X轴旋转0.43633弧度
                    {Eigen::Vector3d(0, 1, 0), H1_hardware_signal.left_shoulder_pitch},  // 绕Y轴旋转
                    {Eigen::Vector3d(1, 0, 0), -0.43633}, // 反向绕X轴旋转0.43633弧度
                    {Eigen::Vector3d(1, 0, 0), H1_hardware_signal.left_shoulder_roll},  // 绕Z轴旋转
                    {Eigen::Vector3d(0, 0, 1), H1_hardware_signal.left_shoulder_yaw},  // 绕Z轴旋转0.5236弧度（30度）
                    {Eigen::Vector3d(0, 1, 0), H1_hardware_signal.left_elbow_yaw},  // 胳膊肘到底是y还是z
                    
                };

            std::vector<Rotation> rotations_right = {
                    {Eigen::Vector3d(1, 0, 0), -0.43633},// 反向绕X轴旋转0.43633弧度
                    {Eigen::Vector3d(0, 1, 0), H1_hardware_signal.right_shoulder_pitch},  // 绕Y轴旋转
                    {Eigen::Vector3d(1, 0, 0), 0.43633}, // 绕X轴旋转0.43633弧度
                    {Eigen::Vector3d(1, 0, 0), H1_hardware_signal.right_shoulder_roll},  // 绕Z轴旋转
                    {Eigen::Vector3d(0, 0, 1), H1_hardware_signal.right_shoulder_yaw},  // 绕Z轴旋转0.5236弧度（30度）
                    {Eigen::Vector3d(0, 1, 0), H1_hardware_signal.right_elbow_yaw},  // 胳膊肘到底是y还是z


                    
                };
                Eigen::Matrix3d left_rotation_matrix = calculateFinalRotationMatrix(rotations_left);
                Eigen::Matrix3d right_rotation_matrix = calculateFinalRotationMatrix(rotations_right);  

                // std::cout << "左臂旋转矩阵:\n" << left_rotation_matrix << std::endl;
                // std::cout << "右臂旋转矩阵:\n" << right_rotation_matrix << std::endl;


            while (ros::ok()) {
                Eigen::Matrix3d rotation_matrix_left = calculateFinalRotationMatrix(rotations_left);
                Eigen::Matrix3d rotation_matrix_right = calculateFinalRotationMatrix(rotations_right);

                // Publish left rotation matrix
                std_msgs::Float32MultiArray rotation_msg_left;
                rotation_msg_left.data.reserve(9);
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        rotation_msg_left.data.push_back(rotation_matrix_left(i, j));
                    }
                }
                rotation_pub_left.publish(rotation_msg_left);

                // Publish right rotation matrix
                std_msgs::Float32MultiArray rotation_msg_right;
                rotation_msg_right.data.reserve(9);
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        rotation_msg_right.data.push_back(rotation_matrix_right(i, j));
                    }
                }
                rotation_pub_right.publish(rotation_msg_right);   
                
                // ros::spinOnce(); 
                break;
            }
            
            // set weight
            msg.motor_cmd().at(JointIndex::kNotUsedJoint).q(1);

            for (int j = 0; j < init_pos.size(); ++j) {
                current_jpos_des.at(j) +=
                    std::clamp(target_pos.at(j) - current_jpos_des.at(j),
                                -max_joint_delta, max_joint_delta);

                current_jpos_des.at(j) = std::clamp(current_jpos_des.at(j), -kPi_2, kPi_2);               

                // set control joints
                for (int j = 0; j < init_pos.size(); ++j) {
                    msg.motor_cmd().at(arm_joints.at(j)).q(current_jpos_des.at(j));
                    msg.motor_cmd().at(arm_joints.at(j)).dq(dq);
                    msg.motor_cmd().at(arm_joints.at(j)).kp(kp);
                    msg.motor_cmd().at(arm_joints.at(j)).kd(kd);
                    msg.motor_cmd().at(arm_joints.at(j)).tau(tau_ff);
                }

                // std::cout << "***********current pos " << j << "*********" << current_jpos_des.at(j) <<  std::endl;
                // std::cout << "***********current pos " << j << "*********" << current_jpos_des.at(j) <<  std::endl;
                // std::cout << "***********current pos " << 5 << "*********" << current_jpos_des.at(5) <<  std::endl;
                // std::cout << "***********current pos " << "*********" << left_shoulder_yaw<<  std::endl;

            }
                
            arm_sdk_publisher->Write(msg);

            std::this_thread::sleep_for(sleep_time);

            #if EchoFrequency
            time_point<high_resolution_clock> ctrl_end = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(ctrl_end - ctrl_start).count();
            double frequency = 1e6 / duration;
            ctrl_start = ctrl_end;
            std::cout << "Control loop: " << frequency << " Hz" << std::endl;
            #endif
        }
    }
}


// Process the newly captured skeleton point data to calculate the joint angles that control the robot
void ProcessNewSkeletonData(k4abt_frame_t bodyFrame) {
    uint32_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);
    float minDistance = std::numeric_limits<float>::max();
    uint32_t closestBodyIndex = 0;

    if (numBodies == 0) {
        return;
    }
    // Take the data of the human body closest to the kinect camera as input
    for (uint32_t i = 0; i < numBodies; i++) {
        k4abt_body_t body;
        if (k4abt_frame_get_body_skeleton(bodyFrame, i, &body.skeleton) != K4A_RESULT_SUCCEEDED) {
            std::cerr << "Get skeleton from body frame failed!" << std::endl;
            continue;
        }

        k4abt_joint_t spineChestJoint = body.skeleton.joints[K4ABT_JOINT_SPINE_CHEST];
        float distance = spineChestJoint.position.xyz.z;

        if (distance < minDistance) {
            minDistance = distance;
            closestBodyIndex = i;
        }
    }

    k4abt_body_t closestBody;
    if (k4abt_frame_get_body_skeleton(bodyFrame, closestBodyIndex, &closestBody.skeleton) != K4A_RESULT_SUCCEEDED) {
        std::cerr << "Get skeleton from body frame failed!" << std::endl;
        return;
    }

    // temp var
    k4a_quaternion_t relativeJointOrientation;
    static k4a_quaternion_t SpineChest_orientation, LeftShoulder_orientation, RightShoulder_orientation;

    if(closestBody.skeleton.joints[K4ABT_JOINT_SPINE_CHEST].confidence_level > K4ABT_JOINT_CONFIDENCE_LOW){
        SpineChest_orientation = closestBody.skeleton.joints[K4ABT_JOINT_SPINE_CHEST].orientation;
        quaternion2Euler(SpineChest_orientation, sc_r, sc_p, sc_y);
    }

    if(closestBody.skeleton.joints[K4ABT_JOINT_SHOULDER_LEFT].confidence_level > K4ABT_JOINT_CONFIDENCE_LOW){
        LeftShoulder_orientation = closestBody.skeleton.joints[K4ABT_JOINT_SHOULDER_LEFT].orientation;
        relativeJointOrientation = calculateRelativeQuaternion(LeftShoulder_orientation, SpineChest_orientation);
        quaternion2Euler(relativeJointOrientation, ls_r, ls_p, ls_y);
    }

    if(closestBody.skeleton.joints[K4ABT_JOINT_ELBOW_LEFT].confidence_level > K4ABT_JOINT_CONFIDENCE_LOW){
        relativeJointOrientation = calculateRelativeQuaternion(closestBody.skeleton.joints[K4ABT_JOINT_ELBOW_LEFT].orientation, LeftShoulder_orientation);
        quaternion2Euler(relativeJointOrientation, le_r, le_p, le_y);
    }

    if(closestBody.skeleton.joints[K4ABT_JOINT_SHOULDER_RIGHT].confidence_level > K4ABT_JOINT_CONFIDENCE_LOW){
        RightShoulder_orientation = closestBody.skeleton.joints[K4ABT_JOINT_SHOULDER_RIGHT].orientation;
        relativeJointOrientation = calculateRelativeQuaternion(RightShoulder_orientation, SpineChest_orientation);
        quaternion2Euler(relativeJointOrientation, rs_r, rs_p, rs_y);

    }
    
    if(closestBody.skeleton.joints[K4ABT_JOINT_ELBOW_RIGHT].confidence_level > K4ABT_JOINT_CONFIDENCE_LOW){
        relativeJointOrientation = calculateRelativeQuaternion(closestBody.skeleton.joints[K4ABT_JOINT_ELBOW_RIGHT].orientation, RightShoulder_orientation);
        quaternion2Euler(relativeJointOrientation, re_r, re_p, re_y);
    }
    
    // For hand open and closing status detect
    #if Enable_Hand
    if(closestBody.skeleton.joints[K4ABT_JOINT_HANDTIP_LEFT].confidence_level > K4ABT_JOINT_CONFIDENCE_LOW && 
        closestBody.skeleton.joints[K4ABT_JOINT_WRIST_LEFT].confidence_level > K4ABT_JOINT_CONFIDENCE_LOW){
        k4a_quaternion_t LeftHandTip_orientation = closestBody.skeleton.joints[K4ABT_JOINT_HANDTIP_LEFT].orientation;
        k4a_quaternion_t LeftWrist_orientation = closestBody.skeleton.joints[K4ABT_JOINT_WRIST_LEFT].orientation;
        lh_a = calculateRelativeAngle(LeftHandTip_orientation, LeftWrist_orientation);
        if(lh_a < 30.0)
            std::cout<<"left hand open."<<std::endl;
        else if(lh_a > 50.0)
            std::cout<<"left hand close."<<std::endl;
    }
    else
        std::cout<<"left hand unknow."<<std::endl;

    if(closestBody.skeleton.joints[K4ABT_JOINT_HANDTIP_RIGHT].confidence_level > K4ABT_JOINT_CONFIDENCE_LOW && 
        closestBody.skeleton.joints[K4ABT_JOINT_WRIST_RIGHT].confidence_level > K4ABT_JOINT_CONFIDENCE_LOW){
        k4a_quaternion_t RightHandTip_orientation = closestBody.skeleton.joints[K4ABT_JOINT_HANDTIP_RIGHT].orientation;
        k4a_quaternion_t RightWrist_orientation = closestBody.skeleton.joints[K4ABT_JOINT_WRIST_RIGHT].orientation;
        rh_a = calculateRelativeAngle(RightHandTip_orientation, RightWrist_orientation);
        if(rh_a < 30.0)
            std::cout<<"right hand open."<<std::endl;
        else if(rh_a > 50.0)
            std::cout<<"right hand close."<<std::endl;
    }
    else
        std::cout<<"right hand unknow."<<std::endl;
    #endif
}

void Main_loop(){

    PrintUsage();

    k4a_device_t device = nullptr;
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    // K4A_DEPTH_MODE_NFOV_2X2BINNED, /**< Depth captured at 320x288. Passive IR is also captured at 320x288. */
    // K4A_DEPTH_MODE_NFOV_UNBINNED,  /**< Depth captured at 640x576. Passive IR is also captured at 640x576. */
    // K4A_DEPTH_MODE_WFOV_2X2BINNED, /**< Depth captured at 512x512. Passive IR is also captured at 512x512. */
    deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

    // Get calibration information
    k4a_calibration_t sensorCalibration;
    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration),
        "Get depth camera calibration failed!");

    // Create Body Tracker
    k4abt_tracker_t tracker = nullptr;
    k4abt_tracker_configuration_t trackerConfig = K4ABT_TRACKER_CONFIG_DEFAULT;
    trackerConfig.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU_CUDA;
    VERIFY(k4abt_tracker_create(&sensorCalibration, trackerConfig, &tracker), "Body tracker initialization failed!");
    // Do not use Kinect's built-in smoothing
    k4abt_tracker_set_temporal_smoothing(tracker, 0.0);

    std::thread KinectRender_thread(KinectRender_loop, sensorCalibration);

    time_point<high_resolution_clock> main_start;
    while (s_isRunning)
    {
        k4a_capture_t sensorCapture = nullptr;
        k4a_wait_result_t getCaptureResult = k4a_device_get_capture(device, &sensorCapture, 0); // without blocking, timeout_in_ms is set to 0 
        if (getCaptureResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            k4a_wait_result_t queueCaptureResult = k4abt_tracker_enqueue_capture(tracker, sensorCapture, 0); // without blocking, timeout_in_ms is set to 0 
            k4a_capture_release(sensorCapture);
            if (queueCaptureResult == K4A_WAIT_RESULT_FAILED)
            {
                std::cout << "Error! Add capture to tracker process queue failed!" << std::endl;
                break;
            }
        }
        else if (getCaptureResult != K4A_WAIT_RESULT_TIMEOUT)
        {
            std::cout << "Get depth capture returned error: " << getCaptureResult << std::endl;
            break;
        }

        k4abt_frame_t bodyFrame = nullptr;
        k4a_wait_result_t popFrameResult = k4abt_tracker_pop_result(tracker, &bodyFrame, 0); // without blocking, timeout_in_ms is set to 0
        if (popFrameResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            if (globalBodyFrameForSkeleton == nullptr)
            {
                // Passing references for increased efficiency, manually managing reference counts
                globalBodyFrameForSkeleton = bodyFrame;
                k4abt_frame_reference(globalBodyFrameForSkeleton);
            }
            ProcessNewSkeletonData(bodyFrame);
            k4abt_frame_release(bodyFrame);

            #if EchoFrequency
            time_point<high_resolution_clock> main_end = high_resolution_clock::now();
            auto duration = duration_cast<milliseconds>(main_end - main_start).count();
            double frequency = 1e3 / duration;
            main_start = main_end;
            std::cout << "Main loop : " << frequency << " Hz" << std::endl;
            #endif
        }
    }

    std::cout << "kinect_teleoperate_robot finished!" << std::endl;

    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);

    k4a_device_stop_cameras(device);
    k4a_device_close(device);

    KinectRender_thread.join();
}


void captureInput(std::atomic<bool>& is_active) {
    char input;
    while (true) {
        std::cin >> input;
        if (input == 's') {
            is_active.store(true);
            std::cout << "start the procseee" << std::endl;
        } else if (input == 'e') {
            is_active.store(false);
            std::cout << "end the process" << std::endl;
        }
    }
}


int main(int argc, char const *argv[])
{

    std::atomic<bool> is_active(false);

    #if Control_G1
    const char* model_path = "../src/unitree_g1/scene.xml";
    #endif
    #if Control_H1
    const char* model_path = "../src/teleoperate_ros/src/unitree_h1/mjcf/scene.xml"; 
    #endif

    ros::init(argc, const_cast<char**>(argv), "control_node");
    ros::NodeHandle nh;
    ros::Publisher rotation_pub_left = nh.advertise<std_msgs::Float32MultiArray>("left_arm_rotation_matrix", 10);
    ros::Publisher rotation_pub_right = nh.advertise<std_msgs::Float32MultiArray>("right_arm_rotation_matrix", 10);


    if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
    }

    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_>
        arm_sdk_publisher;
    unitree_go::msg::dds_::LowCmd_ msg;

    arm_sdk_publisher.reset(
        new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(
            kTopicArmSDK));
    arm_sdk_publisher->InitChannel();

    /*--------------------------------*/
    
    /*--------------------------------*/  
    char error[1000];
    std::cout << model_path << std::endl;
    m = mj_loadXML(model_path, nullptr, error, 1000);
    d = mj_makeData(m);
    mj_resetData(m, d);

    // std::thread control_thread(Control_loop);
    std::thread control_thread(Control_loop, arm_sdk_publisher, std::ref(msg), 
                            std::ref(rotation_pub_left), std::ref(rotation_pub_right), 
                            std::ref(is_active));
    std::thread mujocoRender_thread(MujocoRender_loop);
    std::thread input_thread(captureInput, std::ref(is_active));

    Main_loop();

    control_thread.join();
    mujocoRender_thread.join();
    input_thread.detach();

    int sstep = int(5/control_dt);
    for (int i = 0; i < sstep; ++i) {

        for (int j = 0; j < init_pos.size(); ++j) {
            current_jpos_des.at(j) +=
                std::clamp(init_pos.at(j) - current_jpos_des.at(j),
                            -max_joint_delta, max_joint_delta);

            // set control joints
            for (int j = 0; j < init_pos.size(); ++j) {
                msg.motor_cmd().at(arm_joints.at(j)).q(current_jpos_des.at(j));
                msg.motor_cmd().at(arm_joints.at(j)).dq(dq);
                msg.motor_cmd().at(arm_joints.at(j)).kp(kp);
                msg.motor_cmd().at(arm_joints.at(j)).kd(kd);
                msg.motor_cmd().at(arm_joints.at(j)).tau(tau_ff);
            }

            // std::cout << "***********current pos " << j << "*********" << current_jpos_des.at(j)"*************" <<  std::endl;
        }
        arm_sdk_publisher->Write(msg);
        std::this_thread::sleep_for(sleep_time);
    }
    std::cout << "***********arm down*********" << std::endl;

    return 0;
}