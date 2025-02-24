#pragma once
#include "prx/utilities/defs.hpp"

#include "GLFW/glfw3.h"
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include "mujoco/mujoco.h"

namespace prx {

/**
 * @brief Simple wrapper for MuJoCo simulation and visualization
 */
class MujocoWrapper {
public:
    /**
     * @brief Create wrapper from existing mjModel
     */
    MujocoWrapper(mjModel* m, bool visualize = false) : visualize_(visualize) {
        m_ = m;
        d_ = mj_makeData(m_);

        if (visualize_) {
            initVisualization();
        }
        mj_kinematics(m_, d_);
        mj_forward(m_, d_);
    }

    ~MujocoWrapper() {
        if (visualize_) {
            mjv_freeScene(&scn_);
            mjr_freeContext(&con_);
            glfwDestroyWindow(window_);
            glfwTerminate();
        }
        if (d_) mj_deleteData(d_);
        if (m_) mj_deleteModel(m_);

    }

    // Getters for raw MuJoCo objects
    mjModel* model() { return m_; }
    mjData* data() { return d_; }

    /**
     * @brief Step simulation and update visualization
     */
    void step() {
        // print controls
        mj_step(m_, d_);
        if (visualize_) {
            render();
        }
    }

    /**
     * @brief Reset simulation to initial state
     */
    void reset() {
        mj_resetData(m_, d_);
        mj_forward(m_, d_);
    }

    /**
     * @brief Set position of a geom
     */
    void setRobotPosition(const std::array<double, 3>& pos) {
        // for (int i = 0; i < m_->nq; i++) {
        //     d_->qpos[i] = 0.0;
        // }
        d_->qpos[0] = pos[0];
        d_->qpos[1] = pos[1];
        
        mj_forward(m_, d_);
    }
    /**
     * @brief Set velocity of the robot
     */

    void setZeroVelocity() {
        for (int i = 0; i < m_->nv; i++) {
            d_->qvel[i] = 0.0;
        }
        mj_forward(m_, d_);
    }

    void setRobotVelocity(const std::array<double, 2>& vel) {
        d_->qvel[0] = vel[0];
        d_->qvel[1] = vel[1];
        mj_forward(m_, d_);
    }
    
    /**
     * @brief Get position of a body
     */
    void getBodyPosition(const std::string& name, std::array<double, 3>& pos) {
        int id = mj_name2id(m_, mjOBJ_BODY, name.c_str());
        if (id >= 0) {
            pos[0] = d_->geom_xpos[3 * id];
            pos[1] = d_->geom_xpos[3 * id + 1];
            pos[2] = d_->geom_xpos[3 * id + 2];
        }
    }

    /**
     * @brief Get quaternion of a body
     */
    void getBodyQuaternion(const std::string& name, std::array<double, 4>& quat) {
        int id = mj_name2id(m_, mjOBJ_BODY, name.c_str());
        if (id >= 0) {
            quat[0] = d_->xquat[4 * id];
            quat[1] = d_->xquat[4 * id + 1];
            quat[2] = d_->xquat[4 * id + 2];
            quat[3] = d_->xquat[4 * id + 3];
        }
    }

    /**
     * @brief Set control signals
     */
    void setControl(const double* ctrl, int nctrl) {
        for (int i = 0; i < nctrl && i < m_->nu; i++) {
            d_->ctrl[i] = ctrl[i];
        }
    }

    void setZeroControl() {
        for (int i = 0; i < m_->nu; i++) {
            d_->ctrl[i] = 0.0;
        }
    }

private:
    void initVisualization() {
        if (!glfwInit()) {
            throw std::runtime_error("GLFW initialization failed");
        }

        window_ = glfwCreateWindow(1200, 900, "MuJoCo Simulation", nullptr, nullptr);
        if (!window_) {
            glfwTerminate();
            throw std::runtime_error("Window creation failed");
        }

        glfwMakeContextCurrent(window_);
        mjv_defaultCamera(&cam_);
        mjv_defaultOption(&opt_);
        
        // Enable force visualization
        opt_.flags[mjVIS_CONTACTFORCE] = 1;  // Show contact forces
        // opt_.flags[mjVIS_CONSTRAINT] = 1;    // Show constraint forces
        // opt_.flags[mjVIS_FORCE] = 1;         // Show external forces
        
        // Adjust force visualization scale (optional)
        // opt_.flags[mjVIS_SCALE] = 1;         // Show scale reference
        // opt_.scale.forcewidth = 0.1;         // Adjust force arrow width
        // opt_.scale.contactwidth = 0.1;       // Adjust contact force width
        // opt_.scale.contactheight = 0.1;      // Adjust contact force height
        // opt_.scale.force = 0.1;              // Adjust force scale

        mjv_defaultScene(&scn_);
        mjr_defaultContext(&con_);

        mjv_makeScene(m_, &scn_, 2000);
        mjr_makeContext(m_, &con_, mjFONTSCALE_150);

        // Set up mouse callbacks
        glfwSetWindowUserPointer(window_, this);
        glfwSetMouseButtonCallback(window_, mouse_button_callback);
        glfwSetCursorPosCallback(window_, mouse_move_callback);
        glfwSetScrollCallback(window_, scroll_callback);
    }

    void render() {
        if (!visualize_ || !window_) return;

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
        mjv_updateScene(m_, d_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
        mjr_render(viewport, &scn_, &con_);
        glfwSwapBuffers(window_);
        glfwPollEvents();
    }

    // Mouse interaction handlers
    static void mouse_button_callback(GLFWwindow* window, int button, int act, int mods) {
        MujocoWrapper* wrapper = static_cast<MujocoWrapper*>(glfwGetWindowUserPointer(window));
        wrapper->mouseButton(button, act, mods);
    }

    static void mouse_move_callback(GLFWwindow* window, double xpos, double ypos) {
        MujocoWrapper* wrapper = static_cast<MujocoWrapper*>(glfwGetWindowUserPointer(window));
        wrapper->mouseMove(xpos, ypos);
    }

    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
        MujocoWrapper* wrapper = static_cast<MujocoWrapper*>(glfwGetWindowUserPointer(window));
        wrapper->scroll(yoffset);
    }

    void mouseButton(int button, int act, int mods) {
        // Update button states
        button_left_ = (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
        button_middle_ = (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
        button_right_ = (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

        // Save mouse position
        glfwGetCursorPos(window_, &lastx_, &lasty_);
    }

    void mouseMove(double xpos, double ypos) {
        // Return if no buttons are pressed
        if (!button_left_ && !button_middle_ && !button_right_) {
            return;
        }

        // Compute mouse displacement
        double dx = xpos - lastx_;
        double dy = ypos - lasty_;
        lastx_ = xpos;
        lasty_ = ypos;

        // Get window size
        int width, height;
        glfwGetWindowSize(window_, &width, &height);

        // Get shift key state
        bool mod_shift = (glfwGetKey(window_, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || 
                         glfwGetKey(window_, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

        // Determine action based on mouse button
        mjtMouse action;
        if (button_right_) {
            action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        }
        else if (button_left_) {
            action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        }
        else {
            action = mjMOUSE_ZOOM;
        }

        // Move camera
        mjv_moveCamera(m_, action, dx/height, dy/height, &scn_, &cam_);
    }

    void scroll(double yoffset) {
        // Zoom camera using scroll wheel
        mjv_moveCamera(m_, mjMOUSE_ZOOM, 0, 0.05 * yoffset, &scn_, &cam_);
    }

    // Mouse state variables
    bool button_left_ = false;
    bool button_middle_ = false;
    bool button_right_ = false;
    double lastx_ = 0;
    double lasty_ = 0;

    mjModel* m_ = nullptr;
    mjData* d_ = nullptr;
    bool visualize_;

    // Visualization objects
    GLFWwindow* window_ = nullptr;
    mjvCamera cam_;
    mjvOption opt_;
    mjvScene scn_;
    mjrContext con_;
};

} // namespace prx 