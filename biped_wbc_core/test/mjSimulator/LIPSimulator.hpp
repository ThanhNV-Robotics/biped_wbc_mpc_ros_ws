#pragma once

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <deque>
#include <memory>
#include <string>

#include <Eigen/Dense>

namespace Simulator
{
    // MuJoCo viewer of the LIP visual tests (no physics): a scene of mocap markers, a GLFW window
    // with mouse camera and keyboard requests, decoration lines added each frame.
    // Frame loop: SetMocap(...) -> BeginFrame() -> AddConnector / AddPolyline -> EndFrame(text).
    // Keys: space = pause / run, up / down = velocity command +- 0.05 m/s, backspace = restart.
    // Mouse: left drag = rotate, right drag = move, scroll = zoom.
    class LIPSimulator
    {
        public:
            // colors (as SimDraw.py)
            const float black[4] = {0.f, 0.f, 0.f, 0.5f};
            const float green[4] = {0.13f, 0.55f, 0.13f, 0.85f};
            const float orange[4] = {1.f, 0.55f, 0.f, 0.85f};
            const float blue[4] = {0.f, 0.f, 1.f, 0.4f};
            const float com_traj[4] = {0.f, 0.f, 1.f, 0.7f};
            const float zmp_traj[4] = {1.f, 0.f, 0.f, 0.7f};
            // loads the scene and opens the window; throws std::runtime_error if either fails
            LIPSimulator (const std::string& XML_PATH, const std::string& title = "LIP task planner");
            // frees the scene, the window and the MuJoCo model and data
            virtual ~LIPSimulator();
            // owns raw MuJoCo / GLFW pointers: a copy would free them twice
            LIPSimulator(const LIPSimulator&) = delete;
            LIPSimulator& operator=(const LIPSimulator&) = delete;

            // compiles the xml into mj_model_ / mj_data_ (replaces a loaded one, and its scene if the
            // window is open); false on failure
            bool LoadModel(const std::string& XML_PATH);

            // ---- window and keyboard requests ----
            bool IsOpen() const { return this->window_ && !glfwWindowShouldClose(this->window_); }
            bool IsPaused() const { return this->paused_; }
            // true once per backspace press
            bool ConsumeRestart();
            // [m/s] forward velocity command of the up / down keys (forward only)
            double GetVelocityCommand() const { return this->vx_cmd_; }
            void SetVelocityCommand(double vx) { this->vx_cmd_ = std::max(0.0, vx); }

            // ---- markers ----
            // mocap index of a body declared with mocap="true", -1 if none
            int MocapId(const char* body) const;
            // sets the position of a mocap body (ignored for id < 0)
            void SetMocap(int mocap_id, const Eigen::Vector3d& pos);
            // camera look-at follows (x, y), low-pass filtered with gain per frame
            void FollowTarget(double x, double y, double gain = 0.03);

            // ---- frame ----
            // updates the model geoms of the scene (call after the SetMocap of this frame)
            void BeginFrame();
            // adds a line / capsule from 'from' to 'to' to the scene (decoration only)
            void AddConnector(int type, double width, const Eigen::Vector3d& from, const Eigen::Vector3d& to,
                              const float rgba[4]);
            // adds the lines joining consecutive points
            void AddPolyline(const std::deque<Eigen::Vector3d>& points, double width, const float rgba[4]);
            // renders the scene with a top-left text overlay (labels / values columns), swaps, polls events
            void EndFrame(const char* labels, const char* values);

        private:
            // GLFW window, renderer and callbacks; false on failure
            bool InitWindow(const std::string& title);
            // scene and render context of mj_model_
            void MakeScene();
            void FreeScene();
            // frees everything (destructor, failed constructor)
            void FreeAll();

            // GLFW callbacks are plain functions: they reach the simulator through the window user pointer
            static void keyboard(GLFWwindow* window, int key, int, int act, int);
            static void mouseButton(GLFWwindow* window, int, int, int);
            static void mouseMove(GLFWwindow* window, double x, double y);
            static void scroll(GLFWwindow* window, double, double yoffset);

            GLFWwindow* window_ = nullptr;
            mjModel* mj_model_ = nullptr;
            mjData* mj_data_ = nullptr;

            mjvCamera mjv_cam_;
            mjvOption mjv_opt_;
            mjvScene mjv_scn_;
            mjrContext mjr_context_;
            bool scene_made_ = false;
            mjrRect viewport_ = {0, 0, 0, 0};

            // mouse state for the camera
            bool button_left_ = false, button_right_ = false;
            double last_x_ = 0, last_y_ = 0;

            // keyboard requests, handled in the main loop
            bool paused_ = false;
            bool restart_ = false;
            double vx_cmd_ = 0.1; // [m/s] forward velocity command
    };
}
