#include "LIPSimulator.hpp"

#include <stdexcept>

namespace Simulator
{

    LIPSimulator::LIPSimulator (const std::string& XML_PATH, const std::string& title)
    {
        // compile xml model
        if (!this->LoadModel(XML_PATH))
        {
            throw std::runtime_error("[LIPSimulator] failed to load MuJoCo scene: " + XML_PATH);
        }

        // glfw window and renderer
        if (!this->InitWindow(title))
        {
            this->FreeAll();
            throw std::runtime_error("[LIPSimulator] cannot open the window (no display?)");
        }
    }

    LIPSimulator::~LIPSimulator()
    {
        this->FreeAll();
    }

    void LIPSimulator::FreeAll()
    {
        // scene and context need the window's GL context
        this->FreeScene();
        if (this->window_)
        {
            glfwDestroyWindow(this->window_);
            glfwTerminate();
            this->window_ = nullptr;
        }
        // data first: it belongs to the model
        if (this->mj_data_) mj_deleteData(this->mj_data_);
        if (this->mj_model_) mj_deleteModel(this->mj_model_);
        this->mj_data_ = nullptr;
        this->mj_model_ = nullptr;
    }

    bool LIPSimulator::LoadModel(const std::string& xml_path)
    {
        // free a previously loaded model (and its scene)
        this->FreeScene();
        if (this->mj_data_) mj_deleteData(this->mj_data_);
        if (this->mj_model_) mj_deleteModel(this->mj_model_);
        this->mj_data_ = nullptr;

        // compile xml model
        char loadError[1024] = "";
        this->mj_model_ = mj_loadXML(xml_path.c_str(), nullptr, loadError, sizeof(loadError));
        if (!this->mj_model_) {
            std::fprintf(stderr, "[LIPSimulator] Failed to load MuJoCo scene '%s': %s\n",
                        xml_path.c_str(), loadError);
            return false;
        }
        this->mj_data_ = mj_makeData(this->mj_model_);

        // window already open: new scene for the new model
        if (this->window_) this->MakeScene();
        return true;
    }

    bool LIPSimulator::InitWindow(const std::string& title)
    {
        // window with its GL context
        if (!glfwInit()) return false;
        this->window_ = glfwCreateWindow(1200, 900, title.c_str(), nullptr, nullptr);
        if (!this->window_)
        {
            glfwTerminate();
            return false;
        }
        glfwMakeContextCurrent(this->window_);
        glfwSwapInterval(1); // vsync

        // camera: view of SimDraw.py (elev 25, azim -60)
        mjv_defaultCamera(&this->mjv_cam_);
        mjv_defaultOption(&this->mjv_opt_);
        this->mjv_cam_.azimuth = -60;
        this->mjv_cam_.elevation = -25;
        this->mjv_cam_.distance = 1.6;
        this->mjv_cam_.lookat[2] = 0.2;
        this->MakeScene();

        // callbacks reach this object through the user pointer
        glfwSetWindowUserPointer(this->window_, this);
        glfwSetKeyCallback(this->window_, &LIPSimulator::keyboard);
        glfwSetMouseButtonCallback(this->window_, &LIPSimulator::mouseButton);
        glfwSetCursorPosCallback(this->window_, &LIPSimulator::mouseMove);
        glfwSetScrollCallback(this->window_, &LIPSimulator::scroll);
        return true;
    }

    void LIPSimulator::MakeScene()
    {
        mjv_defaultScene(&this->mjv_scn_);
        mjr_defaultContext(&this->mjr_context_);
        mjv_makeScene(this->mj_model_, &this->mjv_scn_, 4000); // room for the trajectories
        mjr_makeContext(this->mj_model_, &this->mjr_context_, mjFONTSCALE_150);
        this->scene_made_ = true;
    }

    void LIPSimulator::FreeScene()
    {
        if (!this->scene_made_) return;
        mjv_freeScene(&this->mjv_scn_);
        mjr_freeContext(&this->mjr_context_);
        this->scene_made_ = false;
    }

    bool LIPSimulator::ConsumeRestart()
    {
        const bool r = this->restart_;
        this->restart_ = false;
        return r;
    }

    int LIPSimulator::MocapId(const char* body) const
    {
        const int id = mj_name2id(this->mj_model_, mjOBJ_BODY, body);
        return id < 0 ? -1 : this->mj_model_->body_mocapid[id];
    }

    void LIPSimulator::SetMocap(int mocap_id, const Eigen::Vector3d& pos)
    {
        if (mocap_id < 0) return;
        for (int i = 0; i < 3; ++i) this->mj_data_->mocap_pos[3 * mocap_id + i] = pos(i);
    }

    void LIPSimulator::FollowTarget(double x, double y, double gain)
    {
        this->mjv_cam_.lookat[0] += gain * (x - this->mjv_cam_.lookat[0]);
        this->mjv_cam_.lookat[1] += gain * (y - this->mjv_cam_.lookat[1]);
    }

    void LIPSimulator::BeginFrame()
    {
        // markers -> geoms, then the model geoms of the scene
        mj_forward(this->mj_model_, this->mj_data_);
        glfwGetFramebufferSize(this->window_, &this->viewport_.width, &this->viewport_.height);
        mjv_updateScene(this->mj_model_, this->mj_data_, &this->mjv_opt_, nullptr, &this->mjv_cam_,
                        mjCAT_ALL, &this->mjv_scn_);
    }

    void LIPSimulator::AddConnector(int type, double width, const Eigen::Vector3d& from,
                                    const Eigen::Vector3d& to, const float rgba[4])
    {
        if (this->mjv_scn_.ngeom >= this->mjv_scn_.maxgeom) return; // scene full: skip
        mjvGeom* g = &this->mjv_scn_.geoms[this->mjv_scn_.ngeom++];
        mjv_initGeom(g, mjGEOM_NONE, nullptr, nullptr, nullptr, rgba);
        mjv_connector(g, type, width, from.data(), to.data());
        g->category = mjCAT_DECOR;
    }

    void LIPSimulator::AddPolyline(const std::deque<Eigen::Vector3d>& points, double width, const float rgba[4])
    {
        for (std::size_t i = 1; i < points.size(); ++i)
            this->AddConnector(mjGEOM_LINE, width, points[i - 1], points[i], rgba);
    }

    void LIPSimulator::EndFrame(const char* labels, const char* values)
    {
        mjr_render(this->viewport_, &this->mjv_scn_, &this->mjr_context_);
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, this->viewport_, labels, values, &this->mjr_context_);
        glfwSwapBuffers(this->window_);
        glfwPollEvents();
    }

    // ---- GLFW callbacks ----
    void LIPSimulator::keyboard(GLFWwindow* window, int key, int, int act, int)
    {
        auto* self = static_cast<LIPSimulator*>(glfwGetWindowUserPointer(window));
        if (!self || act != GLFW_PRESS) return;
        if (key == GLFW_KEY_SPACE) self->paused_ = !self->paused_;                     // pause / run
        if (key == GLFW_KEY_UP) self->vx_cmd_ += 0.05;                                 // faster
        if (key == GLFW_KEY_DOWN) self->vx_cmd_ = std::max(0.0, self->vx_cmd_ - 0.05); // slower (forward only)
        if (key == GLFW_KEY_BACKSPACE) self->restart_ = true;                          // back to the start pose
    }

    void LIPSimulator::mouseButton(GLFWwindow* window, int, int, int)
    {
        auto* self = static_cast<LIPSimulator*>(glfwGetWindowUserPointer(window));
        if (!self) return;
        self->button_left_ = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        self->button_right_ = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
        glfwGetCursorPos(window, &self->last_x_, &self->last_y_);
    }

    void LIPSimulator::mouseMove(GLFWwindow* window, double x, double y)
    {
        auto* self = static_cast<LIPSimulator*>(glfwGetWindowUserPointer(window));
        if (!self || (!self->button_left_ && !self->button_right_)) return;
        const double dx = x - self->last_x_, dy = y - self->last_y_;
        self->last_x_ = x;
        self->last_y_ = y;
        int width, height;
        glfwGetWindowSize(window, &width, &height);
        // left: rotate, right: move in the horizontal plane
        const mjtMouse action = self->button_right_ ? mjMOUSE_MOVE_H : mjMOUSE_ROTATE_V;
        mjv_moveCamera(self->mj_model_, action, dx / height, dy / height, &self->mjv_cam_);
    }

    void LIPSimulator::scroll(GLFWwindow* window, double, double yoffset)
    {
        auto* self = static_cast<LIPSimulator*>(glfwGetWindowUserPointer(window));
        if (!self) return;
        mjv_moveCamera(self->mj_model_, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &self->mjv_cam_);
    }
}
