// Visual test of LIPTaskPlanner in MuJoCo (no physics): runs the planner in real time and shows
// the same elements as tsid_examples/python/SimDraw.py:
//   CoM (blue) with its ground shadow and vertical projection, ZMP (crimson) with the foot sole
//   outline, capture point (red cross), left / right foot (green / orange), pendulum rod ZMP -> CoM,
//   legs feet -> CoM, CoM and ZMP trajectories. The camera follows the CoM.
//
// Usage:
//   ros2 run biped_wbc_core test_lip_task_planner [task_planner.yaml] [LIP_model.xml]
// Keys: space = pause / run, up / down = forward velocity command +- 0.05 m/s, backspace = restart.
// Mouse: left drag = rotate, right drag = move, scroll = zoom.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <deque>
#include <memory>
#include <string>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "biped_wbc/planner/LIPTaskPlanner.hpp"

using biped_controllers::LIPTaskPlanner;

// default files (source tree, set by CMake); override with the command line
#ifndef TASK_PLANNER_YAML
#define TASK_PLANNER_YAML "task_planner.yaml"
#endif
#ifndef LIP_MODEL_XML
#define LIP_MODEL_XML "LIP_model.xml"
#endif

namespace
{
// ---- MuJoCo visualization state (global: GLFW callbacks are plain functions) ----
mjModel *m = nullptr;
mjData *d = nullptr;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;

// mouse state for the camera
bool button_left = false, button_right = false;
double last_x = 0, last_y = 0;

// keyboard requests, handled in the main loop
bool paused = false;
bool restart = false;
double vx_cmd = 0.1; // [m/s] forward velocity command -> step length vx * Tswing

void keyboard(GLFWwindow *, int key, int, int act, int)
{
    if (act != GLFW_PRESS) return;
    if (key == GLFW_KEY_SPACE) paused = !paused;                 // pause / run
    if (key == GLFW_KEY_UP) vx_cmd += 0.05;                       // faster
    if (key == GLFW_KEY_DOWN) vx_cmd = std::max(0.0, vx_cmd - 0.05); // slower (forward only)
    if (key == GLFW_KEY_BACKSPACE) restart = true;                // back to the start pose
}

void mouse_button(GLFWwindow *window, int, int, int)
{
    button_left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    button_right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    glfwGetCursorPos(window, &last_x, &last_y);
}

void mouse_move(GLFWwindow *window, double x, double y)
{
    if (!button_left && !button_right) return;
    const double dx = x - last_x, dy = y - last_y;
    last_x = x;
    last_y = y;
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    // left: rotate, right: move in the horizontal plane
    const mjtMouse action = button_right ? mjMOUSE_MOVE_H : mjMOUSE_ROTATE_V;
    mjv_moveCamera(m, action, dx / height, dy / height, &cam);
}

void scroll(GLFWwindow *, double, double yoffset)
{
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &cam);
}

// adds a line / capsule from 'from' to 'to' to the scene (decoration only)
void addConnector(int type, double width, const Vector3d &from, const Vector3d &to, const float rgba[4])
{
    if (scn.ngeom >= scn.maxgeom) return; // scene full: skip
    mjvGeom *g = &scn.geoms[scn.ngeom++];
    mjv_initGeom(g, mjGEOM_NONE, nullptr, nullptr, nullptr, rgba);
    mjv_connector(g, type, width, from.data(), to.data());
    g->category = mjCAT_DECOR;
}

// sets the position of a mocap body (bodies declared with mocap="true" in LIP_model.xml)
void setMocap(int mocap_id, const Vector3d &pos)
{
    if (mocap_id < 0) return;
    for (int i = 0; i < 3; ++i) d->mocap_pos[3 * mocap_id + i] = pos(i);
}

int mocapId(const char *body)
{
    const int id = mj_name2id(m, mjOBJ_BODY, body);
    return id < 0 ? -1 : m->body_mocapid[id];
}
} // namespace

int main(int argc, char **argv)
{
    const std::string yaml_path = argc > 1 ? argv[1] : TASK_PLANNER_YAML;
    const std::string xml_path = argc > 2 ? argv[2] : LIP_MODEL_XML;

    // ---- planner ----
    std::unique_ptr<LIPTaskPlanner> planner;
    try
    {
        planner = std::make_unique<LIPTaskPlanner>(yaml_path);
    }
    catch (const std::exception &e)
    {
        std::fprintf(stderr, "%s\n", e.what());
        return 1;
    }
    const double dt = planner->getDt();
    planner->setVelocityCommand(vx_cmd);

    // ---- MuJoCo model (markers only) ----
    char error[1000] = "";
    m = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
    if (!m)
    {
        std::fprintf(stderr, "Cannot load %s: %s\n", xml_path.c_str(), error);
        return 1;
    }
    d = mj_makeData(m);
    const int id_com = mocapId("com"), id_shadow = mocapId("com_shadow"), id_zmp = mocapId("zmp"),
              id_cp = mocapId("cp"), id_lf = mocapId("left_foot"), id_rf = mocapId("right_foot");

    // ---- window and renderer ----
    if (!glfwInit())
    {
        std::fprintf(stderr, "Cannot initialize GLFW (no display?)\n");
        return 1;
    }
    GLFWwindow *window = glfwCreateWindow(1200, 900, "LIP task planner", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // vsync

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 4000); // room for the trajectories
    mjr_makeContext(m, &con, mjFONTSCALE_150);
    cam.azimuth = -60;   // view of SimDraw.py (elev 25, azim -60)
    cam.elevation = -25;
    cam.distance = 1.6;
    cam.lookat[2] = 0.2;

    glfwSetKeyCallback(window, keyboard);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetScrollCallback(window, scroll);

    // ---- trajectory history (one point every 10 ms, last 20 s) ----
    std::deque<Vector3d> com_hist, zmp_hist;
    const std::size_t max_hist = 2000;
    int ticks = 0;

    // colors (as SimDraw.py)
    const float black[4] = {0.f, 0.f, 0.f, 0.5f};
    const float green[4] = {0.13f, 0.55f, 0.13f, 0.85f};
    const float orange[4] = {1.f, 0.55f, 0.f, 0.85f};
    const float blue[4] = {0.f, 0.f, 1.f, 0.4f};
    const float com_traj[4] = {0.f, 0.f, 1.f, 0.7f};
    const float zmp_traj[4] = {1.f, 0.f, 0.f, 0.7f};

    double sim_time = 0.0;
    auto wall_start = std::chrono::steady_clock::now();

    while (!glfwWindowShouldClose(window))
    {
        // ---- restart / velocity command from the keyboard ----
        if (restart)
        {
            planner->reset(Vector2d::Zero(), Vector3d(0.0, 0.135, 0.0), Vector3d(0.0, -0.135, 0.0));
            com_hist.clear();
            zmp_hist.clear();
            restart = false;
        }
        planner->setVelocityCommand(vx_cmd);

        // ---- run the planner in real time: as many dt steps as the wall clock advanced ----
        const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
        if (paused)
        {
            sim_time = wall; // do not catch up after a pause
        }
        while (!paused && sim_time + dt <= wall)
        {
            planner->planWalking();
            sim_time += dt;
            if (++ticks % std::max(1, static_cast<int>(0.01 / dt)) == 0)
            {
                com_hist.push_back(planner->getRefCoMPosition());
                const Vector2d z = planner->getRefZMP();
                zmp_hist.push_back(Vector3d(z.x(), z.y(), 0.0));
                if (com_hist.size() > max_hist) com_hist.pop_front();
                if (zmp_hist.size() > max_hist) zmp_hist.pop_front();
            }
        }

        // ---- planner outputs -> markers ----
        const Vector3d com = planner->getRefCoMPosition();
        const Vector2d zmp2 = planner->getRefZMP(), cp2 = planner->getRefCP();
        const Vector3d zmp(zmp2.x(), zmp2.y(), 0.0), cp(cp2.x(), cp2.y(), 0.0);
        const Vector3d lf = planner->getRefLeftFootPosition(), rf = planner->getRefRightFootPosition();
        setMocap(id_com, com);
        setMocap(id_shadow, Vector3d(com.x(), com.y(), 0.001));
        setMocap(id_zmp, Vector3d(zmp.x(), zmp.y(), 0.002));
        setMocap(id_cp, Vector3d(cp.x(), cp.y(), 0.003));
        setMocap(id_lf, lf);
        setMocap(id_rf, rf);
        mj_forward(m, d);

        // camera follows the CoM, low-pass filtered (smooths the lateral sway)
        cam.lookat[0] += 0.03 * (com.x() - cam.lookat[0]);
        cam.lookat[1] += 0.03 * (com.y() - cam.lookat[1]);

        // ---- render: model geoms, then the lines of SimDraw.py ----
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);

        addConnector(mjGEOM_CAPSULE, 0.004, zmp, com, black);  // pendulum rod
        addConnector(mjGEOM_CAPSULE, 0.010, lf, com, green);   // left leg
        addConnector(mjGEOM_CAPSULE, 0.010, rf, com, orange);  // right leg
        addConnector(mjGEOM_LINE, 1.0, Vector3d(com.x(), com.y(), 0.0), com, blue); // projection
        for (std::size_t i = 1; i < com_hist.size(); ++i)
            addConnector(mjGEOM_LINE, 2.0, com_hist[i - 1], com_hist[i], com_traj);  // CoM trajectory
        for (std::size_t i = 1; i < zmp_hist.size(); ++i)
            addConnector(mjGEOM_LINE, 1.5, zmp_hist[i - 1], zmp_hist[i], zmp_traj);  // ZMP trajectory

        mjr_render(viewport, &scn, &con);

        // ---- text overlay: time, stance leg, phase, command ----
        const char *stance[] = {"left (right swings)", "right (left swings)", "double (init)"};
        char labels[200], values[200];
        std::snprintf(labels, sizeof(labels), "time\nstance\nphase\nvx command\n%s", paused ? "PAUSED" : "");
        std::snprintf(values, sizeof(values), "%.2f s\n%s\n%.2f\n%.2f m/s\n", sim_time,
                      stance[planner->getStanceLeg()], planner->getPhase(), vx_cmd);
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, labels, values, &con);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // ---- clean up ----
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();
    return 0;
}
