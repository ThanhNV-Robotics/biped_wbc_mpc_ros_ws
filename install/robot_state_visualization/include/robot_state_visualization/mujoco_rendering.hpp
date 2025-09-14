
#include <cstdio>
#include <cstring>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

namespace robot_state_visualization
{

class MujocoRendering
{
public:

  static MujocoRendering *get_instance();
  void init(mjModel *mujoco_model, mjData *mujoco_data);
  bool is_close_flag_raised();
  void update();
  void close();

private:
  MujocoRendering();
  static void keyboard_callback(GLFWwindow *window, int key, int scancode, int act, int mods);
  static void mouse_button_callback(GLFWwindow *window, int button, int act, int mods);
  static void mouse_move_callback(GLFWwindow *window, double xpos, double ypos);
  static void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);

  void keyboard_callback_impl(GLFWwindow *window, int key, int scancode, int act, int mods);
  void mouse_button_callback_impl(GLFWwindow *window, int button, int act, int mods);
  void mouse_move_callback_impl(GLFWwindow *window, double xpos, double ypos);
  void scroll_callback_impl(GLFWwindow *window, double xoffset, double yoffset);

  static MujocoRendering *instance_;

  mjModel *mj_model_;
  mjData *mj_data_;

  // Window and primary camera for the simulation's viewer
  GLFWwindow *window_;
  mjvCamera mjv_cam_;

  // Options for the rendering context and scene, all of these are hard coded to defaults.
  mjvOption mjv_opt_;
  mjvScene mjv_scn_;
  mjrContext mjr_con_;

  bool button_left_;
  bool button_middle_;
  bool button_right_;
  double lastx_;
  double lasty_;
};
} 