// Copyright (c) 2025 Sangtaek Lee
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "mujoco_ros2_control/mujoco_rendering.hpp"

#include "sensor_msgs/image_encodings.hpp"

namespace mujoco_ros2_control
{
MujocoRendering *MujocoRendering::instance_ = nullptr;

MujocoRendering *MujocoRendering::get_instance()
{
  if (instance_ == nullptr)
  {
    instance_ = new MujocoRendering();
  }

  return instance_;
}

MujocoRendering::MujocoRendering()
    : mj_model_(nullptr),
      mj_data_(nullptr),
      button_left_(false),
      button_middle_(false),
      button_right_(false),
      lastx_(0.0),
      lasty_(0.0),
      selected_body_(-1),
      selected_geom_(-1),
      force_mode_(false),
      force_scale_(10.0),
      force_active_(false),
      last_click_time_(0.0),
      double_click_time_(0.3)
{
  // Initialize force vectors
  force_pos_[0] = force_pos_[1] = force_pos_[2] = 0.0;
  force_vec_[0] = force_vec_[1] = force_vec_[2] = 0.0;
}

void MujocoRendering::init(mjModel *mujoco_model, mjData *mujoco_data)
{
  mj_model_ = mujoco_model;
  mj_data_ = mujoco_data;

  // create window, make OpenGL context current, request v-sync
  glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
  glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
  window_ = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window_);

  // initialize visualization data structures
  mjv_defaultCamera(&mjv_cam_);
  mjv_defaultOption(&mjv_opt_);
  mjv_defaultScene(&mjv_scn_);
  mjr_defaultContext(&mjr_con_);

  mjv_cam_.type = mjCAMERA_FREE;
  mjv_cam_.distance = 2.;

  // create scene and context
  mjv_makeScene(mj_model_, &mjv_scn_, 2000);
  mjr_makeContext(mj_model_, &mjr_con_, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window_, &MujocoRendering::keyboard_callback);
  glfwSetCursorPosCallback(window_, &MujocoRendering::mouse_move_callback);
  glfwSetMouseButtonCallback(window_, &MujocoRendering::mouse_button_callback);
  glfwSetScrollCallback(window_, &MujocoRendering::scroll_callback);

  // Enable vertical sync for real-time rendering
  // This ensures the rendering is synchronized with the monitor refresh rate
  glfwSwapInterval(1);
}

bool MujocoRendering::is_close_flag_raised() { return glfwWindowShouldClose(window_); }

void MujocoRendering::update()
{
  // get framebuffer viewport
  mjrRect viewport = {0, 0, 0, 0};
  glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
  glfwMakeContextCurrent(window_);

  // Reset the buffer
  mjr_setBuffer(mjFB_WINDOW, &mjr_con_);

  // update scene and render
  mjv_updateScene(mj_model_, mj_data_, &mjv_opt_, NULL, &mjv_cam_, mjCAT_ALL, &mjv_scn_);
  mjr_render(viewport, &mjv_scn_, &mjr_con_);

  // swap OpenGL buffers (blocking call due to v-sync)
  glfwSwapBuffers(window_);

  // process pending GUI events, call GLFW callbacks
  glfwPollEvents();
}

void MujocoRendering::close()
{
  // free visualization storage
  mjv_freeScene(&mjv_scn_);
  mjr_freeContext(&mjr_con_);
  glfwDestroyWindow(window_);

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif
}

void MujocoRendering::keyboard_callback(
  GLFWwindow *window, int key, int scancode, int act, int mods)
{
  get_instance()->keyboard_callback_impl(window, key, scancode, act, mods);
}

void MujocoRendering::mouse_button_callback(GLFWwindow *window, int button, int act, int mods)
{
  get_instance()->mouse_button_callback_impl(window, button, act, mods);
}

void MujocoRendering::mouse_move_callback(GLFWwindow *window, double xpos, double ypos)
{
  get_instance()->mouse_move_callback_impl(window, xpos, ypos);
}

void MujocoRendering::scroll_callback(GLFWwindow *window, double xoffset, double yoffset)
{
  get_instance()->scroll_callback_impl(window, xoffset, yoffset);
}

void MujocoRendering::keyboard_callback_impl(
  GLFWwindow * /* window */, int key, int /* scancode */, int act, int /* mods */)
{
  // backspace: reset simulation
  if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
  {
    mj_resetData(mj_model_, mj_data_);
    mj_forward(mj_model_, mj_data_);
  }
}

void MujocoRendering::mouse_button_callback_impl(
  GLFWwindow *window, int button, int act, int mods)
{
  // update button state
  button_left_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx_, &lasty_);
  
  // Check for Ctrl key (force mode)
  bool ctrl_pressed = (mods & GLFW_MOD_CONTROL) != 0;
  force_mode_ = ctrl_pressed;
  
  // Handle left mouse button events
  if (button == GLFW_MOUSE_BUTTON_LEFT)
  {
    if (act == GLFW_PRESS)
    {
      double current_time = glfwGetTime();
      
      // Check for double-click (object selection)
      if (current_time - last_click_time_ < double_click_time_)
      {
        // Double-click detected - select object
        select_object(lastx_, lasty_);
        force_active_ = false;  // Clear any active force
      }
      else if (ctrl_pressed)
      {
        // Ctrl + left click - start force application
        if (selected_body_ >= 0)
        {
          force_active_ = true;
          // Get the position where force will be applied
          select_object(lastx_, lasty_);
        }
      }
      
      last_click_time_ = current_time;
    }
    else if (act == GLFW_RELEASE)
    {
      // Stop force application when mouse is released
      if (force_active_)
      {
        force_active_ = false;
        clear_force();
      }
    }
  }
}

void MujocoRendering::mouse_move_callback_impl(GLFWwindow *window, double xpos, double ypos)
{
  // compute mouse displacement, save
  double dx = xpos - lastx_;
  double dy = ypos - lasty_;
  lastx_ = xpos;
  lasty_ = ypos;

  // Handle force application (Ctrl + left mouse drag)
  if (force_active_ && button_left_ && force_mode_)
  {
    apply_force(dx, dy);
    return;  // Don't move camera when applying force
  }

  // no buttons down: nothing to do
  if (!button_left_ && !button_middle_ && !button_right_)
  {
    return;
  }

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift =
    (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
     glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right_)
  {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  }
  else if (button_left_)
  {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  }
  else
  {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(mj_model_, action, dx / height, dy / height, &mjv_scn_, &mjv_cam_);
}

void MujocoRendering::scroll_callback_impl(
  GLFWwindow * /* window */, double /* xoffset */, double yoffset)
{
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(mj_model_, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &mjv_scn_, &mjv_cam_);
}

void MujocoRendering::select_object(double xpos, double ypos)
{
  if (!mj_model_ || !mj_data_)
    return;

  // Get framebuffer size
  int width, height;
  glfwGetFramebufferSize(window_, &width, &height);

  // Ray casting for object selection
  mjtNum pnt[3];
  int sel;
  
  // Find closest object using mjv_select
  double aspectratio = (double)width / height;
  double relx = xpos / width;
  double rely = 1.0 - ypos / height;  // Flip Y coordinate
  
  sel = mjv_select(mj_model_, mj_data_, &mjv_opt_, aspectratio, relx, rely, 
                   &mjv_scn_, pnt, &selected_body_, &selected_geom_, NULL);
  
  if (sel >= 0)
  {
    // Object selected
    selected_body_ = mj_model_->geom_bodyid[sel];
    selected_geom_ = sel;
    
    // Store the position where force will be applied
    force_pos_[0] = pnt[0];
    force_pos_[1] = pnt[1];
    force_pos_[2] = pnt[2];
    
    printf("Selected body %d, geom %d at position (%.3f, %.3f, %.3f)\n", 
           selected_body_, selected_geom_, pnt[0], pnt[1], pnt[2]);
  }
  else
  {
    // No object selected
    selected_body_ = -1;
    selected_geom_ = -1;
    printf("No object selected\n");
  }
}

void MujocoRendering::apply_force(double dx, double dy)
{
  if (!mj_model_ || !mj_data_ || selected_body_ < 0)
    return;

  // Get window size for scaling
  int width, height;
  glfwGetWindowSize(window_, &width, &height);
  
  // Convert mouse movement to force vector in world coordinates
  // This is a simplified approach - you might want to make it more sophisticated
  
  // Get camera orientation
  mjtNum cam_forward[3], cam_right[3], cam_up[3];
  
  // Compute camera vectors
  mjtNum cam_pos[3] = {mjv_cam_.lookat[0] + mjv_cam_.distance * mjv_cam_.azimuth * cos(mjv_cam_.elevation),
                       mjv_cam_.lookat[1] + mjv_cam_.distance * mjv_cam_.azimuth * sin(mjv_cam_.elevation),
                       mjv_cam_.lookat[2] + mjv_cam_.distance * sin(mjv_cam_.elevation)};
  
  // Simplified camera vectors (you can make this more sophisticated)
  cam_forward[0] = mjv_cam_.lookat[0] - cam_pos[0];
  cam_forward[1] = mjv_cam_.lookat[1] - cam_pos[1];
  cam_forward[2] = mjv_cam_.lookat[2] - cam_pos[2];
  
  // Normalize forward vector
  double forward_norm = sqrt(cam_forward[0]*cam_forward[0] + cam_forward[1]*cam_forward[1] + cam_forward[2]*cam_forward[2]);
  if (forward_norm > 0)
  {
    cam_forward[0] /= forward_norm;
    cam_forward[1] /= forward_norm;
    cam_forward[2] /= forward_norm;
  }
  
  // Right vector (cross product with up)
  cam_right[0] = cam_forward[1] * 0.0 - cam_forward[2] * 1.0;
  cam_right[1] = cam_forward[2] * 0.0 - cam_forward[0] * 0.0;
  cam_right[2] = cam_forward[0] * 1.0 - cam_forward[1] * 0.0;
  
  // Up vector (cross product with right)
  cam_up[0] = cam_right[1] * cam_forward[2] - cam_right[2] * cam_forward[1];
  cam_up[1] = cam_right[2] * cam_forward[0] - cam_right[0] * cam_forward[2];
  cam_up[2] = cam_right[0] * cam_forward[1] - cam_right[1] * cam_forward[0];
  
  // Scale factors
  double scale_x = force_scale_ / width;
  double scale_y = force_scale_ / height;
  
  // Compute force vector based on mouse movement
  force_vec_[0] = dx * scale_x * cam_right[0] - dy * scale_y * cam_up[0];
  force_vec_[1] = dx * scale_x * cam_right[1] - dy * scale_y * cam_up[1];
  force_vec_[2] = dx * scale_x * cam_right[2] - dy * scale_y * cam_up[2];
  
  // Apply force to the selected body using mj_applyFT
  // Note: mj_applyFT applies force and torque at a point
  mjtNum torque[3] = {0.0, 0.0, 0.0};  // No torque, just force
  mj_applyFT(mj_model_, mj_data_, force_vec_, torque, force_pos_, selected_body_, force_pos_);
  
  // Update force visualization
  update_force_visualization();
  
  printf("Applied force (%.3f, %.3f, %.3f) to body %d\n", 
         force_vec_[0], force_vec_[1], force_vec_[2], selected_body_);
}

void MujocoRendering::update_force_visualization()
{
  // This method can be used to visualize the applied force
  // For now, we'll just print the force magnitude
  double force_magnitude = sqrt(force_vec_[0]*force_vec_[0] + 
                               force_vec_[1]*force_vec_[1] + 
                               force_vec_[2]*force_vec_[2]);
  
  if (force_magnitude > 0.1)  // Only show significant forces
  {
    printf("Force magnitude: %.3f N\n", force_magnitude);
  }
}

void MujocoRendering::clear_force()
{
  // Clear the force vectors
  force_vec_[0] = force_vec_[1] = force_vec_[2] = 0.0;
  force_active_ = false;
  printf("Force cleared\n");
}

}  // namespace mujoco_ros2_control
