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

#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"

#include "mujoco_ros2_control/mujoco_cameras.hpp"
#include "mujoco_ros2_control/mujoco_rendering.hpp"
#include "mujoco_ros2_control/mujoco_ros2_control.hpp"

// for Simulate.cc
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>

#include "mujoco_ros2_control/glfw_adapter.h"
#include "mujoco_ros2_control/simulate.h"
#include "mujoco_ros2_control/array_safety.h"

// MuJoCo data structures
mjModel *mujoco_model = nullptr;
mjData *mujoco_data = nullptr;
std::unique_ptr<mujoco_ros2_control::MujocoRos2Control> mujoco_control;

namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

// constants
const double syncMisalign = 0.1;        // maximum mis-alignment before re-sync (simulation seconds)
const double simRefreshFraction = 0.7;  // fraction of refresh available for simulation
const int kErrorLength = 1024;          // load error string length
using Seconds = std::chrono::duration<double>;

//------------------------------------------- simulation -------------------------------------------

const char* Diverged(int disableflags, const mjData* d) {
  if (disableflags & mjDSBL_AUTORESET) {
    for (mjtWarning w : {mjWARN_BADQACC, mjWARN_BADQVEL, mjWARN_BADQPOS}) {
      if (d->warning[w].number > 0) {
        return mju_warningText(w, d->warning[w].lastinfo);
      }
    }
  }
  return nullptr;
}

mjModel* LoadModel(const char* file, mj::Simulate& sim) {
  // this copy is needed so that the mju::strlen call below compiles
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  // make sure filename is not empty
  if (!filename[0]) {
    return nullptr;
  }

  // load and compile
  char loadError[kErrorLength] = "";
  mjModel* mnew = 0;
  auto load_start = mj::Simulate::Clock::now();

  std::string filename_str(filename);
  std::string extension;
  size_t dot_pos = filename_str.rfind('.');

  if (dot_pos != std::string::npos && dot_pos < filename_str.length() - 1) {
    extension = filename_str.substr(dot_pos);
  }

  if (extension == ".mjb") {
    mnew = mj_loadModel(filename, nullptr);
    if (!mnew) {
      mju::strcpy_arr(loadError, "could not load binary model");
    }
  } else {
    mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);

    // remove trailing newline character from loadError
    if (loadError[0]) {
      int error_length = mju::strlen_arr(loadError);
      if (loadError[error_length-1] == '\n') {
        loadError[error_length-1] = '\0';
      }
    }
  }
  auto load_interval = mj::Simulate::Clock::now() - load_start;
  double load_seconds = Seconds(load_interval).count();

  if (!mnew) {
    std::printf("%s\n", loadError);
    mju::strcpy_arr(sim.load_error, loadError);
    return nullptr;
  }

  // compiler warning: print and pause
  if (loadError[0]) {
    // mj_forward() below will print the warning message
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
    sim.run = 0;
  }

  // if no error and load took more than 1/4 seconds, report load time
  else if (load_seconds > 0.25) {
    mju::sprintf_arr(loadError, "Model loaded in %.2g seconds", load_seconds);
  }

  mju::strcpy_arr(sim.load_error, loadError);

  return mnew;
}

// simulate in background thread (while rendering in main thread)
void PhysicsLoop(mj::Simulate& sim) {
  // cpu-sim syncronization point
  std::chrono::time_point<mj::Simulate::Clock> syncCPU;
  mjtNum syncSim = 0;

  // run until asked to exit
  while (!sim.exitrequest.load()) {
    if (sim.droploadrequest.load()) {
      sim.LoadMessage(sim.dropfilename);
      mjModel* mnew = LoadModel(sim.dropfilename, sim);
      sim.droploadrequest.store(false);

      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.dropfilename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(mujoco_data);
        mj_deleteModel(mujoco_model);

        mujoco_model = mnew;
        mujoco_data = dnew;
        mj_forward(mujoco_model, mujoco_data);

      } else {
        sim.LoadMessageClear();
      }
    }

    if (sim.uiloadrequest.load()) {
      sim.uiloadrequest.fetch_sub(1);
      sim.LoadMessage(sim.filename);
      mjModel* mnew = LoadModel(sim.filename, sim);
      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.filename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(mujoco_data);
        mj_deleteModel(mujoco_model);

        mujoco_model = mnew;
        mujoco_data = dnew;
        mj_forward(mujoco_model, mujoco_data);

      } else {
        sim.LoadMessageClear();
      }
    }

    // sleep for 1 ms or yield, to let main thread run
    //  yield results in busy wait - which has better timing but kills battery life
    if (sim.run && sim.busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

      // run only if model is present
      if (mujoco_model) {
        // running
        if (sim.run) {
          bool stepped = false;

          // record cpu time at start of iteration
          const auto startCPU = mj::Simulate::Clock::now();

          // elapsed CPU and simulation time since last sync
          const auto elapsedCPU = startCPU - syncCPU;
          double elapsedSim = mujoco_data->time - syncSim;

          // requested slow-down factor
          double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

          // misalignment condition: distance from target sim time is bigger than syncMisalign
          bool misaligned =
              std::abs(Seconds(elapsedCPU).count()/slowdown - elapsedSim) > syncMisalign;

          // out-of-sync (for any reason): reset sync times, step
          if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
              misaligned || sim.speed_changed) {
            // re-sync
            syncCPU = startCPU;
            syncSim = mujoco_data->time;
            sim.speed_changed = false;

            // run single step, let next iteration deal with timing
            //mj_step(m, d);
            mujoco_control->update();
            const char* message = Diverged(mujoco_model->opt.disableflags, mujoco_data);
            if (message) {
              sim.run = 0;
              mju::strcpy_arr(sim.load_error, message);
            } else {
              stepped = true;
            }
          }

          // in-sync: step until ahead of cpu
          else {
            bool measured = false;
            mjtNum prevSim = mujoco_data->time;

            double refreshTime = simRefreshFraction/sim.refresh_rate;

            // step while sim lags behind cpu and within refreshTime
            while (Seconds((mujoco_data->time - syncSim)*slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                   mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime)) {
              // measure slowdown before first step
              if (!measured && elapsedSim) {
                sim.measured_slowdown =
                    std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                measured = true;
              }

              // inject noise
              sim.InjectNoise();

              // call mj_step
              //mj_step(m, d);
              mujoco_control->update();
              const char* message = Diverged(mujoco_model->opt.disableflags, mujoco_data);
              if (message) {
                sim.run = 0;
                mju::strcpy_arr(sim.load_error, message);
              } else {
                stepped = true;
              }

              // break if reset
              if (mujoco_data->time < prevSim) {
                break;
              }
            }
          }

          // save current state to history buffer
          if (stepped) {
            sim.AddToHistory();
          }
        }

        // paused
        else {
          // run mj_forward, to update rendering and joint sliders
          mj_forward(mujoco_model, mujoco_data);
          sim.speed_changed = true;
        }
      }
    }  // release std::lock_guard<std::mutex>
  }
}


//-------------------------------------- physics_thread --------------------------------------------

void PhysicsThread(mj::Simulate* sim, const char* filename) {
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr) {
    sim->LoadMessage(filename);
    //mujoco_model = LoadModel(filename, *sim);
    if (mujoco_model) {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      //mujoco_data = mj_makeData(mujoco_model);
    }
    if (mujoco_data) {
      sim->Load(mujoco_model, mujoco_data, filename);

      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      mj_forward(mujoco_model, mujoco_data);

    } else {
      sim->LoadMessageClear();
    }
  }

  PhysicsLoop(*sim);

  // delete everything we allocated
  mj_deleteData(mujoco_data);
  mj_deleteModel(mujoco_model);
}

// main function
int main(int argc, const char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(
    "mujoco_ros2_control_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  RCLCPP_INFO_STREAM(node->get_logger(), "Initializing mujoco_ros2_control node...");
  auto model_path = node->get_parameter("mujoco_model_path").as_string();
  // Get file name from model_path
  std::string file_name_str = model_path.substr(model_path.find_last_of("/\\") + 1);
  const char* file_name = file_name_str.c_str();
  RCLCPP_INFO_STREAM(node->get_logger(), "File name: " << file_name);
  // load and compile model
  char error[1000] = "Could not load binary model";
  if (
    std::strlen(model_path.c_str()) > 4 &&
    !std::strcmp(model_path.c_str() + std::strlen(model_path.c_str()) - 4, ".mjb"))
  {
    mujoco_model = mj_loadModel(model_path.c_str(), 0);
  }
  else
  {
    mujoco_model = mj_loadXML(model_path.c_str(), 0, error, 1000);
  }
  if (!mujoco_model)
  {
    mju_error("Load model error: %s", error);
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco model has been successfully loaded !");
  // make data
  mujoco_data = mj_makeData(mujoco_model);

  // initialize mujoco control
  mujoco_control = std::make_unique<mujoco_ros2_control::MujocoRos2Control>(node, mujoco_model, mujoco_data);

  mujoco_control->init();
  RCLCPP_INFO_STREAM(
    node->get_logger(), "Mujoco ros2 controller has been successfully initialized !");

  // initialize mujoco visualization environment for rendering and cameras
  if (!glfwInit())
  {
    mju_error("Could not initialize GLFW");
  }
  // auto rendering = mujoco_ros2_control::MujocoRendering::get_instance();
  // rendering->init(mujoco_model, mujoco_data);
  // RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco rendering has been successfully initialized !");
  
  //--------------------------------------for GUI rendering-------------------------------
  // simulate object encapsulates the UI

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  auto sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &cam, &opt, &pert, /* is_passive = */ false
  );

  // sim->LoadMessage(file_name);

  // sim->Load(mujoco_model, mujoco_data, file_name);
  // mj_forward(mujoco_model, mujoco_data);

  // start physics thread
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), file_name);

  // start simulation UI loop (blocking call)
  while (rclcpp::ok())
  {
    sim->RenderLoop();
  }
  physicsthreadhandle.join();

  // auto cameras = std::make_unique<mujoco_ros2_control::MujocoCameras>(node);
  // cameras->init(mujoco_model);


  // // run main loop, target real-time simulation and 60 fps rendering with cameras around 6 hz
  // mjtNum last_cam_update = mujoco_data->time;
  // while (rclcpp::ok() && !sim->exitrequest.load())
  // {
  //   // advance interactive simulation for 1/60 sec
  //   //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
  //   //  this loop will finish on time for the next frame to be rendered at 60 fps.
  //   //  Otherwise add a cpu timer and exit this loop when it is time to render.
  //   mjtNum simstart = mujoco_data->time;
  //   while (mujoco_data->time - simstart < 1.0 / 60.0)
  //   {
  //     mujoco_control->update();
  //   }

  //   mj_forward(mujoco_model, mujoco_data);
  //   sim->Sync();
  //   //rendering->update();

  //   // Updating cameras at ~6 Hz
  //   // TODO(eholum): Break control and rendering into separate processes
  //   // if (simstart - last_cam_update > 1.0 / 6.0)
  //   // {
  //   //   //cameras->update(mujoco_model, mujoco_data);
  //   //   last_cam_update = simstart;
  //   // }
  // }

  //rendering->close();

  //cameras->close();

  // free MuJoCo model and data
  mj_deleteData(mujoco_data);
  mj_deleteModel(mujoco_model);

  return 0;
}