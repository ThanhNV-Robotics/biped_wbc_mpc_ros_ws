// This is the main class to solve the Task Space Inverse Dynamic problem based on Quadratic Programming

#pragma once
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/contacts/contact-6d.hpp>
#include <tsid/robots/robot-wrapper.hpp>
#include <tsid/solvers/solver-HQP-factory.hpp>
#include <tsid/solvers/utils.hpp>
#include <tsid/tasks/task-actuation-bounds.hpp>
#include <tsid/tasks/task-com-equality.hpp>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/tasks/task-se3-equality.hpp>
#include <tsid/trajectories/trajectory-base.hpp>

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

namespace biped_controllers
{
    class QP_TSID
    {
    public:
        // Constructors
        QP_TSID(const std::string &yaml_config_path, const std::string &urdf_path);

        // helper functions
        bool soleCornersFromUrdf(const std::string &link_name, const Eigen::Vector3d &normal,
                                             Eigen::Matrix3Xd &corners) const;
        // set up tasks function
        void setupContactTask();
        void setupSwingTask();
        void setupCoMTask();
        void setupBaseOrientationTask();
        void setupPostureTask();
        void setupActuationBoundsTask();

        // robot model (free flyer) used by TSID; the state estimator and the contact fusion use
        // the same one, so it must outlive them
        tsid::robots::RobotWrapper &getRobotWrapper() { return *this->robot_wrapper_; }
    private:
        // weight / priority of a task, read from the yaml, used when the task is added to TSID
        struct TaskSetting
        {
            double weight = 0.0;
            unsigned int priority = 1; // 0 = hard constraint, 1 = soft (weighted) task
        };

        // yaml section of the contact on the given foot ("left" / "right"): the contact_tasks entry
        // whose contact_frame contains that word; throws if there is none
        YAML::Node contactConfig(const std::string &side) const;

        std::string yaml_config_path_;
        std::string urdf_path_;
        YAML::Node yaml_config_;

        // foot frames (contact_frame of the contact_tasks entries), used by the contact and swing tasks
        std::string left_foot_frame_;
        std::string right_foot_frame_;

        // task settings (setup*Task)
        TaskSetting com_setting_;
        TaskSetting base_orientation_setting_;
        TaskSetting posture_setting_;
        TaskSetting actuation_bounds_setting_;
        TaskSetting swing_setting_;
        double left_contact_force_reg_weight_ = 1e-3;  // w_force_reg of addRigidContact
        double right_contact_force_reg_weight_ = 1e-3;
        double contact_transition_time_ = 0.15;        // [s] force ramp before a contact is removed

        // tsid tasks
        std::shared_ptr<tsid::tasks::TaskComEquality> com_task_; // track CoM position
        std::shared_ptr<tsid::tasks::TaskActuationBounds> actuation_bounds_task_; //
        std::shared_ptr<tsid::tasks::TaskJointPosture> posture_task_; // actuated joint tracking task
        std::shared_ptr<tsid::tasks::TaskSE3Equality> base_orientation_task_; // track base euler (orientation)
        
        // contact taskes are added to tsid based on the motion mode (single/double support)
        std::shared_ptr<tsid::contacts::Contact6d> left_foot_contact_task;
        std::shared_ptr<tsid::contacts::Contact6d> right_foot_contact_task;        
        
        // Swing foot tasks are added to tsid also based on the planned motion
        std::shared_ptr<tsid::tasks::TaskSE3Equality> left_foot_swing_task ; 
        std::shared_ptr<tsid::tasks::TaskSE3Equality> right_foot_swing_task ; 

        std::shared_ptr<tsid::robots::RobotWrapper> robot_wrapper_; // share pointer to the RobotWrapper
        // TSID problem (tasks, contacts, QP data). A pointer: its constructor needs the robot,
        // which only exists after the URDF is loaded in the constructor body. Declared after
        // robot_wrapper_: it keeps a reference to the robot, so it must be destroyed first.
        std::unique_ptr<tsid::InverseDynamicsFormulationAccForce> tsid_;        
           
    };
} // namespace biped_controllers
