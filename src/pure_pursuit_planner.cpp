//
// Created by unicornfarm on 24.06.18.
//


#include <ros/ros.h>

#include <pure_pursuit_local_planner/pure_pursuit_planner.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(pure_pursuit_local_planner::PurePursuitPlanner, nav_core::BaseLocalPlanner)

namespace pure_pursuit_local_planner {

    PurePursuitPlanner::PurePursuitPlanner() {
    }

    PurePursuitPlanner::~PurePursuitPlanner()
    {
    }

    void PurePursuitPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ros::NodeHandle private_nh("~/" + name);
        local_plan_publisher_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
        costmap_ros_ = costmap_ros;
        tf_ = tf;
        first_setPlan_ = true;
        rotate_to_global_plan_ = false;
        goal_reached_ = false;
        stand_at_goal_ = false;
        cmd_vel_angular_z_rotate_ = 0;
        cmd_vel_linear_x_ = 0;
        cmd_vel_angular_z_ = 0;

        //Parameter for dynamic reconfigure
        dsrv_ = new dynamic_reconfigure::Server<PurePursuitPlannerConfig>(private_nh);
        dynamic_reconfigure::Server<PurePursuitPlannerConfig>::CallbackType cb = boost::bind(&PurePursuitPlanner::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        joinCostmap_ = new JoinCostmap();

        ROS_INFO("PurePursuitPlanner Initialized");
    }


    void PurePursuitPlanner::reconfigureCB(PurePursuitPlannerConfig &config, uint32_t level)
    {
        if (config.restore_defaults)
        {
            config = default_config_;
            config.restore_defaults = false;
        }
        config_ = config;
    }

    bool PurePursuitPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        global_plan_ = plan;

        //First start of the local plan. First global plan.
        bool first_use = false;
        if(first_setPlan_)
        {
            first_setPlan_ = false;
            pure_pursuit_local_planner::getXPose(*tf_,global_plan_, costmap_ros_->getGlobalFrameID(),old_goal_pose_,global_plan_.size()-1);
            first_use = true;
        }

        pure_pursuit_local_planner::getXPose(*tf_,global_plan_, costmap_ros_->getGlobalFrameID(),goal_pose_,global_plan_.size()-1);
        //Have the new global plan an new goal, reset. Else dont reset.
        if(std::abs(std::abs(old_goal_pose_.getOrigin().getX())-std::abs(goal_pose_.getOrigin().getX())) <= config_.position_accuracy &&
           std::abs(std::abs(old_goal_pose_.getOrigin().getY())-std::abs(goal_pose_.getOrigin().getY())) <= config_.position_accuracy && !first_use
           && std::abs(angles::shortest_angular_distance(tf::getYaw(old_goal_pose_.getRotation()), tf::getYaw(goal_pose_.getRotation()))) <= config_.rotation_accuracy)
        {
            ROS_DEBUG("FTCPlanner: Old Goal == new Goal.");
        }
        else
        {
            //Rotate to first global plan point.
            rotate_to_global_plan_ = true;
            goal_reached_ = false;
            stand_at_goal_ = false;
            ROS_INFO("FTCPlanner: New Goal. Start new routine.");
        }

        old_goal_pose_ = goal_pose_;

        return true;
    }

    bool PurePursuitPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        tf::Stamped<tf::Pose> globalCoordinates;
        costmap_ros_->getRobotPose(globalCoordinates);
        std::vector<double> localCoordiantes;
        double look_ahead = 20;
        getGoalLocalCoordinates(localCoordiantes, globalCoordinates, look_ahead);
        double yaw = tf::getYaw(globalCoordinates.getRotation());
        setControls(localCoordiantes, cmd_vel,yaw);
        double distanceToGoal = getEuclidianDistance(globalCoordinates.getOrigin().x(),
                                                     global_plan_[global_plan_.size()-1].pose.position.x,
                                                     globalCoordinates.getOrigin().y(),
                                                     global_plan_[global_plan_.size()-1].pose.position.y);
        if(distanceToGoal < 0.1)
        {
            goal_reached_ = true;
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
        }


        return true;
    }


    bool PurePursuitPlanner::isGoalReached()
    {
        if(goal_reached_)
        {
            ROS_INFO("PurePursuitPlanner: Goal reached.");
        }
        return goal_reached_;
    }

    void PurePursuitPlanner::getGoalLocalCoordinates(std::vector<double> &localCoordiantes,
                                                     tf::Stamped<tf::Pose> globalCoordinates,
                                                     double look_ahead) {
        double x_global = globalCoordinates.getOrigin().getX();
        double y_global = globalCoordinates.getOrigin().getY();
        double x_goal_global;
        double y_goal_global;
        if (look_ahead < global_plan_.size() - 1) {
            x_goal_global = global_plan_[look_ahead].pose.position.x;
            y_goal_global = global_plan_[look_ahead].pose.position.y;
        } else {
            x_goal_global = global_plan_[global_plan_.size() - 1].pose.position.x;
            y_goal_global = global_plan_[global_plan_.size() - 1].pose.position.y;
        }

        double yaw = tf::getYaw(globalCoordinates.getRotation());

        localCoordiantes.push_back((x_goal_global - x_global) * cos(-yaw) - (y_goal_global - y_global) * sin(-yaw));
        localCoordiantes.push_back((x_goal_global - x_global) * sin(-yaw) + (y_goal_global - y_global) * cos(-yaw));

    }

    void PurePursuitPlanner::setControls(std::vector<double> look_ahead, geometry_msgs::Twist& cmd_vel, double yaw){

        double distance_square = look_ahead[0]*look_ahead[0] + look_ahead[1]*look_ahead[1];

        cmd_vel_linear_x_ = 0.5*(sqrt(distance_square));

        cmd_vel_angular_z_ = 0.5*((2*look_ahead[1]/(distance_square)));
        cmd_vel.angular.z = cmd_vel_angular_z_;
        cmd_vel.linear.x = cmd_vel_linear_x_;

    }

    double PurePursuitPlanner::getEuclidianDistance(const double x_init, const double y_init,
                                                    const double x_end, const double y_end) const {
        double x = (x_init - x_end);
        double y = (y_init - y_end);
        return sqrt(x*x + y*y);
    }

}

