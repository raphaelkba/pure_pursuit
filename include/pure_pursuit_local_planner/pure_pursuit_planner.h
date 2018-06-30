//
// Created by unicornfarm on 24.06.18.
//

#ifndef CATKIN_WS_PURE_PURSUIT_PLANNER_H
#define CATKIN_WS_PURE_PURSUIT_PLANNER_H

#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>

#include <tf/transform_listener.h>

#include <Eigen/Core>

#include <dynamic_reconfigure/server.h>

#include <pure_pursuit/PurePursuitPlannerConfig.h>

#include <nav_core/base_local_planner.h>

#include <pure_pursuit_local_planner/transform_global_plan.h>

#include <pure_pursuit_local_planner/join_costmap.h>

#include <vector>

namespace pure_pursuit_local_planner {

    class PurePursuitPlanner : public nav_core::BaseLocalPlanner {

    public:
        PurePursuitPlanner();
        ~PurePursuitPlanner();

        /**
         * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
         * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
         * @return True if a valid velocity command was found, false otherwise
         */
        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

        /**
         * @brief  Check if the goal pose has been achieved by the local planner
         * @return True if achieved, false otherwise
         */
        bool isGoalReached();

        /**
         * @brief  Set the plan that the local planner is following
         * @param plan The plan to pass to the local planner
         * @return True if the plan was updated successfully, false otherwise
         */
        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

        /**
         * @brief Constructs the local planner
         * @param name The name to give this instance of the local planner
         * @param tf A pointer to a transform listener
         * @param costmap_ros The cost map to use for assigning costs to local plans
         */
        void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

    private:
        /**
        *@brief Reconfigure config_
        */
        void reconfigureCB(PurePursuitPlannerConfig &config, uint32_t level);
        /**
          * @brief Get robot coordinates in local frame
          * @param name The name to give this instance of the local planner
          */
        void getGoalLocalCoordinates(std::vector<double> &localCoordiantes,
                                     tf::Stamped<tf::Pose> globalCoordinates,
                                        double look_ahead);

        void setControls(std::vector<double> look_ahead,geometry_msgs::Twist& cmd_vel, double yaw);

        /**
          * @brief Get euclidean distance
          * @param intial x and y points and end x and y points
          */
        double getEuclidianDistance(const double x_init, const double y_init,
                             const double x_end, const double y_end) const;
        //opublisher where the local plan for visulatation is published
        ros::Publisher local_plan_publisher_;
        //costmap to get the current position
        costmap_2d::Costmap2DROS* costmap_ros_;
        //used for transformation
        tf::TransformListener* tf_;
        //check if plan first at first time
        bool first_setPlan_;
        // true if the robot should rotate to gobal plan if new global goal set
        bool rotate_to_global_plan_;
        //true if the goal point is reache and orientation of goal is reached
        bool goal_reached_;
        //true if the goal point is reache and orientation of goal isn't reached
        bool stand_at_goal_;
        //rotation velocity of previous round for the rotateToOrientation methode
        double cmd_vel_angular_z_rotate_;
        //x velocity of the previous round
        double cmd_vel_linear_x_;
        //rotation velocity of previous round for the dirveToward methode
        double cmd_vel_angular_z_;
        //for dynamic reconfigure
        dynamic_reconfigure::Server<PurePursuitPlannerConfig> *dsrv_;
        //start config
        pure_pursuit_local_planner::PurePursuitPlannerConfig default_config_;
        //reconfigure config
        pure_pursuit_local_planner::PurePursuitPlannerConfig config_;
        //global plan which we run along
        std::vector<geometry_msgs::PoseStamped> global_plan_;
        //transformed global plan in global frame with only the points with are needed for calculation (max_points)
        std::vector<geometry_msgs::PoseStamped> transformed_global_plan_;
        //last point of the global plan in global frame
        tf::Stamped<tf::Pose> goal_pose_;
        // true if the robot should rotate to gobal plan if new global goal set
        tf::Stamped<tf::Pose> old_goal_pose_;

        JoinCostmap *joinCostmap_;

    };
};


#endif //CATKIN_WS_PURE_PURSUIT_PLANNER_H
