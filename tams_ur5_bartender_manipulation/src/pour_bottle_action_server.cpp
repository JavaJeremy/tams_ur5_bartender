/*
Copyright (c) 2017, Daniel Ahlers, Lars Henning Kayser, Jeremias Hartz, Maham Tanveer, Oke Martensen
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#define USE_MATH_DEFINES
#include <tams_ur5_bartender_manipulation/PourBottleAction.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <sstream>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2/exceptions.h>
#include <actionlib/server/simple_action_server.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <tams_ur5_bartender_msgs/BarCollisionObjectArray.h>

//FOR NEW POURING
#include <ros/package.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <visualization_msgs/Marker.h>
#include <tams_pour/PoseStampedArray.h>
#include <rosbag/bag.h>
#include <rosbag/message_instance.h>
#include <rosbag/view.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/Mesh.h>


const std::string ARM_ID = "arm";
const std::string GRIPPER_ID = "gripper";
bool USE_BOTTLE_PUBLISHER = true;
bool OBJECTS_RECOGNIZED = true;
const tams_ur5_bartender_msgs::BarCollisionObjectArray* collision_objects_ = NULL;


const int NUM_RETRIES_AFTER_JAM = 5;
const tf::TransformListener* listener = NULL;
const int planningTime = 5;

class Visualisator {
public:

    Visualisator(std::string baseFrameName) : pnh_("~"), nh_("visualising"), originFrame_(baseFrameName) {
        visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(originFrame_, "/moveit_visual_markers"));
        //Clear collision objects and markers
        visual_tools_->removeAllCollisionObjects();
        clearVisualizations();
    };

    void clearVisualizations() {
        pathPublished_ = 0;
        visual_tools_->deleteAllMarkers();
        visual_tools_->triggerPlanningSceneUpdate();
        ros::Duration(0.1).sleep();
    }

    void publishPath(std::vector<geometry_msgs::Point> path) {
        if (pathPublished_ == 0)
            visual_tools_->publishPath(path, rviz_visual_tools::BLUE, rviz_visual_tools::SMALL);

        else
            visual_tools_->publishPath(path, rviz_visual_tools::WHITE, rviz_visual_tools::SMALL);

        visual_tools_->trigger();
        pathPublished_++;
    }

    void showTrajectory(std::vector<geometry_msgs::Pose> trajectory) {
        ROS_INFO_STREAM("Displaying trajectory");
        std::vector<geometry_msgs::Point> path;
        for (geometry_msgs::Pose& poseOfTrajectory : trajectory) {
            geometry_msgs::Point point = poseOfTrajectory.position;
            path.push_back(point);
        }
        publishPath(path);
    }

private:
    int pathPublished_ = 0;
    ros::NodeHandle pnh_;
    ros::NodeHandle nh_;
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
    const std::string originFrame_;

};

class TrajectoryPourer {
public:

    TrajectoryPourer(std::string glassFrame, std::string tableFrame, std::string endEffectorFrame) :
    handFrame_(endEffectorFrame), glassFrame_(glassFrame), tableFrame_(tableFrame), maxAcceleration_(3.0), pnh_("~"), nh_("pouring"), gripper_("gripper"), arm_("arm") {
        pose_stamped_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose_stamped", 1, true);
        tfBroadcaster_ = new tf::TransformBroadcaster;
        bagPath_ = "/informatik2/students/home/2hartz/NetbeansProjects/Pouring_Thesis/ros_ws/src/tams_pour/bags/trajectories.bag";
        //pnh_.getParam("bagPath", bagPath_);

        //initialize moveit group to move
        arm_.setPlannerId("RRTConnectkConfigDefault");
        arm_.setPlanningTime(500.0);
        arm_.clearPathConstraints();
        arm_.setPoseReferenceFrame(glassFrame_);
    };

    std::string getTableFrame() {
        return tableFrame_;
    }

    std::string getGlassFrame() {
        return glassFrame_;
    }

    std::string getEndFrame() {
        return handFrame_;
    }

    void generatePaths() {
        computeCartesianPaths();
        ROS_INFO_STREAM("Successfully computed " << computedRobotTrajectories_.size() << " Trajectories.");
    }

    void rotatePouringTrajectories() {
        ROS_INFO_STREAM("Rotating original trajectory");
        for (std::vector<geometry_msgs::Pose> &waypoints : trajectories_) {
            rotateTrajectory(waypoints);
            transformedTrajectories_.push_back(transformBottlePoseToHand(waypoints));
        }
    }

    void rotateTrajectory(std::vector<geometry_msgs::Pose> &waypoints) {

        geometry_msgs::PoseStamped armPose = arm_.getCurrentPose();
        geometry_msgs::Pose trajectoryStartPose = waypoints[0];

        float x1 = armPose.pose.position.x;
        float y1 = armPose.pose.position.y;
        float x2 = trajectoryStartPose.position.x;
        float y2 = trajectoryStartPose.position.y;

        float rotation = std::acos(
                (x1 * x2 + y1 * y2) /
                sqrt(pow(x1, 2.0) + pow(y1, 2.0)) +
                sqrt(pow(x2, 2.0) + pow(y2, 2.0))
                );

        float orginialRotation = std::atan(y2 / x2);

        float rotInDegrees = rotation * 180.00 / M_PI;
        float oRotInDegrees = orginialRotation * 180.00 / M_PI;

        ROS_WARN_STREAM("Angle between coordinate system and original waypoint: " << oRotInDegrees);
        ROS_WARN_STREAM("Angle between gripper and original waypoint: " << rotInDegrees);

        //Rotate each point
        for (geometry_msgs::Pose &waypoint : waypoints) {
            float wx = waypoint.position.x;
            float wy = waypoint.position.y;
            float radius = sqrt(pow(wx, 2.0) + pow(wy, 2.0));
            waypoint.position.x = (float) std::cos(std::abs(rotation) + std::abs(orginialRotation)) * radius;
            waypoint.position.y = (float) std::sin(std::abs(rotation) + std::abs(orginialRotation)) * radius;
        }

    }

    void computeCartesianPaths() {
        int trajectoryNo = 0;
        for (std::vector<geometry_msgs::Pose> waypoints : transformedTrajectories_) {
            //Set from where to move
            arm_.setPoseReferenceFrame(glassFrame_);
            //Create Stamped Pose to get right header frame
            geometry_msgs::PoseStamped poseStamped;
            poseStamped.header.frame_id = glassFrame_;
            poseStamped.pose = waypoints.front();
            arm_.setJointValueTarget(poseStamped);
            arm_.setStartState(arm_.getJointValueTarget());

            moveit_msgs::RobotTrajectory trajectory;
            if (computeCartesianPath(waypoints, trajectory, trajectoryNo)) {
                computedRobotTrajectories_.push_back(trajectory);
            } else {
                moveit_msgs::RobotTrajectory emptyTrajectory;
                computedRobotTrajectories_.push_back(emptyTrajectory);
            }

            trajectoryNo++;
        }
        arm_.setStartStateToCurrentState();
    }

    bool computeCartesianPath(const std::vector<geometry_msgs::Pose> waypoints, moveit_msgs::RobotTrajectory &trajectory, int trajectoryNo) {
        //Compute trajectory
        double max_step_distance = 0.5;
        ros::Time begin_time = ros::Time::now();

        //        ROS_INFO_STREAM("Computing path now!");
        double success_percentage = arm_.computeCartesianPath(waypoints, max_step_distance, 5, trajectory);
        //For Debugging
        ros::Duration duration_ros = ros::Time::now() - begin_time;
        double duration = duration_ros.toSec();
        int waypointsGiven = waypoints.size();
        int waypointsComputed = trajectory.joint_trajectory.points.size();
        ros::Duration humanTime = getStampedTrajectoryTime(trajectoryNo);
        ros::Duration robotTime = trajectory.joint_trajectory.points[trajectory.joint_trajectory.points.size() - 1].time_from_start;
        printTrajectoryInformation(success_percentage, duration, waypointsGiven, waypointsComputed, humanTime, robotTime);

        return (success_percentage >= 0.5);
    }

    void printTrajectoryInformation(double success_percentage, double duration, int waypointsGiven, int waypointsComputed, ros::Duration humanTime, ros::Duration robotTime) {
        ROS_INFO("Trajectory %.2f%% achieved, calculated in %lf seconds", success_percentage * 100.0, duration);
        ROS_INFO_STREAM("Given Waypoints:" << waypointsGiven);
        ROS_INFO_STREAM("Computed Waypoints:" << waypointsComputed);
        ROS_INFO_STREAM("Human Trajectory Time: " << humanTime << "sec");
        ROS_INFO_STREAM("Robot Trajectory Time: " << robotTime << "sec");
    }

    std::vector<geometry_msgs::Pose> getTrajectory(int trajectoryNo) {
        std::vector<geometry_msgs::Pose> waypoints;
        if (trajectoryNo >= trajectories_.size()) {
            ROS_INFO_STREAM("No trajectory with index " << trajectoryNo << " found!");
        } else {
            waypoints = trajectories_[trajectoryNo];
        }
        return waypoints;
    }

    bool stampedTrajectoryExists(int trajectoryNo) {
        bool exists = true;
        if (trajectoryNo >= stampedTrajectories_.size()) {
            exists = false;
        }
        return exists;
    }

    std::vector<geometry_msgs::PoseStamped> getStampedTrajectory(int trajectoryNo) {
        std::vector<geometry_msgs::PoseStamped> waypoints;
        if (stampedTrajectoryExists(trajectoryNo)) {
            ROS_ERROR_STREAM("No trajectory with index " << trajectoryNo << " found!");
        } else {
            waypoints = stampedTrajectories_[trajectoryNo];
        }
        return waypoints;
    }

    ros::Duration getStampedTrajectoryTime(int trajectoryNo) {
        ros::Duration duration;
        if (stampedTrajectoryExists(trajectoryNo)) {
            duration = stampedTrajectories_[trajectoryNo].back().header.stamp - stampedTrajectories_[trajectoryNo].front().header.stamp;
        }
        return duration;
    }

    std::vector<geometry_msgs::Pose> getTransformedTrajectory(int trajectoryNo) {
        std::vector<geometry_msgs::Pose> waypoints;
        if (trajectoryNo >= transformedTrajectories_.size()) {
            ROS_ERROR_STREAM("No transformed trajectory with index " << trajectoryNo << " found!");
        } else {
            waypoints = transformedTrajectories_[trajectoryNo];
        }
        return waypoints;
    }

    bool computedTrajectoryExists(int trajectoryNo) {
        bool exists = true;
        if (trajectoryNo >= computedRobotTrajectories_.size() || computedRobotTrajectories_[trajectoryNo].joint_trajectory.points.empty()) {
            exists = false;
        }
        return exists;
    }

    void set_orientation_constraint() {
        //orientation constraints
        moveit_msgs::OrientationConstraint ocm;
        ocm.link_name = "s_model_tool0";
        ocm.header.frame_id = "table_top";
        ocm.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
        ocm.absolute_x_axis_tolerance = 0.6;
        ocm.absolute_y_axis_tolerance = 0.6;
        //        ocm.absolute_x_axis_tolerance = 0.65;
        //        ocm.absolute_y_axis_tolerance = 0.65;
        ocm.absolute_z_axis_tolerance = M_PI;
        ocm.weight = 1.0;

        moveit_msgs::Constraints constraints;
        //constraints.name = "s_model_tool0:upright:20000:high";
        constraints.orientation_constraints.push_back(ocm);
        arm_.setPathConstraints(constraints);
    }

    bool playTrajectory(int trajectoryNo) {
        if (trajectoryNo >= computedRobotTrajectories_.size()) {
            ROS_ERROR_STREAM("No computed robot trajectory with index " << trajectoryNo << " found!");
            return false;
        } else {
            ROS_INFO_STREAM("Playing Trajectory " << trajectoryNo);
            //Move to first point of trajectory
            //set_orientation_constraint();
            arm_.setPoseTarget(transformedTrajectories_[trajectoryNo].front());
            arm_.move();

            //Execute
            moveit_msgs::RobotTrajectory trajectory = computedRobotTrajectories_[trajectoryNo];
            //previous computed path
            if (!executeRobotTrajectory(trajectory)) {
                ROS_ERROR_STREAM("Failed playing trajectory, trying to live compute");
                //live computed path
                if (!playLiveComputedTrajectory(trajectoryNo)) {
                    return 0;
                }
            }

            return true;
        }
    }

    bool executeRobotTrajectory(moveit_msgs::RobotTrajectory trajectory) {
        //Execute trajectory
        robot_trajectory::RobotTrajectory robot_trajectory(arm_.getRobotModel(), "arm");
        robot_trajectory.setRobotTrajectoryMsg((*arm_.getCurrentState()), trajectory);
        moveit::planning_interface::MoveGroupInterface::Plan pour;
        pour.trajectory_ = trajectory;
        return bool(arm_.execute(pour));
    }

    bool playLiveComputedTrajectory(int trajectoryNo) {
        if (trajectoryNo >= transformedTrajectories_.size()) {
            ROS_ERROR_STREAM("No trajectory with index " << trajectoryNo << " found!");
        } else {
            std::vector<geometry_msgs::Pose> waypoints;
            waypoints = transformedTrajectories_[trajectoryNo];

            moveit_msgs::RobotTrajectory trajectory;
            if (computeCartesianPath(waypoints, trajectory, trajectoryNo)) {
                if (!executeRobotTrajectory(trajectory)) {
                    ROS_ERROR_STREAM("Failed playing live computed trajectory!");
                } else {
                    return true;
                }
            } else {
                ROS_ERROR_STREAM("Failed computing live trajectory!");
            }
            return false;
        }
    }

    /**
     * trajectory_msgs/JointTrajectoryPoint.msg:
     * float64[] positions
     * float64[] velocities
     * float64[] accelerations
     * float64[] effort
     * duration time_from_start
     * ----------------------------------------
     * @param trajectory
     * @param stampedWaypoints
     */
    moveit_msgs::RobotTrajectory adjustTrajectoryVelocities(moveit_msgs::RobotTrajectory trajectory, std::vector<geometry_msgs::PoseStamped> stampedWaypoints) {
        //ROS_WARN_STREAM("Old " << trajectory);
        std::vector<trajectory_msgs::JointTrajectoryPoint>& points = trajectory.joint_trajectory.points;
        ros::Duration difference = stampedWaypoints.back().header.stamp - stampedWaypoints.front().header.stamp;
        float avgTimePerPoint = difference.toSec() / points.size();
        //ROS_WARN_STREAM("Third Point Time: " << points[2].time_from_start);
        //ROS_WARN_STREAM("Fourth Point Time: " << points[3].time_from_start);
        ROS_WARN_STREAM("avgTimePerPoint: " << avgTimePerPoint);

        //Iteriere über alle waypoints
        for (int i = 0, lp = (points.size() - 1); i < lp; i++) {
            trajectory_msgs::JointTrajectoryPoint& trajPoint = points[i];
            trajectory_msgs::JointTrajectoryPoint& trajPointNext = points[i + 1];

            geometry_msgs::PoseStamped stampedWaypoint;
            geometry_msgs::PoseStamped stampedWaypointNext;

            if ((i + 1) >= stampedWaypoints.size()) {
                stampedWaypoint = stampedWaypoints[stampedWaypoints.size() - 1];
                stampedWaypointNext = stampedWaypoints.back();
            } else {
                stampedWaypoint = stampedWaypoints[i];
                stampedWaypointNext = stampedWaypoints[i + 1];
            }

            float originalTimeDiff = (stampedWaypointNext.header.stamp - stampedWaypoint.header.stamp).toSec(); //avgTimePerPoint;
            float robotTimeDiff = (trajPointNext.time_from_start - trajPoint.time_from_start).toSec();

            //Iteriere über alle joints
            for (int a = 0, l = trajPoint.positions.size(); a < l; a++) {
                //ROS_INFO_STREAM(trajPoint);
                //ROS_INFO_STREAM(trajPointNext);

                float distDiff = trajPointNext.positions[a] - trajPoint.positions[a];
                float velocity = distDiff / originalTimeDiff;
                float orgVelocity = distDiff / robotTimeDiff;
                float acceleration = velocity / originalTimeDiff;

                //If above maximum, take max of robot
                if (acceleration > maxAcceleration_) {
                    ROS_INFO_STREAM("Distance: " << distDiff << ", vel: " << trajPoint.velocities[a] << ", Starttime: " << trajPoint.time_from_start << ", Endtime: " << trajPointNext.time_from_start);
                    ROS_WARN_STREAM("Robot time diff: " << robotTimeDiff);
                    ROS_WARN_STREAM("Human time diff: " << originalTimeDiff);
                    //ROS_WARN_STREAM("Acceleration too high: " << acceleration);
                    acceleration = maxAcceleration_;
                    velocity = acceleration * originalTimeDiff;
                }

                //For debugging
                //ROS_WARN_STREAM("Prev Velocity: " << trajPoint.velocities[a]);
                //ROS_WARN_STREAM("New Velocity: " << velocity);

                trajPoint.velocities[a] = velocity;
                trajPoint.accelerations[a] = acceleration;
            }

            trajPoint.time_from_start = (stampedWaypoint.header.stamp - stampedWaypoints.front().header.stamp);
        }
        points.back().time_from_start = (stampedWaypoints.back().header.stamp - stampedWaypoints.front().header.stamp);
        //ROS_WARN_STREAM("New " << trajectory);
        return trajectory;
    }

    bool openGripper() {
        gripper_.setNamedTarget("open");
        return bool(gripper_.move());
    }

    bool closeGripper() {
        gripper_.setNamedTarget("closed");
        return bool(gripper_.move());
    }

    std::vector<std::string> getGripperLinks() {
        return gripper_.getLinkNames();
    }

    void setBottleTipTransform() {
        geometry_msgs::Pose pose;
        //geometry_msgs::PoseStamped poseSt = arm_.getCurrentPose();
        //ROS_WARN_STREAM(poseSt);
        //pose = poseSt.pose;
        pose.position.z = bottleHalfHeight_;
        tf::poseMsgToEigen(pose, tip_transform_);
    }

    bool move_to_starting_pose() {
        arm_.setNamedTarget("pour_default");
        return bool(arm_.move());
    }

    std::vector<tams_pour::PoseStampedArray::ConstPtr> readRosBag(std::string bagPath) {
        const std::string trajectoryTopic = "trajectoryPoses";
        const std::string rigidsTopic = "/phasespace_ros/rigids";
        const std::vector<std::string> topics{trajectoryTopic, rigidsTopic};
        std::vector<tams_pour::PoseStampedArray::ConstPtr> trajectoryArray;
        rosbag::Bag bag;

        bag.open(bagPath, rosbag::bagmode::Read);
        rosbag::View view(bag, rosbag::TopicQuery(topics));
        for (const rosbag::MessageInstance& rosbagMsgInstance : view) {
            std::string topic = rosbagMsgInstance.getTopic();
            if (topic == trajectoryTopic) {
                trajectoryArray.push_back(rosbagMsgInstance.instantiate<tams_pour::PoseStampedArray>());
            }
        }
        bag.close();

        return trajectoryArray;
    }

    double getTiltAngleInDegrees(geometry_msgs::Pose p1, geometry_msgs::Pose* p2 = NULL) {
        tf::Vector3 zVector(0, 0, 1);

        tf::Quaternion q(
                p1.orientation.x,
                p1.orientation.y,
                p1.orientation.z,
                p1.orientation.w
                );
        tf::Matrix3x3 m1(q);
        tf::Vector3 v1 = m1 * zVector;
        tf::Vector3 v2 = zVector;

        if (p2 != NULL) {
            tf::Quaternion q(
                    p2->orientation.x,
                    p2->orientation.y,
                    p2->orientation.z,
                    p2->orientation.w
                    );
            tf::Matrix3x3 m2(q);
            v2 = m2 * zVector;
        }

        double scalarProductBottleGlass = v1.dot(v2);

        return (std::acos(scalarProductBottleGlass) * 180.00 / M_PI);
    }

    double getBottleTiltAngle(geometry_msgs::Pose bottlePose) {
        return getTiltAngleInDegrees(bottlePose);
    }

    std::vector<geometry_msgs::Pose> transformBottlePoseToHand(std::vector<geometry_msgs::Pose> waypoints) {
        std::vector<geometry_msgs::Pose> transformed_waypoints;
        Eigen::Affine3d affine;
        for (geometry_msgs::Pose wp : waypoints) {
            tf::poseMsgToEigen(wp, affine);
            geometry_msgs::Pose transformed_wp;
            tf::poseEigenToMsg(affine * tip_transform_.inverse(), transformed_wp);
            transformed_waypoints.push_back(transformed_wp);
        }

        return transformed_waypoints;
    }

    bool enoughDifferenceBetweenPoints(geometry_msgs::Pose p1, geometry_msgs::Pose p2) {
        double distance = sqrt(pow((p2.position.x - p1.position.x), 2.0) + pow((p2.position.y - p1.position.y), 2.0) + pow((p2.position.z - p1.position.z), 2.0));
        double angle = getTiltAngleInDegrees(p1, &p2);
        //ROS_WARN_STREAM("Angle: " << angle);
        return (distance > 0.05) || (angle > 5.0);
    }

    void generatePouringTrajectories() {
        setBottleTipTransform();

        trajectories_.clear();
        stampedTrajectories_.clear();
        transformedTrajectories_.clear();
        std::vector<tams_pour::PoseStampedArray::ConstPtr> trajectoryArrays = readRosBag(bagPath_);

        int trajectoryCount = 0;
        int maxTrajectories = 1;
        int skipPointRange = 10; //min=1 for every Point

        //Each trajectory
        for (const tams_pour::PoseStampedArray::ConstPtr trajectoryArray : trajectoryArrays) {
            std::vector<geometry_msgs::Pose> waypoints;
            std::vector<geometry_msgs::PoseStamped> stampedWaypoints;

            //Each point of trajectory
            int pointCount = 0;
            geometry_msgs::Pose previousPose;
            for (const geometry_msgs::PoseStamped& bottlePoseStamped : trajectoryArray->data) {
                geometry_msgs::Pose bottlePose;
                bottlePose = bottlePoseStamped.pose;

                //Get only relevant points when bottle tilted more than X degrees
                if (pointCount % skipPointRange == 0 && getBottleTiltAngle(bottlePose) > 25) {
                    if (waypoints.size() == 0 || enoughDifferenceBetweenPoints(bottlePose, previousPose)) {
                        waypoints.push_back(bottlePose);
                        stampedWaypoints.push_back(bottlePoseStamped);
                        previousPose = bottlePose;
                    }
                }

                pointCount++;

            }

            if (trajectoryCount < maxTrajectories) {
                trajectories_.push_back(waypoints);
                //stampedTrajectories_.push_back(stampedWaypoints);
            }

            trajectoryCount++;
        }

        ROS_INFO_STREAM("Generated " << trajectories_.size() << " Trajectories.");
    }

    std::vector<geometry_msgs::Pose> filterToLowest(std::vector<geometry_msgs::Pose> waypoints) {
        std::vector<geometry_msgs::Pose> filteredWaypoints;
        filteredWaypoints.push_back(waypoints.front());

        double lowest = waypoints.front().position.z;
        int lowestIndex = 0;
        int count = 0;
        for (geometry_msgs::Pose bottlePose : waypoints) {
            //ROS_INFO_STREAM(bottlePose.position.z);
            if (bottlePose.position.z < lowest) {
                lowest = bottlePose.position.z;
                lowestIndex = count;
            }
            count++;
        }

        //ROS_INFO_STREAM("Lowest: " << lowest << " - " << lowestIndex);

        filteredWaypoints.push_back(waypoints[lowestIndex]);
        filteredWaypoints.push_back(waypoints.back());

        return filteredWaypoints;
    }

    std_msgs::ColorRGBA getBlue() {
        std_msgs::ColorRGBA color;
        color.r = 0.427f;
        color.g = 0.776f;
        color.b = 0.929f;
        color.a = 1.0;

        return color;
    }

    /* Collision objects */
    void importMeshFromResource(const std::string& resource, shape_msgs::Mesh& mesh_msg, float scale) {

        Eigen::Vector3d scaling(scale, scale, scale);
        shapes::Shape* shape = shapes::createMeshFromResource(resource, scaling);
        shapes::ShapeMsg shape_msg;
        shapes::constructMsgFromShape(shape, shape_msg);
        mesh_msg = boost::get<shape_msgs::Mesh>(shape_msg);
    }

    moveit_msgs::CollisionObject getCollisionObjectMsg(const std::string object_id, const std::string frame_id, const shape_msgs::Mesh& mesh_msg, const geometry_msgs::Pose& mesh_pose, const shape_msgs::SolidPrimitive& primitive_msg, const geometry_msgs::Pose& primitive_pose) {
        // create object collision object
        moveit_msgs::CollisionObject object;
        object.header.frame_id = frame_id;
        object.operation = moveit_msgs::CollisionObject::ADD;
        object.id = object_id;
        object.meshes.push_back(mesh_msg);
        object.mesh_poses.push_back(mesh_pose);

        return object;
    }

    moveit_msgs::AttachedCollisionObject getAttachedCollisionObjectMsg(const std::string object_id, const std::string frame_id, const shape_msgs::Mesh& mesh_msg, const geometry_msgs::Pose& mesh_pose, const shape_msgs::SolidPrimitive& primitive_msg, const geometry_msgs::Pose& primitive_pose) {
        // create object collision object
        moveit_msgs::CollisionObject object = getCollisionObjectMsg(object_id, frame_id, mesh_msg, mesh_pose, primitive_msg, primitive_pose);

        moveit_msgs::AttachedCollisionObject attached_object;
        attached_object.link_name = frame_id;
        attached_object.object = object;
        attached_object.touch_links = touch_links_;

        return attached_object;
    }

    moveit_msgs::CollisionObject getMeshObject(const std::string object_id, float x_pos, float y_pos, const std::string mesh_res, float mesh_height, float frame_height = -1.0) {
        // the first primitive defines the object frame and grasp pose in case of the bottle
        shape_msgs::SolidPrimitive primitive;
        primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
        primitive.dimensions.push_back(0.01);
        primitive.dimensions.push_back(0.01);

        geometry_msgs::Pose primitive_pose;
        primitive_pose.orientation.w = 1.0;
        primitive_pose.position.x = x_pos;
        primitive_pose.position.y = y_pos;
        if (frame_height == -1.0)
            frame_height = mesh_height / 2;
        primitive_pose.position.z = frame_height;

        shape_msgs::Mesh mesh;
        importMeshFromResource(mesh_res, mesh, 1.0);
        geometry_msgs::Pose mesh_pose;
        mesh_pose.orientation.w = 1.0;
        mesh_pose.position.x = x_pos;
        mesh_pose.position.y = y_pos;
        mesh_pose.position.z = mesh_height;

        return getCollisionObjectMsg(object_id, tableFrame_, mesh, mesh_pose, primitive, primitive_pose);
    }

    //Careful this is a copy of getMeshObject except for the last line!!!

    moveit_msgs::AttachedCollisionObject getMeshAttachedObject(const std::string object_id, float x_pos, float y_pos, const std::string mesh_res, float mesh_height, float frame_height = -1.0) {
        // the first primitive defines the object frame and grasp pose in case of the bottle
        shape_msgs::SolidPrimitive primitive;
        primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
        primitive.dimensions.push_back(0.01);
        primitive.dimensions.push_back(0.01);

        geometry_msgs::Pose primitive_pose;
        primitive_pose.orientation.w = 1.0;
        primitive_pose.position.x = x_pos;
        primitive_pose.position.y = y_pos;
        if (frame_height == -1.0)
            frame_height = mesh_height / 2;
        primitive_pose.position.z = frame_height;

        shape_msgs::Mesh mesh;
        importMeshFromResource(mesh_res, mesh, 1.0);
        geometry_msgs::Pose mesh_pose;
        mesh_pose.orientation.w = 1.0;
        mesh_pose.position.x = x_pos;
        mesh_pose.position.y = y_pos;
        mesh_pose.position.z = mesh_height;

        return getAttachedCollisionObjectMsg(object_id, handFrame_, mesh, mesh_pose, primitive, primitive_pose);
    }

    void addBottle() {
        //Add glass

        moveit::planning_interface::PlanningSceneInterface psi;
        std::string meshPath = "package://tams_pour/meshes/bottle_binary.stl";
        touch_links_ = getGripperLinks();
        moveit_msgs::AttachedCollisionObject object = getMeshAttachedObject("bottle", 0.0, 0.0, meshPath, bottleHalfHeight_);
        psi.applyAttachedCollisionObject(object);
    }

private:
    //private node handle for reading ros params
    //"~" gets the current node name, then you can do 
    //pnh_.param("rosparam")
    ros::NodeHandle pnh_;
    //For publishing/subscribing use different node handle
    ros::NodeHandle nh_;
    Eigen::Affine3d tip_transform_;
    moveit::planning_interface::MoveGroupInterface arm_;
    moveit::planning_interface::MoveGroupInterface gripper_;
    std::vector< std::vector<geometry_msgs::PoseStamped> > stampedTrajectories_;
    std::vector< std::vector<geometry_msgs::Pose> > trajectories_;
    std::vector< std::vector<geometry_msgs::Pose> > transformedTrajectories_;
    std::vector<moveit_msgs::RobotTrajectory> computedRobotTrajectories_;
    ros::Publisher pose_stamped_pub_;
    tf::TransformBroadcaster* tfBroadcaster_;
    tf::TransformListener* tfListener_;
    std::vector<std::string> touch_links_;
    std::string bagPath_;
    const double maxAcceleration_;
    const double whiteOpacity = 0.8;
    const double bottleHalfHeight_ = 0.14;
    const int lineStripInitId = 1;
    const std::string handFrame_;
    const std::string glassFrame_;
    const std::string originFrame_;
    const std::string tableFrame_;

};

class GrabPourPlace {

    struct ObjectDescription {
        float dim[3];
        float pos[3];
    };

    moveit::planning_interface::MoveGroupInterface arm;
    moveit::planning_interface::MoveGroupInterface gripper;
    tf::TransformBroadcaster* tfBroadcaster_;

protected:
    ros::NodeHandle node_handle;
    ros::ServiceClient planning_scene_diff_client;
    ros::ServiceClient grasp_planning_service;
    ros::Subscriber collision_obj_sub;
    actionlib::SimpleActionServer<tams_ur5_bartender_manipulation::PourBottleAction>* as_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    std::map<std::string, moveit_msgs::CollisionObject> bottles_;
    moveit_msgs::CollisionObject glass_;

    std::stringstream sstm;

public:
    std::map<std::string, ObjectDescription> object_map;

    GrabPourPlace() : arm(ARM_ID), gripper(GRIPPER_ID) {
        //Create dummy objects
        ObjectDescription glass = {
            {0, 0.04, 0.12},
            {-0.20, 0.35, 0.0}
        };
        object_map["glass"] = glass;

        ObjectDescription bottle = {
            {0, 0.04, 0.3},
            {-0.1, -0.1, 0}
        };
        object_map["bottle"] = bottle;

        //instantiate ServiceClient
        planning_scene_diff_client = node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
        planning_scene_diff_client.waitForExistence();

        //Register action service
        as_ = new actionlib::SimpleActionServer<tams_ur5_bartender_manipulation::PourBottleAction>(node_handle, "pour_bottle", boost::bind(&GrabPourPlace::execute, this, _1), false);
        as_->start();

        //subscribe collision objects
        collision_obj_sub = node_handle.subscribe("recognizedObjects", 1000, &GrabPourPlace::onObjectsRecognized, this);

        //configure planner
        arm.setPlannerId("RRTConnectkConfigDefault");
        arm.setPlanningTime(planningTime);
    }

    void despawnObject(std::string object_id) {
        moveit_msgs::ApplyPlanningScene srv;
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.is_diff = true;
        planning_scene.robot_state.is_diff = true;

        moveit_msgs::AttachedCollisionObject aobj;
        aobj.object.id = object_id;
        aobj.object.operation = aobj.object.REMOVE;
        planning_scene.robot_state.attached_collision_objects.push_back(aobj);

        moveit_msgs::CollisionObject object;
        object.id = object_id;
        object.operation = object.REMOVE;
        planning_scene.world.collision_objects.push_back(object);
        srv.request.scene = planning_scene;
        planning_scene_diff_client.call(srv);
    }

    moveit_msgs::CollisionObject spawnObject(std::string objectID, float xPos = -1.0, float yPos = -1.0, float zOffset = 0.0) {

        moveit_msgs::CollisionObject object;

        object.header.frame_id = "table_top";
        object.id = objectID;

        ObjectDescription objProps = object_map[objectID];

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.push_back(objProps.dim[2]);
        primitive.dimensions.push_back(objProps.dim[1]);

        geometry_msgs::Pose pose;
        pose.orientation.w = 1;
        pose.position.x = xPos != -1.0 ? xPos : objProps.pos[0];
        pose.position.y = yPos != -1.0 ? yPos : objProps.pos[1];
        //pose.position.z = objProps.pos[2];
        pose.position.z = objProps.dim[2] / 2 + zOffset;

        object.primitives.push_back(primitive);
        object.primitive_poses.push_back(pose);

        // add object to scene
        object.operation = object.ADD;

        moveit_msgs::ApplyPlanningScene srv;
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.is_diff = true;
        planning_scene.robot_state.is_diff = true;
        planning_scene.world.collision_objects.push_back(object);
        // remove attached object in case it is attached
        /*   moveit_msgs::AttachedCollisionObject aco;
             object.operation = object.REMOVE;
             aco.object = object;
             planning_scene.robot_state.attached_collision_objects.push_back(aco);
         */
        srv.request.scene = planning_scene;
        planning_scene_diff_client.call(srv);

        return object;
    }

    bool grab_bottle(std::string bottle_id) {
        ROS_INFO_STREAM("Starting grab_bottle with bottle_id: " << bottle_id);

        //open gripper
        ROS_INFO("open gripper");
        gripper.setNamedTarget("basic_open");
        gripper.move();

        ros::Duration(0.5).sleep();

        ROS_INFO("grab bottle");
        arm.setSupportSurfaceName("table");
        //Pick Bottle
        if (arm.planGraspsAndPick(bottles_[bottle_id])) {
            ROS_INFO("Execute pick succeded");
            return true;
        } else {
            ROS_ERROR("Execute pick failed");
            return false;
        }
    }

    bool execute_plan(moveit::planning_interface::MoveGroupInterface::Plan plan) {
        return bool(arm.execute(plan));
    }

    void set_orientation_constraint() {
        //Create orientation constraint - bottle up
        moveit_msgs::Constraints constraints;
        moveit_msgs::OrientationConstraint ocm;
        ocm.link_name = "s_model_tool0";
        ocm.header.frame_id = "table_top";
        ocm.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
        ocm.absolute_x_axis_tolerance = M_PI / 5;
        ocm.absolute_y_axis_tolerance = M_PI / 5;
        ocm.absolute_z_axis_tolerance = 2 * M_PI;
        ocm.weight = 1.0;
        constraints.orientation_constraints.push_back(ocm);
        arm.setPathConstraints(constraints);
    }

    moveit::planning_interface::MoveGroupInterface::Plan get_move_bottle_plan(std::string bottle_id, geometry_msgs::Pose pose, float zOffset) {

        //bottle up constraint (this constraint has a precomputed configuration space approximation)
        //look into moveit/moveit_planners/ompl/.../demo_construct_state_database.cpp
        //The database is located at the 'constraints_approximation_database*' or 'cadb*' directory.
        //To use the database, add following param to node 'move_group' in file 'move_group.launch' inside package 'tams_ur5_setup_moveit_config'.
        //    <param name="constraint_approximations_path" value="$(find tams_ur5_bartender_manipulation)/<database path>" />

        //orientation constraints
        moveit_msgs::OrientationConstraint ocm;
        ocm.link_name = "s_model_tool0";
        ocm.header.frame_id = "table_top";
        ocm.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
        ocm.absolute_x_axis_tolerance = 0.65;
        ocm.absolute_y_axis_tolerance = 0.65;
        ocm.absolute_z_axis_tolerance = M_PI;
        ocm.weight = 1.0;

        moveit_msgs::Constraints constraints;
        constraints.name = "s_model_tool0:upright:20000:high";
        constraints.orientation_constraints.push_back(ocm);
        arm.setPathConstraints(constraints);

        //Z offset for pose
        pose.position.z += zOffset;
        ROS_INFO_STREAM("Moving Bottle to pose: " << pose);

        //set target and constraints
        arm.setPoseReferenceFrame("table_top");
        //using fixed pouring position to guarantee successfull pouring
        //arm.setNamedTarget("pour_above_glass_1");
        arm.setPoseTarget(pose);

        //start planning
        moveit::planning_interface::MoveGroupInterface::Plan best_plan;
        bool succeeded = false;
        for (int i = 0; i < 10; i++) {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            arm.plan(plan);
            if (!plan.trajectory_.joint_trajectory.points.empty() && can_pour(plan.trajectory_.joint_trajectory, pose, bottles_[bottle_id])) {
                if (!succeeded) {
                    best_plan = plan;
                    succeeded = true;
                } else if (plan.trajectory_.joint_trajectory.points.size() < best_plan.trajectory_.joint_trajectory.points.size()) {
                    best_plan = plan;
                }
            }
        }
        ROS_INFO_STREAM("Planning move to glass " << (succeeded ? "SUCCEEDED!" : "FAILED!"));
        arm.clearPathConstraints();
        return best_plan;
    }

    /**
     * Tests if a given trajectory meets a specific constraint set and is not empty
     */
    bool trajectory_valid(moveit_msgs::RobotTrajectory trajectory, moveit_msgs::Constraints constraints) {
        std::vector<trajectory_msgs::JointTrajectoryPoint> points = trajectory.joint_trajectory.points;
        if (points.empty()) {
            return false;
        } else {
            kinematic_constraints::KinematicConstraintSet kset(arm.getRobotModel());
            robot_state::Transforms no_transforms(arm.getRobotModel()->getModelFrame());
            kset.add(constraints, no_transforms);
            moveit::core::RobotState rstate(arm.getRobotModel());
            //iterative validation of all waypoints in the given trajectory
            for (int i = 0; i < points.size(); i++) {
                //create robot state for waypoint
                for (int j = 0; j < points[i].positions.size(); j++) {
                    rstate.setJointPositions(trajectory.joint_trajectory.joint_names[j], (double *) &points[i].positions[j]);
                }
                //problems with this function?
                //moveit::core::jointTrajPointToRobotState(trajectory.joint_trajectory, (size_t) i, rstate);
                rstate.update();
                //return false if robot state does not satisfy constraints
                if (!kset.decide(rstate).satisfied) {
                    return false;
                }
            }
            return true;
        }
    }

    bool can_pour(trajectory_msgs::JointTrajectory joint_trajectory, geometry_msgs::Pose pose, moveit_msgs::CollisionObject bottle) {
        moveit_msgs::Constraints constraints = arm.getPathConstraints();
        arm.clearPathConstraints();
        moveit::core::RobotState rstate(arm.getRobotModel());
        moveit::core::jointTrajPointToRobotState(joint_trajectory, (size_t) (joint_trajectory.points.size() - 1), rstate);
        arm.setPoseReferenceFrame("table_top");
        moveit::planning_interface::MoveGroupInterface::Plan plan = get_pour_bottle_plan(rstate, pose, bottle);
        arm.setPathConstraints(constraints);
        return !plan.trajectory_.joint_trajectory.points.empty();
    }

    moveit::planning_interface::MoveGroupInterface::Plan get_pour_bottle_plan(moveit::core::RobotState state, geometry_msgs::Pose& start_pose, moveit_msgs::CollisionObject bottle) {
        moveit_msgs::RobotState start_state;
        moveit::core::robotStateToRobotStateMsg(state, start_state);
        arm.setStartState(start_state);
        moveit_msgs::RobotTrajectory trajectory; //Trajectory

        //Trying around Y-Axis (backwards)
        double success_percentage = arm.computeCartesianPath(compute_pouring_waypoints("Y", false, bottle, start_pose), 0.03, 3, trajectory);
        ROS_INFO("Pouring trajectory (%.2f%% achieved)", success_percentage * 100.0);

        //Try alternative, if trajectory does not reach 100%
        if (success_percentage < 1.0) {
            ROS_INFO("Trying next direction!");
            //Trying around X-Axis to the right
            moveit_msgs::RobotTrajectory temp_traj; //Alternative Trajectory
            double next_percentage = arm.computeCartesianPath(compute_pouring_waypoints("X", true, bottle, start_pose), 0.03, 3, temp_traj);
            ROS_INFO("Pouring trajectory (%.2f%% achieved)", next_percentage * 100.0);
            //If better result, take this one
            if (next_percentage > success_percentage) {
                success_percentage = next_percentage;
                trajectory = temp_traj;
            }
            //Try next, if trajectory does not reach 100%
            if (next_percentage < 1.0) {
                ROS_INFO("Trying next direction!");
                //Trying around X-Axis to the left
                next_percentage = arm.computeCartesianPath(compute_pouring_waypoints("X", false, bottle, start_pose), 0.03, 3, temp_traj);
                ROS_INFO("Pouring trajectory (%.2f%% achieved)", next_percentage * 100.0);
                //If better result, take this one
                if (next_percentage > success_percentage) {
                    success_percentage = next_percentage;
                    trajectory = temp_traj;
                }
            }
        }
        arm.setStartStateToCurrentState();
        moveit::planning_interface::MoveGroupInterface::Plan result_plan;
        if (success_percentage > 0.95) {
            result_plan.trajectory_ = trajectory;
        }
        return result_plan;
    }

    bool pour_bottle_in_glass(moveit_msgs::CollisionObject bottle, float portion_size) {
        ROS_INFO("Pour Bottle");
        arm.setPoseReferenceFrame("world");
        geometry_msgs::Pose start_pose = arm.getCurrentPose().pose;
        moveit::planning_interface::MoveGroupInterface::Plan pour_forward = get_pour_bottle_plan((*arm.getCurrentState()), start_pose, bottle);
        bool success = !pour_forward.trajectory_.joint_trajectory.points.empty();
        if (success) {
            //reverse pouring trajectory
            moveit::planning_interface::MoveGroupInterface::Plan pour_backward;
            reverse_trajectory(pour_forward.trajectory_, pour_backward.trajectory_);
            ros::Duration(0.5).sleep();

            if (!arm.execute(pour_forward)) {
                //TODO: handle failure
                return false;
            }
            double amount = std::max(0.0, std::min(0.6 * (portion_size - 1.5), 20.0));
            ROS_INFO_STREAM("Pouring " << portion_size << "cl");

            ros::Duration(amount).sleep();

            if (!arm.execute(pour_backward)) {
                //TODO: handle failure
                return false;
            }
        }
        return success;
    }

    std::vector<geometry_msgs::Pose> compute_pouring_waypoints(std::string axis, bool inverse_direction, moveit_msgs::CollisionObject bottle, geometry_msgs::Pose start_pose) {
        geometry_msgs::Pose target_pose = start_pose;
        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(start_pose); // first point of trajectory - necessary?
        //pouring is described in a circular motion
        int dFactor = inverse_direction ? 1 : -1;
        int steps = 10; //number of waypoints
        float pScale = 0.65; //max pouring angle in percent (100% is M_PI)
        float radius = get_bottle_height(bottle.id) - bottle.primitives[0].dimensions[0] / 2;
        float glass_radius = glass_.primitives[0].dimensions[1] / 2; //glass radius is half the glass width
        float glass_height = glass_.primitives[0].dimensions[0];
        float glass_offset = 0.0; //vertical distance of bottle tip from glass whenn pouring
        float zDistance = 0.3 + radius - glass_height / 2 - glass_offset;
        float startX = start_pose.position.x;
        float startY = start_pose.position.y;
        float startZ = start_pose.position.z;
        for (int i = 0; i <= steps; i++) {
            float angle = pScale * M_PI * i / steps;
            float tilt_factor = pow((float) i / steps, 2);

            //ROS_INFO_STREAM("computing waypoint for angle " << angle);
            if (axis == "Y") {
                target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -dFactor * angle, 0);
                float translation = dFactor * (sin(angle) * radius + tilt_factor * (glass_radius - 0.005 - 0.015));
                target_pose.position.x = startX + translation;

            } else { //X Axis
                target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0 + dFactor * angle, 0, 0);
                float translation = dFactor * (sin(angle) * radius + tilt_factor * (glass_radius - 0.005));
                target_pose.position.y = startY + translation;
            }
            //	target_pose.position.z = startZ + (cos(M_PI - angle) + 1 - 2 * 1.1 * tilt_factor) * radius;
            //target_pose.position.z = startZ + (cos(M_PI - angle) + 1) * radius - tilt_factor * zDistance;
            target_pose.position.z = startZ + (cos(M_PI - angle) + 1) * radius - ((float) i / steps) * zDistance;
            waypoints.push_back(target_pose);
        }
        return waypoints;
    }

    void cleanup() {
        ROS_INFO("Cleanup");
        for (std::map<std::string, moveit_msgs::CollisionObject>::iterator it = bottles_.begin(); it != bottles_.end(); ++it) {
            despawnObject(it->first);
        }
        bottles_.clear();
        despawnObject("glass");
        despawnObject("bottle");
        gripper.setNamedTarget("basic_open");
        gripper.move();
    }

    bool placeBottle(std::string bottle_id) {
        ROS_INFO("Placing Bottle");

        geometry_msgs::PoseStamped place_pose_stamped;
        try {
            listener->transformPose("table_top", ros::Time(0), arm.getCurrentPose(), "world", place_pose_stamped);
        } catch (tf2::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            return false;
        }

        arm.setPoseReferenceFrame("table_top");
        geometry_msgs::Pose place_pose = place_pose_stamped.pose;
        moveit_msgs::CollisionObject bottle = bottles_[bottle_id];
        float grasp_height = bottle.primitives[0].dimensions[0] / 2;
        place_pose.position.z = grasp_height + 0.005; // table distance and bottle height offset
        ROS_INFO_STREAM(place_pose);

        //move bottle down
        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(place_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit_msgs::RobotTrajectory trajectory;
        double fraction = arm.computeCartesianPath(waypoints, 0.03, 3, trajectory);
        ROS_INFO("Place Down trajectory (%.2f%% acheived)", fraction * 100.0);
        plan.trajectory_ = trajectory;
        if (fraction != 1.0 || !bool(arm.execute(plan))) {
            return false;
        }

        ros::Duration(0.5).sleep();

        //open gripper
        gripper.setNamedTarget("basic_open");
        ROS_INFO("Setting gripper to target: basic_open");
        if (!gripper.move()) {
            return false;
        }

        ros::Duration(0.5).sleep();

        //release object
        despawnObject(bottle_id);
        if (bottles_.count(bottle_id) == 1) {
            ROS_INFO("Respawning bottle");
            bottle = bottles_[bottle_id];
            for (int i = 0; i < bottle.primitive_poses.size(); i++) {
                bottle.primitive_poses[i].position.x = place_pose.position.x;
                bottle.primitive_poses[i].position.y = place_pose.position.y;
            }
            bottle.operation = bottle.ADD;
            planning_scene_interface_.applyCollisionObject(bottle);
        } else {
            bottle = spawnObject(bottle_id);
        }

        ros::Duration(0.5).sleep();

        //move to retreat position
        geometry_msgs::Pose post_place_pose = place_pose;
        post_place_pose.position.x -= 0.15;
        post_place_pose.position.z += 0.15;
        //post_place_pose.position.z += 0.1;
        ROS_INFO_STREAM("Retreating pose" << post_place_pose);
        std::vector<geometry_msgs::Pose> retreat_waypoints;
        retreat_waypoints.push_back(post_place_pose);
        moveit::planning_interface::MoveGroupInterface::Plan retreat_plan;
        moveit_msgs::RobotTrajectory retreat_trajectory;
        double retreat_fraction = arm.computeCartesianPath(retreat_waypoints, 0.03, 3, retreat_trajectory);
        retreat_plan.trajectory_ = retreat_trajectory;
        if (retreat_fraction > 0.3)
            return bool(arm.execute(retreat_plan));
        else
            return false;
    }

    float get_bottle_height(std::string bottle_id) {
        moveit_msgs::CollisionObject bottle = bottles_[bottle_id];
        float height = 0.0;
        for (int i = 0; i < bottle.primitives.size(); i++) {
            height += bottle.primitives[i].dimensions[0];
        }
        return height;
    }

    bool move_back() {
        arm.setNamedTarget("pour_default");
        return bool(arm.move());
    }

    void reverse_trajectory(moveit_msgs::RobotTrajectory &forward, moveit_msgs::RobotTrajectory &backward) {
        robot_trajectory::RobotTrajectory rTrajectory(arm.getRobotModel(), "arm");
        rTrajectory.setRobotTrajectoryMsg((*arm.getCurrentState()), forward);
        rTrajectory.reverse();
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        iptp.computeTimeStamps(rTrajectory);
        rTrajectory.getRobotTrajectoryMsg(backward);
    }

    void onObjectsRecognized(const tams_ur5_bartender_msgs::BarCollisionObjectArray& objArray) {
        if (!OBJECTS_RECOGNIZED) {
            OBJECTS_RECOGNIZED = true;
            bottles_.clear();
            ROS_INFO("creating collision objects");
            for (int i = 0; i < objArray.objects.size(); i++) {
                // add object to scene
                moveit_msgs::CollisionObject object = objArray.objects[i];
                object.operation = object.ADD;
                ROS_INFO_STREAM("Bottle " << object.type.key);
                object.id = object.type.key;
                planning_scene_interface_.applyCollisionObject(object);
                bottles_[object.id] = object;
            }
        }
    }

    void recognizeBottles() {
        OBJECTS_RECOGNIZED = false;
        int retrys = 20;
        while (!OBJECTS_RECOGNIZED && retrys > 0) {
            ros::Duration(1.0).sleep();
            retrys--;
        }
    }

    geometry_msgs::Pose get_glass_pose() {
        geometry_msgs::Pose pose = glass_.primitive_poses[0];
        pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
        return pose;
    }

    void publishCurrentFeedback(std::stringstream &stream) {
        tams_ur5_bartender_manipulation::PourBottleFeedback feedback;
        feedback.task_state = stream.str();
        as_->publishFeedback(feedback);
        stream.str("");
    }

    void execute(const tams_ur5_bartender_manipulation::PourBottleGoalConstPtr& goal) {
        tfBroadcaster_ = new tf::TransformBroadcaster;
        ROS_INFO("Running pour bottle!");

        cleanup();
        glass_ = spawnObject("glass");

        tf::Transform glassTf;
        geometry_msgs::Pose gPose = glass_.primitive_poses[0];
        gPose.orientation.x = 0.0;
        gPose.orientation.y = 0.0;
        gPose.orientation.z = 0.0;
        gPose.orientation.w = 1.0;
        tf::poseMsgToTF(gPose, glassTf);
        ros::Time now = ros::Time::now();
        tf::StampedTransform glassTfStamped(glassTf, now, "table_top", "glass");
        tfBroadcaster_->sendTransform(glassTfStamped);
        ROS_ERROR_STREAM("Glass transform published!");

        recognizeBottles();
        ros::Duration(1.0).sleep();
        //moving arm to default position after collision objects are spawned
        move_back();

        //cleanup();

        std::string bottle_id = goal->bottle_id;
        float portion_size = goal->portion_size;

        moveit_msgs::CollisionObject bottle;

        if (USE_BOTTLE_PUBLISHER) {
            recognizeBottles();
            ros::Duration(1.0).sleep();
            if (bottles_.count(bottle_id) == 1) {
                bottle = bottles_[bottle_id];
            } else {
                publishCurrentFeedback((std::stringstream&) (sstm << "Bottle " << bottle.id << " does not exist!"));
                ROS_INFO_STREAM("Bottle " << bottle.id << " does not exist!");
                as_->setAborted();
                return;
            }
        } else {
            //Spawn bottle
            bottle = spawnObject("bottle");
        }

        ROS_INFO_STREAM("Bottles Recognized");
        geometry_msgs::Pose bottle_pose = bottle.primitive_poses[0];
        bottle_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -M_PI / 2);
        //glass_ = spawnObject("glass");

        tams_ur5_bartender_manipulation::PourBottleResult result;
        result.success = pouringStateMachine(bottle, portion_size);

        if (result.success) {
            as_->setSucceeded(result, "Grasping, constrained motion and pouring succeeded");
        } else {
            as_->setAborted();
        }
    }

    bool pouringStateMachine(moveit_msgs::CollisionObject bottle, float portion_size) {
        int CurrentAttempt = 1;

        enum stateSpace {
            pickBottleUp, pourIntoGlassNew, moveBottleToGlass, pourIntoGlass, moveBottleBack, placeBottleDown, moveArmToDefault, run_cleanup, Exit, OK
        };
        stateSpace state = pickBottleUp;
        stateSpace failedAtState = OK;

        moveit::planning_interface::MoveGroupInterface::Plan move_bottle_back;
        moveit::planning_interface::MoveGroupInterface::Plan move_bottle;
        ROS_INFO_STREAM("TEST 1");
        do {
            switch (state) {

                case pickBottleUp:
                    ROS_INFO_STREAM("--- STATE 1: pickBottleUp; Attempt " << CurrentAttempt);
                    publishCurrentFeedback((std::stringstream&) (sstm << "Picking up bottle: Attempt " << CurrentAttempt));
                    // if SUCCESS, move on to next state, else check number of attempts and either retry state or abort and skip states
                    if (grab_bottle(bottle.id)) {
                        ROS_INFO_STREAM("pickBottleUp succeeded with attempt " << CurrentAttempt);
                        publishCurrentFeedback((std::stringstream&) (sstm << "Picking up bottle succeeded with attempt " << CurrentAttempt));
                        ros::Duration(0.5).sleep();
                        //state = moveBottleToGlass;
                        state = pourIntoGlassNew;
                        CurrentAttempt = 1;
                    } else {
                        if (CurrentAttempt++ == NUM_RETRIES_AFTER_JAM + 1) {
                            ROS_INFO_STREAM("XXX STATE 1: pickBottleUp FAILED after all " << CurrentAttempt << " attempts. Moving on to cleanup.");
                            publishCurrentFeedback((std::stringstream&) (sstm << "Picking up bottle FAILED after " << CurrentAttempt << ". attempt."));
                            state = run_cleanup;
                            failedAtState = pickBottleUp;
                        }
                    }
                    break;
                    //NEW POURING!
                case pourIntoGlassNew:
                {
                    ROS_INFO_STREAM("Test 2");
                    TrajectoryPourer pourer("glass", "table_top", "s_model_tool0");
                    Visualisator visuals(pourer.getGlassFrame());

                    pourer.generatePouringTrajectories();
                    visuals.showTrajectory(pourer.getTrajectory(0));
                    ros::Duration(2.0).sleep();

                    pourer.rotatePouringTrajectories();

                    visuals.showTrajectory(pourer.getTrajectory(0));
                    ros::Duration(1.0).sleep();
                    pourer.computeCartesianPaths();

                    if (pourer.playTrajectory(0)) {
                        state = moveBottleBack;
                    } else {
                        state = run_cleanup;
                        failedAtState = pourIntoGlassNew;
                    }

                    break;
                }

                case moveBottleToGlass:
                    ROS_INFO_STREAM("--- STATE 2: moveBottleToGlass; (planningTime: " << planningTime << ") ; Attempt " << CurrentAttempt);
                    move_bottle = get_move_bottle_plan(bottle.id, get_glass_pose(), .3);
                    if (!move_bottle.trajectory_.joint_trajectory.points.empty()) {
                        if (execute_plan(move_bottle)) {
                            ROS_INFO_STREAM("moveBottleToGlass succeeded with attempt " << CurrentAttempt);
                            publishCurrentFeedback((std::stringstream&) (sstm << "Moving bottle to glass succeeded with attempt " << CurrentAttempt));
                            reverse_trajectory(move_bottle.trajectory_, move_bottle_back.trajectory_); // for later use in moving the bottle back
                            ros::Duration(0.5).sleep();
                            state = pourIntoGlass;
                            CurrentAttempt = 1;
                            break;
                        } else {
                            ROS_INFO_STREAM("moveBottleToGlass (executing path!) FAILED (in attempt " << CurrentAttempt << ")");
                        }
                    }
                    // when plan was found & executed, this isn't reached, else check number of attempts and either retry state or abort and skip states
                    if (CurrentAttempt++ == NUM_RETRIES_AFTER_JAM) {
                        ROS_INFO_STREAM("XXX STATE 2: moveBottleToGlass FAILED after all " << CurrentAttempt << " attempts. Moving on to placeBottleDown.");
                        publishCurrentFeedback((std::stringstream&) (sstm << "Moving bottle to glass FAILED after " << CurrentAttempt << ". attempt. Placing bottle back down!"));
                        state = placeBottleDown;
                        failedAtState = moveBottleToGlass;
                        CurrentAttempt = 1;
                    }
                    break;

                case pourIntoGlass:
                    state = moveBottleBack;
                    if (!pour_bottle_in_glass(bottle, portion_size)) {
                        failedAtState = pourIntoGlass;
                    } else {
                        ROS_INFO_STREAM("pourIntoGlass succeeded");
                        publishCurrentFeedback((std::stringstream&) (sstm << "Pouring succeeded!"));
                    }
                    ros::Duration(0.5).sleep();
                    break;


                case moveBottleBack:
                    ROS_INFO_STREAM("--- STATE 4: moveBottleBack; Attempt " << CurrentAttempt);
                    publishCurrentFeedback((std::stringstream&) (sstm << "Moving bottle back, attempt: " << CurrentAttempt));
                    // if SUCCESS, move on to next state, else check number of attempts and either retry state or abort and skip states
                    if (execute_plan(move_bottle_back)) {
                        ROS_INFO_STREAM("moveBottleBack succeeded with attempt " << CurrentAttempt);
                        publishCurrentFeedback((std::stringstream&) (sstm << "Moving bottle back succeeded with attempt: " << CurrentAttempt));
                        state = placeBottleDown;
                        CurrentAttempt = 1;
                        ros::Duration(0.5).sleep();
                    } else if (CurrentAttempt++ == NUM_RETRIES_AFTER_JAM) {
                        ROS_INFO_STREAM("XXX STATE 4: moveBottleBack FAILED after all " << CurrentAttempt << " attempts. Moving on to ABORT FOR GOOD.");
                        publishCurrentFeedback((std::stringstream&) (sstm << "Moving bottle back failed after " << CurrentAttempt << ". attempt."));
                        state = Exit; // just stay still and abort if moving back didn't work because the hand is still holding the bottle..
                        failedAtState = moveBottleBack;
                    }
                    break;


                case placeBottleDown:
                    ROS_INFO_STREAM("--- STATE 5: placeBottleDown; Attempt " << CurrentAttempt);
                    publishCurrentFeedback((std::stringstream&) (sstm << "Placing bottle down, attempt: " << CurrentAttempt));
                    if (placeBottle(bottle.id)) {
                        ROS_INFO_STREAM("placeBottleDown succeeded with attempt " << CurrentAttempt);
                        publishCurrentFeedback((std::stringstream&) (sstm << "Placing bottle down succeeded with attempt: " << CurrentAttempt));
                        state = moveArmToDefault;
                        CurrentAttempt = 1;
                        ros::Duration(0.5).sleep();
                    } else if (CurrentAttempt++ == NUM_RETRIES_AFTER_JAM) {
                        ROS_INFO_STREAM("XXX STATE 5: placeBottleDown FAILED after all " << CurrentAttempt << " attempts. Moving on to ABORT FOR GOOD.");
                        publishCurrentFeedback((std::stringstream&) (sstm << "Placing bottle down failed after " << CurrentAttempt << ". attempt."));
                        state = Exit;
                        failedAtState = placeBottleDown;
                    }
                    break;


                case moveArmToDefault:
                    CurrentAttempt = 1;
                    ROS_INFO_STREAM("--- STATE 6: moveArmToDefault; Attempt " << CurrentAttempt);
                    publishCurrentFeedback((std::stringstream&) (sstm << "Moving arm to default pose, attempt: " << CurrentAttempt));
                    if (move_back()) {
                        ROS_INFO_STREAM("moveArmToDefault succeeded with attempt " << CurrentAttempt);
                        publishCurrentFeedback((std::stringstream&) (sstm << "Moving arm to default pose succeeded with attempt: " << CurrentAttempt));
                        ros::Duration(1.0).sleep();
                        state = run_cleanup;
                    } else if (CurrentAttempt++ == NUM_RETRIES_AFTER_JAM) {
                        ROS_INFO_STREAM("XXX STATE 6: moveArmToDefault FAILED after all " << CurrentAttempt << " attempts. Moving on to cleanup.");
                        publishCurrentFeedback((std::stringstream&) (sstm << "Moving arm to default pose failed after " << CurrentAttempt << ". attempt."));
                        state = run_cleanup;
                        failedAtState = moveArmToDefault;
                    }
                    break;


                case run_cleanup:
                    ROS_INFO_STREAM("--- STATE 7: run_cleanup: despawn objects and move arm to home (if not already happened)");
                    publishCurrentFeedback((std::stringstream&) (sstm << "Cleaning up (despawning objects, etc.)"));
                    cleanup();
                    // result.success = failedAtState == OK;
                    state = Exit;
                    break;
            }
        } while (state != Exit);

        return failedAtState == OK;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pour_bottle_server");
    listener = new (tf::TransformListener);
    GrabPourPlace gpp;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    while (ros::ok()) {
    }
    return 0;
}
