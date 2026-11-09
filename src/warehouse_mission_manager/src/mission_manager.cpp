#include <memory>
#include <chrono>
#include <cmath>
#include <string>
#include <unordered_map>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/time.h"


#include "warehouse_interfaces/srv/submit_task.hpp"
#include "warehouse_mission_manager/task.hpp"
#include "warehouse_mission_manager/task_queue.hpp"

using namespace std::chrono_literals;
using Trigger = std_srvs::srv::Trigger;
using SetEntityPose = ros_gz_interfaces::srv::SetEntityPose;
using SubmitTask = warehouse_interfaces::srv::SubmitTask;

struct Station
{
    double x;
    double y;
    double yaw;
};

class MissionManager : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigate =
        rclcpp_action::ClientGoalHandle<NavigateToPose>;

    using Trigger = std_srvs::srv::Trigger;
    using SetEntityPose = ros_gz_interfaces::srv::SetEntityPose;

    MissionManager() : Node("warehouse_mission_manager")
    {
        nav_client_ =
            rclcpp_action::create_client<NavigateToPose>(
                this, "navigate_to_pose");

        package_client_ =
            this->create_client<SetEntityPose>(
                "/world/empty/set_pose");

        tf_buffer_ =
            std::make_shared<tf2_ros::Buffer>(this->get_clock());

        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        stations_ = {
            {"storage_A", {3.97826, 2.73529, 0.0}},
            {"packing_station", {-0.32230, -2.45846, 0.0}},
            {"loading_dock", {-4.89033, -2.23598, 0.0}}
        };

        pause_service_ = this->create_service<Trigger>(
            "/warehouse_mission/pause",
            std::bind(&MissionManager::pause_mission, this,
                      std::placeholders::_1, std::placeholders::_2));

        resume_service_ = this->create_service<Trigger>(
            "/warehouse_mission/resume",
            std::bind(&MissionManager::resume_mission, this,
                      std::placeholders::_1, std::placeholders::_2));

        cancel_service_ = this->create_service<Trigger>(
            "/warehouse_mission/cancel",
            std::bind(&MissionManager::cancel_mission, this,
                      std::placeholders::_1, std::placeholders::_2));

        status_service_ = this->create_service<Trigger>(
            "/warehouse_mission/status",
            std::bind(&MissionManager::mission_status, this,
                      std::placeholders::_1, std::placeholders::_2));

        submit_task_service_ = this->create_service<SubmitTask>(
            "/warehouse_mission/submit_task",
            std::bind(&MissionManager::submit_task, this,
              std::placeholders::_1, std::placeholders::_2));

        package_timer_ = this->create_wall_timer(
            100ms,
            std::bind(&MissionManager::update_package_pose, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Warehouse Task Manager started!");

        
    }

private:

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

    rclcpp::Client<SetEntityPose>::SharedPtr package_client_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr package_timer_;

    rclcpp::Service<Trigger>::SharedPtr pause_service_;
    rclcpp::Service<Trigger>::SharedPtr resume_service_;
    rclcpp::Service<Trigger>::SharedPtr cancel_service_;
    rclcpp::Service<Trigger>::SharedPtr status_service_;
    rclcpp::Service<SubmitTask>::SharedPtr submit_task_service_;

    std::unordered_map<std::string, Station> stations_;

    TaskQueue task_queue_;

    WarehouseTask current_task_;
    GoalHandleNavigate::SharedPtr current_goal_handle_;

    bool mission_active_ = false;
    bool paused_ = false;
    bool cancelling_ = false;

    bool carrying_package_ = false;
    bool package_request_in_flight_ = false;
    bool package_drop_pending_ = false;

    int task_stage_ = 0;

    // Approximate map-to-Gazebo-world translation.
    // Calibrated from previously measured robot poses.
    const double world_offset_x_ = -0.296;
    const double world_offset_y_ = -0.070;

    const double package_follow_distance_ = 0.55;
    void submit_task(
    const std::shared_ptr<SubmitTask::Request> request,
    std::shared_ptr<SubmitTask::Response> response)
    {
        if (request->task_id.empty())
        {
            response->success = false;
            response->message = "Task ID cannot be empty.";
            return;
        }

        if (stations_.find(request->pickup_station) == stations_.end())
        {
            response->success = false;
            response->message = "Invalid pickup station.";
            return;
        }

        if (stations_.find(request->dropoff_station) == stations_.end())
        {
            response->success = false;
            response->message = "Invalid drop-off station.";
            return;
        }

        if (request->priority < 1)
        {
            response->success = false;
            response->message = "Priority must be at least 1.";
            return;
        }

        WarehouseTask task{
            request->task_id,
            request->pickup_station,
            request->dropoff_station,
            request->priority
        };

        task_queue_.add_task(task);

        RCLCPP_INFO(
            this->get_logger(),
            "New task submitted: %s | Priority: %d",
            request->task_id.c_str(),
            request->priority);

        response->success = true;
        response->message = "Task successfully added to queue.";

        if (!mission_active_ && !paused_)
        {
            cancelling_ = false;
            start_next_task();
        }
    }
    void start_next_task()
    {
        if (paused_ || cancelling_)
            return;

        if (task_queue_.empty())
        {
            mission_active_ = false;

            RCLCPP_INFO(
                this->get_logger(),
                "ALL WAREHOUSE TASKS COMPLETED!");

            return;
        }

        current_task_ = task_queue_.get_next_task();
        current_task_.status = TaskStatus::IN_PROGRESS;

        task_stage_ = 0;
        mission_active_ = true;
        carrying_package_ = false;
        package_drop_pending_ = false;

        RCLCPP_INFO(
            this->get_logger(),
            "Starting %s | Pickup: %s | Drop-off: %s | Priority: %d",
            current_task_.task_id.c_str(),
            current_task_.pickup_station.c_str(),
            current_task_.dropoff_station.c_str(),
            current_task_.priority);

        navigate_to_task_station();
    }

    void navigate_to_task_station()
    {
        if (!mission_active_ || paused_ || cancelling_)
            return;

        std::string station_name =
            (task_stage_ == 0)
                ? current_task_.pickup_station
                : current_task_.dropoff_station;

        auto it = stations_.find(station_name);

        if (it == stations_.end())
        {
            current_task_.status = TaskStatus::FAILED;
            carrying_package_ = false;

            RCLCPP_ERROR(
                this->get_logger(),
                "Unknown station: %s",
                station_name.c_str());

            start_next_task();
            return;
        }

        if (!nav_client_->wait_for_action_server(5s))
        {
            finish_mission("Nav2 action server unavailable.");
            return;
        }

        const auto &station = it->second;

        auto goal = NavigateToPose::Goal();

        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = this->now();

        goal.pose.pose.position.x = station.x;
        goal.pose.pose.position.y = station.y;

        goal.pose.pose.orientation.z =
            std::sin(station.yaw / 2.0);

        goal.pose.pose.orientation.w =
            std::cos(station.yaw / 2.0);

        auto options =
            rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        options.goal_response_callback =
            [this](const GoalHandleNavigate::SharedPtr &handle)
            {
                if (!handle)
                {
                    if (!paused_ && !cancelling_)
                        finish_mission("Nav2 rejected the goal.");

                    return;
                }

                current_goal_handle_ = handle;

                RCLCPP_INFO(
                    this->get_logger(),
                    "Navigation goal accepted.");
            };

        options.result_callback =
            [this, station_name](
                const GoalHandleNavigate::WrappedResult &result)
            {
                current_goal_handle_.reset();

                if (cancelling_ || paused_)
                    return;

                if (result.code !=
                    rclcpp_action::ResultCode::SUCCEEDED)
                {
                    current_task_.status = TaskStatus::FAILED;
                    carrying_package_ = false;

                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Task %s failed during navigation.",
                        current_task_.task_id.c_str());

                    start_next_task();
                    return;
                }

                if (task_stage_ == 0)
                {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Pickup reached: %s",
                        station_name.c_str());

                    RCLCPP_INFO(
                        this->get_logger(),
                        "Package picked up: %s",
                        current_task_.task_id.c_str());

                    carrying_package_ = true;

                    task_stage_ = 1;

                    navigate_to_task_station();
                }
                else
                {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Drop-off reached: %s",
                        station_name.c_str());

                    place_package_at_dropoff();
                }
            };

        RCLCPP_INFO(
            this->get_logger(),
            "%s navigation for %s",
            task_stage_ == 0 ? "Pickup" : "Drop-off",
            current_task_.task_id.c_str());

        nav_client_->async_send_goal(goal, options);
    }

    void update_package_pose()
    {
        if (package_request_in_flight_)
            return;

        if (!carrying_package_ && !package_drop_pending_)
            return;

        if (!package_client_->service_is_ready())
            return;

        auto request =
            std::make_shared<SetEntityPose::Request>();

        request->entity.name = "warehouse_package";
        request->entity.type = 2;

        if (package_drop_pending_)
        {
            auto it = stations_.find(current_task_.dropoff_station);

            if (it == stations_.end())
                return;

            request->pose.position.x =
                it->second.x + world_offset_x_;

            request->pose.position.y =
                it->second.y + world_offset_y_;

            request->pose.position.z = 0.15;

            request->pose.orientation.z =
                std::sin(it->second.yaw / 2.0);

            request->pose.orientation.w =
                std::cos(it->second.yaw / 2.0);
        }
        else
        {
            try
            {
                auto transform = tf_buffer_->lookupTransform(
                    "map",
                    "base_footprint",
                    tf2::TimePointZero);

                const auto &p = transform.transform.translation;
                const auto &q = transform.transform.rotation;

                double yaw = std::atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));

                request->pose.position.x =
                    p.x + world_offset_x_ -
                    package_follow_distance_ * std::cos(yaw);

                request->pose.position.y =
                    p.y + world_offset_y_ -
                    package_follow_distance_ * std::sin(yaw);

                request->pose.position.z = 0.15;

                request->pose.orientation.z =
                    std::sin(yaw / 2.0);

                request->pose.orientation.w =
                    std::cos(yaw / 2.0);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    3000,
                    "Waiting for robot TF: %s",
                    ex.what());

                return;
            }
        }

        package_request_in_flight_ = true;

        package_client_->async_send_request(
            request,
            [this](rclcpp::Client<SetEntityPose>::SharedFuture future)
            {
                package_request_in_flight_ = false;

                try
                {
                    auto response = future.get();

                    if (!response->success)
                    {
                        RCLCPP_WARN(
                            this->get_logger(),
                            "Gazebo could not update package pose.");

                        return;
                    }

                    if (package_drop_pending_)
                    {
                        package_drop_pending_ = false;
                        carrying_package_ = false;

                        current_task_.status = TaskStatus::COMPLETED;

                        RCLCPP_INFO(
                            this->get_logger(),
                            "Package delivered successfully.");

                        RCLCPP_INFO(
                            this->get_logger(),
                            "TASK COMPLETED: %s",
                            current_task_.task_id.c_str());

                        start_next_task();
                    }
                }
                catch (const std::exception &ex)
                {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Package pose request failed: %s",
                        ex.what());
                }
            });
    }

    void place_package_at_dropoff()
    {
        carrying_package_ = false;
        package_drop_pending_ = true;

        RCLCPP_INFO(
            this->get_logger(),
            "Delivering package for %s...",
            current_task_.task_id.c_str());

        // The timer sends the final package pose.
        // The task is marked complete only after Gazebo confirms it.
    }

    void pause_mission(
        const std::shared_ptr<Trigger::Request>,
        std::shared_ptr<Trigger::Response> response)
    {
        if (!mission_active_ || paused_)
        {
            response->success = false;
            response->message = "No running task to pause.";
            return;
        }

        paused_ = true;

        if (current_goal_handle_)
            nav_client_->async_cancel_goal(current_goal_handle_);

        response->success = true;
        response->message = "Task paused.";

        RCLCPP_INFO(this->get_logger(), "Task paused.");
    }

    void resume_mission(
        const std::shared_ptr<Trigger::Request>,
        std::shared_ptr<Trigger::Response> response)
    {
        if (!mission_active_ || !paused_)
        {
            response->success = false;
            response->message = "No paused task.";
            return;
        }

        paused_ = false;

        response->success = true;
        response->message = "Task resumed.";

        RCLCPP_INFO(this->get_logger(), "Task resumed.");

        navigate_to_task_station();
    }

    void cancel_mission(
        const std::shared_ptr<Trigger::Request>,
        std::shared_ptr<Trigger::Response> response)
    {
        if (!mission_active_)
        {
            response->success = false;
            response->message = "No active task.";
            return;
        }

        cancelling_ = true;
        paused_ = false;
        mission_active_ = false;

        carrying_package_ = false;
        package_drop_pending_ = false;

        if (current_goal_handle_)
        {
            nav_client_->async_cancel_goal(current_goal_handle_);
            current_goal_handle_.reset();
        }

        response->success = true;
        response->message = "Current task cancelled.";

        RCLCPP_WARN(
            this->get_logger(),
            "Current task cancelled.");
    }

    void mission_status(
        const std::shared_ptr<Trigger::Request>,
        std::shared_ptr<Trigger::Response> response)
    {
        response->success = true;

        if (!mission_active_)
        {
            response->message = "No active task.";
            return;
        }

        response->message =
            std::string(paused_ ? "PAUSED: " : "RUNNING: ") +
            current_task_.task_id +
            (task_stage_ == 0 ? " | PICKUP" : " | DROP-OFF") +
            (carrying_package_ ? " | CARRYING PACKAGE" : "");
    }

    void finish_mission(const std::string &reason)
    {
        mission_active_ = false;
        carrying_package_ = false;
        package_drop_pending_ = false;

        RCLCPP_ERROR(
            this->get_logger(),
            "Task manager stopped: %s",
            reason.c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MissionManager>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}