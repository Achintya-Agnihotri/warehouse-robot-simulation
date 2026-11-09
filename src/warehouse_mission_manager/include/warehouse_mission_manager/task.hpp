#ifndef WAREHOUSE_MISSION_MANAGER_TASK_HPP_
#define WAREHOUSE_MISSION_MANAGER_TASK_HPP_

#include <string>

enum class TaskStatus
{
    PENDING,
    IN_PROGRESS,
    COMPLETED,
    FAILED
};

struct WarehouseTask
{
    std::string task_id;
    std::string pickup_station;
    std::string dropoff_station;

    int priority = 1;

    TaskStatus status = TaskStatus::PENDING;
};

#endif
