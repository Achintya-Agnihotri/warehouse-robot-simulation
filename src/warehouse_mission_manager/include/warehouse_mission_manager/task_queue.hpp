#ifndef WAREHOUSE_MISSION_MANAGER_TASK_QUEUE_HPP_
#define WAREHOUSE_MISSION_MANAGER_TASK_QUEUE_HPP_

#include <queue>
#include <vector>
#include "task.hpp"

class TaskQueue
{
public:
    void add_task(const WarehouseTask &task)
    {
        tasks_.push(task);
    }

    bool empty() const
    {
        return tasks_.empty();
    }

    WarehouseTask get_next_task()
    {
        WarehouseTask task = tasks_.top();
        tasks_.pop();
        return task;
    }

    size_t size() const
    {
        return tasks_.size();
    }

private:
    struct ComparePriority
    {
        bool operator()(const WarehouseTask &a,
                        const WarehouseTask &b) const
        {
            return a.priority < b.priority;
        }
    };

    std::priority_queue<
        WarehouseTask,
        std::vector<WarehouseTask>,
        ComparePriority
    > tasks_;
};

#endif