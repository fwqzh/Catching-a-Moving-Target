/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "../include/planner.h"
#include <math.h>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <algorithm>
#include <climits>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

struct Node {
    int x, y;           
    int g, h;           
    int time;           
    Node* parent;       

    Node(int x_, int y_, int g_, int h_, int t_, Node* parent_ = nullptr) 
        : x(x_), y(y_), g(g_), h(h_), time(t_), parent(parent_) {}

    int f() const { return g + h; } 
};

// Comparator for priority queue sorting by f value
struct CompareNode {
    bool operator()(const Node* a, const Node* b) const {
        return a->f() > b->f();
    }
};

int final_destination;
int flag = false;
int goal_index;

void planner(
    int* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr
    )
{
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1, 0, 0, 1, 1, 1}; 
    int dY[NUMOFDIRS] = {-1, 0, 1, -1, 1, -1, 0, 1}; 

    // Time window for future target positions, e.g., considering the next 5 steps as possible goals
    int time_window = 5;
    std::vector<std::pair<int, int>> goal_positions;

    for (int i = curr_time + 1; i <= MIN(curr_time + time_window, target_steps); ++i) {
        int future_goalX = target_traj[i - 1];
        int future_goalY = target_traj[i - 1 + target_steps];
        goal_positions.push_back({future_goalX, future_goalY});
    }

    // Heuristic function to calculate the h from the current node to all target points
    auto getHeuristic = [&](int x, int y, int time) {
        double heuristic = 0;
        int weight_factor = 1;
        for (const auto& goal : goal_positions) {
            heuristic += (abs(goal.first - x) + abs(goal.second - y)) * weight_factor / goal_positions.size();
            weight_factor++;
        }
        heuristic += abs(curr_time + time_window - time); // Time impact factor
        return heuristic;
    };

    // Priority queue (sorted by f value)
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> open_list;

    // Closed set to record visited nodes
    std::unordered_set<int> closed_list;

    // Node hash map to store created nodes and avoid duplication
    std::unordered_map<int, Node*> node_map;

    double initial_h = getHeuristic(robotposeX, robotposeY, curr_time);
    Node* start = new Node(robotposeX, robotposeY, 0, initial_h, curr_time);
    open_list.push(start);
    node_map[GETMAPINDEX(robotposeX, robotposeY, x_size, y_size)] = start;

    // A* search loop
    while (!open_list.empty()) {
        Node* current = open_list.top();
        open_list.pop();

        // Check if any target position has been reached
        for (const auto& goal : goal_positions) {
            if (current->x == goal.first && current->y == goal.second) {
                Node* path_node = current;
                std::vector<std::pair<int, int>> path;

                // Backtrack the path
                while (path_node) {
                    path.push_back({path_node->x, path_node->y});
                    path_node = path_node->parent;
                }

                // Return the next action
                if (path.size() > 1) {
                    action_ptr[0] = path[path.size() - 2].first;
                    action_ptr[1] = path[path.size() - 2].second;
                } else {
                    // If already at the target point, stay in place
                    action_ptr[0] = robotposeX;
                    action_ptr[1] = robotposeY;
                }

                for (auto& entry : node_map) {
                    delete entry.second;
                }
                return;
            }
        }

        // Mark the current node as visited
        int current_index = GETMAPINDEX(current->x, current->y, x_size, y_size);
        closed_list.insert(current_index);

        // Expand neighboring nodes in 8 directions
        for (int dir = 0; dir < NUMOFDIRS; ++dir) {
            int newX = current->x + dX[dir];
            int newY = current->y + dY[dir];

            // Check if the neighbor node is within the map bounds
            if (newX >= 1 && newX <= x_size && newY >= 1 && newY <= y_size) {
                int new_index = GETMAPINDEX(newX, newY, x_size, y_size);
                int cost = map[new_index];  // Get the cost of the neighbor node

                // Skip if the node is non-traversable or has been visited
                if (cost >= collision_thresh || closed_list.find(new_index) != closed_list.end()) {
                    continue;
                }

                // Adjust g value considering the time step
                int g_new = current->g + cost;  
                int time_new = current->time + 1;  

                // Calculate the new heuristic value
                double h_new = getHeuristic(newX, newY, time_new);

                // If the neighbor node has not been created or a better path is found
                if (node_map.find(new_index) == node_map.end() || g_new < node_map[new_index]->g) {
                    Node* neighbor = new Node(newX, newY, g_new, h_new, time_new, current);  
                    open_list.push(neighbor);  
                    node_map[new_index] = neighbor;  
                }
            }
        }
    }

    // If no path is found, stay in place
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;

    for (auto& entry : node_map) {
        delete entry.second;
    }
}