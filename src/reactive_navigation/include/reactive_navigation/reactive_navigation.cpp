#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <stack>
#include <vector>
#include <string>
#include <unordered_set>
#include <tuple>

using namespace std::chrono_literals;

// Helper function to convert position to a unique hashable key
std::string pos_to_key(int y, int x) { // Use (Y, X) for consistency
    return std::to_string(y) + "," + std::to_string(x);
}

// Helper function to get neighbors and check for target ('t')
bool get_neighbors(const std::string &left, const std::string &down,
                   const std::string &up, const std::string &right,
                   int robot_y, int robot_x,
                   std::vector<std::tuple<std::string, int, int>> &neighbors) {
    bool target_found = false;

    // Check UP
    if (up == "t") {
        RCLCPP_INFO(rclcpp::get_logger("DFS"), "Target (UP) found at [%d, %d]", robot_y - 1, robot_x);
        neighbors.clear();
        neighbors.emplace_back("up", robot_y - 1, robot_x);
        target_found = true;
    } else if (up == "f") {
        RCLCPP_INFO(rclcpp::get_logger("DFS"), "Neighbor (UP) added: [%d, %d]", robot_y - 1, robot_x);
        neighbors.emplace_back("up", robot_y - 1, robot_x);
    }

    // Check RIGHT
    if (right == "t") {
        RCLCPP_INFO(rclcpp::get_logger("DFS"), "Target (RIGHT) found at [%d, %d]", robot_y, robot_x + 1);
        neighbors.clear();
        neighbors.emplace_back("right", robot_y, robot_x + 1);
        target_found = true;
    } else if (right == "f") {
        RCLCPP_INFO(rclcpp::get_logger("DFS"), "Neighbor (RIGHT) added: [%d, %d]", robot_y, robot_x + 1);
        neighbors.emplace_back("right", robot_y, robot_x + 1);
    }

    // Check DOWN
    if (down == "t") {
        RCLCPP_INFO(rclcpp::get_logger("DFS"), "Target (DOWN) found at [%d, %d]", robot_y + 1, robot_x);
        neighbors.clear();
        neighbors.emplace_back("down", robot_y + 1, robot_x);
        target_found = true;
    } else if (down == "f") {
        RCLCPP_INFO(rclcpp::get_logger("DFS"), "Neighbor (DOWN) added: [%d, %d]", robot_y + 1, robot_x);
        neighbors.emplace_back("down", robot_y + 1, robot_x);
    }

    // Check LEFT
    if (left == "t") {
        RCLCPP_INFO(rclcpp::get_logger("DFS"), "Target (LEFT) found at [%d, %d]", robot_y, robot_x - 1);
        neighbors.clear();
        neighbors.emplace_back("left", robot_y, robot_x - 1);
        target_found = true;
    } else if (left == "f") {
        RCLCPP_INFO(rclcpp::get_logger("DFS"), "Neighbor (LEFT) added: [%d, %d]", robot_y, robot_x - 1);
        neighbors.emplace_back("left", robot_y, robot_x - 1);
    }

    return target_found;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("move_command_client");
    auto client = node->create_client<cg_interfaces::srv::MoveCmd>("/move_command");

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
    }

    bool goal_reached = false;
    std::unordered_set<std::string> visited;
    std::stack<std::tuple<int, int, std::string>> stack;

    while (!goal_reached) {
        auto status_request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        auto result = client->async_send_request(status_request);

        if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "Failed to call the service.");
            continue;
        }

        auto response = result.get();

        RCLCPP_INFO(node->get_logger(), "Robot Pos: [%d, %d], Target Pos: [%d, %d]",
                    response->robot_pos[1], response->robot_pos[0], // Use (Y, X)
                    response->target_pos[1], response->target_pos[0]); // Use (Y, X)

        RCLCPP_INFO(node->get_logger(), "Direction Info - Left: %s, Down: %s, Up: %s, Right: %s",
                    response->left.c_str(), response->down.c_str(), response->up.c_str(), response->right.c_str());

        // Check if goal is reached
        if (response->robot_pos[0] == response->target_pos[0] &&
            response->robot_pos[1] == response->target_pos[1]) {
            RCLCPP_INFO(node->get_logger(), "Goal reached!");
            goal_reached = true;
            break;
        }

        // Mark current position as visited
        std::string current_key = pos_to_key(response->robot_pos[0], response->robot_pos[1]);
        if (visited.find(current_key) == visited.end()) {
            visited.insert(current_key);
            RCLCPP_INFO(node->get_logger(), "Marking as visited: [%d, %d]",
                        response->robot_pos[0], response->robot_pos[1]);

            // Get neighbors and push them to the stack
            std::vector<std::tuple<std::string, int, int>> neighbors;
            bool target_found = get_neighbors(response->left, response->down, response->up, response->right,
                                              response->robot_pos[0], response->robot_pos[1], neighbors);

            if (target_found) {
                stack = {}; // Clear stack to prioritize direct movement to target
            }

            for (const auto &[direction, y, x] : neighbors) {
                std::string neighbor_key = pos_to_key(y, x);
                if (visited.find(neighbor_key) == visited.end()) {
                    stack.emplace(y, x, direction);
                    RCLCPP_INFO(node->get_logger(), "Pushing to stack: [%d, %d] Direction: %s", y, x, direction.c_str());
                }
            }
        }

        // If the stack is empty, there's no path left to explore
        if (stack.empty()) {
            RCLCPP_ERROR(node->get_logger(), "No more paths to explore. Robot is stuck.");
            break;
        }

        // Get the next position from the stack
        auto [next_y, next_x, next_direction] = stack.top(); // Use (Y, X)
        stack.pop();
        RCLCPP_INFO(node->get_logger(), "Popping from stack: [%d, %d] Direction: %s", next_y, next_x, next_direction.c_str());

        // Send movement command
        auto move_request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        move_request->direction = next_direction;

        auto move_result = client->async_send_request(move_request);

        if (rclcpp::spin_until_future_complete(node, move_result) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "Failed to move the robot.");
            continue;
        }

        RCLCPP_INFO(node->get_logger(), "Moving in direction: %s", next_direction.c_str());
    }

    rclcpp::shutdown();
    return 0;
}
