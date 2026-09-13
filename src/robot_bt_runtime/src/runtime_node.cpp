#include "robot_core/demo.hpp"
#include "robot_interfaces/action/execute_task.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <algorithm>
#include <chrono>
#include <memory>

namespace rc = robot_core;
using ExecuteTask = robot_interfaces::action::ExecuteTask;
using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;
using namespace std::chrono_literals;

// Sessions outlive BT nodes: halting a tree requests stop, the timer drains confirmation.
struct Engine {
  rc::Components components;
  rc::Bindings bindings;
  rc::WorldState world;
  rc::Context context;
  rc::Skills skills;
  rc::Resources resources;
  struct Entry { std::string skill; std::shared_ptr<rc::Session> session; };
  std::vector<Entry> sessions;
  uint64_t sequence{0};
  std::chrono::milliseconds skill_timeout{2000};
  bool fault_latched{false};

  Engine(std::string motion, int ticks, bool fail)
      : bindings(components, {{"perception", "mock_camera"}, {"motion", std::move(motion)}}),
        context{bindings, world} {
    components.add("mock_camera", std::make_shared<rc::MockLocator>());
    components.add("mock_arm", std::make_shared<rc::MockManipulator>(ticks, fail));
    components.add("slow_mock_arm", std::make_shared<rc::MockManipulator>(ticks * 2, fail));
    bindings.get<rc::Manipulator>("motion"); // fail startup on invalid configuration
    rc::register_demo_skills(skills);
  }
  std::shared_ptr<rc::Session> start(std::string skill, std::string implementation, rc::Arguments args) {
    rc::Request request{std::to_string(++sequence), skill, std::move(implementation), std::move(args), skill_timeout};
    auto session = std::make_shared<rc::Session>(skills, resources, context, std::move(request));
    sessions.push_back({std::move(skill), session});
    session->start(rc::Clock::now());
    return session;
  }
  void pump(rc::Time now) {
    for (auto& entry : sessions) {
      auto result = entry.session->tick(now);
      if (result.status == rc::Status::faulted) fault_latched = true;
    }
  }
  void cancel() { for (auto& entry : sessions) entry.session->cancel(); }
  bool stopped() const {
    return resources.empty() && std::all_of(sessions.begin(), sessions.end(),
        [](const Entry& e) { return rc::terminal(e.session->result().status); });
  }
};

class SkillNode final : public BT::StatefulActionNode {
 public:
  SkillNode(const std::string& name, const BT::NodeConfig& config, Engine& engine)
      : BT::StatefulActionNode(name, config), engine_(engine) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("skill"), BT::InputPort<std::string>("implementation", "standard"),
            BT::InputPort<std::string>("object"), BT::InputPort<std::string>("target", "")};
  }
  BT::NodeStatus onStart() override {
    const auto skill = required("skill");
    rc::Arguments args;
    for (const auto& input : engine_.skills.definition(skill).required_inputs)
      args[input] = required(input);
    session_ = engine_.start(skill, required("implementation"), std::move(args));
    return btStatus();
  }
  BT::NodeStatus onRunning() override { return btStatus(); }
  void onHalted() override { if (session_) session_->cancel(); }
 private:
  std::string required(const std::string& key) {
    std::string value;
    if (!getInput(key, value) || value.empty()) throw BT::RuntimeError("missing port: ", key);
    return value;
  }
  BT::NodeStatus btStatus() const {
    const auto status = session_->result().status;
    if (status == rc::Status::succeeded) return BT::NodeStatus::SUCCESS;
    return rc::terminal(status) ? BT::NodeStatus::FAILURE : BT::NodeStatus::RUNNING;
  }
  Engine& engine_;
  std::shared_ptr<rc::Session> session_;
};

class RuntimeNode final : public rclcpp::Node {
 public:
  RuntimeNode() : Node("robot_runtime") {
    const auto motion = declare_parameter<std::string>("motion_component", "mock_arm");
    const auto ticks = declare_parameter<int>("mock_action_ticks", 3);
    const auto fail = declare_parameter<bool>("mock_fail_pick", false);
    const auto period = declare_parameter<int>("tick_period_ms", 20);
    const auto skill_timeout = declare_parameter<int>("skill_timeout_ms", 2000);
    const auto stop_timeout = declare_parameter<int>("stop_timeout_ms", 2000);
    if (ticks <= 0 || ticks > 100000 || period <= 0 || skill_timeout <= 0 || stop_timeout <= 0)
      throw std::invalid_argument("runtime parameters must be positive and mock ticks <= 100000");
    stop_timeout_ = std::chrono::milliseconds(stop_timeout);
    engine_ = std::make_unique<Engine>(motion, ticks, fail);
    engine_->skill_timeout = std::chrono::milliseconds(skill_timeout);
    factory_.registerBuilder<SkillNode>("Skill", [this](const std::string& name, const BT::NodeConfig& config) {
      return std::make_unique<SkillNode>(name, config, *engine_);
    });
    tree_path_ = ament_index_cpp::get_package_share_directory("robot_bt_runtime") + "/trees/pick_place.xml";
    server_ = rclcpp_action::create_server<ExecuteTask>(this, "execute_task",
        [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const ExecuteTask::Goal> goal) {
          if (reserved_ || engine_->fault_latched || !engine_->resources.empty() ||
              goal->task_name != "pick_place" || goal->object_id != "workpiece" ||
              goal->target_id != "tray" || goal->timeout_ms == 0 || goal->timeout_ms > 600000)
            return rclcpp_action::GoalResponse::REJECT;
          reserved_ = true;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<GoalHandle> handle) {
          if (handle != active_) return rclcpp_action::CancelResponse::REJECT;
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle> handle) { accept(handle); });
    timer_ = create_wall_timer(std::chrono::milliseconds(period), [this] { tick(); });
    RCLCPP_INFO(get_logger(), "Mock-only runtime ready: /execute_task; motion=%s", motion.c_str());
  }
  void request_shutdown() {
    if (tree_) tree_->haltTree();
    engine_->cancel();
    if (!engine_->resources.empty())
      RCLCPP_WARN(get_logger(), "Shutdown stop not confirmed. Real hardware needs an independent stop/watchdog.");
  }
 private:
  void accept(const std::shared_ptr<GoalHandle>& handle) {
    active_ = handle;
    stopping_ = false;
    timeout_ = false;
    engine_->sessions.clear();
    engine_->world.observations.clear();
    try {
      auto blackboard = BT::Blackboard::create();
      blackboard->set("object", handle->get_goal()->object_id);
      blackboard->set("target", handle->get_goal()->target_id);
      tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromFile(tree_path_, blackboard));
      deadline_ = rc::Clock::now() + std::chrono::milliseconds(handle->get_goal()->timeout_ms);
    } catch (const std::exception& e) { finish(false, "failed", "INVALID_TASK", e.what()); }
  }
  void tick() {
    const auto now = rc::Clock::now();
    engine_->pump(now);
    if (!active_) return;
    try {
      if (engine_->fault_latched) {
        finish(false, "faulted", "STATE_UNKNOWN", "component fault; runtime latched"); return;
      }
      if (!stopping_ && (active_->is_canceling() || now >= deadline_)) {
        stopping_ = true;
        timeout_ = !active_->is_canceling();
        stop_deadline_ = now + stop_timeout_;
        tree_->haltTree();
        engine_->cancel();
      }
      if (stopping_) {
        if (engine_->stopped()) {
          finish(false, timeout_ ? "timed_out" : "canceled", timeout_ ? "TIMEOUT" : "CANCELED",
                 "all active mock actions confirmed stopped"); return;
        }
        if (now >= stop_deadline_) {
          engine_->fault_latched = true;
          finish(false, "faulted", "STOP_UNCONFIRMED", "stop deadline exceeded; control remains reserved"); return;
        }
      } else {
        const auto status = tree_->tickOnce();
        if (status == BT::NodeStatus::SUCCESS) {
          finish(true, "succeeded", "", "mock object placed and outcome verified"); return;
        }
        if (status == BT::NodeStatus::FAILURE) {
          rc::Result failure{rc::Status::failed, "TASK_FAILED", "behavior tree failed"};
          for (const auto& entry : engine_->sessions)
            if (rc::terminal(entry.session->result().status) && entry.session->result().status != rc::Status::succeeded)
              failure = entry.session->result();
          finish(false, rc::name(failure.status), failure.code, failure.message); return;
        }
      }
      auto feedback = std::make_shared<ExecuteTask::Feedback>();
      if (!engine_->sessions.empty()) {
        const auto& last = engine_->sessions.back();
        feedback->active_skill = last.skill;
        feedback->status = rc::name(last.session->result().status);
        feedback->message = last.session->result().message;
      }
      active_->publish_feedback(feedback);
    } catch (const std::exception& e) {
      engine_->fault_latched = true;
      finish(false, "faulted", "RUNTIME_EXCEPTION", e.what());
    }
  }
  void finish(bool success, const std::string& status, const std::string& code, const std::string& message) {
    if (tree_) tree_->haltTree();
    engine_->cancel();
    auto result = std::make_shared<ExecuteTask::Result>();
    result->success = success;
    result->status = status;
    result->error_code = code;
    result->message = message;
    if (success) active_->succeed(result);
    else if (active_->is_canceling() && engine_->stopped()) active_->canceled(result);
    else active_->abort(result);
    RCLCPP_INFO(get_logger(), "Task %s: %s %s", status.c_str(), code.c_str(), message.c_str());
    active_.reset();
    tree_.reset();
    reserved_ = false;
  }
  std::unique_ptr<Engine> engine_;
  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> tree_;
  std::string tree_path_;
  rclcpp_action::Server<ExecuteTask>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<GoalHandle> active_;
  rc::Time deadline_, stop_deadline_;
  std::chrono::milliseconds stop_timeout_{2000};
  bool reserved_{false}, stopping_{false}, timeout_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<RuntimeNode>();
    // All callbacks and core objects are deliberately serialized in v0.
    rclcpp::spin(node);
    node->request_shutdown();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("robot_runtime"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown();
  return 0;
}
