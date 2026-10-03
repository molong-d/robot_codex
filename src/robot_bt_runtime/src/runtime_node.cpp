#include "robot_core/demo.hpp"
#include "catalog_support.hpp"
#include "plan_support.hpp"
#include "record_support.hpp"
#include "robot_ros_adapters/adapters.hpp"
#include "robot_interfaces/action/execute_task.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <algorithm>
#include <chrono>
#include <memory>
#include <type_traits>

namespace rc = robot_core;
using ExecuteTask = robot_interfaces::action::ExecuteTask;
using ExecutePlan = plan_support::ExecutePlan;
using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;
using PlanGoalHandle = rclcpp_action::ServerGoalHandle<ExecutePlan>;
using namespace std::chrono_literals;

// Sessions outlive BT nodes: halting a tree requests stop, the timer drains confirmation.
struct Engine {
  rc::Components components;
  rc::Bindings bindings;
  rc::WorldState world;
  rc::Context context;
  rc::Skills skills;
  rc::Resources resources;
  rc::ExecutionJournal journal;
  struct Entry { std::string skill; std::shared_ptr<rc::Session> session; };
  std::vector<Entry> sessions;
  uint64_t sequence{0};
  std::chrono::milliseconds skill_timeout{2000};
  bool fault_latched{false};

  Engine(std::string motion, int ticks, int stop_ticks, bool fail_grasp, bool permitted, rclcpp::Node* node,
         bool ros_backend, robot_ros_adapters::Config config, const rc::DemoScene& scene, size_t history_capacity,
         bool outcome_enabled, const std::string& verification_failure, int verification_window_ms)
      : bindings(components, {{"perception", "mock_camera"}, {"motion", motion},
                              {"gripper", "mock_gripper"}, {"safety", "execution_gate"}, {"outcome", "demo_outcome"}}),
        context{bindings, world}, journal(history_capacity) {
    const auto robot = std::make_shared<rc::MockRobotState>();
    components.add("execution_gate", std::make_shared<rc::MockExecutionGate>(permitted));
    components.add("mock_camera", std::make_shared<rc::ConfiguredDemoLocator>(scene));
    rc::ManipulationPolicy policy;
    policy.frame_id = scene.frame();
    policy.allow_synthetic = true;  // this runtime only has demonstration backends
    if (ros_backend) {
      components.add(motion, std::make_shared<robot_ros_adapters::MoveItArm>(node, config));
      components.add("mock_gripper", std::make_shared<robot_ros_adapters::ParallelGripper>(node, config));
    } else {
      components.add("mock_arm", std::make_shared<rc::MockArmMotion>(robot, ticks, false, stop_ticks));
      components.add("slow_mock_arm", std::make_shared<rc::MockArmMotion>(robot, ticks * 2, false, stop_ticks));
      components.add("mock_gripper", std::make_shared<rc::MockGripper>(robot, 2, fail_grasp, stop_ticks));
    }
    const auto arm = bindings.get<rc::ArmMotion>("motion");
    const auto gripper = bindings.get<rc::Gripper>("gripper");
    if (outcome_enabled)
      components.add("demo_outcome", std::make_shared<rc::DemoOutcomeObserver>(arm, gripper, scene,
          ros_backend ? "none" : verification_failure));
    bindings.get<rc::ExecutionGate>("safety");
    rc::VerificationPolicy verification;
    verification.evidence.allow_synthetic = true;
    verification.stable_duration = std::chrono::milliseconds(verification_window_ms);
    rc::register_demo_skills(skills, policy, verification);
  }
  std::shared_ptr<rc::Session> start(std::string skill, std::string implementation, rc::Arguments args) {
    rc::Request request{std::to_string(++sequence), skill, std::move(implementation), std::move(args), skill_timeout};
    auto session = std::make_shared<rc::Session>(skills, resources, context, std::move(request));
    sessions.push_back({std::move(skill), session});
    session->start(rc::Clock::now());
    if (session->result().status == rc::Status::faulted) fault_latched = true;
    record_session(session, rc::Clock::now());
    return session;
  }
  void pump(rc::Time now) {
    for (auto& entry : sessions) {
      auto result = entry.session->tick(now);
      if (result.status == rc::Status::faulted) fault_latched = true;
      if (!journal.active_id().empty()) record_session(entry.session, now);
    }
  }
  void cancel() {
    for (auto& entry : sessions) {
      entry.session->cancel();
      if (!journal.active_id().empty()) record_session(entry.session, rc::Clock::now());
    }
  }
  bool stopped() const {
    return resources.empty() && std::all_of(sessions.begin(), sessions.end(),
        [](const Entry& e) { return rc::terminal(e.session->result().status); });
  }
  rc::ExecutionSnapshot snapshot(rc::Time now) const {
    return {now, world, resources.owners(), resources.empty(), fault_latched, stopped()};
  }
  void record_session(const std::shared_ptr<rc::Session>& session, rc::Time now) {
    std::optional<rc::OutcomeVerification> proof;
    const auto& skill = session->request().skill;
    if (skill == "verify_grasp" || skill == "verify_placement") {
      const auto& values = skill == "verify_grasp" ? world.grasp_verifications : world.placement_verifications;
      const auto it = values.find(session->request().arguments.at("object"));
      if (it != values.end()) proof = it->second;
    }
    journal.observe(session->request(), session->result(), now, std::move(proof));
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
    for (const auto& input : engine_.skills.definition(skill).inputs)
      args[input.name] = required(input.name);
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
    const auto backend = declare_parameter<std::string>("backend", "mock");
    backend_ = backend;
    const auto simulation_only = declare_parameter<bool>("simulation_only", true);
    const auto ros_enabled = declare_parameter<bool>("ros_backend_enabled", false);
    if (backend != "mock" && backend != "panda_ros") throw std::invalid_argument("unknown backend");
    if (backend == "panda_ros" && (!simulation_only || !ros_enabled))
      throw std::invalid_argument("Panda ROS backend requires explicit simulation-only enablement");
    robot_ros_adapters::Config config;
    config.move_action = declare_parameter<std::string>("move_action", config.move_action);
    config.gripper_action = declare_parameter<std::string>("gripper_action", config.gripper_action);
    config.frame = catalog_support::startup_parameter<std::string>(this, "base_frame",
        backend == "panda_ros" ? config.frame : "base_link");
    config.tip = declare_parameter<std::string>("end_effector_link", config.tip);
    config.group = declare_parameter<std::string>("planning_group", config.group);
    config.arm_joints = declare_parameter<std::vector<std::string>>("arm_joints", config.arm_joints);
    config.finger_joint = declare_parameter<std::string>("finger_joint", config.finger_joint);
    config.arm_resource = declare_parameter<std::string>("arm_resource", config.arm_resource);
    config.gripper_resource = declare_parameter<std::string>("gripper_resource", config.gripper_resource);
    config.simulation_grasp_detection = declare_parameter<bool>("simulation_grasp_detection", false);
    const auto outcome_enabled = catalog_support::startup_parameter<bool>(this, "simulation_outcome_evidence", backend == "mock");
    const auto verification_failure = catalog_support::startup_parameter<std::string>(this, "mock_verification_failure", "none");
    const auto verification_window_ms = catalog_support::startup_parameter<int>(this, "verification_window_ms", 100);
    if (verification_failure != "none" && verification_failure != "grasp" && verification_failure != "placement")
      throw std::invalid_argument("mock verification failure must be none, grasp or placement");
    if (verification_window_ms < 1 || verification_window_ms > 30000)
      throw std::invalid_argument("verification window must be 1..30000 ms");
    const auto motion = declare_parameter<std::string>("motion_component", "mock_arm");
    const auto ticks = declare_parameter<int>("mock_action_ticks", 3);
    const auto stop_ticks = catalog_support::startup_parameter<int>(this, "mock_stop_ticks", 2);
    const auto fail = declare_parameter<bool>("mock_fail_pick", false);
    const auto motion_permitted = declare_parameter<bool>("mock_motion_permitted", true);
    const auto period = declare_parameter<int>("tick_period_ms", 20);
    const auto skill_timeout = declare_parameter<int>("skill_timeout_ms", 2000);
    const auto stop_timeout = declare_parameter<int>("stop_timeout_ms", 2000);
    const auto history_capacity = catalog_support::startup_parameter<int>(this, "execution_history_capacity", 32);
    if (ticks <= 0 || ticks > 100000 || stop_ticks <= 0 || stop_ticks > 100000 || period <= 0 || skill_timeout <= 0 || stop_timeout <= 0)
      throw std::invalid_argument("runtime parameters must be positive and mock ticks <= 100000");
    stop_timeout_ = std::chrono::milliseconds(stop_timeout);
    scene_ = std::make_unique<rc::DemoScene>(catalog_support::load_scene(this, config.frame, backend == "panda_ros"));
    tasks_ = std::make_unique<rc::TaskCatalog>(catalog_support::load_tasks(this));
    if (history_capacity < 1 || history_capacity > 128) throw std::invalid_argument("execution history capacity requires 1..128");
    engine_ = std::make_unique<Engine>(motion, ticks, stop_ticks, fail, motion_permitted, this, backend == "panda_ros", config, *scene_,
                                     static_cast<size_t>(history_capacity), outcome_enabled, verification_failure, verification_window_ms);
    tasks_->validate_configuration(*scene_, engine_->skills, engine_->bindings);
    engine_->skill_timeout = std::chrono::milliseconds(skill_timeout);
    factory_.registerBuilder<SkillNode>("Skill", [this](const std::string& name, const BT::NodeConfig& config) {
      return std::make_unique<SkillNode>(name, config, *engine_);
    });
    const auto tree_dir = ament_index_cpp::get_package_share_directory("robot_bt_runtime") + "/trees/";
    // Load reviewed templates at startup, before accepting any task.
    factory_.registerBehaviorTreeFromFile(tree_dir + "pick_place.xml");
    factory_.registerBehaviorTreeFromFile(tree_dir + "locate_object.xml");
    factory_.registerBehaviorTreeFromFile(tree_dir + "verified_pick_place.xml");
    catalog_server_ = create_service<catalog_support::GetCatalog>("get_catalog",
        [this](const std::shared_ptr<catalog_support::GetCatalog::Request>,
               const std::shared_ptr<catalog_support::GetCatalog::Response> response) {
          catalog_support::describe(engine_->skills, engine_->bindings, *tasks_, *scene_, *response);
        });
    state_server_ = create_service<record_support::GetRuntimeState>("get_runtime_state",
        [this](const std::shared_ptr<record_support::GetRuntimeState::Request>,
               const std::shared_ptr<record_support::GetRuntimeState::Response> response) {
          response->schema_version = 2; response->runtime_id = runtime_id_; response->backend = backend_;
          response->simulation_only = true; response->busy = reserved_;
          response->active_execution_id = engine_->journal.active_id();
          response->history_capacity = static_cast<uint32_t>(engine_->journal.capacity());
          response->evicted_records = engine_->journal.evicted_count();
          const auto ids = engine_->journal.recent_ids();
          response->recent_execution_ids.assign(ids.begin(), ids.end());
          response->snapshot = record_support::describe(engine_->snapshot(rc::Clock::now()));
        });
    record_server_ = create_service<record_support::GetExecution>("get_execution",
        [this](const std::shared_ptr<record_support::GetExecution::Request> request,
               const std::shared_ptr<record_support::GetExecution::Response> response) {
          response->schema_version = 2; response->runtime_id = runtime_id_;
          const auto record = engine_->journal.get(request->execution_id, rc::Clock::now());
          response->found = record.has_value();
          if (record) {
            response->lookup_status = rc::terminal(record->result.status) ? "completed" : "active";
            response->record = record_support::describe(*record);
          } else response->lookup_status = request->execution_id.empty() ? "no_records" : "not_found";
        });
    server_ = serve<ExecuteTask>("execute_task", [this](const ExecuteTask::Goal& goal) {
      tasks_->admit(goal.task_name, goal.object_id, goal.target_id, *scene_, engine_->skills, engine_->bindings);
    }, [this](const std::shared_ptr<GoalHandle>& handle) { accept_task(handle); });
    plan_server_ = serve<ExecutePlan>("execute_plan", [this](const ExecutePlan::Goal& goal) {
      rc::validate_plan(plan_support::decode(goal), *scene_, engine_->skills, engine_->bindings, engine_->world);
    }, [this](const std::shared_ptr<PlanGoalHandle>& handle) { accept_plan(handle); });
    timer_ = create_wall_timer(std::chrono::milliseconds(period), [this] { tick(); });
    RCLCPP_INFO(get_logger(), "Demo runtime ready: /execute_task, /execute_plan, /get_catalog, /get_runtime_state, /get_execution; backend=%s; motion=%s; permitted=%s",
                backend.c_str(), motion.c_str(), motion_permitted ? "true" : "false");
  }
  void request_shutdown() {
    if (tree_) tree_->haltTree();
    engine_->cancel();
    if (!engine_->resources.empty())
      RCLCPP_WARN(get_logger(), "Shutdown stop not confirmed. Real hardware needs an independent stop/watchdog.");
  }
 private:
  // Both action APIs share a single active goal and the same stop/resource path.
  struct ActiveGoal {
    const void* identity;
    std::function<bool()> is_canceling;
    std::function<void(bool, const std::string&, const std::string&, const std::string&)> complete;
    std::function<void()> feedback;
  };
  template<class Action, class Validator, class Acceptor>
  typename rclcpp_action::Server<Action>::SharedPtr serve(const std::string& endpoint,
                                                         Validator validate, Acceptor accept) {
    using Handle = rclcpp_action::ServerGoalHandle<Action>;
    return rclcpp_action::create_server<Action>(this, endpoint,
        [this, validate](const rclcpp_action::GoalUUID&, std::shared_ptr<const typename Action::Goal> goal) {
          if (reserved_ || engine_->fault_latched || !engine_->resources.empty() ||
              goal->timeout_ms == 0 || goal->timeout_ms > 600000)
            return rclcpp_action::GoalResponse::REJECT;
          try {
            validate(*goal);
          } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Task rejected before execution: %s", e.what());
            return rclcpp_action::GoalResponse::REJECT;
          }
          reserved_ = true;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<Handle> handle) {
          if (!active_ || handle.get() != active_->identity) return rclcpp_action::CancelResponse::REJECT;
          cancel_requested_ = true;
          engine_->cancel();
          record_state();
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [accept](const std::shared_ptr<Handle> handle) { accept(handle); });
  }
  template<class Action> void begin(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>>& handle,
                                    const std::string& task_name, uint32_t total_steps) {
    active_ = std::make_unique<ActiveGoal>();
    active_->identity = handle.get();
    active_->is_canceling = [handle] { return handle->is_canceling(); };
    active_->complete = [this, handle](bool success, const std::string& status,
                                      const std::string& code, const std::string& message) {
      auto result = std::make_shared<typename Action::Result>();
      result->success = success; result->status = status;
      result->error_code = code; result->message = message;
      if constexpr (std::is_same_v<Action, ExecutePlan>)
        result->completed_steps = static_cast<uint32_t>(std::count_if(engine_->sessions.begin(), engine_->sessions.end(),
            [](const Engine::Entry& e) { return e.session->result().status == rc::Status::succeeded; }));
      if (success) handle->succeed(result);
      else if (handle->is_canceling() && engine_->stopped()) handle->canceled(result);
      else handle->abort(result);
    };
    active_->feedback = [this, handle] {
      auto feedback = std::make_shared<typename Action::Feedback>();
      if (!engine_->sessions.empty()) {
        const auto& last = engine_->sessions.back();
        feedback->active_skill = last.skill;
        feedback->status = rc::name(last.session->result().status);
        feedback->message = last.session->result().message;
        if constexpr (std::is_same_v<Action, ExecutePlan>)
          feedback->step_index = static_cast<uint32_t>(engine_->sessions.size()-1);
      }
      handle->publish_feedback(feedback);
    };
    stopping_ = false;
    cancel_requested_ = false;
    timeout_ = false;
    engine_->sessions.clear();
    engine_->world.observations.clear();
    const auto now = rc::Clock::now();
    engine_->journal.begin(record_support::hex_id(handle->get_goal_id()),
        std::is_same_v<Action, ExecutePlan> ? "execute_plan" : "execute_task", task_name,
        total_steps, handle->get_goal()->timeout_ms, now, record_support::unix_ms());
    record_state();
    deadline_ = now + std::chrono::milliseconds(handle->get_goal()->timeout_ms);
  }
  void accept_task(const std::shared_ptr<GoalHandle>& handle) {
    const auto& definition = tasks_->definitions().at(handle->get_goal()->task_name);
    begin<ExecuteTask>(handle, definition.name, rc::TaskCatalog::step_count(definition));
    try {
      auto blackboard = BT::Blackboard::create();
      blackboard->set("object", handle->get_goal()->object_id);
      blackboard->set("target", handle->get_goal()->target_id);
      const auto& task = tasks_->admit(handle->get_goal()->task_name, handle->get_goal()->object_id,
                                      handle->get_goal()->target_id, *scene_, engine_->skills, engine_->bindings);
      blackboard->set("implementation", task.implementation);
      tree_ = std::make_unique<BT::Tree>(factory_.createTree(
          task.template_id == "pick_place" ? "PickPlace" :
          task.template_id == "verified_pick_place" ? "VerifiedPickPlace" : "LocateObject", blackboard));
      success_message_ = task.template_id == "verified_pick_place" ?
          "synthetic grasp and placement evidence verified; no physical object/contact sensing" :
          task.template_id == "pick_place" ? "release verified; object placement needs perception confirmation" :
          "fresh configured demo motion target obtained";
    } catch (const std::exception& e) { finish(false, "failed", "INVALID_TASK", e.what()); }
  }
  void accept_plan(const std::shared_ptr<PlanGoalHandle>& handle) {
    begin<ExecutePlan>(handle, "", static_cast<uint32_t>(handle->get_goal()->steps.size()));
    try {
      const auto xml = plan_support::tree_xml(plan_support::decode(*handle->get_goal()),
          *scene_, engine_->skills, engine_->bindings, engine_->world);
      tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromText(xml));
      success_message_ = "all skill steps succeeded; consult verification provenance and remaining placement candidates";
    } catch (const std::exception& e) { finish(false, "failed", "INVALID_PLAN", e.what()); }
  }
  void tick() {
    const auto now = rc::Clock::now();
    try {
      // Process stop before pumping: a tick must not start a new sub-action
      // after the task cancellation/deadline was already observed.
      if (active_ && !stopping_ && (cancel_requested_ || active_->is_canceling() || now >= deadline_)) {
        stopping_ = true;
        timeout_ = !cancel_requested_ && !active_->is_canceling();
        stop_deadline_ = now + stop_timeout_;
        if (tree_) tree_->haltTree();
        engine_->cancel();
      }
      engine_->pump(now);
      if (!active_) return;
      record_state();
      if (engine_->fault_latched) {
        finish(false, "faulted", "STATE_UNKNOWN", "component fault; runtime latched"); return;
      }
      if (stopping_) {
        if (engine_->stopped()) {
          finish(false, timeout_ ? "timed_out" : "canceled", timeout_ ? "TIMEOUT" : "CANCELED",
                 "all active actions confirmed stopped by measured feedback"); return;
        }
        if (now >= stop_deadline_) {
          engine_->fault_latched = true;
          finish(false, "faulted", "STOP_UNCONFIRMED", "stop deadline exceeded; control remains reserved"); return;
        }
      } else {
        const auto status = tree_->tickOnce();
        if (status == BT::NodeStatus::SUCCESS) {
          finish(true, "succeeded", "", success_message_); return;
        }
        if (status == BT::NodeStatus::FAILURE) {
          rc::Result failure{rc::Status::failed, "TASK_FAILED", "behavior tree failed"};
          for (const auto& entry : engine_->sessions)
            if (rc::terminal(entry.session->result().status) && entry.session->result().status != rc::Status::succeeded)
              failure = entry.session->result();
          finish(false, rc::name(failure.status), failure.code, failure.message); return;
        }
      }
      record_state();
      active_->feedback();
    } catch (const std::exception& e) {
      engine_->fault_latched = true;
      if (active_) finish(false, "faulted", "RUNTIME_EXCEPTION", e.what());
    }
  }
  void finish(bool success, const std::string& status, const std::string& code, const std::string& message) {
    if (tree_) tree_->haltTree();
    engine_->cancel();
    active_->complete(success, status, code, message);
    rc::Status outcome = rc::Status::failed;
    for (const auto value : {rc::Status::succeeded, rc::Status::failed, rc::Status::canceled,
                             rc::Status::timed_out, rc::Status::faulted})
      if (status == rc::name(value)) outcome = value;
    const auto now = rc::Clock::now();
    engine_->journal.finish({outcome, code, message}, engine_->snapshot(now), now);
    RCLCPP_INFO(get_logger(), "Task %s: %s %s", status.c_str(), code.c_str(), message.c_str());
    active_.reset();
    tree_.reset();
    reserved_ = false;
  }
  void record_state() {
    if (!active_) return;
    const auto now = rc::Clock::now();
    const bool stopping = stopping_ || cancel_requested_;
    engine_->journal.update({stopping ? rc::Status::canceling : rc::Status::running,
        stopping ? (timeout_ ? "DEADLINE" : "CANCEL_REQUESTED") : "",
        stopping ? "awaiting measured stop confirmation" : ""}, engine_->snapshot(now), now);
  }
  std::unique_ptr<Engine> engine_;
  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> tree_;
  std::unique_ptr<rc::DemoScene> scene_;
  std::unique_ptr<rc::TaskCatalog> tasks_;
  std::string success_message_;
  std::string backend_, runtime_id_{record_support::runtime_id()};
  rclcpp::Service<catalog_support::GetCatalog>::SharedPtr catalog_server_;
  rclcpp::Service<record_support::GetRuntimeState>::SharedPtr state_server_;
  rclcpp::Service<record_support::GetExecution>::SharedPtr record_server_;
  rclcpp_action::Server<ExecuteTask>::SharedPtr server_;
  rclcpp_action::Server<ExecutePlan>::SharedPtr plan_server_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<ActiveGoal> active_;
  rc::Time deadline_, stop_deadline_;
  std::chrono::milliseconds stop_timeout_{2000};
  bool reserved_{false}, stopping_{false}, timeout_{false}, cancel_requested_{false};
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
