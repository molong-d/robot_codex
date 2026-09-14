#pragma once

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace robot_core {
using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;
using Arguments = std::map<std::string, std::string>;

enum class Status { idle, running, canceling, succeeded, failed, canceled, timed_out, faulted };
inline const char* name(Status s) {
  switch (s) {
    case Status::idle: return "idle";
    case Status::running: return "running";
    case Status::canceling: return "canceling";
    case Status::succeeded: return "succeeded";
    case Status::failed: return "failed";
    case Status::canceled: return "canceled";
    case Status::timed_out: return "timed_out";
    case Status::faulted: return "faulted";
  }
  return "unknown";
}
inline bool terminal(Status s) {
  return s == Status::succeeded || s == Status::failed || s == Status::canceled ||
         s == Status::timed_out || s == Status::faulted;
}

struct Result {
  Status status{Status::idle};
  std::string code;
  std::string message;
};

// Architecture component, not necessarily a ROS composable node.
class Component {
 public:
  virtual ~Component() = default;
  virtual std::string interface_id() const = 0;
  virtual unsigned interface_version() const { return 1; }
  // Different adapters for one physical resource MUST use the same resource ID.
  virtual std::string resource_id() const = 0;
};

struct Admission {
  bool allowed{false};
  std::string code;
  std::string message;
};

// A hardware-facing implementation may bind this to an E-stop, guarded-area,
// controller-mode, or operator-permission check.
class ExecutionGate : public Component {
 public:
  std::string interface_id() const final { return "execution_gate"; }
  virtual Admission admit(const std::string& skill, const Arguments& arguments, Time now) const = 0;
};

class Components {
 public:
  void add(const std::string& id, std::shared_ptr<Component> component) {
    if (id.empty() || !component || component->resource_id().empty())
      throw std::invalid_argument("invalid component registration");
    if (!items_.emplace(id, std::move(component)).second)
      throw std::invalid_argument("duplicate component: " + id);
  }
  std::shared_ptr<Component> at(const std::string& id) const {
    const auto it = items_.find(id);
    if (it == items_.end()) throw std::invalid_argument("unknown component: " + id);
    return it->second;
  }
 private:
  std::map<std::string, std::shared_ptr<Component>> items_;
};

class Bindings {
 public:
  Bindings(const Components& components, std::map<std::string, std::string> roles)
      : components_(components), roles_(std::move(roles)) {}
  template<class T> std::shared_ptr<T> get(const std::string& role) const {
    const auto it = roles_.find(role);
    if (it == roles_.end()) throw std::invalid_argument("unbound role: " + role);
    auto value = std::dynamic_pointer_cast<T>(components_.at(it->second));
    if (!value) throw std::invalid_argument("component type mismatch: " + role);
    return value;
  }
 private:
  const Components& components_;
  std::map<std::string, std::string> roles_;
};

struct Observation {
  std::string object_id;
  std::string frame_id;
  Time stamp;
  bool valid{false};
};
struct WorldState {
  std::map<std::string, Observation> observations;
};
struct Requirement {
  std::string role;
  std::string interface_id;
  unsigned version{1};
};
struct SkillDefinition {
  std::string id;
  std::string description;
  std::vector<std::string> required_inputs;
  // Descriptive contract; implementations enforce the conditions, not a rule parser.
  std::string precondition;
  std::string invariant;
  std::string success_condition;
};
struct Dependencies {
  std::vector<Requirement> components;
  std::vector<std::string> exclusive_roles;
  // Empty for read-only skills. Actuating skills must name an ExecutionGate role.
  std::string execution_gate_role;
};
struct Context {
  Bindings& bindings;
  WorldState& world;
};

class Skill {
 public:
  virtual ~Skill() = default;
  // Nonblocking calls; a terminal outcome means this skill's commands are quiescent.
  virtual Result start(const Arguments&, Time) = 0;
  virtual Result tick(Time) = 0;
  virtual void cancel() = 0;
};

class Skills {
 public:
  using Factory = std::function<std::unique_ptr<Skill>(Context&)>;
  void define(SkillDefinition definition) {
    if (definition.id.empty()) throw std::invalid_argument("empty skill ID");
    const auto id = definition.id;
    if (!definitions_.emplace(id, std::move(definition)).second)
      throw std::invalid_argument("duplicate skill: " + id);
  }
  void implement(const std::string& skill, const std::string& implementation, Factory factory,
                 Dependencies dependencies) {
    definition(skill);
    if (implementation.empty() || !factory) throw std::invalid_argument("invalid implementation");
    if (!factories_.emplace(std::make_pair(skill, implementation),
                           Implementation{std::move(factory), std::move(dependencies)}).second)
      throw std::invalid_argument("duplicate implementation");
  }
  const Dependencies& dependencies(const std::string& skill, const std::string& implementation) const {
    return factories_.at({skill, implementation}).dependencies;
  }
  const SkillDefinition& definition(const std::string& id) const {
    return definitions_.at(id);
  }
  std::unique_ptr<Skill> create(const std::string& id, const std::string& implementation,
                              Context& context) const {
    auto skill = factories_.at({id, implementation}).factory(context);
    if (!skill) throw std::runtime_error("factory returned null");
    return skill;
  }
 private:
  struct Implementation { Factory factory; Dependencies dependencies; };
  std::map<std::string, SkillDefinition> definitions_;
  std::map<std::pair<std::string, std::string>, Implementation> factories_;
};

// Single-executor ownership. External processes need a shared resource authority.
class Resources {
 public:
  bool acquire(const std::string& owner, const std::vector<std::string>& resources) {
    if (active_.count(owner)) return false;
    for (const auto& resource : resources)
      if (owners_.count(resource)) return false;
    active_.insert(owner);
    for (const auto& resource : resources) owners_[resource] = owner;
    return true;
  }
  void release(const std::string& owner) {
    active_.erase(owner);
    for (auto it = owners_.begin(); it != owners_.end();)
      if (it->second == owner) it = owners_.erase(it); else ++it;
  }
  bool empty() const { return active_.empty(); }
 private:
  std::set<std::string> active_;
  std::map<std::string, std::string> owners_;
};

struct Request {
  std::string id;
  std::string skill;
  std::string implementation;
  Arguments arguments;
  std::chrono::milliseconds timeout{5000};
};

class Session {
 public:
  Session(const Skills& skills, Resources& resources, Context& context, Request request)
      : skills_(skills), resources_(resources), context_(context), request_(std::move(request)) {}
  Session(const Session&) = delete;
  Session& operator=(const Session&) = delete;
  ~Session() {
    // Never release an unconfirmed hardware lease from a destructor.
    if (skill_ && (result_.status == Status::running || result_.status == Status::canceling)) {
      try { skill_->cancel(); } catch (...) {}
    }
  }
  const Result& result() const { return result_; }
  Result start(Time now) {
    if (result_.status != Status::idle) return result_; // never dispatch twice
    try {
      if (request_.id.empty() || request_.timeout.count() <= 0)
        throw std::invalid_argument("ID and positive timeout required");
      const auto& definition = skills_.definition(request_.skill);
      const auto& dependencies = skills_.dependencies(request_.skill, request_.implementation);
      if (request_.arguments.size() != definition.required_inputs.size())
        throw std::invalid_argument("unexpected or missing input");
      for (const auto& key : definition.required_inputs) {
        auto it = request_.arguments.find(key);
        if (it == request_.arguments.end() || it->second.empty())
          throw std::invalid_argument("missing input: " + key);
      }
      for (const auto& requirement : dependencies.components) {
        auto component = context_.bindings.get<Component>(requirement.role);
        if (component->interface_id() != requirement.interface_id ||
            component->interface_version() != requirement.version)
          throw std::invalid_argument("incompatible component: " + requirement.role);
      }
      if (!dependencies.execution_gate_role.empty()) {
        const auto gate = context_.bindings.get<ExecutionGate>(dependencies.execution_gate_role);
        const auto admission = gate->admit(request_.skill, request_.arguments, now);
        if (!admission.allowed)
          return result_ = {Status::failed,
                            admission.code.empty() ? "EXECUTION_DENIED" : admission.code,
                            admission.message.empty() ? "execution gate denied request" : admission.message};
      }
      std::vector<std::string> claimed;
      for (const auto& role : dependencies.exclusive_roles)
        claimed.push_back(context_.bindings.get<Component>(role)->resource_id());
      skill_ = skills_.create(request_.skill, request_.implementation, context_);
      if (!resources_.acquire(request_.id, claimed))
        return result_ = {Status::failed, "RESOURCE_BUSY", "resource already owned"};
      leased_ = true;
    } catch (const std::exception& e) {
      return result_ = {Status::failed, "INVALID_REQUEST", e.what()};
    }
    deadline_ = now + request_.timeout;
    try { accept(skill_->start(request_.arguments, now)); }
    catch (const std::exception& e) { fault(e.what()); }
    return result_;
  }
  Result tick(Time now) {
    if (terminal(result_.status) || result_.status == Status::idle) return result_;
    if (result_.status == Status::running && now >= deadline_) cancel(true);
    if (result_.status == Status::faulted) return result_;
    try { accept(skill_->tick(now)); }
    catch (const std::exception& e) { fault(e.what()); }
    return result_;
  }
  void cancel(bool timeout = false) {
    if (result_.status != Status::running) return;
    stopping_ = true;
    timeout_ = timeout;
    result_ = {Status::canceling, timeout ? "DEADLINE" : "CANCEL_REQUESTED", "awaiting stop confirmation"};
    try { skill_->cancel(); }
    catch (const std::exception& e) { fault(e.what()); }
  }
 private:
  void accept(Result value) {
    if (value.status == Status::idle || value.status == Status::timed_out)
      throw std::runtime_error("invalid skill lifecycle result");
    if (value.status == Status::faulted) { fault(value.message); return; }
    if (stopping_) {
      if (!terminal(value.status)) return; // retain canceling and the lease
      value = {timeout_ ? Status::timed_out : Status::canceled,
               timeout_ ? "TIMEOUT" : "CANCELED", "stop confirmed; inspect state before retry"};
    }
    result_ = std::move(value);
    if (terminal(result_.status) && leased_) {
      resources_.release(request_.id);
      leased_ = false;
    }
  }
  void fault(const std::string& message) {
    result_ = {Status::faulted, "STATE_UNKNOWN", message};
    // Best-effort stop; fault remains latched and ownership is retained.
    try { if (skill_) skill_->cancel(); } catch (...) {}
  }
  const Skills& skills_;
  Resources& resources_;
  Context& context_;
  Request request_;
  std::unique_ptr<Skill> skill_;
  Result result_;
  Time deadline_{};
  bool leased_{false};
  bool stopping_{false};
  bool timeout_{false};
};
}  // namespace robot_core
