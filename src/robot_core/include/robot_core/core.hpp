#pragma once
#include "robot_core/geometry.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace robot_core {
using Arguments = std::map<std::string, std::string>;

// Identifiers can also be used as ROS parameter path segments. No dots or XML.
inline bool valid_id(const std::string& id) {
  if (id.empty() || id.size() > 64) return false;
  const auto alpha = [](char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
  };
  if (!alpha(id.front())) return false;
  return std::all_of(id.begin(), id.end(), [&](char c) { return alpha(c) || (c >= '0' && c <= '9'); });
}

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
  bool contains(const std::string& role) const { return roles_.find(role) != roles_.end(); }
 private:
  const Components& components_;
  std::map<std::string, std::string> roles_;
};

struct EvidenceMetadata {
  std::string source;
  bool synthetic{false};
  // Source-specific score in [0,1], not a calibrated probability. Missing is invalid.
  double quality{-1.0};
};
struct EvidencePolicy {
  std::chrono::milliseconds max_age{500};
  double minimum_quality{0.8};
  bool allow_synthetic{false};
};
inline bool valid_evidence_policy(const EvidencePolicy& p) {
  return p.max_age.count() > 0 && std::isfinite(p.minimum_quality) &&
         p.minimum_quality >= 0.0 && p.minimum_quality <= 1.0;
}
inline bool acceptable_evidence(const EvidenceMetadata& m, Time stamp, Time now, const EvidencePolicy& p) {
  return valid_evidence_policy(p) && valid_id(m.source) && std::isfinite(m.quality) &&
         m.quality >= p.minimum_quality && m.quality <= 1.0 &&
         (!m.synthetic || p.allow_synthetic) && fresh(stamp, now, p.max_age);
}
enum class PoseMeaning { object_pose, motion_target };
inline const char* name(PoseMeaning meaning) {
  return meaning == PoseMeaning::object_pose ? "object_pose" :
         meaning == PoseMeaning::motion_target ? "motion_target" : "unknown";
}
struct Observation {
  std::string object_id;
  std::string frame_id;
  Pose pose;
  Time stamp;
  bool valid{false};
  EvidenceMetadata evidence;
  PoseMeaning meaning{PoseMeaning::object_pose};
};
// Contact evidence is deliberately four-way: partial finger contact and
// unknown feedback are neither a secure grasp nor a confirmed release.
enum class ContactState { unknown, none, left_only, right_only, both };
inline bool dual_finger_grasp(ContactState state) { return state == ContactState::both; }
inline bool no_finger_contact(ContactState state) { return state == ContactState::none; }
struct OutcomeEvidence {
  std::string object_id, target_id;
  Time stamp{};
  bool valid{false}, condition_met{false};
  EvidenceMetadata evidence;
  uint64_t sample_id{0};
  // Optional source-domain measurement time and epoch. Adapters keep this
  // independent from the steady-time stamp used for communication freshness.
  uint64_t source_time_ns{0}, epoch{0};
  ContactState contact_state{ContactState::unknown};
  uint64_t contact_source_time_ns{0};
};
struct OutcomeVerification {
  OutcomeEvidence evidence;
  unsigned samples{0};
  std::chrono::milliseconds stable_for{0};
};
struct WorldState {
  std::map<std::string, Observation> observations;
  std::string attached_object;
  std::map<std::string, std::string> known_locations;
  // An inferred release location is not a new perception measurement.
  std::map<std::string, std::string> placement_candidates;
  Time attachment_stamp{};
  uint64_t attachment_source_time_ns{0}, attachment_epoch{0};
  std::map<std::string, Time> release_stamps;
  std::map<std::string, uint64_t> release_source_times_ns, release_epochs;
  std::map<std::string, OutcomeVerification> grasp_verifications, placement_verifications;
};
struct Requirement {
  std::string role;
  std::string interface_id;
  unsigned version{1};
};
struct SkillDefinition {
  struct Input { std::string name; std::string type; std::string description; };
  std::string id;
  std::string description;
  // v0.4 supports required entity_id inputs only; no implicit string coercion.
  std::vector<Input> inputs;
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

struct ImplementationInfo {
  std::string id;
  Dependencies dependencies;
  bool dependencies_satisfied{false};
  std::string unavailable_reason;
};
struct SkillInfo {
  SkillDefinition definition;
  std::vector<ImplementationInfo> implementations;
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
    if (!valid_id(definition.id)) throw std::invalid_argument("invalid skill ID");
    std::set<std::string> inputs;
    for (const auto& input : definition.inputs)
      if (!valid_id(input.name) || input.type != "entity_id" || !inputs.insert(input.name).second)
        throw std::invalid_argument("invalid or duplicate input schema");
    const auto id = definition.id;
    if (!definitions_.emplace(id, std::move(definition)).second)
      throw std::invalid_argument("duplicate skill: " + id);
  }
  void implement(const std::string& skill, const std::string& implementation, Factory factory,
                 Dependencies dependencies) {
    definition(skill);
    if (!valid_id(implementation) || !factory) throw std::invalid_argument("invalid implementation");
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
  void validate_arguments(const std::string& id, const Arguments& arguments) const {
    const auto& inputs = definition(id).inputs;
    if (arguments.size() != inputs.size()) throw std::invalid_argument("unexpected or missing input");
    for (const auto& input : inputs) {
      const auto it = arguments.find(input.name);
      if (it == arguments.end() || !valid_id(it->second))
        throw std::invalid_argument("invalid entity_id input: " + input.name);
    }
  }
  void validate_dependencies(const std::string& id, const std::string& implementation,
                             const Bindings& bindings) const {
    const auto& deps = dependencies(id, implementation);
    for (const auto& requirement : deps.components) {
      const auto component = bindings.get<Component>(requirement.role);
      if (component->interface_id() != requirement.interface_id ||
          component->interface_version() != requirement.version)
        throw std::invalid_argument("incompatible component: " + requirement.role);
    }
    for (const auto& role : deps.exclusive_roles) bindings.get<Component>(role);
    if (!deps.execution_gate_role.empty()) bindings.get<ExecutionGate>(deps.execution_gate_role);
  }
  std::vector<SkillInfo> catalog(const Bindings& bindings) const {
    std::vector<SkillInfo> result;
    for (const auto& definition : definitions_) {
      SkillInfo info{definition.second, {}};
      for (const auto& factory : factories_) {
        if (factory.first.first != definition.first) continue;
        ImplementationInfo impl{factory.first.second, factory.second.dependencies, false, ""};
        try {
          validate_dependencies(definition.first, impl.id, bindings);
          impl.dependencies_satisfied = true;
        } catch (const std::exception& e) { impl.unavailable_reason = e.what(); }
        info.implementations.push_back(std::move(impl));
      }
      result.push_back(std::move(info));
    }
    return result;
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
  // A copy for diagnostics; reading it cannot acquire or release resources.
  std::map<std::string, std::string> owners() const { return owners_; }
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
  const Request& request() const { return request_; }
  Result start(Time now) {
    if (result_.status != Status::idle) return result_; // never dispatch twice
    try {
      if (request_.id.empty() || request_.timeout.count() <= 0)
        throw std::invalid_argument("ID and positive timeout required");
      skills_.validate_arguments(request_.skill, request_.arguments);
      skills_.validate_dependencies(request_.skill, request_.implementation, context_.bindings);
      const auto& dependencies = skills_.dependencies(request_.skill, request_.implementation);
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
