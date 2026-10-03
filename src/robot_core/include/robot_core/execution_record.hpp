#pragma once
#include "robot_core/core.hpp"
#include <cstdint>
#include <deque>
#include <optional>

namespace robot_core {
struct ExecutionSnapshot {
  Time captured_at;
  WorldState world;
  std::map<std::string, std::string> resource_owners;
  bool resources_empty{true};
  bool fault_latched{false};
  // A terminal task result alone does not imply confirmed actuator stop.
  bool stop_confirmed{false};
};
struct StateTransition { uint64_t elapsed_ms; Result result; };
struct SkillExecutionRecord {
  Request request;
  uint64_t started_ms{0}, updated_ms{0};
  Result result;
  std::vector<StateTransition> transitions;
  std::optional<OutcomeVerification> verification;
};
struct ExecutionRecord {
  std::string id, entrypoint, task_name;
  uint64_t started_unix_ms{0}, elapsed_ms{0};
  uint32_t timeout_ms{0}, total_steps{0};
  Result result{Status::running, "", ""};
  std::vector<SkillExecutionRecord> steps;
  ExecutionSnapshot snapshot;
};

// Single-executor, in-memory diagnostics, not an execution authority or an
// idempotency/recovery ledger. Completed records and their snapshots are frozen.
class ExecutionJournal {
 public:
  explicit ExecutionJournal(size_t capacity = 32) : capacity_(capacity) {
    if (capacity == 0 || capacity > 128) throw std::invalid_argument("history capacity requires 1..128");
  }
  void begin(std::string id, std::string entrypoint, std::string task_name,
             uint32_t total_steps, uint32_t timeout_ms, Time now, uint64_t unix_ms) {
    if (active_ || id.empty() || total_steps == 0 || total_steps > 32 || timeout_ms == 0 ||
        (entrypoint != "execute_task" && entrypoint != "execute_plan") || find(id))
      throw std::invalid_argument("invalid or duplicate execution record");
    started_ = now;
    active_ = ExecutionRecord{};
    active_->id = std::move(id); active_->entrypoint = std::move(entrypoint);
    active_->task_name = std::move(task_name); active_->total_steps = total_steps;
    active_->timeout_ms = timeout_ms; active_->started_unix_ms = unix_ms;
  }
  void observe(const Request& request, const Result& result, Time now,
               std::optional<OutcomeVerification> verification = std::nullopt) {
    auto& record = current();
    auto it = std::find_if(record.steps.begin(), record.steps.end(),
                          [&](const auto& step) { return step.request.id == request.id; });
    if (it == record.steps.end()) {
      if (record.steps.size() >= record.total_steps) throw std::logic_error("execution exceeds planned step count");
      record.steps.push_back({request, elapsed(now), elapsed(now), {}, {}, std::nullopt});
      it = record.steps.end()-1;
    }
    if (terminal(it->result.status)) return;  // immutable terminal evidence
    if (it->transitions.empty() || it->result.status != result.status) {
      if (it->transitions.size() == 8) throw std::logic_error("too many skill state transitions");
      it->transitions.push_back({elapsed(now), result});
    }
    it->updated_ms = elapsed(now);
    it->result = result;
    if (result.status == Status::succeeded) it->verification = std::move(verification);
  }
  void update(Result result, ExecutionSnapshot snapshot, Time now) {
    if (terminal(result.status)) throw std::logic_error("use finish for terminal execution");
    auto& record = current();
    record.result = std::move(result); record.snapshot = std::move(snapshot);
    record.elapsed_ms = elapsed(now);
  }
  void finish(Result result, ExecutionSnapshot snapshot, Time now) {
    if (!terminal(result.status)) throw std::logic_error("execution outcome must be terminal");
    auto& record = current();
    record.result = std::move(result); record.snapshot = std::move(snapshot);
    record.elapsed_ms = elapsed(now);
    history_.push_back(std::move(record));
    active_.reset();
    if (history_.size() > capacity_) { history_.pop_front(); ++evicted_; }
  }
  std::optional<ExecutionRecord> get(const std::string& id, Time now) const {
    const auto* record = find(id);
    if (!record) return std::nullopt;
    auto copy = *record;
    if (active_ && record == &*active_) copy.elapsed_ms = elapsed(now);
    return copy;
  }
  std::string active_id() const { return active_ ? active_->id : ""; }
  std::vector<std::string> recent_ids() const {
    std::vector<std::string> result;
    for (auto it = history_.rbegin(); it != history_.rend(); ++it) result.push_back(it->id);
    return result;
  }
  size_t capacity() const { return capacity_; }
  uint64_t evicted_count() const { return evicted_; }
 private:
  ExecutionRecord& current() {
    if (!active_) throw std::logic_error("no active execution record");
    return *active_;
  }
  const ExecutionRecord* find(const std::string& id) const {
    if (active_ && (id.empty() || active_->id == id)) return &*active_;
    if (id.empty()) return history_.empty() ? nullptr : &history_.back();
    for (const auto& record : history_) if (record.id == id) return &record;
    return nullptr;
  }
  uint64_t elapsed(Time now) const {
    return static_cast<uint64_t>(std::max<int64_t>(0,
        std::chrono::duration_cast<std::chrono::milliseconds>(now-started_).count()));
  }
  size_t capacity_;
  uint64_t evicted_{0};
  Time started_;
  std::optional<ExecutionRecord> active_;
  std::deque<ExecutionRecord> history_;
};
}  // namespace robot_core
