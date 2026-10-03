#include "robot_core/demo.hpp"
#include "robot_core/task_catalog.hpp"
#include "robot_core/plan.hpp"
#include "robot_core/execution_record.hpp"
#include <iostream>
#include <limits>

using namespace robot_core;
using namespace std::chrono_literals;

void check(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
void rejects(const std::function<void()>& run, const char* message) {
  try { run(); } catch (const std::exception&) { return; }
  throw std::runtime_error(message);
}

struct Fixture {
  Components components;
  Bindings bindings;
  WorldState world;
  Context context;
  Skills skills;
  Resources resources;
  std::shared_ptr<MockRobotState> robot{std::make_shared<MockRobotState>()};
  Time now{Clock::now()};
  explicit Fixture(int arm_ticks = 3, bool fail_motion = false, bool permitted = true,
                   bool fail_grasp = false, std::optional<DemoScene> scene = std::nullopt)
      : bindings(components, {{"perception", "camera"}, {"motion", "arm"},
                              {"gripper", "gripper"}, {"safety", "gate"}, {"outcome", "observer"}}),
        context{bindings, world} {
    if (scene) components.add("camera", std::make_shared<ConfiguredDemoLocator>(*scene));
    else components.add("camera", std::make_shared<MockLocator>());
    components.add("arm", std::make_shared<MockArmMotion>(robot, arm_ticks, fail_motion));
    components.add("gripper", std::make_shared<MockGripper>(robot, 2, fail_grasp));
    components.add("gate", std::make_shared<MockExecutionGate>(permitted));
    const auto outcome_scene = scene.value_or(DemoScene("base_link", {
        {"workpiece", EntityRole::object, {0.4, 0.1, 0.2, 0.0, 0.0, 0.0, 1.0}},
        {"tray", EntityRole::target, {0.6, -0.2, 0.15, 0.0, 0.0, 0.0, 1.0}}}));
    components.add("observer", std::make_shared<DemoOutcomeObserver>(bindings.get<ArmMotion>("motion"), bindings.get<Gripper>("gripper"), outcome_scene));
    ManipulationPolicy policy; policy.allow_synthetic = true;
    VerificationPolicy verification; verification.evidence.allow_synthetic = true;
    register_demo_skills(skills, policy, verification);
  }
  Request request(std::string id, std::string skill, Arguments args = {{"object", "workpiece"}}) {
    return {std::move(id), std::move(skill), "standard", std::move(args), 5000ms};
  }
  Result run(Request request) {
    Session session(skills, resources, context, std::move(request));
    auto result = session.start(now);
    for (int i = 0; i < 100 && !terminal(result.status); ++i) {
      now += 10ms;
      result = session.tick(now);
    }
    return result;
  }
  void locate(const std::string& id = "workpiece") {
    check(run(request("locate-" + id, "locate_object", {{"object", id}})).status == Status::succeeded,
          "locate");
  }
};

class ProbeSkill : public Skill {
 public:
  explicit ProbeSkill(bool throw_on_tick = false) : throws_(throw_on_tick) {}
  Result start(const Arguments&, Time) override { return {Status::running, "", ""}; }
  Result tick(Time) override {
    if (throws_) throw std::runtime_error("connection lost");
    return {Status::succeeded, "", ""};
  }
  void cancel() override {}
 private:
  bool throws_;
};

class WrongVersion final : public ObjectLocator {
 public:
  unsigned interface_version() const override { return 99; }
  std::string resource_id() const override { return "other_camera"; }
  std::optional<Observation> locate(const std::string&, Time) override { return std::nullopt; }
};

// A real action server may report success even after a cancel request was sent.
class ProbeMotion final : public ArmMotion {
 public:
  MotionFeedback measured;
  bool dispatched{false}, stop_requested{false}, throw_after_dispatch{false};
  Status status{Status::succeeded};
  std::string resource_id() const override { return "demo_arm"; }
  void begin_move(const CartesianTarget& target) override {
    dispatched = true;
    measured.frame_id = target.frame_id;
    measured.pose = target.pose;
    if (throw_after_dispatch) throw std::runtime_error("transport failed after dispatch");
  }
  Status poll(Time) override { return status; }
  void request_stop() override { stop_requested = true; }
  MotionFeedback feedback() const override { return measured; }
};

class ProbeObserver final : public ManipulationObserver {
 public:
  OutcomeEvidence value{"workpiece", "", {}, true, true, {"sensor", false, 0.95}, 1};
  std::string resource_id() const override { return "probe_sensor"; }
  std::optional<OutcomeEvidence> grasp(const std::string&, Time) override { return value; }
  std::optional<OutcomeEvidence> placement(const std::string&, const std::string&, Time) override { return value; }
};

Plan transfer_plan(const std::string& object = "workpiece", const std::string& target = "tray") {
  return {1, {{"locate_object", "standard", {{"object", object}}},
              {"pick_object", "standard", {{"object", object}}},
              {"locate_object", "standard", {{"object", target}}},
              {"place_object", "standard", {{"object", object}, {"target", target}}}}};
}
DemoScene plan_scene() {
  return DemoScene("base_link", {{"workpiece", EntityRole::object, {}},
                                  {"other", EntityRole::object, {}}, {"tray", EntityRole::target, {}}});
}
Plan verified_transfer_plan() {
  auto plan = transfer_plan();
  plan.steps.insert(plan.steps.begin()+2, {"verify_grasp", "standard", {{"object", "workpiece"}}});
  plan.steps.push_back({"verify_placement", "standard", {{"object", "workpiece"}, {"target", "tray"}}});
  return plan;
}

int main() {
  int passed = 0;
  auto test = [&passed](const char* name, const std::function<void()>& run) {
    run(); ++passed; std::cout << "PASS " << name << '\n';
  };
  try {
    test("evidence policy rejects missing source quality and synthetic data by default", [] {
      const auto now = Clock::now(); EvidenceMetadata value{"camera", false, 0.9}; EvidencePolicy policy;
      check(acceptable_evidence(value, now, now, policy), "accepted real evidence");
      for (double q : {-1.0, 0.2, 1.1, std::numeric_limits<double>::quiet_NaN()}) {
        value.quality = q; check(!acceptable_evidence(value, now, now, policy), "bad quality");
      }
      value.quality = 0.9; value.source.clear(); check(!acceptable_evidence(value, now, now, policy), "missing source");
      value.source = "camera"; value.synthetic = true;
      check(!acceptable_evidence(value, now, now, policy), "synthetic requires explicit enablement");
      policy.allow_synthetic = true; check(acceptable_evidence(value, now, now, policy), "explicit simulation");
      check(!acceptable_evidence(value, now+1ms, now, policy) && !acceptable_evidence(value, now-1s, now, policy), "future/stale");
    });
    test("locate validates source quality and pose meaning before storing", [] {
      Fixture f;
      auto observation = *f.bindings.get<ObjectLocator>("perception")->locate("workpiece", f.now);
      check(observation.meaning == PoseMeaning::motion_target && observation.evidence.synthetic, "demo target provenance");
      Locate real(f.context);
      check(real.start({{"object", "workpiece"}}, f.now).code == "INVALID_OBSERVATION", "default policy rejects demo source");
      check(f.world.observations.empty(), "no invalid evidence retained");
      f.locate(); f.world.observations["workpiece"].evidence.quality = 0.1;
      check(f.run(f.request("quality", "pick_object")).code == "STALE_OBSERVATION" && f.robot->arm_target.empty(), "low quality blocks dispatch");
    });
    test("native object pose cannot be sent directly to arm motion", [] {
      Fixture f; f.locate(); f.world.observations["workpiece"].meaning = PoseMeaning::object_pose;
      check(f.run(f.request("native", "pick_object")).code == "TARGET_RESOLUTION_REQUIRED", "explicit target transform required");
      check(f.robot->arm_target.empty() && f.resources.empty(), "no movement from object-center pose");
    });
    test("six-step verification promotes only accepted placement evidence", [] {
      auto scene = plan_scene(); Fixture f(3, false, true, false, scene); const auto plan = verified_transfer_plan();
      validate_plan(plan, scene, f.skills, f.bindings, f.world);
      ExecutionJournal journal; journal.begin("verified", "execute_plan", "", 6, 5000, f.now, 1);
      for (size_t i = 0; i < plan.steps.size(); ++i) {
        const auto& step = plan.steps[i]; auto request = f.request(std::to_string(i), step.skill, step.arguments);
        const auto result = f.run(request); check(result.status == Status::succeeded, "verified plan step");
        std::optional<OutcomeVerification> proof;
        if (step.skill == "verify_grasp") proof = f.world.grasp_verifications.at("workpiece");
        if (step.skill == "verify_placement") proof = f.world.placement_verifications.at("workpiece");
        journal.observe(request, result, f.now, proof);
      }
      journal.finish({Status::succeeded, "", ""}, {f.now, f.world, {}, true, false, true}, f.now);
      check(f.world.known_locations.at("workpiece") == "tray" && f.world.placement_candidates.empty(), "placement promoted after sensor evidence");
      const auto record = journal.get("verified", f.now);
      check(record->steps[2].verification && record->steps[5].verification &&
            record->steps[2].verification->evidence.evidence.synthetic, "historical grasp/placement provenance retained");
      check(f.world.grasp_verifications.empty(), "grasp proof invalidated before placing");
      const auto& proof = f.world.placement_verifications.at("workpiece");
      check(proof.samples >= 3 && proof.stable_for >= 100ms && proof.evidence.evidence.synthetic, "distinct sample window and provenance");
      f.locate(); f.run(f.request("pick_again", "pick_object"));
      check(f.world.placement_verifications.empty(), "new manipulation invalidates old placement proof");
    });
    test("plan admission validates verification ordering without predicted facts", [] {
      Fixture f; auto scene = plan_scene(); auto plan = verified_transfer_plan();
      validate_plan(plan, scene, f.skills, f.bindings, f.world);
      check(f.world.placement_verifications.empty() && f.world.grasp_verifications.empty(), "admission creates no proof");
      std::swap(plan.steps[1], plan.steps[2]);
      rejects([&] { validate_plan(plan, scene, f.skills, f.bindings, f.world); }, "verification before grasp");
      plan = verified_transfer_plan(); std::swap(plan.steps[4], plan.steps[5]);
      rejects([&] { validate_plan(plan, scene, f.skills, f.bindings, f.world); }, "verification before release");
      plan = verified_transfer_plan(); plan.steps.push_back(plan.steps.back());
      rejects([&] { validate_plan(plan, scene, f.skills, f.bindings, f.world); }, "cannot consume release candidate twice");
    });
    test("verification requires post-effect samples and does not count cached data", [] {
      Fixture f; f.locate(); f.run(f.request("pick", "pick_object"));
      auto observer = std::make_shared<ProbeObserver>(); f.components.add("probe", observer);
      Bindings b(f.components, {{"outcome", "probe"}, {"motion", "arm"}, {"gripper", "gripper"}}); Context c{b, f.world};
      observer->value.stamp = f.world.attachment_stamp;
      VerifyOutcome verify(c, true);
      check(verify.start({{"object", "workpiece"}}, f.now).code == "PRE_EFFECT_EVIDENCE", "completion sample insufficient");
      observer->value.stamp = f.now+1ms;
      check(verify.tick(f.now+1ms).status == Status::running, "first post-effect frame");
      observer->value.stamp = f.now+150ms;  // polling cannot turn an unchanged sample ID into a new frame
      check(verify.tick(f.now+150ms).status == Status::running, "same sample ID not counted");
      observer->value.sample_id = 2;
      check(verify.tick(f.now+150ms).status == Status::running, "only two actual frames");
      observer->value.sample_id = 3; observer->value.stamp = f.now+160ms;
      check(verify.tick(f.now+160ms).status == Status::succeeded, "three distinct post-effect samples");
      check(f.world.grasp_verifications.at("workpiece").samples == 3, "cache polling excluded");
    });
    test("verification rejects malformed negative and misidentified evidence", [] {
      for (int mode = 0; mode < 6; ++mode) {
        Fixture f; f.locate(); f.run(f.request("pick", "pick_object"));
        auto observer = std::make_shared<ProbeObserver>(); observer->value.stamp = f.now+1ms;
        if (mode == 0) observer->value.evidence.quality = 0.2;
        if (mode == 1) observer->value.object_id = "other";
        if (mode == 2) observer->value.condition_met = false;
        if (mode == 3) observer->value.sample_id = 0;
        if (mode == 4) observer->value.stamp = f.now+1s;
        if (mode == 5) observer->value.evidence.synthetic = true;
        f.components.add("probe", observer); Bindings b(f.components, {{"outcome", "probe"}}); Context c{b, f.world};
        VerifyOutcome verify(c, true);
        check(verify.start({{"object", "workpiece"}}, f.now+1ms).status == Status::failed, "invalid proof rejected");
        check(f.world.grasp_verifications.empty(), "no false verified state");
      }
    });
    test("verification rejects source changes regressions and large sample gaps", [] {
      for (int mode = 0; mode < 3; ++mode) {
        Fixture f; f.locate(); f.run(f.request("pick", "pick_object"));
        auto observer = std::make_shared<ProbeObserver>(); observer->value.stamp = f.now+10ms; observer->value.sample_id = 2;
        f.components.add("probe", observer); Bindings b(f.components, {{"outcome", "probe"}}); Context c{b, f.world};
        VerifyOutcome verify(c, true); check(verify.start({{"object", "workpiece"}}, f.now+10ms).status == Status::running, "first frame");
        observer->value.sample_id = mode == 1 ? 1 : 3; observer->value.stamp = f.now+(mode == 2 ? 600ms : 20ms);
        if (mode == 0) observer->value.evidence.source = "another_camera";
        check(verify.tick(observer->value.stamp).status == Status::failed, "discontinuous proof rejected");
        check(f.world.grasp_verifications.empty(), "no continuous stability claim");
      }
    });
    test("failed placement evidence leaves candidate and does not promote location", [] {
      Fixture f; f.locate(); f.run(f.request("pick", "pick_object")); f.locate("tray");
      f.run(f.request("place", "place_object", {{"object", "workpiece"}, {"target", "tray"}}));
      auto observer = std::make_shared<ProbeObserver>(); observer->value.target_id = "tray";
      observer->value.stamp = f.now+1ms; observer->value.condition_met = false;
      f.components.add("probe", observer); Bindings b(f.components, {{"outcome", "probe"}}); Context c{b, f.world};
      VerifyOutcome verify(c, false);
      check(verify.start({{"object", "workpiece"}, {"target", "tray"}}, f.now+1ms).code == "PLACEMENT_NOT_VERIFIED", "failed placement");
      check(f.world.placement_candidates.at("workpiece") == "tray" && f.world.known_locations.empty() &&
            f.world.placement_verifications.empty(), "inference remains inference");
    });
    test("read-only verification cancellation releases leases without manipulating", [] {
      Fixture f; f.locate(); f.run(f.request("pick", "pick_object"));
      Session verify(f.skills, f.resources, f.context, f.request("verify", "verify_grasp"));
      check(verify.start(f.now).status == Status::running && !f.resources.empty(), "verification claims read access exclusively");
      verify.cancel(); check(verify.tick(f.now+1ms).status == Status::canceled && f.resources.empty(), "read-only verification stops promptly");
      check(f.world.attached_object == "workpiece" && f.robot->grasped && f.world.grasp_verifications.empty(), "cancel is not release or proof");
    });
    test("execution journal keeps terminal evidence and snapshot copies", [] {
      Fixture f; ExecutionJournal journal;
      check(!journal.get("", f.now), "initially no records");
      journal.begin("goal", "execute_plan", "", 1, 5000, f.now, 123456);
      auto request = f.request("session", "locate_object");
      journal.observe(request, {Status::running, "", "first"}, f.now);
      journal.observe(request, {Status::running, "", "updated"}, f.now+10ms);
      journal.observe(request, {Status::succeeded, "", "verified"}, f.now+20ms);
      journal.observe(request, {Status::running, "", "regression"}, f.now+30ms);
      f.world.placement_candidates["workpiece"] = "tray";
      journal.finish({Status::succeeded, "", "done"},
          {f.now+20ms, f.world, {}, true, false, true}, f.now+20ms);
      f.world.placement_candidates.clear();
      auto record = journal.get("goal", f.now+100ms);
      check(record && record->elapsed_ms == 20 && record->started_unix_ms == 123456, "frozen duration");
      check(record->steps.size() == 1 && record->steps[0].transitions.size() == 2 &&
            record->steps[0].result.status == Status::succeeded, "status transitions, immutable completion");
      check(record->snapshot.world.placement_candidates.at("workpiece") == "tray" &&
            record->snapshot.world.known_locations.empty(), "copy inferred facts without promoting them");
      record->snapshot.world.placement_candidates.clear();
      check(!journal.get("goal", f.now)->snapshot.world.placement_candidates.empty(), "query copies cannot change history");
    });
    test("execution history evicts oldest completion and preserves active task", [] {
      ExecutionJournal journal(2); const auto now = Clock::now();
      for (int i = 0; i < 3; ++i) {
        journal.begin(std::to_string(i), "execute_task", "inspect_object", 1, 5000, now, 1);
        journal.finish({Status::failed, "TEST", ""}, {}, now);
      }
      check(journal.evicted_count() == 1 && journal.recent_ids() == std::vector<std::string>{"2", "1"}, "bounded newest-first history");
      check(!journal.get("0", now) && !journal.get("missing", now) && journal.get("", now)->id == "2", "missing has no fallback");
      journal.begin("active", "execute_plan", "", 1, 5000, now, 1);
      check(journal.get("", now+50ms)->elapsed_ms == 50 && journal.active_id() == "active", "active lookup");
      check(journal.get("2", now)->result.status == Status::failed, "history remains queryable during next task");
    });
    test("execution recording rejects invalid bounds and duplicate starts", [] {
      const auto now = Clock::now();
      rejects([] { ExecutionJournal invalid(0); }, "zero capacity");
      rejects([] { ExecutionJournal invalid(129); }, "excess capacity");
      ExecutionJournal journal;
      rejects([&] { journal.begin("g", "execute_plan", "", 33, 1, now, 1); }, "excess steps");
      journal.begin("g", "execute_plan", "", 1, 1, now, 1);
      rejects([&] { journal.begin("h", "execute_plan", "", 1, 1, now, 1); }, "second active execution");
      Request request{"1", "locate_object", "standard", {{"object", "tray"}}, 1ms};
      journal.observe(request, {Status::succeeded, "", ""}, now);
      request.id = "2";
      rejects([&] { journal.observe(request, {}, now); }, "excess started steps");
      journal.finish({Status::succeeded, "", ""}, {}, now);
      rejects([&] { journal.begin("g", "execute_plan", "", 1, 1, now, 1); }, "duplicate retained UUID");
    });
    test("fault diagnostic snapshot retains unknown resource ownership", [] {
      Fixture f; ExecutionJournal journal;
      f.skills.implement("pick_object", "broken", [](Context&) { return std::make_unique<ProbeSkill>(true); },
                        {{{"motion", "arm_motion", 2}}, {"motion"}, ""});
      auto request = f.request("broken", "pick_object"); request.implementation = "broken";
      journal.begin("fault", "execute_plan", "", 1, 5000, f.now, 1);
      Session session(f.skills, f.resources, f.context, request);
      journal.observe(session.request(), session.start(f.now), f.now);
      const auto result = session.tick(f.now+10ms);
      journal.observe(session.request(), result, f.now+10ms);
      journal.finish(result, {f.now+10ms, f.world, f.resources.owners(), f.resources.empty(), true, false}, f.now+10ms);
      auto copy = f.resources.owners(); copy.clear();
      const auto record = journal.get("fault", f.now);
      check(record->result.status == Status::faulted && !record->snapshot.stop_confirmed &&
            record->snapshot.fault_latched && record->snapshot.resource_owners.at("demo_arm") == "broken", "fault is not stop");
      check(!f.resources.empty() && !f.resources.acquire("other", {"demo_arm"}), "diagnostic reads leave ownership intact");
    });
    test("cancel recording distinguishes stop request from confirmation", [] {
      Fixture f; f.locate(); ExecutionJournal journal;
      journal.begin("cancel", "execute_task", "pick_place", 4, 5000, f.now, 1);
      Session session(f.skills, f.resources, f.context, f.request("pick", "pick_object"));
      journal.observe(session.request(), session.start(f.now), f.now);
      session.cancel(); journal.observe(session.request(), session.result(), f.now+1ms);
      journal.update({Status::canceling, "CANCEL_REQUESTED", ""},
          {f.now+1ms, f.world, f.resources.owners(), f.resources.empty(), false, false}, f.now+1ms);
      check(!journal.get("", f.now)->snapshot.stop_confirmed && !f.resources.empty(), "request is not confirmation");
      session.tick(f.now+2ms); const auto result = session.tick(f.now+3ms);
      journal.observe(session.request(), result, f.now+3ms);
      journal.finish(result, {f.now+3ms, f.world, f.resources.owners(), f.resources.empty(), false, true}, f.now+3ms);
      const auto record = journal.get("cancel", f.now);
      check(record->steps[0].transitions.size() == 3 && record->result.status == Status::canceled &&
            record->snapshot.stop_confirmed && record->snapshot.resources_empty, "confirmed cancel recorded");
    });
    test("plan validation never writes predicted effects or dispatches", [] {
      Fixture f; auto scene = plan_scene();
      validate_plan(transfer_plan(), scene, f.skills, f.bindings, f.world);
      check(f.world.attached_object.empty() && f.world.observations.empty() &&
            f.world.placement_candidates.empty() && f.resources.empty() && f.robot->arm_target.empty(), "validation is read-only");
    });
    test("late invalid plan step blocks the entire plan before execution", [] {
      Fixture f; auto scene = plan_scene(); auto plan = transfer_plan();
      plan.steps.back().arguments["target"] = "unknown";
      rejects([&] { validate_plan(plan, scene, f.skills, f.bindings, f.world); }, "unknown final target");
      plan = transfer_plan(); plan.steps.back().implementation = "missing";
      rejects([&] { validate_plan(plan, scene, f.skills, f.bindings, f.world); }, "unknown final implementation");
      plan = transfer_plan(); plan.steps.back().arguments["extra"] = "injected";
      rejects([&] { validate_plan(plan, scene, f.skills, f.bindings, f.world); }, "unexpected final input");
      check(!f.robot->grasped && f.robot->arm_target.empty() && f.resources.empty(), "no partial dispatch");
    });
    test("plan checks locate grasp release ordering and matching identities", [] {
      Fixture f; auto scene = plan_scene(); auto plan = transfer_plan();
      std::swap(plan.steps[0], plan.steps[1]);
      rejects([&] { validate_plan(plan, scene, f.skills, f.bindings, f.world); }, "pick before locate");
      plan = transfer_plan(); plan.steps.erase(plan.steps.begin()+2);
      rejects([&] { validate_plan(plan, scene, f.skills, f.bindings, f.world); }, "place without target locate");
      plan = transfer_plan(); plan.steps.back().arguments["object"] = "other";
      rejects([&] { validate_plan(plan, scene, f.skills, f.bindings, f.world); }, "wrong held object");
      plan = transfer_plan(); plan.steps[1].arguments["object"] = "tray";
      rejects([&] { validate_plan(plan, scene, f.skills, f.bindings, f.world); }, "cannot grasp target entity");
    });
    test("plan bounds versions and XML-like inputs are rejected", [] {
      Fixture f; auto scene = plan_scene(); auto plan = transfer_plan();
      plan.schema_version = 2;
      rejects([&] { validate_plan(plan, scene, f.skills, f.bindings, f.world); }, "version");
      plan = {1, {}}; rejects([&] { validate_plan(plan, scene, f.skills, f.bindings, f.world); }, "empty");
      plan.steps.assign(33, transfer_plan().steps.front());
      rejects([&] { validate_plan(plan, scene, f.skills, f.bindings, f.world); }, "too many steps");
      plan.steps.resize(32); validate_plan(plan, scene, f.skills, f.bindings, f.world);
      plan = transfer_plan(); plan.steps[0].arguments["object"] = "{world}";
      rejects([&] { validate_plan(plan, scene, f.skills, f.bindings, f.world); }, "blackboard injection");
      plan = transfer_plan(); plan.steps[0].implementation = "standard\"/>";
      rejects([&] { validate_plan(plan, scene, f.skills, f.bindings, f.world); }, "XML injection");
    });
    test("plan uses actual held state and requires new within-plan observations", [] {
      Fixture f; auto scene = plan_scene(); f.world.attached_object = "workpiece";
      rejects([&] { validate_plan(transfer_plan(), scene, f.skills, f.bindings, f.world); }, "occupied gripper");
      auto place = transfer_plan(); place.steps.erase(place.steps.begin(), place.steps.begin()+2);
      validate_plan(place, scene, f.skills, f.bindings, f.world);
      check(f.world.attached_object == "workpiece", "symbolic release not written");
      f.world.attached_object.clear(); f.locate();
      auto pick = transfer_plan(); pick.steps = {pick.steps[1]};
      rejects([&] { validate_plan(pick, scene, f.skills, f.bindings, f.world); }, "previous observation does not replace planned locate");
    });
    test("plan permits implementation-specific dependencies without constructing skills", [] {
      Fixture f; auto scene = plan_scene(); bool constructed = false;
      f.skills.implement("locate_object", "read_only", [&](Context&) { constructed = true; return std::make_unique<ProbeSkill>(); }, {});
      Plan plan{1, {{"locate_object", "read_only", {{"object", "tray"}}}}};
      validate_plan(plan, scene, f.skills, f.bindings, f.world);
      check(!constructed, "validator does not construct implementation");
    });
    test("validated plan steps still execute through verified skill sessions", [] {
      Fixture f; auto scene = plan_scene(); const auto plan = transfer_plan();
      validate_plan(plan, scene, f.skills, f.bindings, f.world);
      for (size_t i = 0; i < plan.steps.size(); ++i) {
        const auto& step = plan.steps[i];
        check(f.run({"plan_"+std::to_string(i), step.skill, step.implementation, step.arguments, 5000ms}).status == Status::succeeded, "step verified");
      }
      check(f.world.attached_object.empty() && f.world.placement_candidates.at("workpiece") == "tray" && f.resources.empty(), "verified sessions update state");
    });
    test("entity input schema rejects malformed identifiers before dispatch", [] {
      Fixture f;
      for (const auto& id : {"", "object.with.dot", "../../workpiece", "<Skill/>", "1workpiece"}) {
        check(f.run(f.request("invalid", "pick_object", {{"object", id}})).code == "INVALID_REQUEST", "invalid entity ID");
        check(f.resources.empty() && f.robot->arm_target.empty(), "invalid input never dispatches");
      }
      Skills schemas;
      rejects([&] { schemas.define({"bad", "", {{"object", "float", ""}}, "", "", ""}); }, "unsupported input type");
      rejects([&] { schemas.define({"bad", "", {{"object", "entity_id", ""}, {"object", "entity_id", ""}}, "", "", ""}); }, "duplicate input");
    });
    test("demo scene rejects duplicate IDs and invalid poses", [] {
      rejects([] { DemoScene scene("base_link", {{"part", EntityRole::object, {}}, {"part", EntityRole::target, {}}}); }, "duplicate entity");
      rejects([] { DemoScene scene("", {{"part", EntityRole::object, {}}}); }, "empty frame");
      rejects([] { DemoScene scene("base_link", {}); }, "empty scene");
      Pose invalid; invalid.qw = 0.0;
      rejects([&] { DemoScene scene("base_link", {{"part", EntityRole::object, invalid}}); }, "invalid quaternion");
      invalid = {}; invalid.x = std::numeric_limits<double>::infinity();
      rejects([&] { DemoScene scene("base_link", {{"part", EntityRole::object, invalid}}); }, "infinite coordinate");
    });
    test("catalog reports implementation bindings without executing factories or gates", [] {
      Fixture f(3, false, false);
      bool constructed = false;
      f.skills.implement("pick_object", "unbound", [&](Context&) {
        constructed = true; return std::make_unique<ProbeSkill>();
      }, {{{"depth", "object_locator", 1}}, {}, ""});
      const auto catalog = f.skills.catalog(f.bindings);
      check(catalog.size() == 5, "all definitions visible");
      const auto& pick = catalog.at(1);
      check(pick.definition.id == "pick_object" && pick.definition.inputs.at(0).type == "entity_id", "typed inputs");
      check(pick.implementations.size() == 2, "alternative implementations visible");
      check(pick.implementations.at(0).dependencies_satisfied, "bound dependencies despite closed gate");
      check(pick.implementations.at(0).dependencies.components.at(0).version == 2, "component version");
      check(!pick.implementations.at(1).dependencies_satisfied && !pick.implementations.at(1).unavailable_reason.empty(), "unbound implementation explained");
      check(!constructed && f.resources.empty() && f.robot->arm_target.empty(), "read-only catalog");
    });
    test("task admission checks every step before any actuator or factory", [] {
      Fixture f;
      DemoScene scene("base_link", {{"part", EntityRole::object, {}}, {"bin", EntityRole::target, {}}});
      bool constructed = false;
      for (const auto& id : {"locate_object", "pick_object", "place_object"})
        f.skills.implement(id, "alternate", [&](Context&) { constructed = true; return std::make_unique<ProbeSkill>(); },
            id == std::string("place_object") ? Dependencies{{{"missing_gripper", "gripper", 2}}, {}, ""} : Dependencies{});
      TaskCatalog tasks({{"transfer", "pick_place", "alternate"}});
      rejects([&] { tasks.admit("transfer", "part", "bin", scene, f.skills, f.bindings); }, "late step missing dependency");
      rejects([&] { tasks.validate_configuration(scene, f.skills, f.bindings); }, "invalid startup binding");
      check(!constructed && f.resources.empty() && f.robot->arm_target.empty(), "no partial execution");
    });
    test("task aliases validate roles and only select reviewed templates", [] {
      Fixture f;
      DemoScene scene("base_link", {{"part", EntityRole::object, {}}, {"bin", EntityRole::target, {}}});
      TaskCatalog tasks({{"transfer", "pick_place", "standard"}, {"inspect", "locate_object", "standard"}});
      tasks.validate_configuration(scene, f.skills, f.bindings);
      check(tasks.admit("transfer", "part", "bin", scene, f.skills, f.bindings).template_id == "pick_place", "alias");
      check(tasks.admit("inspect", "bin", "", scene, f.skills, f.bindings).template_id == "locate_object", "locate target entity");
      rejects([&] { tasks.admit("transfer", "bin", "part", scene, f.skills, f.bindings); }, "wrong roles");
      rejects([&] { tasks.admit("transfer", "part", "unknown", scene, f.skills, f.bindings); }, "unknown target");
      rejects([&] { tasks.admit("transfer", "unknown", "bin", scene, f.skills, f.bindings); }, "unknown object");
      rejects([&] { tasks.admit("inspect", "part", "bin", scene, f.skills, f.bindings); }, "unexpected target");
      rejects([&] { tasks.admit("unknown", "part", "bin", scene, f.skills, f.bindings); }, "unknown task");
      rejects([] { TaskCatalog invalid({{"task", "../external.xml", "standard"}}); }, "external template");
      rejects([] { TaskCatalog invalid({{"task", "pick_place", "standard"}, {"task", "locate_object", "standard"}}); }, "duplicate task");
      TaskCatalog unknown_impl({{"task", "pick_place", "missing"}});
      rejects([&] { unknown_impl.validate_configuration(scene, f.skills, f.bindings); }, "unknown implementation");
    });
    test("second configured object and target reuse complete manipulation flow", [] {
      const Pose part{0.3, -0.1, 0.2, 0.0, 0.0, 0.0, 1.0};
      const Pose bin{0.5, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0};
      DemoScene scene("base_link", {{"part_two", EntityRole::object, part}, {"bin_two", EntityRole::target, bin}});
      Fixture f(3, false, true, false, scene);
      TaskCatalog tasks({{"transfer_two", "pick_place", "standard"}});
      tasks.admit("transfer_two", "part_two", "bin_two", scene, f.skills, f.bindings);
      f.locate("part_two");
      check(f.run(f.request("pick_two", "pick_object", {{"object", "part_two"}})).status == Status::succeeded, "pick second object");
      check(pose_near(f.robot->arm_pose, part, {}), "configured object pose used");
      f.locate("bin_two");
      check(f.run(f.request("place_two", "place_object", {{"object", "part_two"}, {"target", "bin_two"}})).status == Status::succeeded, "place second object");
      check(pose_near(f.robot->arm_pose, bin, {}) && f.world.placement_candidates.at("part_two") == "bin_two", "configured target used");
      check(f.resources.empty() && f.world.attached_object.empty(), "leases and held state cleared");
    });
    test("pose validation rejects NaN and invalid quaternion", [] {
      Pose p;
      check(valid_pose(p), "identity quaternion");
      p.x = std::numeric_limits<double>::quiet_NaN();
      check(!valid_pose(p), "NaN");
      p = {}; p.qw = 0.0; check(!valid_pose(p), "zero quaternion");
      p.qw = 2.0; check(!valid_pose(p), "nonunit quaternion");
      p = {}; auto q = p; q.qw = -1.0;
      check(pose_near(p, q, {}), "quaternion sign equivalence");
      q.x = 0.1; check(!pose_near(p, q, {}), "position tolerance");
      check(!fresh(Time{}, Clock::now(), 500ms), "unstamped data");
    });
    test("cancel winning completion race never dispatches gripper", [] {
      Fixture f; f.locate();
      auto arm = std::make_shared<ProbeMotion>();
      f.components.add("race", arm);
      Bindings b(f.components, {{"motion", "race"}, {"gripper", "gripper"}, {"safety", "gate"}});
      Context c{b, f.world};
      Session s(f.skills, f.resources, c, f.request("race", "pick_object"));
      s.start(f.now);
      arm->measured.stamp = f.now; arm->measured.valid = true; arm->measured.stopped = true;
      s.cancel();
      check(s.tick(f.now).status == Status::canceled, "race canceled");
      check(arm->stop_requested && !f.robot->grasped, "no grasp dispatched");
      check(f.bindings.get<Gripper>("gripper")->poll(f.now) == Status::idle, "gripper still idle");
    });
    test("terminal result needs fresh stationary feedback to release lease", [] {
      Fixture f; f.locate();
      auto arm = std::make_shared<ProbeMotion>(); f.components.add("probe", arm);
      Bindings b(f.components, {{"motion", "probe"}, {"gripper", "gripper"}, {"safety", "gate"}});
      Context c{b, f.world};
      Session s(f.skills, f.resources, c, f.request("probe", "pick_object")); s.start(f.now);
      arm->measured = {"base_link", {}, f.now-2s, true, true};
      s.cancel(); check(s.tick(f.now).status == Status::canceling && !f.resources.empty(), "stale feedback retains lease");
      arm->measured.stamp = f.now; arm->measured.stopped = false;
      check(s.tick(f.now).status == Status::canceling && !f.resources.empty(), "moving feedback retains lease");
      arm->measured.stopped = true; arm->measured.stamp = f.now-1ms;
      check(s.tick(f.now).status == Status::canceling && !f.resources.empty(), "pre-result stop sample retains lease");
      arm->measured.stamp = f.now;
      arm->measured.stopped = true;
      check(s.tick(f.now).status == Status::canceled && f.resources.empty(), "measured stop releases lease");
    });
    test("outside-tolerance pose blocks grasp after successful action", [] {
      Fixture f; f.locate();
      auto arm = std::make_shared<ProbeMotion>(); f.components.add("probe", arm);
      Bindings b(f.components, {{"motion", "probe"}, {"gripper", "gripper"}, {"safety", "gate"}});
      Context c{b, f.world};
      Session s(f.skills, f.resources, c, f.request("probe", "pick_object")); s.start(f.now);
      arm->measured.stamp = f.now; arm->measured.valid = true; arm->measured.stopped = true;
      arm->measured.pose.x += 0.1;
      check(s.tick(f.now).code == "MOTION_VERIFICATION_FAILED", "pose mismatch");
      check(!f.robot->grasped, "no grasp");
    });
    test("dispatch exception still requests stop and keeps ownership", [] {
      Fixture f; f.locate();
      auto arm = std::make_shared<ProbeMotion>(); arm->throw_after_dispatch = true;
      f.components.add("probe", arm);
      Bindings b(f.components, {{"motion", "probe"}, {"gripper", "gripper"}, {"safety", "gate"}});
      Context c{b, f.world};
      Session s(f.skills, f.resources, c, f.request("probe", "pick_object"));
      check(s.start(f.now).status == Status::faulted, "fault after dispatch");
      check(arm->stop_requested && !f.resources.empty(), "phase set before dispatch");
    });
    test("skill composes perception arm motion and gripper", [] {
      Fixture f; f.locate();
      check(f.run(f.request("pick", "pick_object")).status == Status::succeeded, "pick");
      check(f.world.attached_object == "workpiece" && f.robot->grasped, "grasp verified");
      f.locate("tray");
      check(f.run(f.request("place", "place_object",
                            {{"object", "workpiece"}, {"target", "tray"}})).status == Status::succeeded,
            "place");
      check(f.world.attached_object.empty() && !f.robot->grasped, "release verified");
      check(f.world.placement_candidates.at("workpiece") == "tray", "inferred placement");
      check(f.world.known_locations.count("workpiece") == 0, "release is not a perception observation");
      check(f.resources.empty(), "leases released");
    });
    test("motion implementation can change without skill changes", [] {
      Fixture f(7); f.locate();
      check(f.run(f.request("pick", "pick_object")).status == Status::succeeded, "slow arm backend");
    });
    test("stale and future pose observations are rejected", [] {
      Fixture f; f.locate(); f.now += 3s;
      check(f.run(f.request("old", "pick_object")).code == "STALE_OBSERVATION", "stale");
      f.world.observations["workpiece"].stamp = f.now + 1s;
      check(f.run(f.request("future", "pick_object")).code == "STALE_OBSERVATION", "future");
      check(f.resources.empty(), "precondition failure releases leases");
    });
    test("place requires a fresh target pose", [] {
      Fixture f; f.locate();
      check(f.run(f.request("pick", "pick_object")).status == Status::succeeded, "pick");
      const auto result = f.run(f.request("place", "place_object",
          {{"object", "workpiece"}, {"target", "tray"}}));
      check(result.code == "STALE_OBSERVATION", "missing target observation");
    });
    test("execution gate blocks all actuators before resource claim", [] {
      Fixture f(3, false, false); f.locate();
      const auto result = f.run(f.request("blocked", "pick_object"));
      check(result.code == "SAFETY_INTERLOCK", "gate must reject");
      check(f.resources.empty() && !f.robot->grasped && f.robot->arm_target.empty(), "no dispatch");
    });
    test("invalid arguments fail before dispatch", [] {
      Fixture f;
      auto r = f.request("bad", "pick_object", {{"object", "workpiece"}, {"extra", "x"}});
      check(f.run(r).code == "INVALID_REQUEST", "extra input");
      r.arguments.clear(); check(f.run(r).code == "INVALID_REQUEST", "missing input");
    });
    test("unbound wrong type and wrong version are rejected", [] {
      Fixture f;
      for (const auto& roles : {std::map<std::string, std::string>{},
                               std::map<std::string, std::string>{{"perception", "arm"}}}) {
        Bindings b(f.components, roles); Context c{b, f.world};
        Session s(f.skills, f.resources, c, f.request("bad", "locate_object"));
        check(s.start(f.now).code == "INVALID_REQUEST", "binding must reject");
      }
      f.components.add("v99", std::make_shared<WrongVersion>());
      Bindings b(f.components, {{"perception", "v99"}}); Context c{b, f.world};
      Session s(f.skills, f.resources, c, f.request("v", "locate_object"));
      check(s.start(f.now).code == "INVALID_REQUEST", "version must reject");
    });
    test("unknown implementation fails without resource leak", [] {
      Fixture f; auto r = f.request("unknown", "pick_object"); r.implementation = "missing";
      check(f.run(r).code == "INVALID_REQUEST", "unknown implementation");
      check(f.resources.empty(), "no leak");
    });
    test("resource contention is atomic and owner IDs are unique", [] {
      Resources r;
      check(r.acquire("a", {"arm"}), "first claim");
      check(!r.acquire("b", {"camera", "arm"}), "conflict");
      check(r.acquire("c", {"camera"}), "no partial claim");
      check(!r.acquire("a", {"other"}), "duplicate active owner");
      r.release("a"); r.release("c"); check(r.empty(), "release");
    });
    test("arm and gripper leases span the complete skill", [] {
      Fixture f; f.locate();
      Session s(f.skills, f.resources, f.context, f.request("pick", "pick_object"));
      check(s.start(f.now).status == Status::running, "start");
      for (int i = 0; i < 3; ++i) s.tick(f.now += 10ms);
      check(!f.resources.acquire("other-arm", {"demo_arm"}), "arm remains reserved");
      check(!f.resources.acquire("other-gripper", {"demo_gripper"}), "gripper remains reserved");
      check(s.result().status == Status::running, "gripper phase running");
    });
    test("cancel holds both leases until active component confirms stop", [] {
      Fixture f; f.locate();
      Session s(f.skills, f.resources, f.context, f.request("pick", "pick_object"));
      check(s.start(f.now).status == Status::running, "start");
      s.cancel(); check(!f.resources.empty(), "cancel is not stop");
      check(s.tick(f.now).status == Status::canceling, "waiting for stop");
      check(s.tick(f.now).status == Status::canceled, "stop confirmed");
      check(f.resources.empty() && !f.robot->grasped, "safe release");
    });
    test("timeout uses confirmed cancellation path", [] {
      Fixture f; f.locate(); auto r = f.request("pick", "pick_object"); r.timeout = 1ms;
      Session s(f.skills, f.resources, f.context, r); s.start(f.now);
      check(s.tick(f.now + 2ms).status == Status::canceling, "timeout starts stop");
      check(s.tick(f.now + 3ms).status == Status::timed_out, "timeout result");
      check(f.resources.empty(), "timeout lease release");
    });
    test("start is idempotent across a multi-component skill", [] {
      Fixture f; f.locate();
      Session s(f.skills, f.resources, f.context, f.request("pick", "pick_object"));
      s.start(f.now); s.start(f.now);
      for (int i = 0; i < 10 && !terminal(s.result().status); ++i) s.tick(f.now += 10ms);
      check(s.result().status == Status::succeeded, "must not reset component progress");
    });
    test("motion and gripper failures remain distinct", [] {
      Fixture motion(2, true); motion.locate();
      check(motion.run(motion.request("motion", "pick_object")).code == "MOTION_FAILED", "motion failure");
      Fixture grip(2, false, true, true); grip.locate();
      check(grip.run(grip.request("grip", "pick_object")).code == "GRIPPER_FAILED", "gripper failure");
    });
    test("exceptions latch unknown state and retain control ownership", [] {
      Fixture f;
      f.skills.implement("pick_object", "broken", [](Context&) { return std::make_unique<ProbeSkill>(true); },
                        {{{"motion", "arm_motion", 2}}, {"motion"}, ""});
      auto r = f.request("broken", "pick_object"); r.implementation = "broken";
      Session s(f.skills, f.resources, f.context, r); s.start(f.now);
      check(s.tick(f.now).status == Status::faulted, "fault latched");
      check(!f.resources.empty(), "unknown actuator state keeps lease");
    });
    test("destruction does not falsely confirm a stop", [] {
      Fixture f; f.locate();
      { Session s(f.skills, f.resources, f.context, f.request("pick", "pick_object")); s.start(f.now); }
      check(!f.resources.empty(), "unconfirmed stop retains leases");
    });
    test("skill implementation selection is independent of definition", [] {
      Fixture f;
      f.skills.implement("locate_object", "probe", [](Context&) { return std::make_unique<ProbeSkill>(); }, {});
      auto r = f.request("probe", "locate_object"); r.implementation = "probe";
      check(f.run(r).status == Status::succeeded, "second implementation selected");
      check(f.skills.dependencies("locate_object", "probe").components.empty(), "implementation dependencies");
    });
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n'; return 1;
  }
  std::cout << passed << " tests passed\n";
}
