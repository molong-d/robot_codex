#include "robot_core/demo.hpp"
#include <iostream>

using namespace robot_core;
using namespace std::chrono_literals;

void check(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
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
                   bool fail_grasp = false)
      : bindings(components, {{"perception", "camera"}, {"motion", "arm"},
                              {"gripper", "gripper"}, {"safety", "gate"}}),
        context{bindings, world} {
    components.add("camera", std::make_shared<MockLocator>());
    components.add("arm", std::make_shared<MockArmMotion>(robot, arm_ticks, fail_motion));
    components.add("gripper", std::make_shared<MockGripper>(robot, 2, fail_grasp));
    components.add("gate", std::make_shared<MockExecutionGate>(permitted));
    register_demo_skills(skills);
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

int main() {
  int passed = 0;
  auto test = [&passed](const char* name, const std::function<void()>& run) {
    run(); ++passed; std::cout << "PASS " << name << '\n';
  };
  try {
    test("skill composes perception arm motion and gripper", [] {
      Fixture f; f.locate();
      check(f.run(f.request("pick", "pick_object")).status == Status::succeeded, "pick");
      check(f.world.attached_object == "workpiece" && f.robot->grasped, "grasp verified");
      f.locate("tray");
      check(f.run(f.request("place", "place_object",
                            {{"object", "workpiece"}, {"target", "tray"}})).status == Status::succeeded,
            "place");
      check(f.world.attached_object.empty() && !f.robot->grasped, "release verified");
      check(f.world.known_locations.at("workpiece") == "tray", "observed location");
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
                        {{{"motion", "arm_motion", 1}}, {"motion"}, ""});
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
