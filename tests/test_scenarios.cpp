// Closed-loop regression tests: every scenario in the catalogue must meet its
// expectation (REQ-AEB-070..074, -050, -060).

#include <gtest/gtest.h>

#include <cstring>
#include <string>

#include "sim/simulation.hpp"
#include "sim/text_buffer.hpp"

namespace {

class ScenarioTest : public ::testing::TestWithParam<size_t> {};

TEST_P(ScenarioTest, MeetsExpectation) {
    const sim::Scenario& scenario = sim::kScenarios[GetParam()];
    const sim::Result r = sim::runScenario(scenario, aeb::Config{}, sim::PlantParams{});
    char line[160];
    sim::formatResult(line, sizeof line, scenario, r);
    EXPECT_TRUE(r.passed) << line;
}

INSTANTIATE_TEST_SUITE_P(Catalogue, ScenarioTest, ::testing::Range<size_t>(0, sim::kScenarioCount),
                         [](const ::testing::TestParamInfo<size_t>& info) {
                             std::string name = sim::kScenarios[info.param].name;
                             for (char& c : name) {
                                 if (c == '-') c = '_';
                             }
                             return name;
                         });

TEST(Simulation, IsDeterministic) {
    const sim::Scenario& scenario = sim::kScenarios[4];
    char first[160];
    char second[160];
    sim::formatResult(first, sizeof first, scenario,
                      sim::runScenario(scenario, aeb::Config{}, sim::PlantParams{}));
    sim::formatResult(second, sizeof second, scenario,
                      sim::runScenario(scenario, aeb::Config{}, sim::PlantParams{}));
    EXPECT_STREQ(first, second);
}

TEST(Simulation, WithoutAebEveryApproachCollides) {
    aeb::Config off{};
    off.minActiveSpeedMps = 1000.0f;  // never inside the activation window
    const sim::Result r = sim::runScenario(sim::kScenarios[4], off, sim::PlantParams{});
    EXPECT_TRUE(r.collision);
    EXPECT_NEAR(r.impactSpeedKph, sim::kScenarios[4].egoSpeedKph, 0.5f);
}

TEST(TextBuffer, FormatsNumbers) {
    char buf[64];
    sim::TextBuffer t(buf, sizeof buf);
    t.integer(-42).str(" ").fixed(3.14159f, 2).str(" ").fixed(-0.004f, 2).str(" ").fixed(9.995f, 1);
    EXPECT_STREQ(t.c_str(), "-42 3.14 0.00 10.0");
}

TEST(TextBuffer, TruncatesSafely) {
    char buf[6];
    sim::TextBuffer t(buf, sizeof buf);
    t.str("overflowing text");
    EXPECT_STREQ(t.c_str(), "overf");
    EXPECT_EQ(t.size(), 5U);
}

}  // namespace
