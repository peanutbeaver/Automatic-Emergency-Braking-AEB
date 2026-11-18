// Processor-in-the-loop (PIL) run: the unchanged AEB controller and plant model
// execute on a bare-metal Cortex-M3. Every scenario prints the same "RESULT"
// line as the host SIL run, so the two outputs can be diffed (REQ-AEB-091).

#include "aeb/config.hpp"
#include "semihosting.hpp"
#include "sim/simulation.hpp"
#include "sim/text_buffer.hpp"

int main() {
    semihost::write("AEB processor-in-the-loop run (Cortex-M3, QEMU mps2-an385)\n");

    const aeb::Config config{};
    const sim::PlantParams plant{};
    int32_t passed = 0;

    for (size_t i = 0; i < sim::kScenarioCount; ++i) {
        const sim::Scenario& scenario = sim::kScenarios[i];
        const sim::Result result = sim::runScenario(scenario, config, plant);

        char line[160];
        sim::formatResult(line, sizeof line, scenario, result);
        semihost::write(line);
        semihost::write("\n");
        passed += result.passed ? 1 : 0;
    }

    char summary[64];
    sim::TextBuffer text(summary, sizeof summary);
    text.str("SUMMARY ")
        .integer(passed)
        .str("/")
        .integer(static_cast<int32_t>(sim::kScenarioCount))
        .str(" scenarios passed\n");
    semihost::write(summary);

    return passed == static_cast<int32_t>(sim::kScenarioCount) ? 0 : 1;
}
