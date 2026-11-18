// Software-in-the-loop (SIL) runner: executes every scenario on the host,
// prints a result table and optionally writes per-scenario CSV time series.
//
//   aeb_sil                 run all scenarios
//   aeb_sil --csv out/      also write out/<scenario>.csv for plotting

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>

#include "aeb/config.hpp"
#include "sim/simulation.hpp"

namespace {

void writeCsvRow(const sim::Sample& s, void* context) {
    auto* file = static_cast<std::FILE*>(context);
    std::fprintf(file, "%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%s\n", static_cast<double>(s.timeS),
                 static_cast<double>(s.gapM), static_cast<double>(s.egoSpeedMps),
                 static_cast<double>(s.leadSpeedMps), static_cast<double>(s.ttcS),
                 static_cast<double>(s.brakeRequestMps2), static_cast<double>(s.brakeActualMps2),
                 aeb::toString(s.state));
}

}  // namespace

int main(int argc, char** argv) {
    std::string csvDir;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--csv") == 0 && i + 1 < argc) {
            csvDir = argv[++i];
        } else {
            std::fprintf(stderr, "usage: %s [--csv <dir>]\n", argv[0]);
            return 2;
        }
    }
    if (!csvDir.empty()) {
        std::filesystem::create_directories(csvDir);
    }

    const aeb::Config config{};
    const sim::PlantParams plant{};
    size_t passed = 0;

    std::printf("AEB software-in-the-loop run (host)\n");
    for (size_t i = 0; i < sim::kScenarioCount; ++i) {
        const sim::Scenario& scenario = sim::kScenarios[i];

        std::FILE* csv = nullptr;
        if (!csvDir.empty()) {
            const std::string path = csvDir + "/" + scenario.name + ".csv";
            csv = std::fopen(path.c_str(), "w");
            if (csv == nullptr) {
                std::fprintf(stderr, "cannot write %s\n", path.c_str());
                return 2;
            }
            std::fprintf(csv, "time_s,gap_m,ego_mps,lead_mps,ttc_s,brake_req_mps2,brake_act_mps2,state\n");
        }

        const sim::Result result =
            sim::runScenario(scenario, config, plant, csv != nullptr ? writeCsvRow : nullptr, csv);
        if (csv != nullptr) {
            std::fclose(csv);
        }

        char line[160];
        sim::formatResult(line, sizeof line, scenario, result);
        std::printf("%s\n", line);
        passed += result.passed ? 1U : 0U;
    }

    std::printf("SUMMARY %zu/%zu scenarios passed\n", passed, sim::kScenarioCount);
    return passed == sim::kScenarioCount ? 0 : 1;
}
