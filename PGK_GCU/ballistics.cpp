// ballistics.cpp
// PGK Fire Control — C++ compute engine
// Reads key=value pairs from stdin, writes results to stdout.

#include <iostream>
#include <string>
#include <sstream>
#include <map>
#include <cmath>
#include <iomanip>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double to_d(const std::map<std::string,std::string>& m,
                   const std::string& k, double def = 0.0) {
    auto it = m.find(k);
    if (it == m.end()) return def;
    try { return std::stod(it->second); }
    catch (...) { return def; }
}

static std::string to_s(const std::map<std::string,std::string>& m,
                        const std::string& k, const std::string& def = "") {
    auto it = m.find(k);
    return (it == m.end()) ? def : it->second;
}

int main() {
    std::map<std::string, std::string> data;
    std::string line;
    while (std::getline(std::cin, line)) {
        if (line.empty()) continue;
        auto pos = line.find('=');
        if (pos == std::string::npos) continue;
        std::string k = line.substr(0, pos);
        std::string v = line.substr(pos + 1);
        // trim
        while (!k.empty() && isspace(k.front())) k.erase(k.begin());
        while (!k.empty() && isspace(k.back()))  k.pop_back();
        while (!v.empty() && isspace(v.front())) v.erase(v.begin());
        while (!v.empty() && isspace(v.back()))  v.pop_back();
        data[k] = v;
    }

    // --- Read inputs into C++ variables ---
    double launch_long  = to_d(data, "launch_long");
    double launch_lat   = to_d(data, "launch_lat");
    double launch_height= to_d(data, "launch_height");
    double impact_long  = to_d(data, "impact_long");
    double impact_lat   = to_d(data, "impact_lat");
    double impact_height= to_d(data, "impact_height");
    std::string det_type= to_s(data, "detonation_type", "Impact");
    std::string command = to_s(data, "command", "CALCULATE");

    // --- Ballistics math ---
    double dx = (impact_long - launch_long) * 111320.0 *
                std::cos(((launch_lat + impact_lat) / 2.0) * M_PI / 180.0);
    double dy = (impact_lat  - launch_lat)  * 110540.0;
    double dz = impact_height - launch_height;

    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (distance < 1.0) distance = 1.0;

    double avg_velocity = 800.0;                // m/s
    double time_to_impact = distance / avg_velocity;
    double time_at_detonation = time_to_impact;

    if (det_type == "Time")            time_at_detonation = std::max(0.0, time_to_impact - 2.5);
    else if (det_type == "Proximity")  time_at_detonation = std::max(0.0, time_to_impact - 0.5);
    else if (det_type == "IED")        time_at_detonation = std::max(0.0, time_to_impact - 1.0);
    else if (det_type == "Manual Override") time_at_detonation = 0.0;

    double progress = std::min(100.0, (distance / 30000.0) * 100.0);
    std::string mono_code = "DIST: " + std::to_string((int)distance) + "m";
    std::string status = "OK";

    // --- Command handling (buttons) ---
    if (command == "MONO_CAST") {
        status = "MONO_CAST_COMPLETE";
        mono_code = "MONO COMPLETE @ " + std::to_string((int)distance) + "m";
    }
    else if (command == "MASS_CAST") {
        status = "MASS_CAST_COMPLETE";
        mono_code = "MASS CAST COMPLETE";
    }
    else if (command == "INSTANT_DETONATION") {
        status = "INSTANT_FIRE";
        time_to_impact     = 0.0;
        time_at_detonation = 0.0;
        progress           = 100.0;
        mono_code          = "INSTANT FIRE TRIGGERED!";
    }

    // --- Output to stdout ---
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "distance="            << distance            << "\n";
    std::cout << "time_to_impact="      << time_to_impact      << "\n";
    std::cout << "time_to_detonation="  << time_at_detonation  << "\n";
    std::cout << "progress="            << (int)progress       << "\n";
    std::cout << "mono_code="           << mono_code           << "\n";
    std::cout << "status="              << status              << "\n";
    return 0;
}