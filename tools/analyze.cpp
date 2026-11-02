#include "guanid/public_units.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace {
std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> result;
    std::istringstream stream(line);
    std::string cell;
    while (std::getline(stream,cell,',')) result.push_back(cell);
    if (!line.empty() && line.back() == ',') result.push_back("");
    return result;
}
double number(const std::string& value) {
    std::size_t end;
    const double n = std::stod(value,&end);
    if (end != value.size() || !std::isfinite(n)) throw std::runtime_error("invalid number: " + value);
    return n;
}
std::size_t windowSize(const std::string& value) {
    const double n = number(value);
    if (n < 1 || n > 10001 || std::floor(n) != n) throw std::runtime_error("invalid window size");
    return static_cast<std::size_t>(n);
}
void load(const std::string& path, std::vector<guanid::Record>& rows, std::uint32_t& sequence) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("cannot open " + path);
    std::string line;
    if (!std::getline(file,line)) throw std::runtime_error("empty CSV: " + path);
    if (!line.empty() && line.back() == '\r') line.pop_back();
    auto headers = split(line);
    std::map<std::string,std::size_t> columns;
    for (std::size_t i = 0; i < headers.size(); ++i)
        if (!columns.emplace(headers[i],i).second) throw std::runtime_error("duplicate CSV column");
    const bool legacy = columns.count("left_applied_volts");
    const std::vector<std::string> required = legacy ? std::vector<std::string>{
        "test","direction","elapsed_s","commanded_volts","left_applied_volts","left_position_m",
        "left_velocity_mps","right_applied_volts","right_position_m","right_velocity_mps"} :
        std::vector<std::string>{"run_id","test","direction","time_s","motor","units","voltage","position","velocity"};
    for (const auto& key : required) if (!columns.count(key)) throw std::runtime_error("missing column: " + key);
    std::map<std::tuple<std::string,guanid::Test,guanid::Direction>,std::uint32_t> ids;
    std::size_t line_number = 1;
    while (std::getline(file,line)) {
        ++line_number;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;
        try {
            const auto cells = split(line);
            if (cells.size() != headers.size()) throw std::runtime_error("wrong column count");
            auto get = [&](const std::string& key) -> const std::string& { return cells.at(columns.at(key)); };
            const auto test = get("test"), direction = get("direction");
            if ((test != "quasistatic" && test != "dynamic") || (direction != "forward" && direction != "reverse"))
                throw std::runtime_error("invalid test/direction");
            guanid::Record row;
            row.test = test == "dynamic" ? guanid::Test::dynamic : guanid::Test::quasistatic;
            row.direction = direction == "forward" ? guanid::Direction::forward : guanid::Direction::reverse;
            const auto original_id = columns.count("run_id") ? get("run_id") : "0";
            if (original_id.empty() || original_id.find_first_not_of("0123456789") != std::string::npos)
                throw std::runtime_error("invalid run_id");
            const auto key = std::make_tuple(original_id,row.test,row.direction);
            if (!ids.count(key)) ids[key] = sequence++;
            row.run_id = ids.at(key);
            row.time_s = number(get(legacy ? "elapsed_s" : "time_s"));
            if (row.time_s < 0) throw std::runtime_error("negative timestamp");
            if (legacy) {
                if (std::abs(number(get("commanded_volts"))) > 12) throw std::runtime_error("invalid commanded voltage");
                for (const auto* side : {"left","right"}) {
                    const std::string s = side;
                    row.motor = {}; row.motor.name = s;
                    row.motor.voltage(number(get(s+"_applied_volts"))).linearPosition(number(get(s+"_position_m")))
                        .linearVelocity(number(get(s+"_velocity_mps")));
                    if (!row.motor.valid()) throw std::runtime_error("invalid motor sample");
                    rows.push_back(row);
                }
            } else {
                row.motor.name = get("motor"); row.motor.voltage(number(get("voltage")));
                if (get("units") == "m") row.motor.linearPosition(number(get("position"))).linearVelocity(number(get("velocity")));
                else if (get("units") == "rad") row.motor.angularPosition(number(get("position"))).angularVelocity(number(get("velocity")));
                else if (get("units") == "in") row.motor.linearPosition(number(get("position"))*guanid::metersPerInch).linearVelocity(number(get("velocity"))*guanid::metersPerInch);
                else if (get("units") == "deg") row.motor.angularPosition(number(get("position"))*guanid::radiansPerDegree).angularVelocity(number(get("velocity"))*guanid::radiansPerDegree);
                else throw std::runtime_error("units must be m, rad, in or deg");
                if (!row.motor.valid()) throw std::runtime_error("invalid motor sample");
                rows.push_back(row);
            }
        } catch (const std::exception& e) { throw std::runtime_error(path+":"+std::to_string(line_number)+": "+e.what()); }
    }
    if (file.bad()) throw std::runtime_error("CSV read failure");
}
void jsonNumber(std::ostream& out,double n) { if (std::isfinite(n)) out << n; else out << "null"; }
void fitJson(std::ostream& out,const guanid::Fit& fit, double scale = 1) {
    out << "{\"ok\":" << (fit.ok ? "true" : "false");
    if (!fit.ok) out << ",\"error\":" << std::quoted(fit.error);
    else {
        const auto& g = fit.gains;
        out << ",\"kS\":" << g.kS << ",\"kV\":" << g.kV*scale << ",\"kA\":" << g.kA*scale << ",\"kG\":" << g.kG
            << ",\"voltage_rmse\":" << fit.voltage_rmse << ",\"excitation_ratio\":" << fit.excitation_ratio
            << ",\"acceleration_r_squared\":";
        jsonNumber(out,fit.acceleration_r_squared); out << ",\"voltage_r_squared\":"; jsonNumber(out,fit.voltage_r_squared);
    }
    out << ",\"samples\":" << fit.samples << "}";
}
// Self-contained SVG diagnostics: one row per run, columns velocity, volts, residual.
void plot(const std::string& path,const guanid::Analysis& analysis, double scale = 1, const char* unit = "SI") {
    using Key = std::tuple<std::uint32_t,guanid::Test,guanid::Direction>;
    std::map<Key,std::vector<guanid::Diagnostic>> runs;
    for (const auto& p : analysis.diagnostics) runs[{p.record.run_id,p.record.test,p.record.direction}].push_back(p);
    std::ofstream svg(path);
    if (!svg) throw std::runtime_error("cannot write " + path);
    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"1200\" height=\"" << runs.size()*250
        << "\"><rect width=\"100%\" height=\"100%\" fill=\"white\"/>";
    std::size_t run_index = 0;
    for (const auto& run : runs) {
        const auto& points = run.second;
        const double t0 = points.front().record.time_s, span = std::max(1e-6,points.back().record.time_s-t0);
        svg << "<text x=\"10\" y=\"" << run_index*250+20 << "\" font-size=\"15\">"
            << guanid::stateName(guanid::testState(std::get<1>(run.first),std::get<2>(run.first)))
            << " run " << std::get<0>(run.first) << "</text>";
        for (int panel = 0; panel < 3; ++panel) {
            double lo = 0, hi = 0;
            auto value = [&](const guanid::Diagnostic& p,int series) {
                if (panel == 0) return (series ? p.velocity : p.record.motor.velocity) / scale;
                if (panel == 1) return series ? p.predicted_voltage : p.record.motor.volts;
                return p.residual;
            };
            for (const auto& p : points) for (int s = 0; s < 2; ++s) { lo=std::min(lo,value(p,s)); hi=std::max(hi,value(p,s)); }
            const double range = std::max(1e-6,hi-lo), x = panel*400+55, y = run_index*250+50;
            svg << "<g font-size=\"12\"><text x=\"" << x << "\" y=\"" << y-10 << "\">"
                << (panel == 0 ? "velocity: raw blue, filtered orange" : panel == 1 ? "volts: measured blue, model orange" : "residual volts")
                << " (" << (panel == 0 ? unit : "V") << ")</text><path d=\"M" << x << ',' << y << "v160h320\" fill=\"none\" stroke=\"gray\"/>"
                << "<text x=\"" << x-50 << "\" y=\"" << y+10 << "\">" << std::setprecision(3) << hi
                << "</text><text x=\"" << x-50 << "\" y=\"" << y+160 << "\">" << lo << "</text>"
                << "<text x=\"" << x << "\" y=\"" << y+180 << "\">" << t0 << " s</text>"
                << "<text x=\"" << x+260 << "\" y=\"" << y+180 << "\">" << t0+span << " s</text></g>";
            for (int s=0; s < (panel == 2 ? 1 : 2); ++s) {
                svg << "<polyline fill=\"none\" stroke=\"" << (s ? "#ea580c" : "#2563eb") << "\" points=\"";
                for (const auto& p : points) svg << x+320*(p.record.time_s-t0)/span << ',' << y+160-160*(value(p,s)-lo)/range << ' ';
                svg << "\"/>";
            }
        }
        ++run_index;
    }
    svg << "</svg>\n";
    if (!svg) throw std::runtime_error("plot write failure");
}
}
int main(int argc,char** argv) {
    try {
        guanid::AnalysisConfig config;
        std::vector<std::string> files;
        std::string output, plots, model_name = "simple", selected, left, right, derived;
        double track = 0, tau = 0, omega = 0;
        bool publicUnits = true, explicitMinVelocity = false;
        for (int i=1; i<argc; ++i) {
            const std::string arg = argv[i];
            auto value = [&]() -> std::string { if (++i >= argc) throw std::runtime_error("missing value for "+arg); return argv[i]; };
            if (arg == "--help") {
                std::cout << "guanid_analyze CSV... --out gains.json [--model simple|elevator|arm] [--motor NAME]\n"
                    "  --plot-dir DIR --min-velocity N --trim-start N --trim-end N --max-gap N\n"
                    "  --median-window N --derivative-window N --allow-incomplete\n"
                    "  --units in-deg|si (default in-deg); --min-velocity uses output axis units/s\n"
                    "  --combine LEFT RIGHT NAME | --angular-inches LEFT RIGHT NAME TRACK_WIDTH_IN\n"
                    "  --angular LEFT RIGHT NAME TRACK_WIDTH_M (legacy SI width)\n"
                    "  --velocity-tau SECONDS --position-omega RAD_PER_SEC\n"; return 0;
            } else if (arg == "--out") output = value();
            else if (arg == "--plot-dir") plots = value();
            else if (arg == "--units") { const auto unit = value(); if (unit != "si" && unit != "in-deg") throw std::runtime_error("units must be in-deg or si"); publicUnits = unit == "in-deg"; }
            else if (arg == "--motor") selected = value();
            else if (arg == "--model") {
                model_name = value();
                if (model_name == "simple") config.model = guanid::Model::simple;
                else if (model_name == "elevator") config.model = guanid::Model::elevator;
                else if (model_name == "arm") config.model = guanid::Model::arm;
                else throw std::runtime_error("unknown model");
            } else if (arg == "--min-velocity") { config.min_velocity = number(value()); explicitMinVelocity = true; }
            else if (arg == "--trim-start") config.trim_start_s = number(value());
            else if (arg == "--trim-end") config.trim_end_s = number(value());
            else if (arg == "--max-gap") config.max_gap_s = number(value());
            else if (arg == "--median-window") config.median_window = windowSize(value());
            else if (arg == "--derivative-window") config.derivative_window = windowSize(value());
            else if (arg == "--allow-incomplete") config.require_four_tests = false;
            else if (arg == "--velocity-tau") { tau = number(value()); if (tau <= 0) throw std::runtime_error("tau must be positive"); }
            else if (arg == "--position-omega") { omega = number(value()); if (omega <= 0) throw std::runtime_error("omega must be positive"); }
            else if (arg == "--combine" || arg == "--angular" || arg == "--angular-inches") {
                left=value(); right=value(); derived=value();
                if (arg != "--combine") { track=number(value()); if (arg == "--angular-inches") track *= guanid::metersPerInch; if (track <= 0) throw std::runtime_error("track width must be positive"); }
            } else if (arg.rfind("--",0) == 0) throw std::runtime_error("unknown option: "+arg);
            else files.push_back(arg);
        }
        if (files.empty() || output.empty() || !config.valid()) throw std::runtime_error("specify CSV files, --out and valid settings; see --help");
        std::vector<guanid::Record> records;
        std::uint32_t sequence = 0;
        for (const auto& path : files) load(path,records,sequence);
        if (!derived.empty() && !guanid::deriveDrivetrain(records,left,right,derived,records,track))
            throw std::runtime_error("drivetrain derivation requires matching linear samples and distinct channel names");
        std::set<std::string> names;
        for (const auto& r : records) if (selected.empty() || r.motor.name == selected) names.insert(r.motor.name);
        if (names.empty()) throw std::runtime_error("no selected channels");
        std::map<std::string,guanid::Analysis> results;
        for (const auto& name : names) {
            auto channelConfig = config;
            if (publicUnits && explicitMinVelocity) {
                for (const auto& row : records) if (row.motor.name == name) {
                    channelConfig.min_velocity *= guanid::siPerPublicUnit(row.motor.units); break;
                }
            }
            auto result = guanid::analyze(records,name,channelConfig);
            if (!result.ok) throw std::runtime_error(name+": "+result.error);
            results.emplace(name,std::move(result));
        }
        if (!plots.empty()) std::filesystem::create_directories(plots);
        std::ofstream out(output);
        if (!out) throw std::runtime_error("cannot write output");
        out << std::setprecision(12) << "{\"schema_version\":" << (publicUnits ? 3 : 2) << ",\"model\":" << std::quoted(model_name)
            << ",\"settings\":{\"min_velocity_explicit\":" << (explicitMinVelocity ? "true" : "false") << ",\"min_velocity\":";
        if (publicUnits && !explicitMinVelocity) out << "null";
        else out << config.min_velocity;
        out << ",\"trim_start_s\":" << config.trim_start_s
            << ",\"trim_end_s\":" << config.trim_end_s << ",\"max_gap_s\":" << config.max_gap_s
            << ",\"median_window\":" << config.median_window << ",\"derivative_window\":" << config.derivative_window
            << "},\"channels\":{";
        bool first = true;
        for (const auto& item : results) {
            const auto& name = item.first; const auto& a = item.second;
            const double scale = publicUnits ? guanid::siPerPublicUnit(a.units) : 1;
            const std::string unit = a.units == guanid::Units::linear ? (publicUnits ? "in" : "m") : (publicUnits ? "deg" : "rad");
            if (!first) out << ',';
            first = false;
            out << '\n' << std::quoted(name) << ":{\"position_unit\":" << std::quoted(unit)
                << ",\"velocity_unit\":" << std::quoted(unit+"/s")
                << ",\"acceleration_unit\":" << std::quoted(unit+"/s^2")
                << ",\"kV_unit\":" << std::quoted("V*s/"+unit)
                << ",\"kA_unit\":" << std::quoted("V*s^2/"+unit)
                << ",\"min_velocity\":" << (explicitMinVelocity ? config.min_velocity : config.min_velocity/scale)
                << ",\"fit\":"; fitJson(out,a,scale); out << ",\"forward\":"; fitJson(out,a.forward,scale);
            out << ",\"reverse\":"; fitJson(out,a.reverse,scale); out << ",\"warnings\":[";
            for (std::size_t i=0; i<a.warnings.size(); ++i) { if (i) out << ','; out << std::quoted(a.warnings[i]); }
            out << ']';
            auto feedback = [&](const char* key,guanid::FeedbackGains gains) {
                out << ",\"" << key << "\":{\"valid\":" << (gains.valid ? "true" : "false")
                    << ",\"kP\":" << gains.kP*scale << ",\"kI\":" << gains.kI*scale << ",\"kD\":" << gains.kD*scale << '}';
            };
            if (tau > 0) feedback("velocity_feedback",guanid::suggestVelocityFeedback(a.gains,tau));
            if (omega > 0) feedback("position_feedback",guanid::suggestPositionFeedback(a.gains,omega));
            out << '}';
            if (!plots.empty()) {
                const auto prefix = (std::filesystem::path(plots)/name).string();
                plot(prefix+".svg",a,scale,(unit+"/s").c_str());
                std::ofstream diagnostics(prefix+".csv");
                diagnostics << std::setprecision(12) << "run_id,test,direction,time_s,position_unit,raw_velocity,filtered_velocity,acceleration,voltage,model_voltage,residual\n";
                for (const auto& p : a.diagnostics) diagnostics << p.record.run_id << ','
                    << (p.record.test == guanid::Test::dynamic ? "dynamic" : "quasistatic") << ','
                    << (p.record.direction == guanid::Direction::forward ? "forward" : "reverse") << ','
                    << p.record.time_s << ',' << unit << ',' << p.record.motor.velocity/scale << ',' << p.velocity/scale << ',' << p.acceleration/scale
                    << ',' << p.record.motor.volts << ',' << p.predicted_voltage << ',' << p.residual << '\n';
                if (!diagnostics) throw std::runtime_error("diagnostics write failure");
            }
            std::cout << name << ": kS=" << a.gains.kS << " kV=" << a.gains.kV*scale << " kA=" << a.gains.kA*scale << " kG=" << a.gains.kG << " (" << unit << ", seconds, volts)" << '\n';
            for (const auto& warning : a.warnings) std::cerr << name << ": warning: " << warning << '\n';
        }
        out << "\n}}\n";
        out.close(); if (!out) throw std::runtime_error("output write failure");
        return 0;
    } catch (const std::exception& e) { std::cerr << "guanid: " << e.what() << '\n'; return 2; }
}
