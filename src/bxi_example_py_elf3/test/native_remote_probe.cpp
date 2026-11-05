// Test harness for the unchanged native mapper. Never executes system actions.
#include <iostream>
#include <vector>
#include "remote_controller/input_mapper.hpp"

int main(int argc, char **argv) {
    if (argc != 2 && argc != 3) return 2;
    remote_controller::InputMapper mapper(remote_controller::load_remote_config(argv[1]));
    const bool crsf=argc==3 && std::string(argv[2])=="crsf";
    const std::vector<std::string> sources=crsf ? std::vector<std::string>{
        "crsf.xbox.button.lb", "crsf.xbox.button.rb", "crsf.xbox.trigger_right",
        "crsf.xbox.button.a", "crsf.xbox.button.b", "crsf.xbox.button.x",
        "crsf.xbox.button.y", "crsf.xbox.button.back", "crsf.xbox.trigger_left"
    } : std::vector<std::string>{"js.button.6", "js.button.7", "js.axis.4",
        "js.button.0", "js.button.1", "js.button.3", "js.button.4", "js.button.11", "js.axis.5"};
    double first;
    while (std::cin >> first) {
        std::vector<std::pair<std::string,double>> values={{sources[0],first}};
        for (std::size_t i=1;i<sources.size();++i) {
            double v; if (!(std::cin >> v)) return 3;
            values.emplace_back(sources[i],v);
        }
        const auto actions=mapper.set_signals(values);
        communication::msg::MotionCommands m; mapper.fill_message(m);
        std::cout << "MSG " << m.btn_1 << ' ' << m.btn_2 << ' ' << m.btn_3 << ' '
            << m.btn_4 << ' ' << m.btn_5 << ' ' << m.btn_6 << ' ' << m.btn_7 << ' '
            << m.btn_8 << ' ' << m.btn_9 << ' ' << m.btn_10;
        for (const auto &action:actions) std::cout << ' ' << action;
        std::cout << '\n';
    }
}
