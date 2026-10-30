// Exercise the actual YAML parser, mapper and Linux joystick driver. No ROS
// node, system action or robot publisher is started by this executable.
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/stat.h>
#include <unistd.h>

#include "remote_controller/drivers/driver_registry.hpp"
#include "remote_controller/input_mapper.hpp"

using remote_controller::InputMapper;
using remote_controller::RemoteConfig;
using Message = communication::msg::MotionCommands;

namespace {
void check(bool condition, const std::string &reason)
{
    if (!condition) {throw std::runtime_error(reason);}
}

std::array<int, 10> buttons(const Message &m)
{
    return {{m.btn_1, m.btn_2, m.btn_3, m.btn_4, m.btn_5,
        m.btn_6, m.btn_7, m.btn_8, m.btn_9, m.btn_10}};
}

Message read(InputMapper &mapper)
{
    Message m;
    mapper.fill_message(m);
    return m;
}

void only(const Message &m, int slot, int value)
{
    auto expected = std::array<int, 10>{{}};
    expected[slot - 1] = value;
    expected[7] = 3;  // Existing shoulder release marker, not an event.
    check(buttons(m) == expected, "overlapping command for slot " + std::to_string(slot));
}

void trace(std::ofstream &stream, const std::string &name,
    const std::string &phase, InputMapper &mapper)
{
    if (!stream.is_open()) {return;}
    stream << name << '\t' << phase;
    for (int v : buttons(read(mapper))) {stream << '\t' << v;}
    stream << '\n';
}

struct Request {const char *name; int button; int slot; int value;};
const Request requests[] = {
    {"continue", 1, 9, 3}, {"dance", 0, 6, 3},
    {"sonic06", 3, 6, 4}, {"sonic11", 4, 6, 5},
};

void test_requests(const RemoteConfig &config, std::ofstream &stream)
{
    for (const auto &request : requests) {
        std::array<int, 3> order{{0, 1, 2}};
        int permutation = 0;
        do {
            InputMapper mapper(config);
            const std::string name = std::string(request.name) + "_" +
                std::to_string(permutation++);
            const std::string face = "js.button." + std::to_string(request.button);
            const std::string parts[] = {"js.button.6", "js.axis.4", face};
            mapper.set_signal("js.axis.4", -1.0);
            trace(stream, name, "neutral", mapper);
            // Both modifier press orders are supported. Face is pressed last.
            const int first = permutation % 2;
            mapper.set_signal(parts[first], 1.0);
            trace(stream, name, "modifier", mapper);
            mapper.set_signal(parts[1 - first], 1.0);
            trace(stream, name, "modifier", mapper);
            mapper.set_signal(face, 1.0);
            only(read(mapper), request.slot, request.value);
            trace(stream, name, "request", mapper);
            trace(stream, name, "held", mapper);
            // Test every physical release order, including RT first. Native
            // legacy outputs may reappear; the suite filter must suppress them.
            for (int index : order) {
                mapper.set_signal(parts[index], index == 1 ? -1.0 : 0.0);
                trace(stream, name, "release", mapper);
            }
            check(buttons(read(mapper)) == std::array<int, 10>{{}}, "not neutral");
            mapper.set_signals({{"js.button.6", 1.0}, {"js.axis.4", 1.0}, {face, 1.0}});
            only(read(mapper), request.slot, request.value);
            trace(stream, name, "repeat", mapper);
        } while (std::next_permutation(order.begin(), order.end()));
    }
}

void test_conflicts_and_escape(const RemoteConfig &config, std::ofstream &stream)
{
    // Exhaust all two-or-more face-button combinations under the new modifier.
    const int faces[] = {0, 1, 3, 4};
    for (int mask = 1; mask < 16; ++mask) {
        if ((mask & (mask - 1)) == 0) {continue;}
        InputMapper mapper(config);
        mapper.set_signals({{"js.button.6", 1.0}, {"js.axis.4", 1.0}});
        for (int i = 0; i < 4; ++i) {
            if (mask & (1 << i)) {mapper.set_signal("js.button." + std::to_string(faces[i]), 1.0);}
        }
        const auto m = read(mapper);
        check(m.btn_6 == 0 && m.btn_9 == 0, "ambiguous face buttons selected a model");
    }
    for (double rt : {-1.0, 1.0}) {
        InputMapper mapper(config);
        mapper.set_signals({{"js.axis.4", rt}, {"js.button.7", 1.0}, {"js.button.3", 1.0}});
        only(read(mapper), 1, 1);  // RB+X normal, RT need not be released.
        const std::string name = rt < 0 ? "normal_rt0" : "normal_rt1";
        trace(stream, name, "normal", mapper);
        mapper.set_signal("js.button.7", 0.0);
        trace(stream, name, "release", mapper);
        mapper.set_signal("js.button.3", 0.0);
        trace(stream, name, "release", mapper);
        mapper.set_signal("js.button.7", 1.0);
        mapper.set_signals({{"js.button.3", 0.0}, {"js.button.6", 1.0}, {"js.button.1", 1.0}});
        const auto m = read(mapper);
        check(m.btn_8 == 2 && m.btn_6 == 0 && m.btn_9 == 0, "zero torque overlaps a new request");
    }
    InputMapper threshold(config);
    threshold.set_signals({{"js.button.6", 1.0}, {"js.axis.4", .89}, {"js.button.4", 1.0}});
    check(read(threshold).btn_6 == 0, "RT below activation threshold");
    threshold.set_signal("js.axis.4", .91);
    check(read(threshold).btn_6 == 5, "RT above activation threshold");
    threshold.set_signal("js.axis.4", .82);
    check(read(threshold).btn_6 == 5, "RT hysteresis lost");
    threshold.set_signal("js.axis.4", .79);
    check(read(threshold).btn_6 == 0, "RT did not release");
}

void test_linux_driver(const RemoteConfig &config)
{
    // Feed real Linux js_event packets through the official driver via a FIFO.
    // This validates decoding/normalization, not a physical gamepad's layout.
    char temp[] = "/tmp/elf3-gamepad-XXXXXX";
    check(mkdtemp(temp) != nullptr, "mkdtemp");
    const std::string path = std::string(temp) + "/js";
    check(mkfifo(path.c_str(), 0600) == 0, "mkfifo");
    const int fifo = open(path.c_str(), O_RDWR | O_NONBLOCK);
    check(fifo >= 0, "open FIFO");
    InputMapper mapper(config);
    std::mutex lock;
    auto device = config.input_devices.at(0);
    check(device.type == "joystick", "expected official joystick source");
    device.device = path;
    auto driver = remote_controller::create_input_driver(device, mapper, lock,
        [](const std::vector<std::string> &outputs) {check(outputs.empty(), "unexpected system event");},
        [](const std::string &) {});
    check(driver != nullptr, "joystick factory");
    driver->start();
    auto send = [&](int type, int number, int value) {
        js_event event{};
        event.type = type;
        event.number = number;
        event.value = value;
        check(write(fifo, &event, sizeof(event)) == static_cast<ssize_t>(sizeof(event)), "write js_event");
    };
    auto await_buttons = [&](int slot, int value) {
        for (int i = 0; i < 200; ++i) {
            {
                const std::lock_guard<std::mutex> guard(lock);
                const auto m = read(mapper);
                if (buttons(m)[slot - 1] == value && driver->is_ready()) {return;}
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        throw std::runtime_error("Linux driver packet timeout");
    };
    send(JS_EVENT_AXIS | JS_EVENT_INIT, 4, -32767);
    send(JS_EVENT_BUTTON, 6, 1);
    send(JS_EVENT_AXIS, 4, 32767);
    send(JS_EVENT_BUTTON, 3, 1);
    await_buttons(6, 4);  // SONIC06 from actual decoded packets.
    send(JS_EVENT_BUTTON, 3, 0);
    send(JS_EVENT_BUTTON, 6, 0);
    send(JS_EVENT_AXIS, 4, -32767);
    await_buttons(8, 0);
    driver->stop();
    close(fifo);
    unlink(path.c_str());
    rmdir(temp);
}
}  // namespace

int main(int argc, char **argv)
{
    try {
        check(argc == 2 || argc == 3, "usage: test CONFIG [TRACE.tsv]");
        const auto config = remote_controller::load_remote_config(argv[1]);
        std::ofstream stream;
        if (argc == 3) {stream.open(argv[2]); check(stream.good(), "trace open");}
        test_requests(config, stream);
        test_conflicts_and_escape(config, stream);
        test_linux_driver(config);
        std::cout << "PASS: 4 suite chords, 24 release sequences, 11 ambiguous combinations, "
            "normal/zero-torque escapes, RT hysteresis, native Linux joystick packets\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
