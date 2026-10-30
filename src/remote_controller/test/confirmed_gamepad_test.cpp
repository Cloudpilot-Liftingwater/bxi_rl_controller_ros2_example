// Actual parser/mapper with a deterministic monotonic clock; no ROS publisher.
#include <algorithm>
#include <array>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include "remote_controller/input_mapper.hpp"

using remote_controller::InputMapper;
using remote_controller::RemoteConfig;
using Message = communication::msg::MotionCommands;

namespace {
enum Key {LB, RB, A, B, X, Y, STOP};
const char *raw[] = {"js.button.6", "js.button.7", "js.button.0",
    "js.button.1", "js.button.3", "js.button.4", "js.button.10"};
void check(bool value, const std::string &reason)
{
    if (!value) {throw std::runtime_error(reason);}
}
std::array<int, 10> slots(const Message &m)
{
    return {{m.btn_1, m.btn_2, m.btn_3, m.btn_4, m.btn_5,
        m.btn_6, m.btn_7, m.btn_8, m.btn_9, m.btn_10}};
}
struct Harness {
    std::chrono::steady_clock::time_point now{};
    InputMapper mapper;
    explicit Harness(const RemoteConfig &config) : mapper(config, [this]() {return now;})
    {mapper.set_signal(raw[STOP], 0.0);}
    void key(Key key, bool down) {mapper.set_signal(raw[key], down ? 1.0 : (0.0));}
    void wait(int ms) {now += std::chrono::milliseconds(ms); mapper.tick();}
    Message read() {Message m; mapper.fill_message(m); return m;}
};
void expect(const Message &m, int slot = 0, int value = 0)
{
    auto actual = slots(m);
    if (actual[7] == 3) {actual[7] = 0;}  // Neutral/rearm marker is not an action.
    std::array<int, 10> wanted{{}};
    if (slot) {wanted[slot - 1] = value;}
    check(actual == wanted, "unexpected action; expected slot " + std::to_string(slot));
    check(m.vel_des.x == 0 && m.vel_des.y == 0 && m.yawdot_des == 0, "walking velocity leaked");
}
Message sample(Harness &h, std::ofstream &out, const std::string &name, const std::string &phase)
{
    const auto m = h.read();
    if (out.is_open()) {
        out << name << '\t' << phase;
        for (int value : slots(m)) {out << '\t' << value;}
        out << '\n';
    }
    return m;
}
struct Action {const char *name; std::vector<Key> keys; int slot; int value;};
const Action actions[] = {
    {"pd", {RB, B}, 3, 1}, {"idle", {LB, B}, 8, 1},
    {"continue", {RB, Y}, 9, 3}, {"dance", {LB, A}, 6, 3},
    {"sonic06", {LB, X}, 6, 4}, {"sonic11", {LB, Y}, 6, 5},
};

int test_permutations(const RemoteConfig &config, std::ofstream &out)
{
    int count = 0;
    for (const auto &a : actions) {
        auto press = a.keys;
        std::sort(press.begin(), press.end());
        int p = 0;
        do {
            auto release = a.keys;
            std::sort(release.begin(), release.end());
            int r = 0;
            do {
                Harness h(config);
                const std::string name = std::string(a.name) + "_p" + std::to_string(p) + "_r" + std::to_string(r++);
                expect(sample(h, out, name, "neutral"));
                for (Key key : press) {
                    h.key(key, true);
                    if (key == press.back()) {expect(sample(h, out, name, "request"), a.slot, a.value);}
                    else {expect(sample(h, out, name, "prefix"));h.wait(20);}
                }
                h.wait(3000);expect(sample(h, out, name, "held"));
                for (Key key : release) {
                    h.key(key, false);h.wait(20);
                    expect(sample(h, out, name, "release"));
                }
                for (Key key : press) {h.key(key, true);}
                expect(sample(h, out, name, "repeat"), a.slot, a.value);
                ++count;
            } while (std::next_permutation(release.begin(), release.end()));
            ++p;
        } while (std::next_permutation(press.begin(), press.end()));
    }
    return count;
}

void test_click_and_prefixes(const RemoteConfig &config, std::ofstream &out)
{
    Harness click(config);
    click.key(B, true);expect(sample(click, out, "sequence", "held"));
    click.key(B, false);expect(sample(click, out, "sequence", "request"), 9, 2);
    click.wait(5000);expect(sample(click, out, "sequence", "released"));
    // No delay, even for a down/up occurring at the same monotonic timestamp.
    Harness tap(config);tap.key(B, true);tap.key(B, false);expect(tap.read(), 9, 2);
    // A B combined with any other button must never replay a lone B on release.
    for (Key modifier : {LB, RB, STOP, A, X, Y}) {
        Harness h(config);h.key(B, true);h.key(modifier, true);
        if (modifier == LB) {expect(h.read(), 8, 1);}
        else if (modifier == RB) {expect(h.read(), 3, 1);}
        else if (modifier == STOP) {expect(h.read(), 8, 2);}
        else {expect(h.read());}
        h.key(modifier, false);h.key(B, false);expect(h.read());
    }
    // RT alone has no meaning in the test profile; RT+Y cannot request lie-down.
    Harness rt(config);rt.mapper.set_signal("js.axis.4", 1.0);rt.key(Y, true);
    expect(rt.read());rt.wait(5000);expect(rt.read());rt.key(Y, false);expect(rt.read());
    // Obsolete double-shoulder chords may not turn into valid commands as
    // their buttons are released. Normal is the only intentional exception.
    for (Key face : {A, B, X, Y}) {
        std::vector<Key> release{LB, RB, face};std::sort(release.begin(), release.end());
        do {
            Harness invalid(config);
            invalid.mapper.set_signals({{raw[LB], 1.0}, {raw[RB], 1.0}, {raw[face], 1.0}});
            bool rb = true;bool x = face == X;
            expect(invalid.read(), x ? 1 : 0, x ? 1 : 0);
            for (Key key : release) {
                invalid.key(key, false);if (key == RB) {rb = false;}
                if (key == X) {x = false;}
                expect(invalid.read(), rb && x ? 1 : 0, rb && x ? 1 : 0);
            }
        } while (std::next_permutation(release.begin(), release.end()));
    }
}

void test_normal_and_zero_override(const RemoteConfig &config, std::ofstream &out)
{
    Harness h(config);h.key(LB, true);h.key(A, true);expect(h.read(), 6, 3);
    // Initiating LB need not be released to return to normal.
    h.key(RB, true);h.key(X, true);expect(sample(h, out, "escape", "normal"), 1, 1);
    h.wait(3000);expect(h.read(), 1, 1);
    h.key(A, false);h.key(LB, false);h.key(RB, false);
    expect(sample(h, out, "escape", "release"));
    h.key(X, false);expect(sample(h, out, "escape", "release"));
    // View/Back works immediately with every combination of other keys held.
    const Key keys[] = {LB, RB, A, B, X, Y};
    for (int mask = 0; mask < 64; ++mask) {
        Harness z(config);
        for (int i = 0; i < 6; ++i) {if (mask & (1 << i)) {z.key(keys[i], true);}}
        z.key(STOP, true);expect(z.read(), 8, 2);
        z.wait(10000);expect(z.read(), 8, 2);
        z.key(STOP, false);expect(z.read(), ((mask & 2) && (mask & 16)) ? 1 : 0,
            ((mask & 2) && (mask & 16)) ? 1 : 0);
    }
    // Source activation/full-release barriers must never delay explicit stop.
    Harness initial(config);initial.mapper.set_input_edges_enabled(false);
    initial.key(LB, true);initial.key(A, true);initial.key(STOP, true);
    expect(initial.read(), 8, 2);
    initial.mapper.set_input_edges_enabled(true);expect(initial.read(), 8, 2);
    // Stop wins over a pending, not yet published model request and normal.
    Harness pending(config);pending.key(LB, true);pending.key(A, true);
    pending.key(RB, true);pending.key(X, true);pending.key(STOP, true);
    expect(sample(pending, out, "stop", "request"), 8, 2);
    pending.key(STOP, false);pending.key(RB, false);pending.key(X, false);
    expect(sample(pending, out, "stop", "release"));
    pending.key(A, false);pending.key(LB, false);expect(sample(pending, out, "stop", "release"));
    // Unpublished model pulses are dropped if more than one face key appears.
    const Key faces[] = {A, B, X, Y};
    for (int mask = 1; mask < 16; ++mask) {
        if ((mask & (mask - 1)) == 0) {continue;}
        for (int modifiers = 0; modifiers < 4; ++modifiers) {
            Harness multi(config);
            if (modifiers & 1) {multi.key(LB, true);}
            if (modifiers & 2) {multi.key(RB, true);}
            for (int i = 0; i < 4; ++i) {if (mask & (1 << i)) {multi.key(faces[i], true);}}
            multi.wait(5000);
            const bool normal = (modifiers & 2) && (mask & (1 << 2));
            expect(multi.read(), normal ? 1 : 0, normal ? 1 : 0);
        }
    }
}

void test_activation_loss_and_cancel(const RemoteConfig &config)
{
    Harness h(config);h.mapper.set_input_edges_enabled(false);
    h.key(LB, true);h.key(Y, true);h.mapper.set_input_edges_enabled(true);
    h.wait(5000);expect(h.read());
    h.key(LB, false);h.key(Y, false);h.key(LB, true);h.key(Y, true);expect(h.read(), 6, 5);
    Harness lost(config);lost.key(B, true);lost.mapper.clear_signals_with_prefix("js.");expect(lost.read());
    Harness pending(config);pending.key(LB, true);pending.key(A, true);
    pending.key(RB, true);pending.key(X, true);expect(pending.read(), 1, 1);
}

}  // namespace

int main(int argc, char **argv)
{
    try {
        check(argc == 2 || argc == 3, "usage: test CONFIG [TRACE.tsv]");
        const auto config = remote_controller::load_remote_config(argv[1]);
        check(config.gestures.enabled && config.system_commands.empty() &&
            config.analog_outputs.empty(), "test-only profile must isolate legacy/system/walking outputs");
        std::ofstream out;
        if (argc == 3) {out.open(argv[2]);check(out.good(), "trace open");}
        const int n = test_permutations(config, out);
        test_click_and_prefixes(config, out);
        test_normal_and_zero_override(config, out);
        test_activation_loss_and_cancel(config);
        std::cout << "PASS: " << n << " immediate press/release permutations; lone-B release; "
            "44 multi-face states; immediate normal; single-key zero over 64 held-key states; reconnect/loss\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';return 1;
    }
}
