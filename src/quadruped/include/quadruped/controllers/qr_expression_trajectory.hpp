// SPDX-License-Identifier: MIT
#pragma once

#include <algorithm>
#include <cstddef>

namespace Quadruped {

// ROS-independent sampler used by the simulator controller and its tests.
struct ExpressionPose {
    float height = 0, velocity = 0, roll = 0, pitch = 0;
};

class ExpressionTrajectory {
public:
    enum class Action { NONE, HOP, STOMP, RECOVERY };

    static float Smooth(float u) {
        u = std::max(0.f, std::min(1.f, u));
        return u*u*u*(u*(6*u-15)+10);
    }
    static float SmoothDerivative(float u) {
        u = std::max(0.f, std::min(1.f, u));
        return 30*u*u*(u-1)*(u-1);
    }

    Action action() const { return active; }

    void Start(Action next, double now, const ExpressionPose& current) {
        active = next;
        started = now;
        from = output = current;
    }

    void Cancel(double now) {
        // Repeated cancels must not indefinitely extend the recovery deadline.
        if (active != Action::NONE && active != Action::RECOVERY) {
            Start(Action::RECOVERY, now, output);
        }
    }

    ExpressionPose Sample(double now, const ExpressionPose& live) {
        if (active == Action::NONE) return output = live;
        if (now < started) {
            active = Action::NONE;
            return output = live;
        }
        float elapsed = static_cast<float>(now - started);
        const float duration = active == Action::HOP ? .55f : .75f;
        if (active != Action::RECOVERY && elapsed >= duration) {
            // Both actions end at the nominal, level stance with zero speed.
            Start(Action::RECOVERY, started + duration, ExpressionPose{});
            elapsed = static_cast<float>(now - started);
        }
        if (active == Action::RECOVERY) {
            if (elapsed >= .35f) {
                active = Action::NONE;
                return output = live;
            }
            // Blend toward the *live* target, not a captured pose followed by
            // a jump when recovery finishes. The initial vertical velocity is
            // preserved by the quintic Hermite boundary term.
            output = Blend(from, live, elapsed, .35f);
        } else {
            if (active == Action::HOP) {
                const float times[] = {0, .08f, .22f, .38f, .55f};
                const float heights[] = {0, -.069f, .100f, -.050f, 0};
                output = Keyframes(elapsed, times, heights);
            } else {
                const float times[] = {0, .08f, .18f, .29f, .40f, .51f, .63f, .75f};
                const float heights[] = {0, -.069f, .100f, -.088f, .063f, -.075f, .044f, 0};
                output = Keyframes(elapsed, times, heights);
            }
            // Enter from the currently commanded pose without a step. By the
            // first crouch the correction is zero: impacts remain symmetric.
            const auto correction = Blend(from, ExpressionPose{}, elapsed,
                                          .08f);
            output.height += correction.height;
            output.velocity += correction.velocity;
            output.roll = correction.roll;
            output.pitch = correction.pitch;
        }
        output.velocity = std::max(-1.125f, std::min(1.125f, output.velocity));
        return output;
    }

private:
    static ExpressionPose Blend(const ExpressionPose& first, const ExpressionPose& last,
                                float elapsed, float duration) {
        const float u = std::max(0.f, std::min(1.f, elapsed / duration));
        const float s = Smooth(u), ds = SmoothDerivative(u) / duration;
        // Hermite basis for nonzero initial velocity, zero final velocity.
        const float h = u - 6*u*u*u + 8*u*u*u*u - 3*u*u*u*u*u;
        const float dh = 1 - 18*u*u + 32*u*u*u - 15*u*u*u*u;
        ExpressionPose pose;
        pose.height = first.height + (last.height-first.height)*s + first.velocity*duration*h;
        pose.velocity = (last.height-first.height)*ds + first.velocity*dh;
        pose.roll = first.roll + (last.roll-first.roll)*s;
        pose.pitch = first.pitch + (last.pitch-first.pitch)*s;
        return pose;
    }

    template <std::size_t N>
    static ExpressionPose Keyframes(float elapsed, const float (&times)[N],
                                   const float (&heights)[N]) {
        for (std::size_t i = 1; i < N; ++i) {
            if (elapsed <= times[i]) {
                ExpressionPose first, last;
                first.height = heights[i-1];
                last.height = heights[i];
                return Blend(first, last, elapsed-times[i-1], times[i]-times[i-1]);
            }
        }
        ExpressionPose result;
        result.height = heights[N-1];
        return result;
    }

    Action active = Action::NONE;
    double started = 0;
    ExpressionPose from, output;
};

}  // namespace Quadruped
