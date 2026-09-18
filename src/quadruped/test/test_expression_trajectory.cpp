// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>
#include <cmath>
#include "controllers/qr_expression_trajectory.hpp"

using Quadruped::ExpressionPose;
using Quadruped::ExpressionTrajectory;
using Action = ExpressionTrajectory::Action;

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(ExpressionTrajectory, ExactSymmetricStompKeyframes) {
    ExpressionTrajectory trajectory;
    trajectory.Start(Action::STOMP, 0, {});
    const double times[] = {0, .08, .18, .29, .40, .51, .63, .75};
    const float heights[] = {0, -.069, .100, -.088, .063, -.075, .044, 0};
    for (int i = 0; i < 8; ++i) {
        const auto pose = trajectory.Sample(times[i], {});
        EXPECT_NEAR(pose.height, heights[i], 1e-6);
        EXPECT_NEAR(pose.velocity, 0, 1e-5);
        EXPECT_FLOAT_EQ(pose.roll, 0);
        EXPECT_FLOAT_EQ(pose.pitch, 0);
    }
}

TEST(ExpressionTrajectory, BoundedVelocityAndDerivative) {
    for (const auto action : {Action::STOMP, Action::HOP}) {
        ExpressionTrajectory trajectory;
        trajectory.Start(action, 0, {});
        auto before = trajectory.Sample(0, {});
        for (int tick = 1; tick < 1900; ++tick) {
            const auto pose = trajectory.Sample(tick * .001, {});
            EXPECT_TRUE(std::isfinite(pose.height));
            EXPECT_LE(std::abs(pose.velocity), 1.125001);
            EXPECT_LE(std::abs(pose.height), .100001);
            // A capped feed-forward velocity may differ at the hop's peak.
            const float derivative = (pose.height - before.height) / .001f;
            EXPECT_NEAR(std::max(-1.125f, std::min(1.125f, derivative)), pose.velocity, .075);
            before = pose;
        }
    }
}

TEST(ExpressionTrajectory, CancelAtMultiplePhasesIsContinuousAndSettles) {
    for (const auto action : {Action::STOMP, Action::HOP}) {
        for (double time : {.01, .06, .12, .22, .38, .50}) {
            ExpressionTrajectory trajectory;
            trajectory.Start(action, 0, {});
            const auto before = trajectory.Sample(time, {});
            trajectory.Cancel(time);
            ExpressionPose target;
            target.height = -.012f;
            target.roll = .08f;
            target.pitch = -.06f;
            const auto atCancel = trajectory.Sample(time, target);
            EXPECT_FLOAT_EQ(before.height, atCancel.height);
            EXPECT_FLOAT_EQ(before.velocity, atCancel.velocity);
            EXPECT_FLOAT_EQ(before.roll, atCancel.roll);
            EXPECT_FLOAT_EQ(before.pitch, atCancel.pitch);
            EXPECT_EQ(trajectory.action(), Action::RECOVERY);
            auto last = atCancel;
            for (int tick = 1; tick <= 351; ++tick) {
                const auto pose = trajectory.Sample(time + tick*.001, target);
                EXPECT_LT(std::abs(pose.height-last.height), .0012);
                EXPECT_LT(std::abs(pose.roll-last.roll), .002);
                last = pose;
                trajectory.Cancel(time + tick*.001); // repeated cancel is idempotent
            }
            EXPECT_EQ(trajectory.action(), Action::NONE);
            EXPECT_FLOAT_EQ(last.height, target.height);
            EXPECT_FLOAT_EQ(last.roll, target.roll);
            EXPECT_FLOAT_EQ(last.pitch, target.pitch);
            EXPECT_FLOAT_EQ(last.velocity, 0);
        }
    }
}

TEST(ExpressionTrajectory, MovingRecoveryTargetDoesNotSnapAtEnd) {
    ExpressionTrajectory trajectory;
    trajectory.Start(Action::STOMP, 0, {});
    trajectory.Sample(.4, {});
    trajectory.Cancel(.4);
    ExpressionPose target;
    auto previous = trajectory.Sample(.4, target);
    for (int tick = 1; tick <= 360; ++tick) {
        target.height = .02f * tick/360.f;
        target.roll = -.1f * tick/360.f;
        const auto pose = trajectory.Sample(.4 + tick*.001, target);
        EXPECT_LT(std::abs(pose.height-previous.height), .0005);
        EXPECT_LT(std::abs(pose.roll-previous.roll), .002);
        previous = pose;
    }
    EXPECT_FLOAT_EQ(previous.height, target.height);
}

TEST(ExpressionTrajectory, InterruptingRecoveryWithNewActionHasNoStep) {
    ExpressionTrajectory trajectory;
    ExpressionPose initial;
    initial.height = -.015f;
    initial.roll = .10f;
    trajectory.Start(Action::STOMP, 0, initial);
    auto pose = trajectory.Sample(0, {});
    EXPECT_FLOAT_EQ(pose.height, initial.height);
    EXPECT_FLOAT_EQ(pose.roll, initial.roll);
    trajectory.Sample(.6, {});
    trajectory.Cancel(.6);
    const auto before = trajectory.Sample(.7, initial);
    trajectory.Start(Action::HOP, .7, before);
    pose = trajectory.Sample(.7, {});
    EXPECT_FLOAT_EQ(pose.height, before.height);
    EXPECT_FLOAT_EQ(pose.velocity, before.velocity);
    EXPECT_FLOAT_EQ(pose.roll, before.roll);
    pose = trajectory.Sample(.78, {});
    EXPECT_NEAR(pose.roll, 0, 1e-6);
    EXPECT_NEAR(pose.height, -.069, 1e-6);
}
