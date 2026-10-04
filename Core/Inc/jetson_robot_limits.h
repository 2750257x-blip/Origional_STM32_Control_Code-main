#ifndef JETSON_ROBOT_LIMITS_H
#define JETSON_ROBOT_LIMITS_H

#include <float.h>

/* Compile-time software limits, in radians.
 * These values preserve the deployed source defaults. Rebuild and flash after
 * editing. Motor limits use physical motor coordinates, after sign/lean mapping.
 * These are software clips, not the motor protocol's representable range.
 * Set an ENABLED macro to 0 to bypass that software clip explicitly.
 */
#ifndef JETSON_MOTOR_TARGET_CLIP_ENABLED
#define JETSON_MOTOR_TARGET_CLIP_ENABLED 1
#endif
#ifndef JETSON_FEEDBACK_ERROR_CLIP_ENABLED
#define JETSON_FEEDBACK_ERROR_CLIP_ENABLED 1
#endif
#ifndef JETSON_FEEDBACK_MAX_ERROR_RAD
#define JETSON_FEEDBACK_MAX_ERROR_RAD 0.5f
#endif

/* Left and right joints of the same type share the existing limits. */
#ifndef JETSON_LIMIT_LEG_PITCH_MIN_RAD
#define JETSON_LIMIT_LEG_PITCH_MIN_RAD -2.0f
#endif
#ifndef JETSON_LIMIT_LEG_PITCH_MAX_RAD
#define JETSON_LIMIT_LEG_PITCH_MAX_RAD 2.0f
#endif
#ifndef JETSON_LIMIT_LEG_ROLL_MIN_RAD
#define JETSON_LIMIT_LEG_ROLL_MIN_RAD -0.8f
#endif
#ifndef JETSON_LIMIT_LEG_ROLL_MAX_RAD
#define JETSON_LIMIT_LEG_ROLL_MAX_RAD 0.8f
#endif
#ifndef JETSON_LIMIT_LEG_YAW_MIN_RAD
#define JETSON_LIMIT_LEG_YAW_MIN_RAD -1.5f
#endif
#ifndef JETSON_LIMIT_LEG_YAW_MAX_RAD
#define JETSON_LIMIT_LEG_YAW_MAX_RAD 1.5f
#endif
#ifndef JETSON_LIMIT_KNEE_MIN_RAD
#define JETSON_LIMIT_KNEE_MIN_RAD -2.0f
#endif
#ifndef JETSON_LIMIT_KNEE_MAX_RAD
#define JETSON_LIMIT_KNEE_MAX_RAD 2.0f
#endif
#ifndef JETSON_LIMIT_ANKLE_PITCH_MIN_RAD
#define JETSON_LIMIT_ANKLE_PITCH_MIN_RAD -1.5f
#endif
#ifndef JETSON_LIMIT_ANKLE_PITCH_MAX_RAD
#define JETSON_LIMIT_ANKLE_PITCH_MAX_RAD 1.5f
#endif
#ifndef JETSON_LIMIT_ANKLE_ROLL_MIN_RAD
#define JETSON_LIMIT_ANKLE_ROLL_MIN_RAD -1.5f
#endif
#ifndef JETSON_LIMIT_ANKLE_ROLL_MAX_RAD
#define JETSON_LIMIT_ANKLE_ROLL_MAX_RAD 1.5f
#endif

#if (JETSON_MOTOR_TARGET_CLIP_ENABLED != 0) && (JETSON_MOTOR_TARGET_CLIP_ENABLED != 1)
#error "JETSON_MOTOR_TARGET_CLIP_ENABLED must be 0 or 1"
#endif
#if (JETSON_FEEDBACK_ERROR_CLIP_ENABLED != 0) && (JETSON_FEEDBACK_ERROR_CLIP_ENABLED != 1)
#error "JETSON_FEEDBACK_ERROR_CLIP_ENABLED must be 0 or 1"
#endif

/* Float constant comparisons need constant folding beyond ISO C's integer
 * constant expressions. GCC/Clang support this for static assertions; older
 * Keil compilers can use the equivalent file-scope array size check.
 * Suppress only this extension diagnostic, not failed assertions.
 */
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wgnu-folding-constant"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#if defined(__clang__) || (defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L)
#define JETSON_LIMITS_ASSERT(condition, name) _Static_assert((condition), #name)
#else
#define JETSON_LIMITS_ASSERT(condition, name) typedef char name[(condition) ? 1 : -1]
#endif

JETSON_LIMITS_ASSERT(JETSON_FEEDBACK_MAX_ERROR_RAD >= 0.0f &&
                    JETSON_FEEDBACK_MAX_ERROR_RAD <= FLT_MAX,
                    feedback_error_must_be_finite_and_nonnegative);
/* The fixed +/-12.56 rad encoding bounds match motor_el05.h P_MIN/P_MAX.
 * A software clip cannot widen the motor protocol's representable range.
 * Comparisons reject reversed intervals, NaN and infinity, even if disabled.
 */
#define JETSON_CHECK_JOINT_LIMIT(kind) \
    JETSON_LIMITS_ASSERT(JETSON_LIMIT_##kind##_MIN_RAD >= -12.56f && \
                        JETSON_LIMIT_##kind##_MAX_RAD <= 12.56f && \
                        JETSON_LIMIT_##kind##_MIN_RAD <= JETSON_LIMIT_##kind##_MAX_RAD, \
                        kind##_limits_must_be_ordered_finite_and_encodable)
JETSON_CHECK_JOINT_LIMIT(LEG_PITCH);
JETSON_CHECK_JOINT_LIMIT(LEG_ROLL);
JETSON_CHECK_JOINT_LIMIT(LEG_YAW);
JETSON_CHECK_JOINT_LIMIT(KNEE);
JETSON_CHECK_JOINT_LIMIT(ANKLE_PITCH);
JETSON_CHECK_JOINT_LIMIT(ANKLE_ROLL);
#undef JETSON_CHECK_JOINT_LIMIT
#undef JETSON_LIMITS_ASSERT
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

/* Bound the feedback error relative to the last target, not its per-frame change.
 * Keep this pure helper usable in host tests without the STM32 HAL.
 * MAX_ERROR_RAD must be nonnegative. 0 is a zero-width window, not disable.
 */
static inline float JetsonLimits_ClipFeedbackPosition(float position, float target)
{
#if JETSON_FEEDBACK_ERROR_CLIP_ENABLED
    float delta = position - target;
    if (delta > JETSON_FEEDBACK_MAX_ERROR_RAD) delta = JETSON_FEEDBACK_MAX_ERROR_RAD;
    if (delta < -JETSON_FEEDBACK_MAX_ERROR_RAD) delta = -JETSON_FEEDBACK_MAX_ERROR_RAD;
    return target + delta;
#else
    (void)target;
    return position;
#endif
}

#endif /* JETSON_ROBOT_LIMITS_H */
