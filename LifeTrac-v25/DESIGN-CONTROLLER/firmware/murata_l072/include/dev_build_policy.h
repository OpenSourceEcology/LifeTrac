#ifndef DEV_BUILD_POLICY_H
#define DEV_BUILD_POLICY_H

/*
 * Development defaults for all non-final LifeTrac controller firmware.
 * These remain enabled unless a final-release build explicitly overrides them.
 */

#ifndef FINAL_RELEASE_BUILD
#define FINAL_RELEASE_BUILD 0
#endif

#ifndef RADIO_MINIMIZER_DEFAULT
#define RADIO_MINIMIZER_DEFAULT 1
#endif

#ifndef POWER_CYCLE_AVOIDANCE_DEFAULT
#define POWER_CYCLE_AVOIDANCE_DEFAULT 1
#endif

/*
 * Bench/test images must explicitly opt in to active RF behavior.
 * Default is 0 so development images come up quiet on the bench.
 */
#ifndef RADIO_ACTIVE_TEST_DEFAULT
#define RADIO_ACTIVE_TEST_DEFAULT 0
#endif

#endif /* DEV_BUILD_POLICY_H */