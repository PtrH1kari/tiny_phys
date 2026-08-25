/*
 * tp_test.h -- a ~40 line test harness. No dependency worth adding for this.
 *
 * Usage:
 *   #include "tp_test.h"
 *   int main(void) {
 *       TP_TEST("thing does the thing") {
 *           TP_CHECK(1 + 1 == 2);
 *           TP_CHECK_NEAR(0.1f + 0.2f, 0.3f, 1e-6f);
 *       }
 *       return tp_test_summary();
 *   }
 */
#ifndef TP_TEST_H
#define TP_TEST_H

#include <stdio.h>
#include <math.h>

static int tp_test_failures = 0;
static int tp_test_checks   = 0;

#define TP_TEST(name) \
    for (int tp_once_ = (printf("-- %s\n", (name)), 1); tp_once_; tp_once_ = 0)

#define TP_CHECK(cond)                                                  \
    do {                                                                \
        tp_test_checks++;                                               \
        if (!(cond)) {                                                  \
            tp_test_failures++;                                         \
            printf("   FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);   \
        }                                                               \
    } while (0)

#define TP_CHECK_NEAR(a, b, eps)                                             \
    do {                                                                     \
        tp_test_checks++;                                                    \
        double a_ = (double)(a), b_ = (double)(b);                           \
        if (!(fabs(a_ - b_) <= (double)(eps))) {                             \
            tp_test_failures++;                                              \
            printf("   FAIL %s:%d  %s = %.9g, expected %.9g (tol %.3g)\n",   \
                   __FILE__, __LINE__, #a, a_, b_, (double)(eps));           \
        }                                                                    \
    } while (0)

#define TP_CHECK_V3_NEAR(v, ex, ey, ez, eps)  \
    do {                                      \
        TP_CHECK_NEAR((v).x, (ex), (eps));    \
        TP_CHECK_NEAR((v).y, (ey), (eps));    \
        TP_CHECK_NEAR((v).z, (ez), (eps));    \
    } while (0)

static int tp_test_summary(void) {
    printf("%d checks, %d failures\n", tp_test_checks, tp_test_failures);
    return tp_test_failures == 0 ? 0 : 1;
}

#endif /* TP_TEST_H */
