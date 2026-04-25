#ifndef TEST_HELPERS_H
#define TEST_HELPERS_H

#include <stdio.h>
#include <stdlib.h>

extern int test_failed;

#define EXPECT_EQ(actual, expected) do {                              \
    long _a = (long)(actual), _e = (long)(expected);                  \
    if (_a != _e) {                                                   \
        fprintf(stderr, "%s:%d: EXPECT_EQ failed: "                   \
                "actual=%ld expected=%ld\n",                          \
                __FILE__, __LINE__, _a, _e);                          \
        test_failed = 1;                                              \
    }                                                                 \
} while (0)

#define EXPECT_WITHIN(actual, expected, tol) do {                     \
    long _a = (long)(actual), _e = (long)(expected), _t = (long)(tol);\
    if (labs(_a - _e) > _t) {                                         \
        fprintf(stderr, "%s:%d: EXPECT_WITHIN failed: "               \
                "actual=%ld expected=%ld tol=%ld\n",                  \
                __FILE__, __LINE__, _a, _e, _t);                      \
        test_failed = 1;                                              \
    }                                                                 \
} while (0)

#define EXPECT_TRUE(cond) do {                                        \
    if (!(cond)) {                                                    \
        fprintf(stderr, "%s:%d: EXPECT_TRUE failed: %s\n",            \
                __FILE__, __LINE__, #cond);                           \
        test_failed = 1;                                              \
    }                                                                 \
} while (0)

#endif /* TEST_HELPERS_H */
