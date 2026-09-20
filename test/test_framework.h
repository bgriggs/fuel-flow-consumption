// Minimal self-contained test framework.
//
// Deliberately has no external dependencies so the suite builds with nothing
// but a C++ compiler, on a developer machine or in CI, with no package step.
#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <vector>
#include <string>

namespace testing {

struct TestCase {
  const char* suite;
  const char* name;
  void (*fn)();
};

// Function-local static: avoids any dependence on translation-unit
// initialization order while tests self-register.
inline std::vector<TestCase>& registry() {
  static std::vector<TestCase> cases;
  return cases;
}

inline int& currentFailures() {
  static int n = 0;
  return n;
}

inline std::vector<std::string>& failureLog() {
  static std::vector<std::string> log;
  return log;
}

struct Registrar {
  Registrar(const char* suite, const char* name, void (*fn)()) {
    TestCase c;
    c.suite = suite;
    c.name = name;
    c.fn = fn;
    registry().push_back(c);
  }
};

inline void recordFailure(const char* file, int line, const std::string& what) {
  currentFailures()++;
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%s:%d: ", file, line);
  failureLog().push_back(std::string(buf) + what);
}

template <typename T>
inline std::string toStr(const T& v) {
  return std::to_string(v);
}
inline std::string toStr(bool v) { return v ? "true" : "false"; }
inline std::string toStr(const char* v) { return v ? std::string(v) : "(null)"; }

inline int run() {
  int failed = 0;
  int passed = 0;
  const char* lastSuite = nullptr;
  for (size_t i = 0; i < registry().size(); i++) {
    const TestCase& c = registry()[i];
    if (lastSuite == nullptr || std::string(lastSuite) != c.suite) {
      std::printf("\n[%s]\n", c.suite);
      lastSuite = c.suite;
    }
    currentFailures() = 0;
    failureLog().clear();
    c.fn();
    if (currentFailures() == 0) {
      std::printf("  PASS  %s\n", c.name);
      passed++;
    } else {
      std::printf("  FAIL  %s\n", c.name);
      for (size_t j = 0; j < failureLog().size(); j++) {
        std::printf("          %s\n", failureLog()[j].c_str());
      }
      failed++;
    }
  }
  std::printf("\n%d passed, %d failed, %d total\n", passed, failed,
              passed + failed);
  return failed == 0 ? 0 : 1;
}

}  // namespace testing

#define TEST(suite_name, test_name)                                       \
  static void suite_name##_##test_name();                                 \
  static testing::Registrar reg_##suite_name##_##test_name(               \
      #suite_name, #test_name, suite_name##_##test_name);                 \
  static void suite_name##_##test_name()

#define CHECK(cond)                                                       \
  do {                                                                    \
    if (!(cond)) {                                                        \
      testing::recordFailure(__FILE__, __LINE__,                          \
                             std::string("CHECK(") + #cond + ") failed"); \
    }                                                                     \
  } while (0)

#define CHECK_EQ(actual, expected)                                        \
  do {                                                                    \
    auto a_ = (actual);                                                   \
    auto e_ = (expected);                                                 \
    if (!(a_ == e_)) {                                                    \
      testing::recordFailure(                                             \
          __FILE__, __LINE__,                                             \
          std::string(#actual) + " == " + #expected + " failed: got " +   \
              testing::toStr(a_) + ", want " + testing::toStr(e_));       \
    }                                                                     \
  } while (0)

#define CHECK_NEAR(actual, expected, tol)                                 \
  do {                                                                    \
    double a_ = (double)(actual);                                         \
    double e_ = (double)(expected);                                       \
    if (!(std::fabs(a_ - e_) <= (double)(tol))) {                         \
      testing::recordFailure(                                             \
          __FILE__, __LINE__,                                             \
          std::string(#actual) + " near " + #expected + " failed: got " + \
              std::to_string(a_) + ", want " + std::to_string(e_) +       \
              " +/- " + std::to_string((double)(tol)));                   \
    }                                                                     \
  } while (0)

#endif  // TEST_FRAMEWORK_H
