#pragma once

#include <cstddef>
#include <iostream>
#include <vector>

#define _EMT_STRINGIFY(x) #x

#define EMT_TESTSET_BEGIN(set_name) \
{ \
    const std::string __test_set_name = set_name; \
    int __test_count = 0; \
    int __fail_count = 0; \
    std::cout << "=> Test set: " << __test_set_name << std::endl; \
    std::cout << "=======================================================" << std::endl;

#define EMT_TESTSET_END \
    int __passed_count = __test_count - __fail_count; \
    std::cout << "=======================================================" << std::endl; \
    if (__fail_count != 0) { \
        std::cout << "In " << __test_set_name << ", " << __passed_count << " passed, " << __fail_count << " failed out of " << __test_count << " total." << std::endl; \
    } else { \
        std::cout << "In " << __test_set_name << ", all tests (" << __test_count << ") passed." << std::endl; \
    } \
    std::cout << std::endl; \
}

#define EMT_TEST_BEGIN(test_name) \
{\
    const std::string __test_name = test_name; \
    std::vector<std::string> __test_messages; \
    bool __test_failed = false;

#define EMT_TEST_END \
    if (__test_failed) { \
        std::cout << "    -> " << __test_name << ": FAILED" << std::endl; \
        for (size_t i = 0; i < __test_messages.size(); i ++) { \
            std::cout << "        * " << __test_messages[i] << std::endl; \
        } \
    } else { \
        std::cout << "    -> " << __test_name << ": PASSED" << std::endl; \
    } \
}

#define EMT_ASSERT_NOTHROW_BEGIN \
{ \
    const std::string __nothrow_test_name = std::string(__FILE__) + std::string(", ") + std::to_string(__LINE__); \
    bool __nothrow_test_failed = false; \
    try {

#define EMT_ASSERT_NOTHROW_END \
    } catch (...) { \
        __nothrow_test_failed = true; \
    } \
    __test_count ++; \
    if (__nothrow_test_failed) { \
        __test_failed = true; \
        __test_messages.push_back(std::string(__FILE__) + std::string(", ") + std::to_string(__LINE__) + ": ASSERT_NOTHROW failed: test threw an exception."); \
    } \
}

#define EMT_ASSERT_EQ(tested, expected) \
{\
    __test_count ++; \
    if (!(tested == expected)) {\
        __fail_count ++; \
        __test_failed = true; \
        __test_messages.push_back(std::string(__FILE__) + std::string(", ") + std::to_string(__LINE__) + ": ASSERT_EQ failed: \"" + _EMT_STRINGIFY(tested) + "\" was not equal to \"" + _EMT_STRINGIFY(expected) + "\""); \
    } \
}

#define EMT_ASSERT_NOT_EQ(tested, expected) \
{\
    __test_count ++; \
    if (tested == expected) {\
        __fail_count ++; \
        __test_failed = true; \
        __test_messages.push_back(std::string(__FILE__) + std::string(", ") + std::to_string(__LINE__) + ": ASSERT_EQ failed: \"" + _EMT_STRINGIFY(tested) + "\" was not equal to \"" + _EMT_STRINGIFY(expected) + "\""); \
    } \
}
