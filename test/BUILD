cc_test(
    name = "test_transformation",
    srcs = ["test_transformation.cpp"],
    tags = ["unit"],
    deps = [
        "//src:transformation",
        "@gtest",
    ],
)

# bazel run "//test:simple_test2"
cc_binary(
    name = "simple_test2",
    srcs = [
        # "simple_test.cpp",
        "simple_test2.cpp",
    ],
    deps = [
        "@eigen",
    ],
)

# bazel run "//test:simple_test1"
cc_binary(
    name = "simple_test",
    srcs = [
        "simple_test.cpp",
    ],
    deps = [
        "@eigen",
    ],
)
