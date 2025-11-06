cc_test(
    name = "test_transformation",
    srcs = ["test_transformation.cpp"],
    tags = ["unit"],
    deps = [
        "//src:transformation",
        "@gtest",
    ],
)

# bazel run "//test:simple_test"  
cc_binary(
    name = "simple_test",
    srcs = [
        # "simple_test.cpp",
        "simple_test2.cpp",
    ],
    deps = [
        "@eigen",
    ],
)   