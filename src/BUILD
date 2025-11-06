# bazel build "//src:transformation"
cc_library(
    name = "transformation",
    srcs = [
        "Object.cpp",
        "Transformation.cpp",
    ],
    hdrs = [
        "Object.h",
        "Transformation.h",
        "types.h",
    ],
    visibility = ["//visibility:public"],
    deps = ["@eigen"],
)
