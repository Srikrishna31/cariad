# bazel build "//src:object"
cc_library(
    name = "object",
    srcs = ["Object.cpp"],
    hdrs = [
        "Object.h",
        "types.h",
    ],
    visibility = ["//visibility:public"],
    deps = ["@eigen"],
)

# bazel build "//src:transformation"
cc_library(
    name = "transformation",
    srcs = ["Transformation.cpp"],
    hdrs = [
        "Transformation.h",
        "types.h",
    ],
    visibility = ["//visibility:public"],
    deps = ["@eigen"],
)
