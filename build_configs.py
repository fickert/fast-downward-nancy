release = ["-DCMAKE_BUILD_TYPE=Release"]
debug = ["-DCMAKE_BUILD_TYPE=Debug"]
releasenolp = ["-DCMAKE_BUILD_TYPE=Release", "-DUSE_LP=NO"]
debugnolp = ["-DCMAKE_BUILD_TYPE=Debug", "-DUSE_LP=NO"]
minimal = ["-DCMAKE_BUILD_TYPE=Release", "-DDISABLE_PLUGINS_BY_DEFAULT=YES"]
minrt = ["-DCMAKE_BUILD_TYPE=Release", "-DDISABLE_PLUGINS_BY_DEFAULT=TRUE", "-DPLUGIN_REAL_TIME_ENABLED=TRUE", "-DPLUGIN_LANDMARK_CUT_HEURISTIC_ENABLED=TRUE", "-DPLUGIN_NULL_PRUNING_METHOD_ENABLED=TRUE"]
debugrt = ["-DCMAKE_BUILD_TYPE=Debug", "-DDISABLE_PLUGINS_BY_DEFAULT=TRUE", "-DPLUGIN_REAL_TIME_ENABLED=TRUE", "-DPLUGIN_LANDMARK_CUT_HEURISTIC_ENABLED=TRUE", "-DPLUGIN_NULL_PRUNING_METHOD_ENABLED=TRUE"]

DEFAULT = "minrt"
DEBUG = "debugrt"
