#pragma once

#include <string>

#include "plane_seg/plane_segmenter.hpp"

namespace plane_seg {

// Reads a flat "key: value" YAML file (see config/default.yaml) into Params.
// Keys that are absent keep their default. Throws std::runtime_error on an
// unknown key or a malformed value.
Params loadParams(const std::string& path, Params params = {});

}  // namespace plane_seg
