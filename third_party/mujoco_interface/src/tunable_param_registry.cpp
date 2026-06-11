#include "tunable_param_registry.h"
#include <algorithm>

// ========== 极简注册接口（只需 name + 指针，其余由 YAML 提供）==========

void TunableParamRegistry::registerFloat(const std::string& name, float* ptr) {
    Param p;
    p.name     = name;
    p.label    = name;   // fallback: 用 name 作为标签
    p.group    = "";
    p.data_ptr = static_cast<void*>(ptr);
    p.type     = ParamType::FLOAT;
    p.min_val  = 0.0;
    p.max_val  = 0.0;
    p.format   = "%.3f";
    params_.push_back(p);
}

void TunableParamRegistry::registerDouble(const std::string& name, double* ptr) {
    Param p;
    p.name     = name;
    p.label    = name;
    p.group    = "";
    p.data_ptr = static_cast<void*>(ptr);
    p.type     = ParamType::DOUBLE;
    p.min_val  = 0.0;
    p.max_val  = 0.0;
    p.format   = "%.3f";
    params_.push_back(p);
}

void TunableParamRegistry::registerInt(const std::string& name, int* ptr) {
    Param p;
    p.name     = name;
    p.label    = name;
    p.group    = "";
    p.data_ptr = static_cast<void*>(ptr);
    p.type     = ParamType::INT;
    p.min_val  = 0.0;
    p.max_val  = 0.0;
    p.format   = "%d";
    params_.push_back(p);
}

void TunableParamRegistry::registerBool(const std::string& name, bool* ptr) {
    Param p;
    p.name     = name;
    p.label    = name;
    p.group    = "";
    p.data_ptr = static_cast<void*>(ptr);
    p.type     = ParamType::BOOL;
    p.min_val  = 0.0;
    p.max_val  = 1.0;
    p.format   = "";
    params_.push_back(p);
}

// ========== 查询接口 ==========

std::vector<TunableParamRegistry::Param>& TunableParamRegistry::getParams() {
    return params_;
}

const std::vector<TunableParamRegistry::Param>& TunableParamRegistry::getParams() const {
    return params_;
}

std::vector<std::string> TunableParamRegistry::getGroupNames() const {
    std::vector<std::string> names;
    for (const auto& p : params_) {
        if (std::find(names.begin(), names.end(), p.group) == names.end()) {
            names.push_back(p.group);
        }
    }
    return names;
}

const TunableParamRegistry::Param* TunableParamRegistry::findParam(const std::string& name) const {
    for (const auto& p : params_) {
        if (p.name == name) {
            return &p;
        }
    }
    return nullptr;
}

TunableParamRegistry::Param* TunableParamRegistry::findParam(const std::string& name) {
    for (auto& p : params_) {
        if (p.name == name) {
            return &p;
        }
    }
    return nullptr;
}

size_t TunableParamRegistry::size() const {
    return params_.size();
}

void TunableParamRegistry::clear() {
    params_.clear();
}
