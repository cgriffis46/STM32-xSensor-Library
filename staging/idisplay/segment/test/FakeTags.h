// A tag source for tests: names and values in a small table, changed at will.
#pragma once
#include <cmath>
#include <cstring>
#include <string>
#include <vector>
#include "iTagSource.h"

struct FakeTags : idisplay::iTagSource {
    struct T { std::string name; idisplay::xTagValue v; bool readable = true; };
    std::vector<T> tags;
    std::vector<std::pair<std::string, idisplay::xTagValue>> writes;
    bool acceptWrites = true;

    void num(const char* n, double v) { slot(n).v = idisplay::xTagValue::num(v); }
    void boolean(const char* n, bool v) { slot(n).v = idisplay::xTagValue::boolean(v); }
    void unreadable(const char* n) { slot(n).readable = false; }

    int32_t find(const char* name) override {
        for (size_t i = 0; i < tags.size(); i++) if (tags[i].name == name) return (int32_t)i;
        return -1;
    }
    bool read(int32_t h, idisplay::xTagValue& out) override {
        if (h < 0 || (size_t)h >= tags.size() || !tags[h].readable) return false;
        out = tags[h].v;
        return true;
    }
    bool write(int32_t h, const idisplay::xTagValue& v) override {
        if (h < 0 || (size_t)h >= tags.size()) return false;
        writes.push_back({tags[h].name, v});
        if (acceptWrites) tags[h].v = v;
        return acceptWrites;
    }
private:
    T& slot(const char* n) {
        for (auto& t : tags) if (t.name == n) return t;
        tags.push_back(T{n, {}, true});
        return tags.back();
    }
};
