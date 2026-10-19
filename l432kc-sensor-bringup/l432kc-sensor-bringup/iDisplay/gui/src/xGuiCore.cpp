#include "xGuiCore.h"

namespace idisplay {

xGuiCore::xGuiCore(iTextSurface& surface, xScreen& home)
    : surface_(surface), home_(home), depth_(1) {
    stack_[0] = &home_;
}

void xGuiCore::start() {
    depth_ = 1;
    stack_[0] = &home_;
    home_.onEnter(*this);
}

bool xGuiCore::handle(const xGuiEvent& ev) {
    switch (ev.type) {
    case xGuiEventType::Key:
        top().onKey(ev.key, ev.action, *this);
        return true;
    case xGuiEventType::Refresh:
        top().onRefresh(ev.arg);
        return true;
    case xGuiEventType::Show:
        if (ev.screen == nullptr) return false;
        show(*ev.screen);
        return true;
    case xGuiEventType::Push:
        if (ev.screen == nullptr) return false;
        push(*ev.screen);
        return true;
    case xGuiEventType::Pop:
        pop();
        return true;
    case xGuiEventType::Home:
        home();
        return true;
    }
    return false;
}

void xGuiCore::render() {
    surface_.clear();
    top().render(surface_);
}

void xGuiCore::show(xScreen& s) {
    if (&s == &home_) {
        home();
        return;
    }
    if (depth_ == 1) {
        stack_[depth_++] = &s;     // never replace home itself
    } else {
        stack_[depth_ - 1] = &s;
    }
    s.onEnter(*this);
}

void xGuiCore::push(xScreen& s) {
    if (&s == &home_) {
        home();
        return;
    }
    if (depth_ < kMaxDepth) {
        stack_[depth_++] = &s;
    } else {
        stack_[depth_ - 1] = &s;
    }
    s.onEnter(*this);
}

void xGuiCore::pop() {
    if (depth_ <= 1) return;
    --depth_;
    top().onResume(*this);
}

void xGuiCore::home() {
    depth_ = 1;
    home_.onEnter(*this);
}

} // namespace idisplay
