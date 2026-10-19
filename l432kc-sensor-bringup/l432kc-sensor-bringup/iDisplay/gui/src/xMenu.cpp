#include "xMenu.h"

namespace idisplay {

void xMenu::clear() {
    count_ = 0;
    sel_ = 0;
    first_ = 0;
}

bool xMenu::addItem(const char* text, Kind kind, xScreen* screen, xMenuAction action, void* ctx) {
    if (count_ >= kMaxItems) return false;
    Item& it = items_[count_++];
    it.text = text;
    it.kind = kind;
    it.screen = screen;
    it.action = action;
    it.ctx = ctx;
    return true;
}

bool xMenu::add(const char* text, xScreen& target) {
    return addItem(text, Kind::Screen, &target, nullptr, nullptr);
}

bool xMenu::add(const char* text, xMenuAction action, void* ctx) {
    return addItem(text, Kind::Action, nullptr, action, ctx);
}

bool xMenu::addBack(const char* text) {
    return addItem(text, Kind::Back, nullptr, nullptr, nullptr);
}

void xMenu::choose(xNavigator& nav) {
    if (sel_ >= count_) return;
    const Item& it = items_[sel_];
    switch (it.kind) {
    case Kind::Screen:
        if (it.screen != nullptr) nav.push(*it.screen);
        break;
    case Kind::Action:
        if (it.action != nullptr) it.action(nav, it.ctx);
        break;
    case Kind::Back:
        nav.pop();
        break;
    }
}

bool xMenu::onKey(xKey key, xKeyAction action, xNavigator& nav) {
    const bool press = action == xKeyAction::Pressed;
    const bool step = press || action == xKeyAction::Repeat;
    switch (key) {
    case xKey::Up:
        if (step) up();
        return step;
    case xKey::Down:
        if (step) down();
        return step;
    case xKey::Enter:
        if (press) choose(nav);
        else if (action == xKeyAction::Held) nav.pop();
        return press || action == xKeyAction::Held;
    case xKey::Back:
        if (press) nav.pop();
        return press;
    default:
        return false;
    }
}

void xMenu::render(iTextSurface& s) {
    uint8_t row = s.cursorRow();
    if (title_ != nullptr && row < s.rows()) {
        s.setCursor(0, row);
        s.setInverse(true);
        s.print(title_);
        s.padToEndOfRow();
        s.setInverse(false);
        ++row;
    }
    if (row >= s.rows()) return;
    const uint8_t visible = (uint8_t)(s.rows() - row);

    // Scroll just enough to keep the selection in view.
    if (sel_ < first_) first_ = sel_;
    if (sel_ >= first_ + visible) first_ = (uint8_t)(sel_ - visible + 1);
    if (first_ + visible > count_) first_ = count_ > visible ? (uint8_t)(count_ - visible) : 0;

    for (uint8_t i = first_; i < count_ && row < s.rows(); ++i, ++row) {
        const bool selected = i == sel_;
        s.setCursor(0, row);
        s.setInverse(selected);
        s.putChar(selected ? '>' : ' ');
        s.print(items_[i].text);
        if (selected) s.padToEndOfRow();
        s.setInverse(false);
    }
}

} // namespace idisplay
