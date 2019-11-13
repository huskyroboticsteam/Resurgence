#pragma once

namespace Math {
class Vec2I;
class Vec2 {
public:
    float x, y;

    inline Vec2(float x = 0.0f, float y = 0.0f):
        x(x),
        y(y) {
    }
    
    inline Vec2(const Vec2 & copy_from) = default;
    inline Vec2(Vec2 && move_from) = default;
    
    inline ~Vec2() = default;

    inline Vec2 & operator=(const Vec2 & copy_from) = default;
    inline Vec2 & operator=(Vec2 && move_from) = default;

    inline Vec2 operator+() const {
        return *this;
    }

    inline Vec2 operator-() const {
        return Vec2(- x, - y);
    }

    inline Vec2 operator+(Vec2 rhs) const {
        return Vec2(x + rhs.x, y + rhs.y);
    }

    inline Vec2 operator-(Vec2 rhs) const {
        return Vec2(x - rhs.x, y - rhs.y);
    }

    inline Vec2 operator*(float scalar) const {
        return Vec2(x * scalar, y * scalar);
    }

    inline Vec2 operator/(float denom) const {
        return Vec2(x / denom, y / denom);
    }

    inline bool operator==(Vec2 rhs) const {
        return x == rhs.x && y == rhs.y;
    }

    inline bool operator!=(Vec2 rhs) const {
        return !(* this == rhs);
    }

    inline Vec2I into_Vec2I() const;
};

class Vec2I {
public:
    int x, y;

    Vec2I(int x = 0, int y = 0):
        x(x),
        y(y) {
    }

    inline Vec2I(const Vec2I & copy_from) = default;
    inline Vec2I(Vec2I && move_from) = default;
    
    inline ~Vec2I() = default;

    inline Vec2I & operator=(const Vec2I & copy_from) = default;
    inline Vec2I & operator=(Vec2I && move_from) = default;

    inline Vec2I operator+() const {
        return *this;
    }

    inline Vec2I operator-() const {
        return Vec2I(- x, - y);
    }

    inline Vec2I operator+(Vec2I rhs) const {
        return Vec2I(x + rhs.x, y + rhs.y);
    }

    inline Vec2I operator-(Vec2I rhs) const {
        return Vec2I(x - rhs.x, y - rhs.y);
    }

    inline Vec2I operator*(float scalar) const {
        return Vec2I(x * scalar, y * scalar);
    }

    inline Vec2I operator/(float denom) const {
        return Vec2I(x / denom, y / denom);
    }

    inline bool operator==(Vec2I rhs) const {
        return x == rhs.x && y == rhs.y;
    }

    inline bool operator!=(Vec2I rhs) const {
        return !(* this == rhs);
    }

    inline Vec2 into_Vec2() const;
};
}

inline Math::Vec2I Math::Vec2::into_Vec2I() const {
    return Vec2I(x, y);
}

inline Math::Vec2 Math::Vec2I::into_Vec2() const {
    return Vec2(x, y);
}