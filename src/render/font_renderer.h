#pragma once

#include <GLFW/glfw3.h>

#include <array>
#include <string_view>

class FontRenderer {
public:
    bool init(int pixelHeight);
    void shutdown();
    void drawText(float x, float y, float scale, std::string_view text) const;
    float measureText(std::string_view text, float scale) const;
    float lineHeight(float scale) const;

private:
    struct Glyph {
        GLuint texture = 0;
        int width = 0;
        int height = 0;
        int bearingX = 0;
        int bearingY = 0;
        long advance = 0;
    };

    std::array<Glyph, 128> glyphs_ {};
    int pixelHeight_ = 0;
    bool ready_ = false;
};
