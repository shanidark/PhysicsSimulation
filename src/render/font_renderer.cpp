#include "render/font_renderer.h"

#include <ft2build.h>
#include FT_FREETYPE_H

#include <array>
#include <vector>

namespace {
constexpr std::array<const char*, 5> fontCandidates {{
    "/usr/share/fonts/noto/NotoSans-Regular.ttf",
    "/usr/share/fonts/TTF/DejaVuSans.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/liberation/LiberationSans-Regular.ttf",
    "/usr/share/fonts/gnu-free/FreeSans.otf",
}};

const char* findUsableFont(FT_Library library, int pixelHeight, FT_Face& face) {
    for (const char* path : fontCandidates) {
        if (FT_New_Face(library, path, 0, &face) == 0) {
            FT_Set_Pixel_Sizes(face, 0, static_cast<FT_UInt>(pixelHeight));
            return path;
        }
    }
    return nullptr;
}
}

bool FontRenderer::init(int pixelHeight) {
    pixelHeight_ = pixelHeight;

    FT_Library library = nullptr;
    if (FT_Init_FreeType(&library) != 0) {
        return false;
    }

    FT_Face face = nullptr;
    if (!findUsableFont(library, pixelHeight, face)) {
        FT_Done_FreeType(library);
        return false;
    }

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    for (unsigned char c = 32; c < 127; ++c) {
        if (FT_Load_Char(face, c, FT_LOAD_RENDER) != 0) {
            continue;
        }

        GLuint texture = 0;
        const int width = static_cast<int>(face->glyph->bitmap.width);
        const int height = static_cast<int>(face->glyph->bitmap.rows);

        if (width > 0 && height > 0) {
            std::vector<unsigned char> pixels(static_cast<std::size_t>(width * height * 2));
            for (int i = 0; i < width * height; ++i) {
                pixels[static_cast<std::size_t>(i * 2)] = 255;
                pixels[static_cast<std::size_t>(i * 2 + 1)] = face->glyph->bitmap.buffer[i];
            }

            glGenTextures(1, &texture);
            glBindTexture(GL_TEXTURE_2D, texture);
            glTexImage2D(GL_TEXTURE_2D,
                         0,
                         GL_LUMINANCE_ALPHA,
                         static_cast<GLsizei>(width),
                         static_cast<GLsizei>(height),
                         0,
                         GL_LUMINANCE_ALPHA,
                         GL_UNSIGNED_BYTE,
                         pixels.data());

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        }

        glyphs_[c] = {
            texture,
            width,
            height,
            face->glyph->bitmap_left,
            face->glyph->bitmap_top,
            face->glyph->advance.x,
        };
    }

    glBindTexture(GL_TEXTURE_2D, 0);

    FT_Done_Face(face);
    FT_Done_FreeType(library);

    ready_ = true;
    return true;
}

void FontRenderer::shutdown() {
    for (Glyph& glyph : glyphs_) {
        if (glyph.texture != 0) {
            glDeleteTextures(1, &glyph.texture);
            glyph.texture = 0;
        }
    }
    ready_ = false;
}

void FontRenderer::drawText(float x, float y, float scale, std::string_view text) const {
    if (!ready_) {
        return;
    }

    glEnable(GL_TEXTURE_2D);

    float cursor = x;
    for (char raw : text) {
        const unsigned char c = static_cast<unsigned char>(raw);
        if (c >= glyphs_.size()) {
            continue;
        }

        const Glyph& glyph = glyphs_[c];
        if (glyph.texture == 0) {
            cursor += 8.0f * scale;
            continue;
        }

        const float xpos = cursor + static_cast<float>(glyph.bearingX) * scale;
        const float ypos = y - static_cast<float>(glyph.bearingY) * scale;
        const float width = static_cast<float>(glyph.width) * scale;
        const float height = static_cast<float>(glyph.height) * scale;

        glBindTexture(GL_TEXTURE_2D, glyph.texture);
        glBegin(GL_QUADS);
        glTexCoord2f(0.0f, 0.0f);
        glVertex2f(xpos, ypos);
        glTexCoord2f(1.0f, 0.0f);
        glVertex2f(xpos + width, ypos);
        glTexCoord2f(1.0f, 1.0f);
        glVertex2f(xpos + width, ypos + height);
        glTexCoord2f(0.0f, 1.0f);
        glVertex2f(xpos, ypos + height);
        glEnd();

        cursor += static_cast<float>(glyph.advance >> 6) * scale;
    }

    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);
}

float FontRenderer::measureText(std::string_view text, float scale) const {
    float width = 0.0f;
    for (char raw : text) {
        const unsigned char c = static_cast<unsigned char>(raw);
        if (c >= glyphs_.size()) {
            continue;
        }

        const Glyph& glyph = glyphs_[c];
        width += glyph.texture != 0 ? static_cast<float>(glyph.advance >> 6) * scale : 8.0f * scale;
    }
    return width;
}

float FontRenderer::lineHeight(float scale) const {
    return static_cast<float>(pixelHeight_) * scale;
}

bool FontRenderer::ready() const {
    return ready_;
}
