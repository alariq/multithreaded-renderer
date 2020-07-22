#ifndef GAMEOS_RENDER_H
#define GAMEOS_RENDER_H

#include <SDL2/SDL.h>

namespace graphics {

struct RenderWindow {
    SDL_Window* window_;
    int width_;
    int height_;
};

};

#endif // GAMEOS_GRAPHICS_H


