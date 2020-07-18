#ifndef GAMEOS_GRAPHICS_H
#define GAMEOS_GRAPHICS_H

namespace graphics {

struct RenderWindow;
typedef RenderWindow*   RenderWindowHandle;

void set_verbose(bool is_verbose);

RenderWindowHandle  create_window           (const char* pwinname, int width, int height, unsigned int rhi_flags);
bool                resize_window           (RenderWindowHandle rw_handle, int width, int height);
void                grab_window_input       (RenderWindowHandle rw_handle, bool grab);
void                get_window_size         (RenderWindowHandle rw_handle, int* width, int* height);
// may be different than window size (e.g. when switching to fullscreen)
void                get_drawable_size       (RenderWindowHandle rw_handle, int* width, int* height);
void                swap_window             (RenderWindowHandle h);
void                destroy_window          (RenderWindowHandle rw_handle);
bool                set_window_fullscreen   (RenderWindowHandle rw_handle, bool fullscreen);
bool                is_mode_supported       (int width, int height, int bpp);
int                 get_window_display_index(RenderWindowHandle rw_handle);
bool                get_desktop_display_mode(int display_index, int* width, int* height, int* bpp);
int                 get_num_display_modes   (int display_index);
bool                get_display_mode_by_index(int display_index, int mode_index, int* width, int* height, int* bpp);
bool				get_required_extensions	(RenderWindowHandle rw_handle, unsigned int* count, const char** names);


};

#endif // GAMEOS_GRAPHICS_H


