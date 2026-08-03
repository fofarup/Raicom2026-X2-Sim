#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <png.h>
extern int XTestFakeMotionEvent(Display *, int, int, int, unsigned long);
extern int XTestFakeButtonEvent(Display *, unsigned int, int, unsigned long);
extern int XTestFakeKeyEvent(Display *, unsigned int, int, unsigned long);

static Window find_named(Display *d, Window w, const char *needle) {
  char *name = NULL;
  if (XFetchName(d, w, &name) && name) {
    int match = strstr(name, needle) != NULL;
    XFree(name);
    if (match) return w;
  }
  Window root, parent, *children = NULL;
  unsigned count = 0;
  if (!XQueryTree(d, w, &root, &parent, &children, &count)) return 0;
  Window result = 0;
  for (unsigned i = 0; i < count && !result; ++i)
    result = find_named(d, children[i], needle);
  if (children) XFree(children);
  return result;
}

static void print_tree(Display *d, Window w, int depth) {
  XWindowAttributes a; char *name = NULL;
  if (!XGetWindowAttributes(d, w, &a)) return;
  XFetchName(d, w, &name);
  printf("%*s%lu %dx%d+%d+%d map=%d name=%s\n", depth, "", w,
         a.width, a.height, a.x, a.y, a.map_state, name ? name : "");
  if (name) XFree(name);
  Window root, parent, *children = NULL; unsigned count = 0;
  if (XQueryTree(d, w, &root, &parent, &children, &count)) {
    for (unsigned i = 0; i < count; ++i) print_tree(d, children[i], depth + 2);
    if (children) XFree(children);
  }
}

int main(int argc, char **argv) {
  if (argc < 3) {
    fprintf(stderr, "usage: %s WINDOW_SUBSTR info|shot FILE|click X Y|reset\n", argv[0]);
    return 2;
  }
  Display *d = XOpenDisplay(NULL);
  if (!d) { fprintf(stderr, "cannot open DISPLAY\n"); return 1; }
  Window w = find_named(d, DefaultRootWindow(d), argv[1]);
  if (!w) { fprintf(stderr, "window not found: %s\n", argv[1]); return 1; }
  XWindowAttributes a;
  XGetWindowAttributes(d, w, &a);
  if (!strcmp(argv[2], "info")) {
    printf("window=%lu width=%d height=%d map_state=%d\n", w, a.width, a.height, a.map_state);
  } else if (!strcmp(argv[2], "tree")) {
    print_tree(d, w, 0);
  } else if (!strcmp(argv[2], "key") && argc == 4) {
    KeySym sym = XStringToKeysym(argv[3]);
    KeyCode code = XKeysymToKeycode(d, sym);
    XRaiseWindow(d, w); XSetInputFocus(d, w, RevertToParent, CurrentTime);
    int down = 0, up = 0;
    XKeyEvent ke = {0};
    ke.display=d; ke.window=w; ke.root=DefaultRootWindow(d); ke.same_screen=True;
    ke.keycode=code; ke.type=KeyPress;
    XSendEvent(d, w, True, KeyPressMask, (XEvent *)&ke); XFlush(d); usleep(100000);
    ke.type=KeyRelease;
    XSendEvent(d, w, True, KeyReleaseMask, (XEvent *)&ke); XFlush(d);
    fprintf(stderr, "key sym=%lu code=%u down=%d up=%d\n", sym, code, down, up);
  } else if (!strcmp(argv[2], "click") && argc == 5) {
    int x = atoi(argv[3]), y = atoi(argv[4]);
    Window child;
    int root_x, root_y;
    XTranslateCoordinates(d, w, DefaultRootWindow(d), x, y, &root_x, &root_y, &child);
    XRaiseWindow(d, w); XSetInputFocus(d, w, RevertToParent, CurrentTime);
    XWarpPointer(d, None, w, 0, 0, 0, 0, x, y); XFlush(d); usleep(100000);
    fprintf(stderr, "click local=%d,%d root=%d,%d\n", x, y, root_x, root_y);
    int down = XTestFakeButtonEvent(d, Button1, True, CurrentTime); XSync(d, False); usleep(500000);
    int up = XTestFakeButtonEvent(d, Button1, False, CurrentTime); XSync(d, False);
    fprintf(stderr, "button down=%d up=%d\n", down, up);
  } else if (!strcmp(argv[2], "reset") && argc == 3) {
    /* MuJoCo's default 200%% UI places Reset at these window-relative
       coordinates.  Relative positioning keeps this valid across the two
       window sizes used by the competition image. */
    int x = (int)(a.width * 0.079), y = (int)(a.height * 0.872);
    XRaiseWindow(d, w); XSetInputFocus(d, w, RevertToParent, CurrentTime);
    XWarpPointer(d, None, w, 0, 0, 0, 0, x, y); XFlush(d); usleep(100000);
    int down = XTestFakeButtonEvent(d, Button1, True, CurrentTime);
    XSync(d, False); usleep(250000);
    int up = XTestFakeButtonEvent(d, Button1, False, CurrentTime);
    XSync(d, False);
    fprintf(stderr, "reset local=%d,%d down=%d up=%d\n", x, y, down, up);
  } else if (!strcmp(argv[2], "shot") && argc == 4) {
    XImage *im = XGetImage(d, w, 0, 0, a.width, a.height, AllPlanes, ZPixmap);
    if (!im) { fprintf(stderr, "XGetImage failed\n"); return 1; }
    FILE *f = fopen(argv[3], "wb");
    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    png_infop info = png_create_info_struct(png);
    png_init_io(png, f);
    png_set_IHDR(png, info, a.width, a.height, 8, PNG_COLOR_TYPE_RGB,
                 PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png, info);
    unsigned char *line = calloc(a.width * 3, 1);
    for (int y = 0; y < a.height; ++y) {
      for (int x = 0; x < a.width; ++x) {
        unsigned long p = XGetPixel(im, x, y);
        line[3*x] = (p >> 16) & 255; line[3*x+1] = (p >> 8) & 255; line[3*x+2] = p & 255;
      }
      png_write_row(png, line);
    }
    png_write_end(png, NULL); png_destroy_write_struct(&png, &info); free(line);
    fclose(f); XDestroyImage(im);
  } else { fprintf(stderr, "bad command\n"); return 2; }
  XCloseDisplay(d);
  return 0;
}
