/*******************************************************************************
 * Size: 10 px
 * Bpp: 4
 * Opts: --bpp 4 --size 10 --no-compress --font LeagueGothic-Italic-2.otf --symbols No pain No gain --format lvgl -o league.c
 ******************************************************************************/

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

#ifndef LEAGUE
#define LEAGUE 1
#endif

#if LEAGUE

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+0020 " " */

    /* U+004E "N" */
    0x2, 0xf0, 0xf0, 0x5, 0xf2, 0xd0, 0x8, 0xf6,
    0x90, 0xb, 0xea, 0x60, 0xe, 0xae, 0x30, 0x1d,
    0x8f, 0x0, 0x4a, 0x7d, 0x0, 0x77, 0x7a, 0x0,

    /* U+0061 "a" */
    0x2, 0xd8, 0x9, 0x7c, 0x2, 0xc9, 0x1d, 0xc6,
    0x79, 0xd3, 0x6c, 0xf0,

    /* U+0067 "g" */
    0x0, 0x3e, 0xb6, 0x0, 0xb7, 0xc0, 0x0, 0xf6,
    0x90, 0x2, 0xd9, 0x50, 0x0, 0xea, 0x0, 0x8,
    0xc6, 0x0, 0xa, 0x4e, 0x40, 0xd, 0xe9, 0x0,

    /* U+0069 "i" */
    0x2, 0xd0, 0x1, 0x8, 0x70, 0xb4, 0xe, 0x11,
    0xe0, 0x4b, 0x7, 0x80,

    /* U+006E "n" */
    0x8, 0xdc, 0xb, 0x7d, 0xe, 0x6a, 0x1e, 0x87,
    0x4b, 0xb4, 0x78, 0xe1,

    /* U+006F "o" */
    0x4, 0xe8, 0xc, 0x6b, 0xe, 0x69, 0x3b, 0x96,
    0x69, 0xc2, 0x3e, 0x70,

    /* U+0070 "p" */
    0x8, 0xec, 0xb, 0x8d, 0xe, 0x6a, 0x2d, 0x87,
    0x5c, 0xc4, 0x8d, 0xc0, 0xb4, 0x0, 0xe1, 0x0
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 25, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 0, .adv_w = 63, .box_w = 6, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 24, .adv_w = 53, .box_w = 4, .box_h = 6, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 36, .adv_w = 55, .box_w = 6, .box_h = 8, .ofs_x = -1, .ofs_y = -2},
    {.bitmap_index = 60, .adv_w = 29, .box_w = 3, .box_h = 8, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 72, .adv_w = 54, .box_w = 4, .box_h = 6, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 84, .adv_w = 51, .box_w = 4, .box_h = 6, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 96, .adv_w = 54, .box_w = 4, .box_h = 8, .ofs_x = 0, .ofs_y = -2}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/

static const uint16_t unicode_list_0[] = {
    0x0, 0x2e, 0x41, 0x47, 0x49, 0x4e, 0x4f, 0x50
};

/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 32, .range_length = 81, .glyph_id_start = 1,
        .unicode_list = unicode_list_0, .glyph_id_ofs_list = NULL, .list_length = 8, .type = LV_FONT_FMT_TXT_CMAP_SPARSE_TINY
    }
};



/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

#if LVGL_VERSION_MAJOR == 8
/*Store all the custom data of the font*/
static  lv_font_fmt_txt_glyph_cache_t cache;
#endif

#if LVGL_VERSION_MAJOR >= 8
static const lv_font_fmt_txt_dsc_t font_dsc = {
#else
static lv_font_fmt_txt_dsc_t font_dsc = {
#endif
    .glyph_bitmap = glyph_bitmap,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .kern_dsc = NULL,
    .kern_scale = 0,
    .cmap_num = 1,
    .bpp = 4,
    .kern_classes = 0,
    .bitmap_format = 0,
#if LVGL_VERSION_MAJOR == 8
    .cache = &cache
#endif
};



/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LVGL_VERSION_MAJOR >= 8
const lv_font_t league = {
#else
lv_font_t league = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 10,          /*The maximum line height required by the font*/
    .base_line = 2,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = 0,
    .underline_thickness = 1,
#endif
    .dsc = &font_dsc,          /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
#if LV_VERSION_CHECK(8, 2, 0) || LVGL_VERSION_MAJOR >= 9
    .fallback = NULL,
#endif
    .user_data = NULL,
};



#endif /*#if LEAGUE*/

