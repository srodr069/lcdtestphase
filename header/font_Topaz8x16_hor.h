/* File generated by pnm2font                          */
/* http://bax.comlab.uni-rostock.de                    */
/*                                                     */

#ifndef PROGMEM
#define PROGMEM
#endif

extern const unsigned char PROGMEM font_image_8_16[1520];
extern const Bax_font font_8_16;
#ifndef FONT_MAIN
/* font image width 760 height 16 */
/* attention: characters in row-by-row order, first    */
/* pixel is MSB of first byte (other implementations   */
/* typically use vertical strips of 8 pixels).         */
#ifdef BAX_FONT
/* verify min/max (first two entries) before use! */
const Bax_font font_8_16 PROGMEM = {
 32,126,16,8,16,font_image_8_16,(0) };
#ifndef DEF_FONT
#define DEF_FONT &font_8_16
#endif /* DEF_FONT */
#else /* BAX_FONT */
#define FONT_STRIDE 16
#define FONT_WIDTH  8
#define FONT_HEIGHT 16
#define FONT_MIN    32
#define FONT_MAX    126
#define FONT_IMAGE  font_image_8_16
#endif
const unsigned char PROGMEM font_image_8_16[1520] = {
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x18,0x18,0x3c,0x3c,0x3c,0x3c,0x18,0x18,0x18,0x18,0x00,0x00,0x18,0x18,0x00,0x00,
0x6c,0x6c,0x6c,0x6c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x6c,0x6c,0x6c,0x6c,0xfe,0xfe,0x6c,0x6c,0xfe,0xfe,0x6c,0x6c,0x6c,0x6c,0x00,0x00,
0x18,0x18,0x3e,0x3e,0x60,0x60,0x3c,0x3c,0x06,0x06,0x7c,0x7c,0x18,0x18,0x00,0x00,
0x00,0x00,0xc6,0xc6,0xcc,0xcc,0x18,0x18,0x30,0x30,0x66,0x66,0xc6,0xc6,0x00,0x00,
0x38,0x38,0x6c,0x6c,0x68,0x68,0x76,0x76,0xdc,0xdc,0xcc,0xcc,0x76,0x76,0x00,0x00,
0x18,0x18,0x18,0x18,0x30,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x0c,0x0c,0x18,0x18,0x30,0x30,0x30,0x30,0x30,0x30,0x18,0x18,0x0c,0x0c,0x00,0x00,
0x30,0x30,0x18,0x18,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x18,0x18,0x30,0x30,0x00,0x00,
0x00,0x00,0x66,0x66,0x3c,0x3c,0xff,0xff,0x3c,0x3c,0x66,0x66,0x00,0x00,0x00,0x00,
0x00,0x00,0x18,0x18,0x18,0x18,0x7e,0x7e,0x18,0x18,0x18,0x18,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x18,0x18,0x30,0x30,
0x00,0x00,0x00,0x00,0x00,0x00,0x7e,0x7e,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x18,0x18,0x00,0x00,
0x03,0x03,0x06,0x06,0x0c,0x0c,0x18,0x18,0x30,0x30,0x60,0x60,0xc0,0xc0,0x00,0x00,
0x3c,0x3c,0x66,0x66,0x6e,0x6e,0x7e,0x7e,0x76,0x76,0x66,0x66,0x3c,0x3c,0x00,0x00,
0x18,0x18,0x38,0x38,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x7e,0x7e,0x00,0x00,
0x3c,0x3c,0x66,0x66,0x06,0x06,0x1c,0x1c,0x30,0x30,0x66,0x66,0x7e,0x7e,0x00,0x00,
0x3c,0x3c,0x66,0x66,0x06,0x06,0x1c,0x1c,0x06,0x06,0x66,0x66,0x3c,0x3c,0x00,0x00,
0x1c,0x1c,0x3c,0x3c,0x6c,0x6c,0xcc,0xcc,0xfe,0xfe,0x0c,0x0c,0x1e,0x1e,0x00,0x00,
0x7e,0x7e,0x60,0x60,0x7c,0x7c,0x06,0x06,0x06,0x06,0x66,0x66,0x3c,0x3c,0x00,0x00,
0x1c,0x1c,0x30,0x30,0x60,0x60,0x7c,0x7c,0x66,0x66,0x66,0x66,0x3c,0x3c,0x00,0x00,
0x7e,0x7e,0x66,0x66,0x06,0x06,0x0c,0x0c,0x18,0x18,0x18,0x18,0x18,0x18,0x00,0x00,
0x3c,0x3c,0x66,0x66,0x66,0x66,0x3c,0x3c,0x66,0x66,0x66,0x66,0x3c,0x3c,0x00,0x00,
0x3c,0x3c,0x66,0x66,0x66,0x66,0x3e,0x3e,0x06,0x06,0x0c,0x0c,0x38,0x38,0x00,0x00,
0x00,0x00,0x18,0x18,0x18,0x18,0x00,0x00,0x00,0x00,0x18,0x18,0x18,0x18,0x00,0x00,
0x00,0x00,0x18,0x18,0x18,0x18,0x00,0x00,0x00,0x00,0x18,0x18,0x18,0x18,0x30,0x30,
0x0c,0x0c,0x18,0x18,0x30,0x30,0x60,0x60,0x30,0x30,0x18,0x18,0x0c,0x0c,0x00,0x00,
0x00,0x00,0x00,0x00,0x7e,0x7e,0x00,0x00,0x00,0x00,0x7e,0x7e,0x00,0x00,0x00,0x00,
0x30,0x30,0x18,0x18,0x0c,0x0c,0x06,0x06,0x0c,0x0c,0x18,0x18,0x30,0x30,0x00,0x00,
0x3c,0x3c,0x66,0x66,0x06,0x06,0x0c,0x0c,0x18,0x18,0x00,0x00,0x18,0x18,0x00,0x00,
0x7c,0x7c,0xc6,0xc6,0xde,0xde,0xde,0xde,0xde,0xde,0xc0,0xc0,0x78,0x78,0x00,0x00,
0x18,0x18,0x3c,0x3c,0x3c,0x3c,0x66,0x66,0x7e,0x7e,0xc3,0xc3,0xc3,0xc3,0x00,0x00,
0xfc,0xfc,0x66,0x66,0x66,0x66,0x7c,0x7c,0x66,0x66,0x66,0x66,0xfc,0xfc,0x00,0x00,
0x3c,0x3c,0x66,0x66,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0x66,0x66,0x3c,0x3c,0x00,0x00,
0xf8,0xf8,0x6c,0x6c,0x66,0x66,0x66,0x66,0x66,0x66,0x6c,0x6c,0xf8,0xf8,0x00,0x00,
0xfe,0xfe,0x66,0x66,0x60,0x60,0x78,0x78,0x60,0x60,0x66,0x66,0xfe,0xfe,0x00,0x00,
0xfe,0xfe,0x66,0x66,0x60,0x60,0x78,0x78,0x60,0x60,0x60,0x60,0xf0,0xf0,0x00,0x00,
0x3c,0x3c,0x66,0x66,0xc0,0xc0,0xce,0xce,0xc6,0xc6,0x66,0x66,0x3e,0x3e,0x00,0x00,
0x66,0x66,0x66,0x66,0x66,0x66,0x7e,0x7e,0x66,0x66,0x66,0x66,0x66,0x66,0x00,0x00,
0x7e,0x7e,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x7e,0x7e,0x00,0x00,
0x0e,0x0e,0x06,0x06,0x06,0x06,0x06,0x06,0x66,0x66,0x66,0x66,0x3c,0x3c,0x00,0x00,
0xe6,0xe6,0x66,0x66,0x6c,0x6c,0x78,0x78,0x6c,0x6c,0x66,0x66,0xe6,0xe6,0x00,0x00,
0xf0,0xf0,0x60,0x60,0x60,0x60,0x60,0x60,0x62,0x62,0x66,0x66,0xfe,0xfe,0x00,0x00,
0x82,0x82,0xc6,0xc6,0xee,0xee,0xfe,0xfe,0xd6,0xd6,0xc6,0xc6,0xc6,0xc6,0x00,0x00,
0xc6,0xc6,0xe6,0xe6,0xf6,0xf6,0xde,0xde,0xce,0xce,0xc6,0xc6,0xc6,0xc6,0x00,0x00,
0x38,0x38,0x6c,0x6c,0xc6,0xc6,0xc6,0xc6,0xc6,0xc6,0x6c,0x6c,0x38,0x38,0x00,0x00,
0xfc,0xfc,0x66,0x66,0x66,0x66,0x7c,0x7c,0x60,0x60,0x60,0x60,0xf0,0xf0,0x00,0x00,
0x38,0x38,0x6c,0x6c,0xc6,0xc6,0xc6,0xc6,0xc6,0xc6,0x6c,0x6c,0x3c,0x3c,0x06,0x06,
0xfc,0xfc,0x66,0x66,0x66,0x66,0x7c,0x7c,0x6c,0x6c,0x66,0x66,0xe3,0xe3,0x00,0x00,
0x3c,0x3c,0x66,0x66,0x70,0x70,0x38,0x38,0x0e,0x0e,0x66,0x66,0x3c,0x3c,0x00,0x00,
0x7e,0x7e,0x5a,0x5a,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x3c,0x3c,0x00,0x00,
0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x3e,0x3e,0x00,0x00,
0xc3,0xc3,0xc3,0xc3,0x66,0x66,0x66,0x66,0x3c,0x3c,0x3c,0x3c,0x18,0x18,0x00,0x00,
0xc6,0xc6,0xc6,0xc6,0xc6,0xc6,0xd6,0xd6,0xfe,0xfe,0xee,0xee,0xc6,0xc6,0x00,0x00,
0xc3,0xc3,0x66,0x66,0x3c,0x3c,0x18,0x18,0x3c,0x3c,0x66,0x66,0xc3,0xc3,0x00,0x00,
0xc3,0xc3,0xc3,0xc3,0x66,0x66,0x3c,0x3c,0x18,0x18,0x18,0x18,0x3c,0x3c,0x00,0x00,
0xfe,0xfe,0xc6,0xc6,0x8c,0x8c,0x18,0x18,0x32,0x32,0x66,0x66,0xfe,0xfe,0x00,0x00,
0x3c,0x3c,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x3c,0x3c,0x00,0x00,
0xc0,0xc0,0x60,0x60,0x30,0x30,0x18,0x18,0x0c,0x0c,0x06,0x06,0x03,0x03,0x00,0x00,
0x3c,0x3c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x3c,0x3c,0x00,0x00,
0x10,0x10,0x38,0x38,0x6c,0x6c,0xc6,0xc6,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xfe,0xfe,
0x18,0x18,0x18,0x18,0x0c,0x0c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x3c,0x3c,0x06,0x06,0x1e,0x1e,0x66,0x66,0x3b,0x3b,0x00,0x00,
0xe0,0xe0,0x60,0x60,0x6c,0x6c,0x76,0x76,0x66,0x66,0x66,0x66,0x3c,0x3c,0x00,0x00,
0x00,0x00,0x00,0x00,0x3c,0x3c,0x66,0x66,0x60,0x60,0x66,0x66,0x3c,0x3c,0x00,0x00,
0x0e,0x0e,0x06,0x06,0x36,0x36,0x6e,0x6e,0x66,0x66,0x66,0x66,0x3b,0x3b,0x00,0x00,
0x00,0x00,0x00,0x00,0x3c,0x3c,0x66,0x66,0x7e,0x7e,0x60,0x60,0x3c,0x3c,0x00,0x00,
0x1c,0x1c,0x36,0x36,0x30,0x30,0x78,0x78,0x30,0x30,0x30,0x30,0x78,0x78,0x00,0x00,
0x00,0x00,0x00,0x00,0x3b,0x3b,0x66,0x66,0x66,0x66,0x3c,0x3c,0xc6,0xc6,0x7c,0x7c,
0xe0,0xe0,0x60,0x60,0x6c,0x6c,0x76,0x76,0x66,0x66,0x66,0x66,0xe6,0xe6,0x00,0x00,
0x18,0x18,0x00,0x00,0x38,0x38,0x18,0x18,0x18,0x18,0x18,0x18,0x3c,0x3c,0x00,0x00,
0x06,0x06,0x00,0x00,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x66,0x66,0x3c,0x3c,
0xe0,0xe0,0x60,0x60,0x66,0x66,0x6c,0x6c,0x78,0x78,0x6c,0x6c,0xe6,0xe6,0x00,0x00,
0x38,0x38,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x3c,0x3c,0x00,0x00,
0x00,0x00,0x00,0x00,0x66,0x66,0x77,0x77,0x6b,0x6b,0x63,0x63,0x63,0x63,0x00,0x00,
0x00,0x00,0x00,0x00,0x7c,0x7c,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x00,0x00,
0x00,0x00,0x00,0x00,0x3c,0x3c,0x66,0x66,0x66,0x66,0x66,0x66,0x3c,0x3c,0x00,0x00,
0x00,0x00,0x00,0x00,0xdc,0xdc,0x66,0x66,0x66,0x66,0x7c,0x7c,0x60,0x60,0xf0,0xf0,
0x00,0x00,0x00,0x00,0x3d,0x3d,0x66,0x66,0x66,0x66,0x3e,0x3e,0x06,0x06,0x07,0x07,
0x00,0x00,0x00,0x00,0xec,0xec,0x76,0x76,0x66,0x66,0x60,0x60,0xf0,0xf0,0x00,0x00,
0x00,0x00,0x00,0x00,0x3e,0x3e,0x60,0x60,0x3c,0x3c,0x06,0x06,0x7c,0x7c,0x00,0x00,
0x08,0x08,0x18,0x18,0x3e,0x3e,0x18,0x18,0x18,0x18,0x1a,0x1a,0x0c,0x0c,0x00,0x00,
0x00,0x00,0x00,0x00,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x3b,0x3b,0x00,0x00,
0x00,0x00,0x00,0x00,0x66,0x66,0x66,0x66,0x66,0x66,0x3c,0x3c,0x18,0x18,0x00,0x00,
0x00,0x00,0x00,0x00,0x63,0x63,0x6b,0x6b,0x6b,0x6b,0x36,0x36,0x36,0x36,0x00,0x00,
0x00,0x00,0x00,0x00,0x63,0x63,0x36,0x36,0x1c,0x1c,0x36,0x36,0x63,0x63,0x00,0x00,
0x00,0x00,0x00,0x00,0x66,0x66,0x66,0x66,0x66,0x66,0x3c,0x3c,0x18,0x18,0x70,0x70,
0x00,0x00,0x00,0x00,0x7e,0x7e,0x4c,0x4c,0x18,0x18,0x32,0x32,0x7e,0x7e,0x00,0x00,
0x0e,0x0e,0x18,0x18,0x18,0x18,0x70,0x70,0x18,0x18,0x18,0x18,0x0e,0x0e,0x00,0x00,
0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x00,0x00,
0x70,0x70,0x18,0x18,0x18,0x18,0x0e,0x0e,0x18,0x18,0x18,0x18,0x70,0x70,0x00,0x00,
0x72,0x72,0x9c,0x9c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
};
#endif /* FONT_MAIN */
