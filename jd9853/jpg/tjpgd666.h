/*----------------------------------------------------------------------------/
/ TJpgDec - Tiny JPEG Decompressor include file               (C)ChaN, 2020
/----------------------------------------------------------------------------*/
#ifndef DEF_TJPGDEC
#define DEF_TJPGDEC

/*---------------------------------------------------------------------------*/
/* System Configurations */

#define	JD_SZBUF		512		/* Size of stream input buffer */
#define JD_FORMAT		0		/* 0: RGB888 (3 bytes/pixel), 1: RGB666 (2 bytes/pixel) */
#define	JD_USE_SCALE	1		/* Use descaling feature for output */
#define JD_TBLCLIP		1		/* Use table for saturation */

/*---------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

/* Error codes (sin cambios) */
typedef enum { JDR_OK = 0, JDR_INTR, JDR_INP, JDR_MEM1, JDR_MEM2, JDR_PAR, JDR_FMT1, JDR_FMT2, JDR_FMT3 } JRESULT;

typedef struct { uint16_t left, right, top, bottom; } JRECT;

typedef struct JDEC JDEC;
struct JDEC {
	unsigned int dctr;
	uint8_t* dptr;
	uint8_t* inbuf;
	uint8_t dmsk;
	uint8_t scale;
	uint8_t msx, msy;
	uint8_t qtid[3];
	int16_t dcv[3];
	uint16_t nrst;
	uint16_t width, height;
	uint8_t* huffbits[2][2];
	uint16_t* huffcode[2][2];
	uint8_t* huffdata[2][2];
	int32_t* qttbl[4];
	void* workbuf;
	uint8_t* mcubuf;
	void* pool;
	unsigned int sz_pool;
	unsigned int (*infunc)(JDEC*, uint8_t*, unsigned int);
	void* device;
	uint16_t x_offs;
	uint16_t y_offs;
};

JRESULT jd_prepare (JDEC* jd, unsigned int (*infunc)(JDEC*,uint8_t*,unsigned int), void* pool, unsigned int sz_pool, void* dev);
JRESULT jd_decomp (JDEC* jd, int (*outfunc)(JDEC*,void*,JRECT*), uint8_t scale);

#ifdef __cplusplus
}
#endif

#endif /* _TJPGDEC */
