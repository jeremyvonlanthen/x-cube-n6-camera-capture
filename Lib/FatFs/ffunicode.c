/*------------------------------------------------------------------------*/
/* Minimal Unicode / OEM conversion for FatFs LFN support (ASCII only)     */
/*------------------------------------------------------------------------*/
/* The full ffunicode.c from the FatFs distribution carries large code-page */
/* conversion tables (CP437 here).  DIAS only ever creates 7-bit ASCII file  */
/* names (digits, '-', '_', '.', and the JPG/MP4 extensions), so those       */
/* tables are unnecessary: these three functions are all that ff.c (with     */
/* FF_USE_LFN != 0) needs to link, implemented for the ASCII range only.     */
/* Characters outside 7-bit ASCII are reported as unsupported (0).           */
/*------------------------------------------------------------------------*/

#include "ff.h"

#if FF_USE_LFN != 0

/* OEM (code page) code -> Unicode */
WCHAR ff_oem2uni (WCHAR oem, WORD cp)
{
    (void)cp;
    return (oem < 0x80) ? oem : 0;
}

/* Unicode -> OEM (code page) code */
WCHAR ff_uni2oem (DWORD uni, WORD cp)
{
    (void)cp;
    return (uni < 0x80) ? (WCHAR)uni : 0;
}

/* Unicode up-case conversion (ASCII) */
DWORD ff_wtoupper (DWORD uni)
{
    if (uni >= (DWORD)'a' && uni <= (DWORD)'z')
        return uni - ((DWORD)'a' - (DWORD)'A');
    return uni;
}

#endif /* FF_USE_LFN != 0 */
